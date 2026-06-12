# Phase 6: HUD Live-Update Fix - Context

**Gathered:** 2026-06-12
**Status:** Ready for planning
**Source:** Diagnosis from hardware UAT failure of Phase 4 / DISP-02 (debug session 2026-06-12)

<domain>
## Phase Boundary

Remediation of a single rendering defect in the OpenSim Live HUD. The selected
joint-angle readout in the Simbody viewport is frozen on the placeholder string
`knee_r: --.--°` and never updates, even though IK solves correctly and the
console logs the true changing angle every frame.

In scope:
- Make the in-viewport joint-angle text update live, each visualizer frame.
- Keep the existing display-filter behavior (only selected joints, abbreviated
  1-decimal format `knee_r: 42.1°`, empty-selection clean default) intact.
- Apply the fix to BOTH copies of `opensim_live_realtime.py` (see canonical refs)
  so the executed copy and the source-of-truth copy match.

Out of scope:
- C++ plugin / device editor changes (joint selection already works, Phase 2).
- Trigger wiring (Phase 3), IK math, UDP packet format.
- Any new requirement — this only closes the deferred DISP-02 verification.
</domain>

<decisions>
## Implementation Decisions

### Root cause (locked — confirmed by code trace)
- `_init_screen_text_hud()` creates the HUD once with
  `osim.DecorativeText("knee_r: --.--°")` and registers it via
  `viz.addDecoration(0, xform, text)`.
- Simbody's `Visualizer::addDecoration` stores the decoration **by value (a copy)**.
  The per-frame update path `_render_joint_display_hud` → `_hud_screen_text.setText(compact)`
  mutates the original retained Python handle, which the renderer no longer references.
  Result: the on-screen text is permanently the constructor string.
- `_read_coord_value()` and the IK loop are correct — `[COORD]` / `[HUD-DIAG]`
  console lines show the real changing `knee_angle_r`. Only the render path is dead.

### Option 1 (DecorationGenerator) is BLOCKED — confirmed by research 2026-06-12
- `06-RESEARCH.md` introspected the installed `C:\OpenSim 4.5` Python bindings:
  `SimTK::DecorationGenerator` is **not wrapped** (zero `*Generator` types in the
  861-symbol module), so it cannot be constructed or subclassed — no SWIG director.
  Every in-viewport live-text path is also dead: `addDecoration` copies by value,
  there is no `removeDecoration`/`clearDecorations`, and `updDecoration(i)` returns
  base `DecorativeGeometry` with no `setText`. **Do not attempt option 1 or any
  in-Simbody-viewport text mutation on this build.**

### Chosen fix — LAYERED strategy (operator-selected 2026-06-12): window-title → UDP
The readout cannot live inside the Simbody viewport on this build, so it moves to
the OpenSim window title bar, with the already-wired UDP feed as the fallback.

1. **Runtime capability probe** (locked): rewrite the hard-coded `_pick_hud_strategy`
   (currently `return "window_title"` ~line 712) to actually probe the live `viz`
   object via `hasattr`/`dir(viz)` and select a strategy. Use the verified probe
   recipe in `06-RESEARCH.md` Q5. On this build it must resolve to `window_title`
   (if the wrapped-String title works) else `udp_feedback`.
2. **Strategy `window_title`** (primary, keeps readout in the OpenSim window):
   revive `setWindowTitle` by passing a **SWIG-wrapped `SimTK.String(title)`**
   instead of a plain Python `str` (the plain-str typemap is what failed in
   `_try_set_window_title`, line 543, latching `_window_title_supported=False`).
   Update the title each frame with the compact HUD string. The window-title-wrap
   is the one in-OpenSim path still worth trying; it is UNVERIFIED — the plan must
   test it at runtime and fall through cleanly if the wrap also rejects.
3. **Strategy `udp_feedback`** (fallback, already transmitting): the per-frame
   `_send_angle_feedback` packet on port 5001 is the confirmed-working live readout.
   When this strategy is active, the operator reads the angle in the Open Ephys
   plugin UI. **No C++ plugin rebuild in this phase** — Python already sends the
   packet; document the consumer as the operator-visible readout and note
   plugin-side rendering as a separate follow-up if needed.
- The active strategy must be chosen once at startup by the probe and logged
  (`[JOINT-DISPLAY] strategy=...`) so UAT can see which path is live.
- Remove or gate the dead `addDecoration`+`setText` `screen_text` path so it can
  never silently re-freeze the readout (this is what masked the bug through Phase 4).

### Claude's Discretion
- Exact module structure of the generator (nested class vs module-level), the
  shared-state holder mechanism, and how `_pick_hud_strategy` selects between
  `decoration_generator` / `window_title` / `udp_feedback` strategies.
- Whether to remove the now-dead `setText` branch or keep it behind the strategy switch.
</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### The defect and both copies of the script
- `opensim_live_realtime.py` (source-of-truth copy in `C:\Users\justi\Documents\Plugin\`) —
  functions `_init_screen_text_hud`, `_pick_hud_strategy`, `_render_joint_display_hud`,
  `_build_joint_hud_lines`, `_read_coord_value`.
- `C:\Users\justi\Open-Sim--Bio-Mech\opensim_live_realtime.py` — the copy actually
  launched by `start_opensim_live.bat`; the fix MUST land here too or it won't run.
- `opensim_joint_catalog.py` — `abbrev_for`, `coordinate_for`, `format_angle_line`
  (HUD string formatting; unchanged but referenced).

### Prior phase that introduced the HUD
- `.planning/phases/01-display-config-contract-spike/01-RESEARCH.md` — original
  Simbody text-overlay spike (what was tried).
- `.planning/phases/04-filtered-display-beside-sim-clock/04-VERIFICATION.md` —
  the `human_needed` verification that deferred DISP-02 to manual UAT.
</canonical_refs>

<specifics>
## Specific Ideas

- Verification must be observable, not inspection-only this time: the operator
  moves the knee and the displayed number (OpenSim window TITLE BAR under the
  `window_title` strategy, or the Open Ephys plugin-UI readout under
  `udp_feedback`) changes in lockstep with the console `[HUD-DIAG] knee_angle_r=`
  value; switching the selected joint changes the displayed label.
- The chosen strategy is logged once at startup (`[JOINT-DISPLAY] strategy=...`);
  Phase 5's manual UAT checklist should be updated to look at the active readout
  surface rather than the (now-removed) in-viewport text.
- The visualizer loop target is ~20 Hz (`OPENSIM_LIVE_VISUALIZER_RATE`); the
  generator must not call IK or do per-frame allocation that stalls it.
</specifics>

<deferred>
## Deferred Ideas

- Multi-line HUD layout / per-joint color coding — out of scope, DISP-02 is a
  single compact line.
- Rebuilding the C++ plugin — not required for this Python-only render fix.
</deferred>

---

*Phase: 06-hud-live-update-fix*
*Context gathered: 2026-06-12 via debug diagnosis (option 1 selected by operator)*
