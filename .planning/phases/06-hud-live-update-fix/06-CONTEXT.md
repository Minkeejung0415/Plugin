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

### Chosen fix — Option 1: DecorationGenerator
- Replace the one-shot `addDecoration` + `setText` pattern with a Simbody
  `DecorationGenerator` subclass whose `generateDecorations(state, decorations)`
  is invoked by the visualizer every `report()`/`show()` and appends a freshly
  constructed `DecorativeText(compact)` carrying the current HUD string.
- Register it via `viz.addDecorationGenerator(gen)` (exact method name to be
  confirmed by research against the installed OpenSim 4.5 build).
- The generator reads the current compact HUD string from shared state that the
  IK loop updates each frame (e.g. a module-level holder set in
  `_render_joint_display_hud`), so the generator stays cheap and never calls IK.

### Capability probe (locked requirement)
- Before committing to option 1, the plan MUST include a runtime probe that prints
  whether `addDecorationGenerator` exists on the live `viz` object and whether
  `simbody.DecorationGenerator` is Python-subclassable on this build
  (`_pick_hud_strategy` already enumerates `dir(viz)` near line 575/581 — extend it).

### Fallback chain (locked, if option 1 is unavailable on this SWIG build)
1. Window-title augmentation via `setWindowTitle` — BUT this build already logged
   `[WARN] setWindowTitle unavailable` (`_try_set_window_title`), so treat as likely dead.
2. Drive the readout through the existing port-5001 angle-feedback UDP packet
   (`_send_angle_feedback`, already wired) into the Open Ephys plugin UI.
   Document this as the operator-visible readout if the viewport cannot update.

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
  moves the knee and the viewport number changes in lockstep with the console
  `[HUD-DIAG] knee_angle_r=` value; switching the selected joint changes the label.
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
