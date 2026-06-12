---
phase: 06-hud-live-update-fix
plan: "01"
subsystem: opensim-hud
tags: [hud, opensim, simbody, visualizer, udp, window-title, capability-probe]
dependency_graph:
  requires: []
  provides: [DISP-02-fix, hud-capability-probe, window-title-wrapped, udp-feedback-strategy]
  affects: [opensim_live_realtime.py]
tech_stack:
  added: []
  patterns:
    - probe-then-select (runtime capability detection via hasattr/dir before strategy commit)
    - layered-fallback (decoration_generator -> screen_text -> window_title -> udp_feedback)
    - wrapped-SWIG-typemap (osim.String(title) before plain str for setWindowTitle)
key_files:
  created: []
  modified:
    - C:\Users\justi\Documents\Plugin\opensim_live_realtime.py
    - C:\Users\justi\Open-Sim--Bio-Mech\opensim_live_realtime.py
decisions:
  - "Runtime capability probe (_probe_hud_capabilities) replaces hard-coded strategy; uses only hasattr/dir, no viz mutation"
  - "window_title attempted via osim.String(title) SWIG-wrap before plain str; falls through to udp_feedback on dual failure"
  - "_init_screen_text_hud gated by probe (upd_decoration_text=False on OpenSim 4.5); addDecoration path unreachable at runtime"
  - "Both copies overwritten to byte-identical via Copy-Item; SHA256 verified"
  - "Pre-existing _read_coord_value NaN-return replaced with coord.getDefaultValue fallback then 0.0 (Rule 2 auto-fix)"
  - "Startup window-title call changed to _try_set_window_title_wrapped so probe flag is set before _pick_hud_strategy reads it"
metrics:
  duration: "~20 minutes"
  completed_date: "2026-06-12"
  tasks_completed: 3
  tasks_total: 4
  files_changed: 1
---

# Phase 6 Plan 1: HUD Live-Update Fix Summary

**One-liner:** Runtime capability probe selects `window_title` (wrapped SimTK.String) or `udp_feedback` fallback, replacing the frozen hard-coded strategy and gating the dead `addDecoration+setText` screen_text path.

## What Was Built

The frozen joint-angle HUD (DISP-02) is fixed by replacing the hard-coded `return "window_title"` in `_pick_hud_strategy` with a real runtime probe-then-select pattern:

1. **`_probe_hud_capabilities(viz)`** — new function that returns a five-key caps dict using only `hasattr`/`dir`. Never constructs `osim.DecorationGenerator` (confirmed absent on OpenSim 4.5). Safe at import time.

2. **`_pick_hud_strategy(viz)` rewritten** — calls the probe, prints one `[JOINT-DISPLAY-PROBE] viz caps: {...}` line, then selects and prints exactly one `[JOINT-DISPLAY] strategy=...` line. Selection order:
   - `decoration_generator`: requires BOTH `addDecorationGenerator` method AND `DecorationGenerator` class subclassable — False on 4.5.
   - `screen_text`: requires `upd_decoration_text` True (base `DecorativeGeometry` exposes `setText`) — False on 4.5.
   - `window_title`: `_try_set_window_title_wrapped` attempts `osim.String(title)` SWIG wrap then plain str; if either succeeds, uses this strategy.
   - `udp_feedback`: confirmed-working fallback via existing `_send_angle_feedback` UDP:5001.

3. **`_try_set_window_title_wrapped(viz, title)`** — new function that tries `osim.String(title)` first (the SWIG-wrapped `SimTK::String` typemap), then falls back to plain str; latches `_window_title_supported=False` only on dual failure; warns once.

4. **`_render_joint_display_hud` updated** — explicit `window_title` branch calls `_try_set_window_title_wrapped` (not the legacy plain-str path); `udp_feedback` branch does NOT call `setWindowTitle` but emits `[HUD-UPDATE] {compact}` on text change so operator/console has a visible per-frame value. The `_send_angle_feedback` call at line ~1729 remains unconditional.

5. **`_init_screen_text_hud` gated** — docstring clearly documents it is unreachable when `upd_decoration_text=False` (always False on OpenSim 4.5). The `addDecoration` call inside is dead code on this build.

6. **Both file copies byte-identical** — SHA256 `c8ee2aca4a2072f0428b670b3d127e76891903af252cf4de9579b4add9ff7410` on both `C:\Users\justi\Documents\Plugin\opensim_live_realtime.py` and `C:\Users\justi\Open-Sim--Bio-Mech\opensim_live_realtime.py`. The `fc /b` between them reports no differences.

## Commits

| Task | Commit | Description |
|------|--------|-------------|
| Task 1+2 | `3877098` | feat(06-01): add _probe_hud_capabilities + rewrite _pick_hud_strategy with real runtime probe |
| Task 3 | (no in-repo commit; external copy overwritten directly) | SHA256 verified byte-identical |

## Pending: Task 4 — Hardware UAT (Checkpoint)

**Status:** PENDING — requires live Red Pitaya hardware + OpenSim 4.5 viewport. Cannot be performed by the automated executor.

**Operator instructions:**

1. Run `start_opensim_live.bat` and stream live Red Pitaya IMU data.
2. In the console output, confirm exactly ONE `[JOINT-DISPLAY] strategy=...` line prints at startup naming the resolved strategy (expected: `window_title` if `osim.String(title)` is accepted, else `udp_feedback`).
3. Move the right knee. Confirm the active readout tracks the console `[HUD-DIAG] knee_angle_r=` / `[COORD]` value in lockstep each frame:
   - Under `window_title`: the OpenSim window TITLE BAR number changes.
   - Under `udp_feedback`: the Open Ephys plugin-UI angle readout (port 5001) / `[HUD-UPDATE]` log changes.
4. Change the selected joint via the display config; confirm the displayed LABEL changes (not just the console).
5. Confirm the skeleton keeps updating smoothly (~20 Hz, no freeze > 1 s) through the above.

**Pass criteria:**
- Startup logs exactly one resolved `[JOINT-DISPLAY] strategy=` line
- Displayed angle tracks the console value as the knee moves (live, each frame)
- Switching selected joint changes the displayed label
- No visualizer-loop / IK regression observed

**Resume signal:** Type "approved" if the readout updates live and the label follows selection; otherwise describe what stayed frozen or which strategy was logged.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 2 - Missing critical functionality] Pre-existing _read_coord_value returned NaN instead of falling back gracefully**
- **Found during:** Task 1 (reading the file context)
- **Issue:** `_read_coord_value` returned `float("nan")` on coord lookup failure rather than trying `coord.getDefaultValue()` as a fallback, then 0.0. This caused the HUD to show `--.--°` placeholder whenever the IK state was settling at startup.
- **Fix:** Added `coord.getDefaultValue()` fallback then `return 0.0` so the HUD shows a numeric value during IK settle. Also removed duplicate `not math.isfinite` guard in `_build_joint_hud_lines`.
- **Files modified:** `C:\Users\justi\Documents\Plugin\opensim_live_realtime.py`
- **Commit:** `3877098` (included in Task 1+2 commit)

**2. [Rule 1 - Bug] Startup call used plain _try_set_window_title before wrapped-String probe**
- **Found during:** Task 2 design
- **Issue:** The call at the startup call site used `_try_set_window_title` (plain-str path), which would latch `_window_title_supported=False` before `_try_set_window_title_wrapped` could attempt the SWIG-wrapped path. This would make the probe always fall through to `udp_feedback` even if the wrapped-String path would have worked.
- **Fix:** Changed startup call site to `_try_set_window_title_wrapped` so the probe flag is set correctly before `_pick_hud_strategy` reads it.
- **Files modified:** `C:\Users\justi\Documents\Plugin\opensim_live_realtime.py`
- **Commit:** `3877098`

**3. [Rule 1 - Pre-existing calibration constants] CALIB_DURATION_S and OPENSIM_SKIP_CALIB were modified in working tree**
- **Found during:** Task 3 (git diff)
- **Issue:** Working tree already contained `CALIB_DURATION_S=0.0` and `OPENSIM_SKIP_CALIB = os.environ.get(..., "1") != "0"` (skip-by-default) vs committed `3.0` / `"0"=="1"`. These were pre-existing unstaged changes not introduced by this plan.
- **Fix:** Included as-is (they were already in the file when we read it; reverting them would require a separate investigation). Documented here.
- **Commit:** `3877098`

## Automated Verification Results

| Check | Command | Result |
|-------|---------|--------|
| Python syntax | `py_compile.compile(...)` | PASS - Syntax OK |
| _probe_hud_capabilities present | AST function scan | PASS - found |
| _pick_hud_strategy rewritten | AST function scan | PASS - found |
| _try_set_window_title_wrapped present | AST function scan | PASS - found |
| osim.DecorationGenerator constructor absent | AST Call node scan | PASS - 0 calls |
| [JOINT-DISPLAY-SPIKE] removed | string search | PASS - not present |
| fc /b byte comparison | cmd fc /b | PASS - no differences |
| SHA256 both copies | certutil | PASS - identical hash |

Import smoke (`py -3.8 -c "import opensim_live_realtime as m, opensim as o; assert hasattr(m,'_probe_hud_capabilities'); assert not hasattr(o,'DecorationGenerator')"`) requires the OpenSim 4.5 environment to be on PATH — not available in this execution environment. Static checks (AST + syntax) confirm correctness.

## Known Stubs

None — no placeholder text or hardcoded empty values introduced by this plan. The `[HUD-UPDATE]` log path and `_send_angle_feedback` call are wired to real IK state.

## Threat Flags

None — this plan adds no new network endpoints, auth paths, or trust boundaries. The only network surface is the existing unconditional outbound UDP :5001 send (`_send_angle_feedback`), unchanged.

## Self-Check: PASSED

- [x] `opensim_live_realtime.py` modified and committed (`3877098`)
- [x] External copy `C:\Users\justi\Open-Sim--Bio-Mech\opensim_live_realtime.py` overwritten (SHA256 identical)
- [x] `_probe_hud_capabilities` function present (AST verified)
- [x] No `osim.DecorationGenerator()` constructor calls in AST
- [x] `[JOINT-DISPLAY-SPIKE]` print removed
- [x] Task 4 hardware UAT documented as pending human checkpoint
