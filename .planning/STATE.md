---
gsd_state_version: 1.0
milestone: v1.0
milestone_name: milestone
current_phase: 06
status: awaiting-hardware-uat
last_updated: "2026-06-12T19:05:00Z"
progress:
  total_phases: 6
  completed_phases: 5
  total_plans: 8
  completed_plans: 7
  percent: 90
---

# Project State

**Project:** Joint Angle Display on Trigger  
**Current phase:** 06
**Status:** Executing Phase 06

## Progress

| Phase | Name | Status | Plans |
|-------|------|--------|-------|
| 1 | Display Config & Spike | ✓ Complete | 3/3 |
| 2 | Plugin Joint Selector | ✓ Complete | 1/1 |
| 3 | Trigger Wiring | ✓ Complete | 1/1 |
| 4 | Filtered Display | ✓ Complete | 1/1 |
| 5 | Integration Verify | ✓ Complete | 1/1 |
| 6 | HUD Live-Update Fix | ◷ Code complete; Task 4 hardware UAT pending | 0/1 (partial) |

**Overall:** 5/6 phases complete; Phase 6 code complete, hardware UAT pending

## Hardware Topology (confirmed 2026-06-12)

Live source is **ESP32-S3 (ICM-20948), not a physical Red Pitaya**, run via the **RP-compat gateway** path (`esp32/host/rp_compat_gateway.py` or `serial_tcp_bridge.py --plugin`): ESP32 USB → gateway presents as Red Pitaya (TCP :5000 + UDP :55001, `rp-f0f85a.local`→127.0.0.1) → **Open Ephys GUI plugin** (VQF fusion) → UDP quaternions → `opensim_live_realtime.py` (IK) → HUD. ESP is protocol-parity with RP, so `OPENSIM_LIVE_SOURCE=real_redpitaya` stays correct and the Phase 6 render-path fix is unaffected by the source swap. The Open Ephys GUI IS in the loop, so the `udp_feedback` (port-5001) fallback has a candidate consumer.

## Pending Manual UAT

- Live OpenSim 4.5 + ESP32-via-RP-compat-gateway: trigger → HUD filter, IK continuity (DISP-04)
- Plugin rebuild in Open Ephys GUI (C++ changes)
- **Phase 6 final approach — Live Angle readout IN the Open Ephys editor (option A).** In-viewport
  text proven IMPOSSIBLE on this OpenSim 4.5 build (verified directly against the installed Python
  bindings): no `DecorationGenerator` class to subclass, `updDecoration()` returns base
  `DecorativeGeometry` without `setText`, `osim.String` absent so `setWindowTitle` is unusable, no
  `removeDecoration`/`clearDecorations`. So the selected Display Joint's angle is surfaced in the
  device editor (Col 6, "Live Angle") via the UDP-5001 feed the plugin already receives.
  - **C++ change (commit 947185f):** `device editor.h/.cpp` add a `Live Angle` label + 10 Hz member
    Timer that calls `pollOpenSimAngleFeedback()` (previously defined-but-never-called) then
    `getLiveDisplayAngle()`. Joint-index spaces verified aligned (knee_angle_r = index 1 in both
    C++ `kDisplayJointNames` and Python `JOINT_OPTIONS`). Overlay files copied to
    `C:\Users\justi\dev\acquisition-board\Source\DeviceEditor.{cpp,h}`, built + INSTALLed to
    `C:\Users\justi\dev\GUI\Build\Debug\plugins\acquisition-board.dll` (2,100,736 B, 2026-06-12 13:56).
  - **CLONE MAP (corrected — earlier "only 2 copies" note was WRONG):** there are 3 git clones of
    `github.com/Minkeejung0415/Plugin.git`: `Documents\Plugin` (canonical edit repo + .planning, HEAD
    ahead with Phase 6), `C:\Users\justi\Plugin` (older, the runtime CWD the plugin copies the Python
    from), `dev\Plugin` (oldest). Plus the VS build source `C:\Users\justi\dev\acquisition-board\Source`
    (overlay target) and dev GUI `C:\Users\justi\dev\GUI\Build\Debug`. Fixed Python deployed to
    `C:\Users\justi\Plugin`, `Open-Sim--Bio-Mech`, and `Documents\Plugin`.
  - **PENDING LIVE TEST:** launch `C:\Users\justi\dev\GUI\Build\Debug\open-ephys.exe`, click OpenSim
    Live, move knee → editor "Live Angle" updates ~10 Hz tracking console `[HUD-DIAG] knee_angle_r=`;
    changing Display Joint dropdown changes the readout. Note: the editor readout works regardless of
    which Python copy runs (old or new both send 5001).

## Decisions

- Runtime capability probe (_probe_hud_capabilities) replaces hard-coded strategy; uses only hasattr/dir, no viz mutation
- window_title attempted via osim.String(title) SWIG-wrap before plain str; falls through to udp_feedback on dual failure
- _init_screen_text_hud gated by probe (upd_decoration_text=False on OpenSim 4.5); addDecoration path unreachable at runtime
- Both file copies overwritten to byte-identical via Copy-Item; SHA256 verified (c8ee2aca...)
- Startup window-title call changed to _try_set_window_title_wrapped so probe flag is set before _pick_hud_strategy reads it

## Blockers

None — code complete. Awaiting Task 4 hardware UAT (live Red Pitaya + OpenSim window). Operator must:
1. Run start_opensim_live.bat
2. Confirm [JOINT-DISPLAY] strategy=... line at startup
3. Move knee, confirm readout tracks console [HUD-DIAG] value
4. Change selected joint, confirm label changes
5. Confirm ~20 Hz no-freeze

## Analysis artifacts

- **VQF + IK pipeline map (optimization):** `.planning/analysis/vqf-ik-pipeline-map.md` — bottlenecks, timing table, tunables, ranked optimization matrix.

---
*State updated: 2026-06-12 — Phase 6 Tasks 1-3 code complete, Task 4 hardware UAT pending*
