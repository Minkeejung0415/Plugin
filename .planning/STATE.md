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
- **Phase 6 operator flow (confirmed):** the operator launches ONLY the Open Ephys GUI; the acquisition-board plugin starts OpenSim Live itself. `AcqBoardRedPitaya::launchOpenSimLive()` (acqboard.ccp:1694) runs `kOpenSimWorkDir\opensim_live_realtime.py` = `C:\Users\justi\Open-Sim--Bio-Mech\opensim_live_realtime.py` — the executed copy that was fixed. UAT: turn on Open Ephys, click "OpenSim Live", move knee → live angle tracks the OpenSim-console `[HUD-DIAG]`/`[HUD-UPDATE]` value on the active readout; switching selected joint changes the label.
  - **Launch-path verified safe:** only TWO copies of opensim_live_realtime.py exist (repo + Open-Sim--Bio-Mech), both at fixed SHA `c8ee2aca…`. No stale copy at the GUI launch dir (`Downloads\open-ephys-v1.0.2-windows\open-ephys\`). `launchOpenSimLive()` also copies the script from the GUI's CWD over the workdir copy (acqboard.ccp:1708-1712) — harmless here because (a) deployed DLL is Jun 11, copy-logic source is Jun 12 (likely not in the running binary), and (b) no source file exists at the GUI CWD to copy. If the plugin is ever REBUILT, ensure the GUI's working dir contains the fixed script (or none) so it cannot clobber the fix.
  - **Residual gap (not fixed this phase):** whether the Open Ephys plugin UI actually *renders* the port-5001 angle packet is UNVERIFIED (plugin C++ untouched). If active strategy is `udp_feedback` and the plugin UI shows nothing, the `[HUD-UPDATE]` console line is ground-truth that the fix works; plugin-side rendering would be a separate follow-up.

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
