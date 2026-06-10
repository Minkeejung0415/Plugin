# Pitfalls Research — OpenSim View Angle on Trigger

**Project:** Open Ephys Red Pitaya Plugin  
**Researched:** 2026-06-10  
**Confidence:** MEDIUM-HIGH

## Pitfall 1: Simbody Python Camera API Gap

**Warning signs:** `AttributeError` on `setCameraTransform` or similar during Phase 2  
**Prevention:** Phase 1 spike — enumerate `dir(viz)` on target OpenSim 4.5 install; document fallback (window title, fixed camera presets via model coordinates)  
**Phase:** 1

## Pitfall 2: UDP Packet Format Regression

**Warning signs:** Live IK freezes or wrong sensor count after plugin changes  
**Prevention:** Keep view config in separate JSON file; do not append fields to v2 quaternion packets  
**Phase:** 1

## Pitfall 3: File Write Race (Plugin vs Python)

**Warning signs:** Partial JSON read, stale view after rapid triggers  
**Prevention:** Write to temp file + atomic rename; include monotonic `seq` field; Python ignores older seq  
**Phase:** 2

## Pitfall 4: Trigger Never Fires View Change

**Warning signs:** UI selection works but view unchanged during experiment  
**Prevention:** Also write config on selection change (optional preview) AND on trigger; add "Apply View Now" test button in device editor for debugging  
**Phase:** 3

## Pitfall 5: Label Not Visible Beside Sim Clock

**Warning signs:** Simbody has no text overlay API  
**Prevention:** Research SimbodyVisualizer text APIs; fallback: composite window title `"Connect OpenSim | Lateral Right | t=1.23s"` or OS-level overlay — document limitation if native overlay impossible  
**Phase:** 4

## Pitfall 6: Hardcoded Windows Paths

**Warning signs:** Config written to wrong directory on different machine  
**Prevention:** Reuse `kOpenSimWorkDir` constant from `acqboard.ccp`; centralize in one header  
**Phase:** 2

## Pitfall 7: Acquisition Thread Blocking

**Warning signs:** Sample drops when trigger fires  
**Prevention:** View config write on message thread or short non-blocking IO; never block UDP send path in `sendOpenSimQuaternionPacket`  
**Phase:** 3
