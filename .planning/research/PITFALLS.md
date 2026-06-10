# Pitfalls Research — Joint Angle Display on Trigger

**Project:** Open Ephys Red Pitaya Plugin  
**Researched:** 2026-06-10  
**Revised:** 2026-06-10 (scope correction)  
**Confidence:** MEDIUM-HIGH

## Pitfall 1: Simbody On-Screen Text API Gap

**Warning signs:** No method to render custom text beside `setShowSimTime` output  
**Prevention:** Phase 1 spike — enumerate `dir(viz)` on target OpenSim 4.5 install; document fallback (window title with joint values, external status line)  
**Phase:** 1

## Pitfall 2: UDP Packet Format Regression

**Warning signs:** Live IK freezes or wrong sensor count after plugin changes  
**Prevention:** Keep display config in separate JSON file; do not append fields to v2 quaternion packets  
**Phase:** 1

## Pitfall 3: File Write Race (Plugin vs Python)

**Warning signs:** Partial JSON read, stale filter after rapid triggers  
**Prevention:** Write to temp file + atomic rename; include monotonic `seq` field; Python ignores older seq  
**Phase:** 3

## Pitfall 4: Sensor Names vs Coordinate Names Confusion

**Warning signs:** UI shows `femur_r_imu` but display expects `hip_flexion_r`  
**Prevention:** Document mapping in catalog; plugin stores coordinate names; optional UI grouping by segment  
**Phase:** 2

## Pitfall 5: Display Clutter / Unreadable HUD

**Warning signs:** Too many joints selected; text overflows beside clock  
**Prevention:** Cap recommended joints (open question); compact format (`knee_r: 42.1°`); v2 overlay styling  
**Phase:** 4

## Pitfall 6: Hardcoded Windows Paths

**Warning signs:** Config written to wrong directory on different machine  
**Prevention:** Reuse `kOpenSimWorkDir` constant from `acqboard.ccp`; centralize in one header  
**Phase:** 2

## Pitfall 7: Acquisition Thread Blocking

**Warning signs:** Sample drops when trigger fires  
**Prevention:** Display config write on message thread or short non-blocking IO; never block UDP send path in `sendOpenSimQuaternionPacket`  
**Phase:** 3

## Pitfall 8: Invalid Coordinate Names in Config

**Warning signs:** `coord_set.get(name)` throws; display blank  
**Prevention:** Plugin validates against curated catalog; Python skips unknown names with warning log  
**Phase:** 2

## Superseded Pitfalls

Simbody **camera** API gap (prior Pitfall 1) — no longer applicable.
