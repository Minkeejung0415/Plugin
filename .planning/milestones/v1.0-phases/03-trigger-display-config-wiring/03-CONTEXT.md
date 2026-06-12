# Phase 3: Trigger → Display Config Wiring - Context

**Gathered:** 2026-06-10  
**Status:** Ready for planning

<domain>
## Phase Boundary

Connect `ACQBOARD TRIGGER` and **Apply Display** to atomic writes of `opensim_joint_display_config.json` in `kOpenSimWorkDir`. Must not block UDP v2 streaming.

</domain>

<decisions>
## Implementation Decisions

### Trigger hook
- After valid TTL trigger in `devicethread.cpp`, call `writeJointDisplayConfig()` on Red Pitaya board

### Atomic write
- `.tmp` file then rename; increment `seq`; include `trigger_ts`

### Apply Display
- Utility button in device editor calls same write path (OPS-01)

### Claude's Discretion
- Config write on trigger thread — keep JSON write fast, no file I/O in UDP hot path

</decisions>

<deferred>
## Deferred Ideas

- Per-TTL preset maps (v2)

</deferred>
