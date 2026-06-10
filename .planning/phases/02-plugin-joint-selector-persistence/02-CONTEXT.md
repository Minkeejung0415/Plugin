# Phase 2: Plugin Joint Selector & Persistence - Context

**Gathered:** 2026-06-10  
**Status:** Ready for planning  
**Mode:** Auto-generated (autonomous smart discuss — locked defaults)

<domain>
## Phase Boundary

Add operator-facing multi-select for curated IK joint coordinates in the Red Pitaya device editor, with XML persistence. No trigger→config write in this phase (Phase 3).

</domain>

<decisions>
## Implementation Decisions

### UI layout
- Checkbox toggles labeled with catalog abbrev (`knee_r`, `hip_r`, …) below existing Red Pitaya controls
- Max 6 selections enforced in UI with status message when exceeded
- Optional JOIN-04: orange label when joint segment matches active `streamSensorNames`

### Catalog
- Mirror `opensim_joint_catalog.py` / locked defaults (7 flexion DOFs)
- C++ header `opensim_joint_catalog.h` shared with Python names

### Persistence
- XML child `<JOINT_DISPLAY><JOINT coordinate=… selected=…/></JOINT_DISPLAY>`
- `JointDisplaySeq` attribute preserves monotonic seq across save/load

### Claude's Discretion
- Toggle grid layout and exact pixel bounds follow existing editor columns

</decisions>

<code_context>
## Existing Code Insights

### Reusable Assets
- `device editor.cpp` Red Pitaya column layout, `saveVisualizerEditorParameters` / `loadVisualizerEditorParameters`
- `opensim_joint_catalog.py` curated list

### Integration Points
- `AcqBoardRedPitaya` holds selection state for Phase 3 config writes

</code_context>

<specifics>
## Specific Ideas

Locked defaults from `.planning/PROJECT.md` (2026-06-10).

</specifics>

<deferred>
## Deferred Ideas

- Apply Display button wiring (Phase 3 OPS-01)
- Trigger writes (Phase 3)

</deferred>
