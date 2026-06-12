---
phase: 6
slug: hud-live-update-fix
status: draft
nyquist_compliant: false
wave_0_complete: false
created: 2026-06-12
---

# Phase 6 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | None for Python (only standalone scripts: `test_udp_sender.py`, `udp_receive_test.py`; C tests under `tests/`). No pytest/conftest — do NOT add one (no-new-deps constraint). |
| **Config file** | none — see Wave 0 |
| **Quick run command** | `py -3.8 -c "import opensim_live_realtime"` (import-smoke: proves no `osim.DecorationGenerator` AttributeError reintroduced) |
| **Full suite command** | Manual hardware UAT (operator moves knee, observes the live number on the active readout surface) — DISP-02 is observation-based per CONTEXT.md |
| **Estimated runtime** | ~5 seconds (import smoke); UAT is manual |

---

## Sampling Rate

- **After every task commit:** Run `py -3.8 -c "import opensim_live_realtime"` (import smoke, <5 s)
- **After every plan wave:** caps-probe unit check + two-copy `fc /b` diff
- **Before `/gsd:verify-work`:** import smoke green AND two copies byte-identical
- **Max feedback latency:** 5 seconds (automated portion)

---

## Per-Task Verification Map

| Task ID | Plan | Wave | Requirement | Threat Ref | Secure Behavior | Test Type | Automated Command | File Exists | Status |
|---------|------|------|-------------|------------|-----------------|-----------|-------------------|-------------|--------|
| 06-01-xx | 01 | 1 | DISP-02 | — | N/A (no new trust boundary) | smoke | `py -3.8 -c "import opensim_live_realtime"` | ✅ | ⬜ pending |
| 06-01-xx | 01 | 1 | DISP-02 | — | N/A | unit | `py -3.8 -c "import opensim_live_realtime as m; assert hasattr(m,'_probe_hud_capabilities')"` | ❌ W0 (probe fn new) | ⬜ pending |
| 06-01-xx | 01 | 2 | DISP-02 | — | N/A | integration | `fc /b "C:\Users\justi\Documents\Plugin\opensim_live_realtime.py" "C:\Users\justi\Open-Sim--Bio-Mech\opensim_live_realtime.py"` returns no differences | ❌ W0 | ⬜ pending |
| 06-01-xx | 01 | 2 | DISP-02 | — | N/A | manual | hardware UAT — operator moves knee, active readout tracks `[HUD-DIAG] knee_angle_r=` | n/a | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

- [ ] `_probe_hud_capabilities` function (new) — runtime `hasattr`/`dir(viz)` probe; basis for the unit smoke (recipe in 06-RESEARCH.md Q5).
- [ ] Two-copy diff check baked into verification (`fc /b` on Windows, or `diff`).
- [ ] No pytest install — import-smoke + `fc` suffice (no-new-deps).

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| Live readout changes when operator moves the joint | DISP-02 | Requires live Red Pitaya IMU stream + OpenSim 4.5 viewport; observation-based per CONTEXT.md | Start `start_opensim_live.bat`, stream IMU, move right knee; confirm the active readout (OpenSim window title bar under `window_title`, else Open Ephys plugin UI / logged `[HUD-UPDATE]` under `udp_feedback`) tracks console `[HUD-DIAG] knee_angle_r=`; then change selected joint and confirm the label changes. |
| Chosen strategy logged at startup | DISP-02 | Depends on live `viz` object capabilities | Confirm one `[JOINT-DISPLAY] strategy=...` line prints at startup naming the resolved strategy. |

---

## Security Domain

Not applicable in the conventional sense — no auth, no new network-input parsing, no crypto. The
only network surface is the existing outbound UDP :5001 send (no new listener, no new parsing). No
ASVS category is engaged by this render-path fix.

---

## Validation Sign-Off

- [ ] All tasks have `<automated>` verify or Wave 0 dependencies
- [ ] Sampling continuity: no 3 consecutive tasks without automated verify
- [ ] Wave 0 covers all MISSING references (`_probe_hud_capabilities`, two-copy diff)
- [ ] No watch-mode flags
- [ ] Feedback latency < 5s
- [ ] `nyquist_compliant: true` set in frontmatter

**Approval:** pending
