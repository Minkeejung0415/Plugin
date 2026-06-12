# Phase 6: HUD Live-Update Fix - Research

**Researched:** 2026-06-12
**Domain:** Simbody (SimTK) visualizer decoration API via OpenSim 4.5 Python/SWIG bindings
**Confidence:** HIGH (introspected the actual installed `C:\OpenSim 4.5` Python 3.8 bindings + SimTK `Visualizer.h` header)

## Summary

The locked "Option 1: DecorationGenerator" fix **cannot be implemented as written** on this
build. The `SimTK::DecorationGenerator` C++ class is **not wrapped at all** in the OpenSim 4.5
Python bindings — there is no `opensim.DecorationGenerator`, no `*.simbody.DecorationGenerator`,
and no `*Generator` type anywhere in the 861-symbol module namespace. The visualizer **method**
`addDecorationGenerator(generator)` does exist, but it requires a `DecorationGenerator*` argument
that is impossible to construct or subclass from Python (no class, therefore no SWIG director).
**Q1 = partial / Q2 = NO.** The plan must lead with a fallback, not Option 1.

The good news: the actual root cause is now fully confirmed against the SimTK header, and there
is a **documented, supported live-update path that the current code never tried** —
`viz.updDecoration(i)` returns a *writable reference to the visualizer's internally-stored
decoration*, and the renderer reads that stored array every `report()`. The catch (also
confirmed empirically): `updDecoration` returns the **base** type `DecorativeGeometry`, which
does **not** expose `setText()`, and no `safeDownCast` helper exists. So the live-text path is
viable for geometry that the base class can mutate (color, transform, scale), but **not for
the text string of a `DecorativeText`** through `updDecoration` alone.

**Primary recommendation:** Plan should (a) implement the locked capability probe and let it
prove `DecorationGenerator` is absent at runtime, then (b) fall straight through to the
**port-5001 UDP angle-feedback path (`_send_angle_feedback`)** as the operator-visible live
readout, because both in-viewport text strategies are dead on this exact build and
`setWindowTitle` already failed in production. See Q3/Q4 for the definitive reasoning and the
one remaining (lower-confidence) in-viewport option to evaluate during planning.

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| Live joint-angle string render | Simbody visualizer (in-process C++ via SWIG) | Open Ephys plugin UI (UDP 5001) | Visualizer owns the viewport; UDP is the cross-process readout fallback |
| Per-frame HUD string production | Python IK loop (`_render_joint_display_hud`) | — | Already reads IK state; cheap string build only |
| Capability detection | Python (`_pick_hud_strategy`) | — | Must probe the live SWIG object, not assume |
| Operator-visible readout when viewport text is impossible | Open Ephys plugin (consumes UDP 5001) | — | Already wired (`_send_angle_feedback`) |

## User Constraints (from CONTEXT.md)

### Locked Decisions
- **Root cause (confirmed):** `addDecoration` stores the `DecorativeText` **by value (a copy)**;
  the retained Python handle that `_hud_screen_text.setText()` mutates is no longer the object
  the renderer references. Result: on-screen text is frozen at the constructor string. IK loop
  and `_read_coord_value()` are correct — only the render path is dead.
- **Chosen fix — Option 1: DecorationGenerator** subclass whose `generateDecorations(state, decorations)`
  emits a fresh `DecorativeText(compact)` each frame, registered via `viz.addDecorationGenerator(gen)`.
  → **RESEARCH FINDING: not implementable on this build (see Q2). Plan must treat the locked
  Option 1 as blocked and proceed to the fallback chain.**
- **Capability probe (locked requirement):** the plan MUST include a runtime probe printing whether
  `addDecorationGenerator` exists on the live `viz` and whether `DecorationGenerator` is
  subclassable. Extend `_pick_hud_strategy` (`dir(viz)` near line 709). → Snippet provided in Q5.
- **Fallback chain (locked):** (1) `setWindowTitle` — already logs `[WARN] setWindowTitle
  unavailable`, treat as likely dead. (2) port-5001 UDP angle-feedback (`_send_angle_feedback`,
  already wired) as the operator-visible readout.
- **Apply to BOTH copies** of `opensim_live_realtime.py` so the executed copy and source-of-truth match.

### Claude's Discretion
- Exact module structure of the generator (nested vs module-level), shared-state holder mechanism,
  and how `_pick_hud_strategy` selects between `decoration_generator` / `window_title` / `udp_feedback`.
- Whether to remove the dead `setText` branch or keep it behind the strategy switch.

### Deferred Ideas (OUT OF SCOPE)
- Multi-line HUD layout / per-joint color coding.
- Rebuilding the C++ plugin.

## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| DISP-02 | Filtered joint angle values rendered beside the Simbody simulation time/clock, updating live each frame | Q1–Q5 below. Option 1 blocked; viable live path is UDP-5001 feedback (HIGH) plus one in-viewport candidate to evaluate (`updDecoration` color/transform — text NOT mutable; MEDIUM). Probe recipe in Q5 lets the implementation choose at runtime. |

---

## Primary Research Questions — Answered

### Q1. Does the binding expose a per-frame decoration generator registration on the SimbodyVisualizer?

**Partially. The METHOD exists; the required ARGUMENT TYPE does not.** [VERIFIED: introspection of `C:\OpenSim 4.5\sdk\Python` under Python 3.8]

- The object the code holds is `model.updVisualizer().updSimbodyVisualizer()` → Python type
  **`opensim.simbody.SimTKVisualizer`** (this is SimTK's `Visualizer`, wrapped). Confirmed: `ModelVisualizer.updSimbodyVisualizer` exists and returns it.
- `SimTKVisualizer` exposes **`addDecorationGenerator`** (verified `hasattr == True`). Its SWIG
  docstring matches the C++ contract: *"Add a DecorationGenerator that will be invoked to add
  dynamically generated geometry to each frame... The Visualizer assumes ownership... :rtype: int."*
- Header confirms the C++ signature: `int addDecorationGenerator(DecorationGenerator* generator);`
  (`C:\OpenSim 4.5\sdk\Simbody\include\simbody\internal\Visualizer.h:636`). [CITED: installed Visualizer.h]
- **Blocker:** the parameter type `DecorationGenerator` is **not wrapped in the Python module**
  (see Q2). The method is callable but un-feedable. Confidence: **HIGH**.

### Q2. Can `SimTK::DecorationGenerator` be subclassed in Python (SWIG director)?

**NO.** [VERIFIED: full namespace scan]

- Scanned `opensim`, `opensim.simbody`, `opensim.simulation`, `opensim.common`: **zero**
  `DecorationGenerator` symbols. A scan of all 861 top-level names for anything ending in
  `Generator` returned **NONE**.
- SWIG directors require the C++ class to be wrapped *and* compiled with `%feature("director")`.
  Since the class is not wrapped at all, **there is no Python type to subclass and no director**.
  Overriding the C++ virtual `generateDecorations(const State&, Array_<DecorativeGeometry>&)` from
  Python is impossible on this build.
- **Practical consequence:** Option 1 from CONTEXT.md is unbuildable. The plan must not write a
  `class HudDecorationGenerator(osim.DecorationGenerator)` — it will `AttributeError` at import.
  Confidence: **HIGH**.

### Q3. If directors are unavailable, what is the correct working pattern for live in-viewport text?

Three candidates evaluated. **None gives reliable live in-viewport TEXT on this exact build.**

**(a) Re-add / replace the decoration each frame — DOES NOT WORK for live update.**
- `addDecoration` is documented as *"Add an **always-present**, body-fixed piece of geometry"*
  (Visualizer.h:599). It **accumulates** — there is **no `removeDecoration` and no
  `clearDecorations`** (verified `hasattr == False` for both). Calling `addDecoration` every frame
  would pile up an unbounded number of stacked text decorations (memory + overdraw growth at 20 Hz),
  not replace the old one. Reject.

**(b) `updDecoration(i)` writable-reference path — the documented live path, but blocked for TEXT.**
- `updDecoration(i)` returns *"a writable reference to the i'th decoration... allowed for a const
  Visualizer since it is just a decoration"* (Visualizer.h:610–612). This **is** the
  Simbody-intended way to mutate a stored decoration live; the renderer reads the stored array each
  `report()`. **This is the live path the current code never tried.** [CITED: Visualizer.h]
- **BUT** the C++ return type is the **base** `DecorativeGeometry&`, and SWIG hands back a Python
  object typed `DecorativeGeometry`, which **does not expose `setText()`** (verified:
  `hasattr(osim.DecorativeGeometry, "setText") == False`; `setText` exists only on `DecorativeText`).
  There is **no `safeDownCast` / `DownCast` helper** for decorations (verified: none in namespace).
- So `viz.updDecoration(i).setText(...)` raises `AttributeError`. The base class CAN mutate
  `setColor`, `setTransform`, `setScaleFactors`, `setOpacity` live — but **not the text string**.
- Net: `updDecoration` cannot live-update HUD *text* on this build. (It could drive a non-text
  indicator — e.g. a colored bar via `DecorativeBrick` whose color encodes angle — but that is a
  scope change and not requested.) Confidence: **HIGH** that text is not mutable this way.

**(c) `addEphemeralDecoration` / per-frame decoration API — DOES NOT EXIST.**
- Verified `hasattr(SimTKVisualizer, "addEphemeralDecoration") == False`. No such API on this build.

**Why the current code fails (definitive):** `addDecoration` copies the `DecorativeText` into the
visualizer's internal store (by value, Visualizer.h:599 "always-present... like the one passed in").
The Python `_hud_screen_text` handle still points at the *original* object outside the store, so
`_hud_screen_text.setText(compact)` mutates a copy the renderer never reads. The renderer redraws
the stored copy unchanged every frame. There is no Python-reachable way to reach into the stored
copy's text (base-type `updDecoration` lacks `setText`; no downcast; no generator). Confidence: **HIGH**.

### Q4. Fallback chain viability

**(a) `setWindowTitle` — exists but already failed in production; treat as DEAD.**
- The method exists (`hasattr == True`) and its docstring is normal ("Change the title on the main
  visualizer window"). [VERIFIED]
- However the running build already logs `[WARN] setWindowTitle unavailable (...)` via
  `_try_set_window_title` (line 543). The SWIG typemap for the `String` arg rejects a plain Python
  `str` on this build (documented in the code comment, line 544: *"OpenSim 4.5 SWIG rejects plain
  Python str for setWindowTitle"*). After the first failure the code latches `_window_title_supported
  = False` and never retries. Confidence the title path is dead in the field: **MEDIUM-HIGH**
  (method present, but observed-failing; the plan should let the probe re-confirm rather than assume,
  and may try a `SimTK.String(title)` wrap as a cheap last attempt — see Open Questions).

**(b) Port-5001 UDP angle-feedback (`_send_angle_feedback`) — VIABLE and ALREADY WIRED.** [VERIFIED: code]
- `_send_angle_feedback(t_stream, joint_index, angle_deg)` (line 501) packs 4×float32
  `[t_stream, 3.1 version tag, joint_index, angle_deg]` and sends to `ANGLE_FEEDBACK_IP:5001`. It is
  already called in the live loop (line 1587) every frame for the selected joint, independent of the
  dead viewport text. The Open Ephys plugin "optionally reads this port to display the real-time
  angle in the Open Ephys UI" (docstring). This is the **only** confirmed-working operator-visible
  live readout on this build. Confidence: **HIGH** it transmits; **MEDIUM** that the plugin UI
  currently renders it (plugin-side consumption is outside this Python file — see Open Questions).

### Q5. Capability-probe recipe (runtime strategy selection)

Drop-in extension for `_pick_hud_strategy(viz)` (replaces the current hard-coded
`return "window_title"` at line 712). Verified against the installed bindings — these checks return
the values shown on `C:\OpenSim 4.5`:

```python
def _probe_hud_capabilities(viz):
    """Return a dict of what the live Simbody visualizer can actually do on this build.

    Verified values on C:\\OpenSim 4.5 (OpenSim 4.5, Python 3.8):
      has_add_decoration_generator = True   (method present...)
      decoration_generator_subclassable = False  (...but the class is unwrapped -> unusable)
      upd_decoration_text = False           (updDecoration returns base DecorativeGeometry; no setText)
      has_set_window_title = True           (present, but rejects str at runtime on this build)
    """
    import opensim as osim
    caps = {}
    caps["has_add_decoration_generator"] = hasattr(viz, "addDecorationGenerator")
    # The method is useless without a constructible/subclassable generator class:
    caps["decoration_generator_subclassable"] = hasattr(osim, "DecorationGenerator")
    # updDecoration returns base DecorativeGeometry, which lacks setText:
    caps["upd_decoration_text"] = hasattr(osim.DecorativeGeometry, "setText")
    caps["has_set_window_title"] = hasattr(viz, "setWindowTitle")
    caps["has_remove_decoration"] = hasattr(viz, "removeDecoration") or hasattr(viz, "clearDecorations")
    return caps


def _pick_hud_strategy(viz):
    caps = _probe_hud_capabilities(viz)
    print(f"[JOINT-DISPLAY-PROBE] viz caps: {caps}")
    # Decoration-generator path requires BOTH the method AND a subclassable class:
    if caps["has_add_decoration_generator"] and caps["decoration_generator_subclassable"]:
        print("[JOINT-DISPLAY] strategy=decoration_generator")
        return "decoration_generator"
    # In-viewport live text via updDecoration only works if the base type exposes setText:
    if caps["upd_decoration_text"]:
        print("[JOINT-DISPLAY] strategy=screen_text (updDecoration)")
        return "screen_text"
    # Window title is present on 4.5 but rejects str at runtime; _try_set_window_title latches off.
    # UDP feedback is the only confirmed live operator-visible readout on this build:
    print("[JOINT-DISPLAY] strategy=udp_feedback (in-viewport text unavailable on this build)")
    return "udp_feedback"
```

On `C:\OpenSim 4.5` this returns **`udp_feedback`** (generator class absent → text not
mutable via updDecoration → title unreliable). Confidence: **HIGH**.

---

## Standard Stack

No new packages. The phase is a render-path fix inside an existing Python 3.8 script using the
already-installed `opensim` 4.5 bindings. [VERIFIED: `osim.GetVersion()` == "4.5"]

| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| `opensim` | 4.5 (`C:\OpenSim 4.5\sdk\Python`) | IK + Simbody visualizer + decorations | Already the proven bridge; frozen per CLAUDE.md |
| `socket` (stdlib) | 3.8 | UDP angle-feedback fallback (port 5001) | Already wired in `_send_angle_feedback` |

**Installation:** none. Do not `pip install` anything (no-new-runtime-dependencies constraint).

## Package Legitimacy Audit

Not applicable — this phase installs **no external packages**. All APIs come from the
pre-installed `C:\OpenSim 4.5` SDK and the Python standard library.

## Architecture Patterns

### Data flow (live HUD readout)

```
IMU UDP :5000 ──> _udp_ahrs_thread ──> _frame_queue
                                          │
                              IK loop (~20 Hz) ──> ikSolver.assemble(state)
                                          │
                       _build_joint_hud_lines(model, state)   (reads IK angles, cheap)
                                          │
                    ┌─────────────────────┴───────────────────────┐
                    │  strategy = _pick_hud_strategy(viz)          │
                    ▼                                              ▼
   strategy == "udp_feedback" (THIS BUILD)        strategy == decoration_generator / screen_text
   _send_angle_feedback(t, joint_idx, deg) ─UDP:5001─>            (BLOCKED on 4.5 — see Q2/Q3)
        Open Ephys plugin UI / log
                                          │
                            model.getVisualizer().show(state)  (renders model + clock)
```

### Pattern: probe-then-select, degrade gracefully
**What:** Never assume an API works — call the runtime probe (Q5), print the caps dict, pick the
highest-fidelity strategy the live object actually supports.
**When to use:** Every frame strategy is fixed once at startup (probe is cheap, runs once in
`_pick_hud_strategy`); the per-frame `_render_joint_display_hud` just dispatches on the chosen string.

### Pattern: cheap per-frame generator-shaped holder (even without a real generator)
**What:** Keep a module-level holder updated by the IK loop (the CONTEXT.md "shared-state holder"
idea is still correct) so whichever strategy renders reads a pre-formatted string and never calls IK.
**When to use:** Keeps the 20 Hz loop unstalled regardless of strategy. The string build
(`_build_joint_hud_lines`) already does only `coord.getValue` reads + formatting — no allocation
storms.

### Anti-Patterns to Avoid
- **Subclassing `osim.DecorationGenerator`:** `AttributeError` at import — the class does not exist.
- **Calling `addDecoration` every frame:** unbounded accumulation; no remove/clear API exists.
- **`updDecoration(i).setText(...)`:** `AttributeError` — base `DecorativeGeometry` has no `setText`.
- **Assuming `setWindowTitle` works because the method exists:** it rejects `str` at runtime on this
  build and `_try_set_window_title` already latched it off.

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Live operator readout | Custom GL text overlay / new window | Existing `_send_angle_feedback` UDP:5001 | Already wired and frame-driven; viewport text is genuinely unavailable on this build |
| Strategy detection | Hard-coded `return "window_title"` | Runtime caps probe (Q5) | The current hard-code is exactly what hid the dead path through Phase 4 UAT |
| Decoration removal | Manual index bookkeeping to "replace" text | (nothing — there is no remove API) | `addDecoration` is always-present/accumulating by design |

**Key insight:** The reason DISP-02 silently "passed" inspection in Phase 4 but failed hardware UAT
is that the code hard-coded `window_title` and never observed it failing live. The fix is as much
about *honest runtime capability reporting* as about the render call.

## Runtime State Inventory

> This is a rename/sync-sensitive phase (two file copies must end identical). Inventory below.

| Category | Items Found | Action Required |
|----------|-------------|------------------|
| Stored data | None — no datastore keys involved. Verified: phase touches only in-memory HUD render. | none |
| Live service config | `opensim_joint_display_config.json` in `WORK_DIR` drives the filter; **unchanged** by this phase. | none |
| OS-registered state | `start_opensim_live.bat` launches the **executed copy** at `C:\Users\justi\Open-Sim--Bio-Mech\` via `py -3.8 -u opensim_live_realtime.py`. The fix MUST land in that copy or it won't run. | edit executed copy |
| Secrets/env vars | `OPENSIM_LIVE_SOURCE=real_redpitaya`, `OPENSIM_LIVE_VISUALIZER_RATE` — name-only, unchanged. | none |
| Build artifacts | None — pure `.py` edit, no compiled package. | none |

**Two-copy divergence (plan-critical):** The source-of-truth (`C:\Users\justi\Documents\Plugin\opensim_live_realtime.py`,
66896 bytes) and the executed copy (`C:\Users\justi\Open-Sim--Bio-Mech\opensim_live_realtime.py`,
57019 bytes) currently **differ by ~348 changed lines** — they are NOT identical today. Both share
the same `WORK_DIR`. The executed copy already contains the HUD functions (11 references) but at an
older revision. The plan must decide whether to (a) port only the HUD fix into both, or (b) copy the
newer source-of-truth wholesale to the executed location. **Recommend option (b) — overwrite the
executed copy from the source-of-truth — then apply the fix once, to guarantee they end identical**,
unless the executed copy has machine-specific edits (none found; both use the same paths/WORK_DIR).
Confidence: **MEDIUM** — verify with a diff during planning before choosing copy-wholesale.

## Common Pitfalls

### Pitfall 1: Trusting `hasattr(viz, "addDecorationGenerator")` as "generator works"
**What goes wrong:** The method exists, so a naive probe says "yes," but the call is unfeedable.
**Why:** SWIG wraps the method but not the parameter class.
**How to avoid:** Probe `hasattr(osim, "DecorationGenerator")` (the class), not just the method (Q5).
**Warning signs:** `AttributeError: module 'opensim' has no attribute 'DecorationGenerator'`.

### Pitfall 2: `updDecoration` looks like the answer but returns the base type
**What goes wrong:** `viz.updDecoration(0).setText(...)` → `AttributeError`.
**Why:** C++ return is `DecorativeGeometry&`; SWIG gives a base-typed Python object with no `setText`,
and there is no downcast helper.
**How to avoid:** Probe `hasattr(osim.DecorativeGeometry, "setText")` (it's `False`) before choosing.

### Pitfall 3: Forgetting the second file copy
**What goes wrong:** Fix lands in the repo copy; `start_opensim_live.bat` runs the other one; UAT
fails again exactly as it did for Phase 4.
**How to avoid:** Make "both copies byte-identical" an explicit verification step (diff returns empty).

### Pitfall 4: Per-frame allocation / IK in the render path stalling 20 Hz
**What goes wrong:** Visualizer loop drops below `OPENSIM_LIVE_VISUALIZER_RATE`.
**How to avoid:** Render path reads a pre-built string from the shared holder; no `ikSolver` calls,
no new socket per frame (`_angle_feedback_sock` is already a cached module global).

## Code Examples

### Confirmed-absent generator (do NOT write this)
```python
# WILL RAISE AttributeError AT IMPORT on OpenSim 4.5 — DecorationGenerator is unwrapped.
class HudGen(osim.DecorationGenerator):   # <-- osim has no DecorationGenerator
    def generateDecorations(self, state, decorations): ...
```

### Confirmed-working live readout (use this)
```python
# Already present at line 501/1587. Frame-driven, cached socket, no allocation storm.
_send_angle_feedback(t_imu, joint_index, angle_deg)   # -> UDP 5001 -> Open Ephys UI
```

### Strategy probe (Q5) — drop-in for line 709
See Q5 block above. Returns `"udp_feedback"` on this build.

## State of the Art

| Old Approach (in code) | Current Reality on 4.5 | Impact |
|------------------------|------------------------|--------|
| `addDecoration` + retained handle `.setText` | Stored by value; handle is a stale copy | Frozen text (the bug) |
| Planned: `DecorationGenerator` subclass | Class unwrapped in Python bindings | Option 1 unbuildable |
| `setWindowTitle` fallback | Method present but rejects `str` on this build | Already latched off in field |
| `updDecoration().setText` | Base type lacks `setText`; no downcast | In-viewport text not mutable |
| **UDP :5001 angle feedback** | **Wired, frame-driven, working** | **The viable live readout** |

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | Open Ephys plugin actually renders the UDP :5001 packet in its UI (Python side only transmits) | Q4(b) | If plugin ignores it, operator still has no live readout — would need a tiny C++ plugin display change (out of this phase's scope) |
| A2 | Copying the source-of-truth wholesale over the executed copy introduces no machine-specific regressions | Runtime State Inventory | Could overwrite a local-only tweak; mitigated by diffing first |
| A3 | `setWindowTitle` cannot be salvaged via `SimTK.String(title)` wrapping | Q4(a) / Open Questions | If it CAN, a free in-viewport-adjacent readout returns; low effort to test |

## Open Questions (RESOLVED in 06-01-PLAN.md)

1. **Can `setWindowTitle` be revived by wrapping the arg?** — RESOLVED (deferred to runtime test).
   - Known: method exists; plain `str` rejected; `_try_set_window_title` latches off.
   - Unclear: whether `viz.setWindowTitle(osim.String(title))` (if `osim.String`/`SimTK::String`
     is wrapped) passes the typemap.
   - **Resolution:** Task 2 attempts `osim.String(title)` at runtime with clean fall-through to
     `udp_feedback` if it is also rejected. Residual uncertainty is intentional and fully mitigated
     by the fall-through (plain-str dead: HIGH; wrapped-str works: LOW — proven at execution).

2. **Does the Open Ephys plugin consume UDP :5001 today?** — RESOLVED (scoped out + UAT-covered).
   - Known: Python sends it every frame; docstring says plugin "optionally reads" it.
   - **Resolution:** plugin C++ is out of scope this phase (no rebuild). Task 4 UAT confirms the
     operator can SEE the number change on the active readout surface (plugin UI or logged readout);
     plugin-side rendering is flagged as a separate follow-up only if the UAT shows it is absent.

3. **Copy-wholesale vs. surgical port for the two files?** — RESOLVED.
   - **Resolution:** Task 3 overwrites the executed copy from source-of-truth, then asserts `fc /b`
     empty — the copy-wholesale default recommended here.

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| OpenSim Python bindings | All rendering | ✓ | 4.5 (verified `GetVersion`) | — |
| Python 3.8 | Executed script | ✓ | `C:\...\Python38\python.exe` present; `.bat` uses `py -3.8` | — |
| `SimTK::DecorationGenerator` (Python) | Locked Option 1 | ✗ | unwrapped | UDP :5001 feedback |
| `setWindowTitle(str)` | Title fallback | ✗ (runtime str rejection) | method present | UDP :5001 feedback |
| UDP :5001 path (`_send_angle_feedback`) | Operator readout | ✓ | wired | — |

**Missing dependencies with no fallback:** none — UDP :5001 covers the gap.
**Missing dependencies with fallback:** DecorationGenerator and window-title both fall back to UDP :5001.

## Validation Architecture

> `nyquist_validation: true` in config → section required.

### Test Framework
| Property | Value |
|----------|-------|
| Framework | None for Python (only standalone scripts: `test_udp_sender.py`, `udp_receive_test.py`; C tests under `tests/`). No pytest/conftest. |
| Config file | none — see Wave 0 |
| Quick run command | `py -3.8 -c "import opensim_live_realtime"` (import-smoke: proves no `DecorationGenerator` AttributeError) |
| Full suite command | Manual hardware UAT (operator moves knee, observes live number) — DISP-02 is observation-based by CONTEXT.md |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| DISP-02 | Probe correctly reports caps and selects `udp_feedback` on this build | unit | `py -3.8 -c "import opensim_live_realtime as m, opensim as o; print(m._probe_hud_capabilities.__name__)"` after adding probe | ❌ Wave 0 (probe fn new) |
| DISP-02 | Module imports with no `osim.DecorationGenerator` reference | smoke | `py -3.8 -c "import opensim_live_realtime"` | ✅ |
| DISP-02 | Both file copies byte-identical post-fix | integration | `fc /b "<src>" "<executed>"` (or `diff`) returns no differences | ❌ Wave 0 |
| DISP-02 | Live readout changes when operator moves the joint | manual-only | hardware UAT — observable per CONTEXT.md "observable, not inspection-only" | n/a |

### Sampling Rate
- **Per task commit:** `py -3.8 -c "import opensim_live_realtime"` (import smoke, <5 s)
- **Per wave merge:** caps-probe unit check + two-copy `fc /b` diff
- **Phase gate:** hardware UAT — operator moves knee, the chosen readout (UDP :5001 in Open Ephys UI
  or logged `[HUD-UPDATE]`) tracks the console `[HUD-DIAG] knee_angle_r=` value; switching selected
  joint changes the label.

### Wave 0 Gaps
- [ ] `_probe_hud_capabilities` function (new; basis for the unit smoke).
- [ ] Two-copy diff check baked into verification (`fc /b` on Windows).
- [ ] No pytest install needed — import-smoke + `fc` suffice; do not add a test framework (no-new-deps).

## Security Domain

Not applicable in the conventional sense — no auth, network input parsing change, or crypto. The
only network surface is the existing outbound UDP :5001 send (no new listener, no new parsing).
No ASVS category is engaged by this render-path fix. (security_enforcement: this phase introduces
no new input-validation or trust-boundary surface.)

## Sources

### Primary (HIGH confidence)
- Live introspection of `C:\OpenSim 4.5\sdk\Python` opensim module under `Python38\python.exe`:
  `GetVersion()=="4.5"`; full `dir()` of `SimTKVisualizer`, `DecorativeGeometry`, `DecorativeText`,
  `ModelVisualizer`; namespace scans proving `DecorationGenerator` and all `*Generator` types absent;
  `hasattr` matrix for `addDecorationGenerator`/`updDecoration`/`setWindowTitle`/`removeDecoration`/`clearDecorations`.
- `C:\OpenSim 4.5\sdk\Simbody\include\simbody\internal\Visualizer.h` — lines 39, 599–612, 636–643
  (decoration storage semantics, `updDecoration` writable-ref contract, `addDecorationGenerator` signature).
- `opensim_live_realtime.py` (source-of-truth) — defect functions at lines 452, 501, 543, 695, 709, 716, 1403.
- `start_opensim_live.bat` — confirms the executed copy path and `py -3.8` launcher.

### Secondary (MEDIUM confidence)
- Two-copy byte/line divergence measured via `diff` (~348 changed lines; 66896 vs 57019 bytes).

### Tertiary (LOW confidence)
- Whether a `SimTK::String`-wrapped `setWindowTitle` call succeeds (untested; Open Question 1).
- Whether the Open Ephys plugin UI currently renders UDP :5001 (plugin C++ not inspected here).

## Metadata

**Confidence breakdown:**
- DecorationGenerator unavailability (Q1/Q2): HIGH — direct namespace + header evidence.
- In-viewport text impossibility (Q3): HIGH — base-type `updDecoration`, no remove API, no downcast.
- UDP fallback viability (Q4b): HIGH transmit / MEDIUM plugin-render.
- Window-title death (Q4a): MEDIUM-HIGH — method present but observed-failing.
- Two-copy reconciliation: MEDIUM — recommend diff-first.

**Research date:** 2026-06-12
**Valid until:** stable until the OpenSim install changes (no version churn expected) — ~90 days.
