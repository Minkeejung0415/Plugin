# Features Research — Joint Angle Display on Trigger

**Project:** Open Ephys Red Pitaya Plugin  
**Researched:** 2026-06-10  
**Revised:** 2026-06-10 (scope correction)  
**Confidence:** HIGH

## Table Stakes (Must Have for This Milestone)

| Feature | Complexity | Notes |
|---------|------------|-------|
| Joint coordinate multi-select in plugin UI | Medium | Checkbox list + XML persistence |
| Apply display filter on trigger | Medium | Hook broadcast/TTL path |
| Show filtered joint angles near sim clock | Medium | Simbody overlay or title fallback |
| No regression to live IK stream | Low | Sidecar must not block UDP thread |

## Differentiators (v1 Included)

| Feature | Complexity | Notes |
|---------|------------|-------|
| Persisted joint selection across sessions | Low | Existing XML save pattern |
| Curated coordinate catalog (instrumented limbs) | Low | Fixed list from Rajagopal model |

## Deferred (v2)

| Feature | Reason |
|---------|--------|
| Per-trigger joint set mapping (different TTL lines → different joint lists) | Adds UI complexity; v1 uses single selected set |
| Auto-select joints from active sensor count | Convenience; manual selection sufficient for v1 |
| Full model coordinate picker | 80+ DOFs overwhelming; curated catalog first |

## Anti-Features

| Anti-Feature | Why Avoid |
|--------------|-----------|
| Camera/view angle presets | User correction — not requested |
| Showing all model coordinates | Explicitly the noise problem being solved |
| Breaking UDP v2 format | Live pipeline depends on stable packet layout |

## Dependencies

```
UI joint selector → settings XML → trigger handler → display config file → Python watcher → filtered HUD beside sim time
```

Selector must be readable before trigger fires; Python watcher must run for full Live session; IK loop already provides coordinate values.
