# Features Research — OpenSim View Angle on Trigger

**Project:** Open Ephys Red Pitaya Plugin  
**Researched:** 2026-06-10  
**Confidence:** HIGH

## Table Stakes (Must Have for This Milestone)

| Feature | Complexity | Notes |
|---------|------------|-------|
| View preset selector in plugin UI | Low | ComboBox + XML persistence |
| Apply view on trigger | Medium | Hook broadcast/TTL path |
| Show active view label near sim clock | Medium | Simbody overlay or title hack |
| No regression to live IK stream | Low | Sidecar must not block UDP thread |

## Differentiators (v1 Included)

| Feature | Complexity | Notes |
|---------|------------|-------|
| Persisted preset across sessions | Low | Existing XML save pattern |
| Standard anatomical preset library | Low | Fixed enum, not user-defined |

## Deferred (v2)

| Feature | Reason |
|---------|--------|
| Per-trigger preset mapping (different TTL lines → different views) | Adds UI complexity; v1 uses single selected preset |
| Animated camera transitions | Not requested; instant switch sufficient |
| View sync from OpenSim back to plugin | One-way control enough for operator workflow |

## Anti-Features

| Anti-Feature | Why Avoid |
|--------------|-----------|
| Joint angle HUD replacing camera label | User asked for view angle beside clock, not coordinate dump |
| Breaking UDP v2 format | Live pipeline depends on stable packet layout |

## Dependencies

```
UI selector → settings XML → trigger handler → view config file → Python watcher → Simbody camera + label
```

Selector must be readable before trigger fires; Python watcher must run for full Live session.
