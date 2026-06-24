---
phase: 08-slave-monitor-sync-csv
created: 2026-06-24
status: complete
---

# Phase 08 Research: Slave Monitor, Sync Evidence, and CSV Export

## Findings

### Existing Firmware Capability

- Master and slave Arduino sketches already share 14-channel sample layout: accel, gyro, mag, quaternion, DIO.
- ESP-NOW command relay exists for start/stop/record start/record stop.
- Slave SD binary layout is compact and safe for acquisition: `STP1` header plus fixed records.
- `dio` is already recorded in every row as level plus edge count, but current sampling is loop/debounce based.
- Slave status can be piggybacked over ESP-NOW as a compact packet because the monitor rate is low.

### Existing Plugin Capability

- Acquisition Board plugin already polls/prints status text and has a constrained UI.
- The UI is height-limited, so a fixed-height dropdown/details pattern is safer than per-slave cards.
- Existing `device editor.cpp` already owns status presentation and control layout.

### CSV Tooling

- `esp32/host/analyze_sample_rate.py` validates SD binaries for sequence/timestamp behavior.
- `esp32/host/rec_download.py` contains SD binary parsing and CSV writing logic.
- `esp32/host/sd_bin_to_csv.py` now converts a copied SD binary directly to CSV.

## Recommended Architecture

1. Slaves send a compact status/preview packet to the master about 5 Hz.
2. Master stores recent slave packets in a fixed small table keyed by MAC or explicit `slave_id`.
3. Master exposes status in a machine-readable line protocol over TCP, e.g. `SLAVE_STATUS ...`.
4. Plugin parses these lines, maintains a map of detected slaves, and renders one selected slave in a fixed-height UI panel.
5. Post-acquisition tools batch-convert copied SD `.bin` files and produce summary evidence.

## Risks

- ESP-NOW packets can collide or drop; monitor data must be treated as best-effort and stale after a short age threshold.
- Multiple slaves currently cannot all use static `192.168.4.2`; switch to DHCP or unique slave IDs/IPs.
- UI height is constrained; avoid verbose tables.
- DIO precision is currently limited by loop sampling/debounce; interrupt timestamps may be needed if sync tolerance is tighter than ~10-20 ms.
