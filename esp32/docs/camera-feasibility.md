# Camera Feasibility — TEVM-AR0234 & Technexion S32 Lens

## Lab Hardware

| Item | Interface | Notes |
|------|-----------|-------|
| **TEVM-AR0234** | MIPI CSI-2 (up to 4 lanes) | onsemi AR0234 2.3 MP global shutter, onboard ISP + IMU |
| **TEVM-AR0234-C-S32-IR** | Same MIPI module + **S32** M12 lens (32° D-FOV) + IR cut | Confirmed Technexion SKU family |
| **Seeed XIAO ESP32-S3 Sense** | **DVP** (OV3660/OV5640) | No native MIPI CSI on ESP32-S3 |

## ESP32-S3 (Seeed) — Verdict

**TEVM-AR0234 cannot attach directly to XIAO ESP32-S3 Sense.**

- ESP32-S3 provides **LCD_CAM DVP** only (8-bit parallel sensor bus).
- TEVM-AR0234 requires **MIPI CSI-2** and 70-pin / MINI-SAS style camera connectors (TechNexion ecosystem).
- Espressif documents: MIPI CSI on **ESP32-P4** via `esp_driver_cam` / `esp-video-components`, not ESP32-S3.

### Recommended paths

1. **Near-term (v1 firmware in this repo):** XIAO Sense **OV3660 DVP** for motion-score action verification; keep channel map compatible with STEP host.
2. **Target global shutter (TEVM):** **ESP32-P4** dev kit (e.g. ESP32-P4-EYE) + MIPI CSI + port `esp_cam_sensor` driver — evaluate VizionSDK on Linux host for AR0234 tuning, then port critical capture path to ESP-IDF.
3. **Lab gateway option:** TEVM on **NXP i.MX8 EVK** (TechNexion `TEVM-AR0234-*-NXP` kits) → stream JPEG or motion features to ESP32-S3 or PC over Ethernet/USB.

## S32 Lens Variant

The **S32** lens (32° field of view) is a mechanical/optical option on the same AR0234 MIPI module. It does not change the MIPI electrical requirement. Any ESP32-P4 MIPI bring-up applies equally; framing/ROI constants differ for verification thresholds.

## Action Verification Impact

| Path | Global shutter | Motion blur | Fit for gait cross-check |
|------|----------------|-------------|---------------------------|
| OV3660 DVP @ 10–30 Hz | Rolling shutter | Higher on fast ankle motion | Acceptable for phase-level check |
| TEVM-AR0234 MIPI | Global shutter | Low | Preferred for high-speed limbs |

## Decision (Phase 0)

| Option | Proceed v1 | Owner action |
|--------|------------|--------------|
| DVP on XIAO Sense | **Yes** — implement in Phase 5 | Flash `firmware/` with Sense board |
| TEVM on ESP32-S3 | **No** — hardware mismatch | — |
| TEVM on ESP32-P4 | **Spike in v2** | Procure P4 + MIPI carrier |
| TEVM via NXP EVK | **Optional lab path** | Use existing TEVM cable kit |

---
*Phase 0 deliverable — 2026-06-01*
