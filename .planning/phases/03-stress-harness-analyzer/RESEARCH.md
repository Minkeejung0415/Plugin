# Phase 3: Stress Harness & Analyzer — Implementation State Research

**Researched:** 2026-06-15
**Domain:** ESP32 acquisition reliability — sweep harness, artifact analyzer, summary reporting
**Confidence:** HIGH (all claims derived from direct file reads of the actual codebase and artifact files)

---

## Summary

Phase 3 is substantially complete. The two primary scripts (`stress_test_serial.py` and
`analyze_sample_rate.py`) are fully implemented and have been exercised against real hardware.
59 per-rate CSV artifacts exist, covering 18 distinct frequencies and all four mode combinations
(filter on/off × SD on/off) for most rates. A SUMMARY.md and SUMMARY.json are auto-generated
by the harness, and the documentation file (`esp32/docs/stress-test-sample-rate.md`) exists and
is current.

The main gap is that the **SUMMARY.md/JSON only reflects the most recent partial run** (4 rows
at 1100–1250 Hz, filter on, SD on), not a consolidated view across all 59 CSV artifacts. The
first-failure reason is captured in the `note` field of each SUMMARY.json record but is **not
surfaced as a dedicated human-readable sentence** in SUMMARY.md. Success criterion 4 ("clear
reason for first failure") is therefore partially met in machine-readable form only.

**Primary recommendation:** The phase is ~85% done. Three targeted additions complete it:
(1) a consolidated re-analysis pass that rebuilds SUMMARY from all CSV artifacts, (2) a
dedicated "first failure reason" line in SUMMARY.md, and (3) a final full sweep run so the
summary reflects all mode combinations.

---

## Success Criteria Assessment

### SC1 — `stress_test_serial.py` runs sweeps over frequency and mode combinations

**Status: DONE**

Evidence:
- File `esp32/host/stress_test_serial.py` (677 lines) implements a full sweep harness. [VERIFIED: direct file read]
- `--hz` accepts an arbitrary list (default 12 rates: 50–2000 Hz). [VERIFIED: line 31, 512]
- `--filter both` and `--sd both` enumerate all 4 mode combos via `itertools.product`. [VERIFIED: lines 594–598]
- 59 per-rate CSV artifacts exist in `stress_results/`, covering 18 distinct frequencies. [VERIFIED: directory listing]
- Rates present: 50, 100, 150, 200, 250, 300, 400, 500, 750, 900, 910, 1000, 1100, 1150, 1200, 1250, 1300, 1500, 2000. [VERIFIED: directory listing]
- Mode coverage per frequency:
  - Full 4-combo coverage (filter_off/on × sd_off/on): 50, 100, 150, 200, 250, 300, 400, 500, 750, 1000, 1500, 2000
  - Partial 2-combo (filter_on/sd_on + filter_off/sd_off only): 1100, 1150, 1200, 1250
  - Single-combo (filter_on/sd_on only): 900, 910, 1300
- Binary frame decoding and CSV text mode are both supported; auto-detection is implemented. [VERIFIED: lines 492–504, 415–423]

**Gap:** The 1100–1250 Hz range is missing `filter_on/sd_off` and `filter_off/sd_on` combos.
This means filter cost and SD cost cannot be isolated at the upper boundary. This is a data
gap (re-run needed), not a code gap.

---

### SC2 — Test artifacts include per-rate CSV/JSON/Markdown summaries

**Status: PARTIAL**

Evidence:
- Per-rate CSV files exist for every tested (hz, mode) combination. [VERIFIED: 59 files in stress_results/]
- CSV format: two columns, `seq,time_s`, one row per sample. [VERIFIED: rate_1100__filter_on__sd_on.csv first 5 rows]
- SUMMARY.md is written after every run (single-session consolidated view). [VERIFIED: lines 634–664]
- SUMMARY.json is written after every run containing full `RateResult.__dict__` records. [VERIFIED: lines 665–669]

**Gap 1 — Per-rate JSON is not written.** `analyze_sample_rate.py` can emit JSON via
`--json` flag but only to stdout; no per-rate `.json` sidecar file is written alongside each
CSV. If the success criterion means one JSON file per rate/mode combination, this is missing.

**Gap 2 — SUMMARY reflects only the last run, not all artifacts.** The current SUMMARY.md has
4 rows (1100–1250 Hz, filter on, SD on — the most recent sweep). The 59 CSV artifacts from
all prior runs are not represented. There is no mechanism to rebuild SUMMARY from the full
artifact directory. Running the script again on all combinations would overwrite and regenerate
a complete summary only for that run.

**Gap 3 — No per-run Markdown summary.** Each invocation overwrites `SUMMARY.md` rather than
appending or dating a run-specific file. Historical sweep results are preserved only as CSVs.

---

### SC3 — Analyzer can validate SD files and host-visible streams using same sequence/timestamp rules

**Status: DONE**

Evidence:
- `analyze_sample_rate.py` has two paths: `analyze_csv()` for host-side streams and
  `analyze_sd_bin()` for SD binary files. [VERIFIED: lines 113–166]
- Both paths call the same `_summarize()` function which applies identical sequence-gap,
  duplicate, and timestamp-gap logic. [VERIFIED: lines 51–110]
- SD binary parser reads the STEP STP1 magic header, decodes `seq` (uint32) and `timestamp_us`
  (int64) from each record, converts to seconds, and feeds the same `_summarize()` path.
  [VERIFIED: lines 130–166]
- SD format constants: magic `0x31505453`, header struct `<IHHHHq` (20 bytes), record struct
  `<Iq` (12 bytes). [VERIFIED: lines 13–15, 154]
- Auto-detection by 4-byte magic: if file starts with `STP1` (LE), format is `sd-bin`.
  [VERIFIED: lines 218–224]
- `stress_test_serial.py` delegates to the analyzer via subprocess for host CSV validation
  and exposes `--sd-file` shortcut for SD binary validation. [VERIFIED: lines 360–388, 541–543]
- Metrics applied identically in both modes: consecutive duplicate sequences, sequence jumps
  > 1, timestamp gaps > N × median dt. [VERIFIED: lines 65–93]

**No gaps for SC3.**

---

### SC4 — Summary reports highest passing rate and recommended cap with clear reason for first failure

**Status: PARTIAL**

Evidence — What IS present:

- "Highest passing Hz: **N**" and "Recommended cap (80% of highest pass): **N Hz**" are written
  to SUMMARY.md after every run. [VERIFIED: lines 660–661]
- The `note` field in each SUMMARY.json record captures machine-readable failure reasons:
  - `"loop_overruns=2"` for 1100 Hz failure [VERIFIED: SUMMARY.json line 17]
  - `"no SD counters"` for 1250 Hz failure [VERIFIED: SUMMARY.json line 67]
- The `RateResult.note` string is built from failure signals: `insufficient rows`, `no SD
  counters`, `sd_errors=N`, `loop_overruns=N`. [VERIFIED: lines 456–463]

Evidence — What is MISSING:

- SUMMARY.md does **not** include a dedicated "First failure reason:" sentence or paragraph.
  The `note` field from the JSON is not rendered into the Markdown summary. [VERIFIED: lines
  634–664 — no note field rendered]
- The current SUMMARY.md (4 rows) shows the 1100 Hz row as FAIL but its reason (loop_overruns)
  is only visible by reading SUMMARY.json. A reader of SUMMARY.md alone cannot determine why
  1100 Hz failed — only that it did. [VERIFIED: SUMMARY.md full content]
- The "first failure" concept applies to the first rate that fails in ascending order. The
  harness does not compute or label this explicitly; it only marks each row PASS/FAIL. The
  `note` column is absent from the Markdown table entirely. [VERIFIED: SUMMARY.md table columns]
- The SUMMARY.md currently covers only 4 rates out of ~59 CSV artifacts, which means the
  recommended cap (960 Hz) is based on an incomplete sweep of just the upper boundary. There
  is no documented cap for `filter_off/sd_off` or `filter_on/sd_off` mode separately.

**Gap:** Add `note` column to SUMMARY.md table and add a "First failure at N Hz: [reason]"
sentence in the footer section. Optionally, add per-mode cap lines (worst-case and per-mode).

---

## Docs Assessment

### `esp32/docs/stress-test-sample-rate.md`

**Status: EXISTS AND CURRENT**

File exists at `C:\Users\justi\Documents\Plugin\esp32\docs\stress-test-sample-rate.md`.
[VERIFIED: direct file read]

Content covers:
- Ground-truth definition (SD continuity is authoritative, host stream is transport check)
- Recommended sweep procedure (4 steps, filter/SD combinations)
- Automated sweep CLI examples with real flags
- Pass criteria (all 4 conditions enumerated)
- Artifact outputs (CSV, SUMMARY.md, SUMMARY.json paths)
- SD ground-truth verification workflow
- Firmware command table
- Manual serial capture instructions
- Failure interpretation guide (duplicates/gaps/SD errors/loop overruns diagnostic tree)

**No doc gaps.** The documentation accurately describes the current tooling.

---

## Artifact Directory State

| Artifact | Present | Notes |
|----------|---------|-------|
| `stress_results/SUMMARY.md` | YES | 4 rows only (last run); does not represent full sweep history |
| `stress_results/SUMMARY.json` | YES | 4 records matching SUMMARY.md |
| Per-rate CSVs (59 files) | YES | 18 frequencies; partial mode coverage at 1100–1300 Hz |
| Per-rate JSON files | NO | Not produced by current tooling; stdout-only via `--json` flag |
| SD binary artifact | NO | No `step_session.bin` in repo (expected — on physical SD card) |
| `esp32/docs/stress-test-sample-rate.md` | YES | Current and accurate |

---

## Code Analysis: SUMMARY.md Gap Detail

The harness writes SUMMARY at lines 634–664. The note field from `RateResult` is built at
lines 454–463 and stored in the dataclass, but is not included in the SUMMARY.md table
render (lines 644–653). The JSON write at lines 665–669 does include all fields via
`r.__dict__`, which is why `note` is visible in SUMMARY.json but not SUMMARY.md.

To add note to SUMMARY.md, the table row format string needs one additional column. The footer
section (lines 654–661) would need one new line such as:

```
First failure at {first_fail_hz} Hz ({first_fail_mode}): {first_fail_note}
```

This is a small, localized change.

---

## Gap Summary for Planning

| Gap | Criterion | Type | Effort |
|-----|-----------|------|--------|
| `note` column absent from SUMMARY.md table | SC4 | Code (1 line) | Trivial |
| "First failure reason" sentence missing from SUMMARY.md footer | SC4 | Code (~5 lines) | Trivial |
| SUMMARY.md only reflects the last run; no consolidated view of all 59 CSVs | SC2 | Architecture | Small |
| No per-rate JSON sidecar files (if required by SC2) | SC2 | Code (~10 lines) | Small |
| Missing `filter_on/sd_off` and `filter_off/sd_on` combos at 1100–1250 Hz | SC1 (data) | Re-run required | Hardware |
| SUMMARY reports cap based on incomplete boundary sweep | SC4 | Data (re-run) | Hardware |

---

## Confidence Assessment

| Area | Level | Reason |
|------|-------|--------|
| SC1 sweep implementation | HIGH | Code read + 59 artifact files confirm live runs |
| SC2 artifact coverage | HIGH | Directory listing enumerated all files |
| SC3 analyzer correctness | HIGH | Both code paths verified to use same _summarize() |
| SC4 first-failure reporting | HIGH | Confirmed note field present in JSON, absent from MD |
| Docs completeness | HIGH | File read directly |

**Research date:** 2026-06-15
**Valid until:** N/A — this is a static codebase snapshot analysis, not library version research
