# Sample Rate Change Bug Analysis

## Executive Summary

The sample rate change at the plugin level does **not propagate correctly** to OpenEphys or internal recordings due to **three critical issues**:

1. **Dynamic sample rate changes during streaming are ignored** - The hardware timing loop doesn't recalculate when the rate changes mid-stream
2. **CSV timestamps use hardcoded rate** - Internal recordings have incorrect timestamps  
3. **Sample rate not reflected in OpenEphys** - May require signal chain update notification

---

## Issue 1: Sample Rate Changes During Streaming Are Ignored

### Root Cause
The streaming hardware timing (`run_stream()` in RedPitaya_justin.c) calculates `ticks_per_sample` **once at startup** based on `g_stream_hw_hz`:

```c
// RedPitaya_justin.c, line 1339-1342
static int run_stream(int client_fd, HardwareContext *ctx, FILE *bin_file, FILE *csv_file, int base_channels) {
    int hw_hz = g_stream_hw_hz;  // READ g_stream_hw_hz ONCE
    if (hw_hz < 1) hw_hz = 1;
    if (hw_hz > 2000) hw_hz = 2000;
    uint32_t ticks_per_sample = (uint32_t)(CTR_CLK_RATE / hw_hz);  // CALCULATED ONCE
```

Later, when the FREQ command is received **during active streaming**, it updates `g_stream_hw_hz`:

```c
// RedPitaya_justin.c, line 948-953
if (strstr(cmd, "FREQ:")) {
    int frequency_hz = atoi(strstr(cmd, "FREQ:") + 5);
    if (frequency_hz < 1) frequency_hz = 1;
    if (frequency_hz > 2000) frequency_hz = 2000;
    g_stream_hw_hz = frequency_hz;  // ← UPDATES GLOBAL
    printf("Hardware stream tick rate set to %d Hz...\n", g_stream_hw_hz);
}
```

**But**: The `run_stream()` function never re-reads `g_stream_hw_hz` from the volatile global variable. It continues using the old `ticks_per_sample` value calculated at startup.

### Evidence
- Line 1378 in run_stream loop uses hardcoded `ticks_per_sample`:
  ```c
  while (1) {
      uint32_t now = *ctx->gpio_counter;
      if ((now - last_counter) < ticks_per_sample) {  // ← NEVER CHANGES
          // ...
          continue;
      }
      last_counter += ticks_per_sample;  // ← USES OLD VALUE
  ```

### Impact
- ✗ USB/network streaming rate does **not change**
- ✗ Binary file recording rate does **not change**
- ✗ CSV logging rate does **not change**
- ✓ Plugin-side `settings.boardSampleRate` **is updated** (but OpenEphys doesn't know)

---

## Issue 2: CSV Timestamps Use Hardcoded Rate Instead of Actual Rate

### Root Cause
The CSV row writer uses the **compile-time constant** `DESIRED_SAMPLE_RATE_HZ` (100 Hz) instead of the dynamic `g_stream_hw_hz`:

```c
// RedPitaya_justin.c, line 496 (write_csv_row function)
fprintf(fp, "%d,%.6f", sample_index, (double)sample_index / (double)DESIRED_SAMPLE_RATE_HZ);
//                                                           ^^^^^^^^^^^^^^^^^^^^^^^^
//                                                           HARDCODED 100 Hz!
```

### Impact
- If user sets sample rate to 1000 Hz, but `DESIRED_SAMPLE_RATE_HZ = 100`:
  - Sample 1000 gets timestamp `1000 / 100 = 10.0` seconds ❌
  - Should be `1000 / 1000 = 1.0` seconds ✓
- All internal CSV recordings have **grossly incorrect timestamps** (~10x error if hardware rate is 1000 Hz)

---

## Issue 3: VQF Fusion Statistics Report Uses Hardcoded Rate

### Root Cause
Similar hardcoded issue in the VQF performance reporting:

```c
// RedPitaya_justin.c, line 719 (maybe_report_vqf_stats function)
if (with_fusion && *vqf_call_count > 0 && (sample_number % DESIRED_SAMPLE_RATE_HZ) == 0) {
//                                               ^^^^^^^^^^^^^^^^^^^^^^^^
//                                               Reports every 100 samples, not every 1 sec at actual rate
```

### Impact
- VQF timing stats are printed at wrong intervals
- Minor compared to Issues 1 & 2, but indicates systemic hardcoded rate problem

---

## Issue 4: OpenEphys May Not Know About Sample Rate Changes

### Potential Root Cause
When the sample rate label changes in the UI (device editor.cpp, line 908-925):

```cpp
void DeviceEditor::labelTextChanged (Label* labelThatHasChanged)
{
    if (labelThatHasChanged == sampleRateLabel.get())
    {
        // ...
        board->setSampleRate (clamped);  // ✓ Updates plugin-side rate
        board->updateSampleFrequency (newFreq);  // ✓ Sends to Red Pitaya
        
        if (redPitayaSensorUiBuilt && acquisitionIsActive)
        {
            repopulateSensorRateComboForHwHz (newFreq > 0 ? newFreq : 100);
            CoreServices::updateSignalChain (this);  // ← Notifies OpenEphys
        }
    }
}
```

**Note**: `CoreServices::updateSignalChain()` is only called if `redPitayaSensorUiBuilt && acquisitionIsActive`. 

### Questions to Verify
1. Is the signal chain update happening for all board types or only Red Pitaya?
2. Does the signal chain update properly notify OpenEphys of the new sample rate?
3. Should this update happen **before** streaming starts, or can it happen mid-stream?

---

## Data Flow Diagram

```
User changes sample rate in UI
          ↓
labelTextChanged() (device editor.cpp:908)
          ↓
    ┌─────┴─────┐
    ↓           ↓
setSampleRate() updateSampleFrequency()
(C++ plugin)   (sends FREQ:XXX to Red Pitaya)
    ↓           ↓
    │      ┌────┴────┐
    │      ↓         ↓
    │   (receives  (updates
    │    in        g_stream_hw_hz
    │    process_  during active
    │    stream_   streaming)
    │    commands)    ↓
    │      │      ❌ BUT ticks_per_sample
    │      │         NOT recalculated
    │      │
    ↓      ↓
OpenEphys knows   Hardware ignores
the new rate      the new rate
    ✓ (if          ✗ Still uses old
       notified)      ticks_per_sample

     MISMATCH!
```

---

## Data Flow Issues - USB/Binary/CSV Recording

### Current Flow
1. USB streaming sends frames at `ticks_per_sample` interval (calculated at startup)
2. Binary recording copies those frames as-is
3. CSV logging writes timestamps using **hardcoded** `DESIRED_SAMPLE_RATE_HZ`

### What Breaks When User Changes Rate During Streaming
- Frame rate from Red Pitaya: **OLD rate** (because `ticks_per_sample` not recalculated)
- OpenEphys expects: **NEW rate** (because `updateSignalChain()` was called)
- CSV timestamps: **Wildly wrong** (hardcoded 100 Hz)

---

## Fixes Required

### Fix 1: Make `ticks_per_sample` Dynamic (CRITICAL)

**Location**: `run_stream()` in RedPitaya_justin.c

**Current Code** (lines 1378-1380):
```c
while (1) {
    uint32_t now = *ctx->gpio_counter;
    if ((now - last_counter) < ticks_per_sample) {  // ← STALE VALUE
```

**Solution**: Recalculate on each iteration:
```c
while (1) {
    int current_hw_hz = g_stream_hw_hz;  // READ volatile global
    if (current_hw_hz < 1) current_hw_hz = 1;
    if (current_hw_hz > 2000) current_hw_hz = 2000;
    uint32_t current_ticks_per_sample = (uint32_t)(CTR_CLK_RATE / current_hw_hz);
    
    uint32_t now = *ctx->gpio_counter;
    if ((now - last_counter) < current_ticks_per_sample) {
```

**Impact**: Allows real-time sample rate changes to take effect

---

### Fix 2: Use Dynamic Rate for CSV Timestamps (CRITICAL)

**Location**: `write_csv_row()` in RedPitaya_justin.c, line 496

**Current Code**:
```c
fprintf(fp, "%d,%.6f", sample_index, (double)sample_index / (double)DESIRED_SAMPLE_RATE_HZ);
```

**Solution**: Accept `g_stream_hw_hz` as parameter or read volatile global:
```c
int actual_hz = g_stream_hw_hz > 0 ? g_stream_hw_hz : DESIRED_SAMPLE_RATE_HZ;
fprintf(fp, "%d,%.6f", sample_index, (double)sample_index / (double)actual_hz);
```

**Impact**: CSV timestamps will be correct for internal recordings

---

### Fix 3: Use Dynamic Rate for VQF Reporting (MINOR)

**Location**: `maybe_report_vqf_stats()` in RedPitaya_justin.c, line 719

**Current Code**:
```c
if (with_fusion && *vqf_call_count > 0 && (sample_number % DESIRED_SAMPLE_RATE_HZ) == 0) {
```

**Solution**:
```c
int actual_hz = g_stream_hw_hz > 0 ? g_stream_hw_hz : DESIRED_SAMPLE_RATE_HZ;
if (with_fusion && *vqf_call_count > 0 && (sample_number % actual_hz) == 0) {
```

**Impact**: VQF stats reported at correct intervals

---

### Fix 4: Verify OpenEphys Update Happens for All Boards

**Location**: `DeviceEditor::labelTextChanged()` in device editor.cpp, line 908

**Current Code** (line 927):
```cpp
if (redPitayaSensorUiBuilt && acquisitionIsActive)
{
    // ...
    CoreServices::updateSignalChain (this);
}
```

**Issue**: This ONLY updates for Red Pitaya when acquisition is active!

**Solution**: Ensure `CoreServices::updateSignalChain()` is called for all board types:
```cpp
void DeviceEditor::labelTextChanged (Label* labelThatHasChanged)
{
    if (labelThatHasChanged == sampleRateLabel.get())
    {
        // ... existing code ...
        
        // Notify OpenEphys of sample rate change for ALL boards
        if (acquisitionIsActive)
            CoreServices::updateSignalChain (this);
    }
}
```

**Impact**: OpenEphys will always be notified of rate changes

---

## Testing Checklist

- [ ] Change sample rate **before** starting acquisition → verify OpenEphys shows new rate
- [ ] Start acquisition at rate A → change to rate B → verify:
  - [ ] USB streaming receives frames at rate B (check with protocol analyzer)
  - [ ] Binary file has correct sample count at rate B
  - [ ] CSV timestamps are correct
  - [ ] OpenEphys display reflects rate B
- [ ] Verify `g_stream_hw_hz` is being read dynamically (add logging)
- [ ] Verify `ticks_per_sample` is recalculated each iteration
- [ ] Verify CSV timestamp formula uses actual hardware rate

---

## Summary Table

| Issue | Severity | Location | Status |
|-------|----------|----------|--------|
| Stale `ticks_per_sample` during stream | **CRITICAL** | RedPitaya_justin.c:1378 | ❌ Not fixed |
| Hardcoded CSV timestamps | **CRITICAL** | RedPitaya_justin.c:496 | ❌ Not fixed |
| Hardcoded VQF reporting rate | **MINOR** | RedPitaya_justin.c:719 | ❌ Not fixed |
| OpenEphys update only for Red Pitaya | **HIGH** | device editor.cpp:927 | ⚠️ Partial coverage |

