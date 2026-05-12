# PREEMPT_RT Integration Plan

Documented 2026-05-12. Not yet implemented — read this fully before starting.

---

## Problem statement

The Red Pitaya server (`RedPitaya_justin.c`) has a hardware GPIO counter-based timing loop
that runs at up to 2000 Hz. Average loop timing is acceptable (~600 µs with two sensors).
The problem is **rare worst-case spikes** that push iteration time above 1 ms.

These spikes are not caused by other userspace processes competing for the CPU. They are
caused by **kernel-level work** (hardware IRQ handlers, softirqs, RCU callbacks) preempting
the sensor loop without warning. On a stock Linux kernel, even a `SCHED_FIFO` priority-99
thread can be frozen for hundreds of microseconds while the kernel handles an interrupt.

CPU core isolation (`isolcpus=`) would solve this but is not viable here because both
Cortex-A9 cores are occupied by parallel sensor threads.

PREEMPT_RT is the correct long-term fix. It converts IRQ handlers into schedulable kernel
threads, allowing a `SCHED_FIFO` userspace task to preempt them. Worst-case latency drops
from 500–2000 µs to 10–50 µs.

---

## Red Pitaya environment

```
OS:      Ubuntu 22.04.5 LTS
Kernel:  5.15.0-xilinx (armv7l — ARM Cortex-A9, 32-bit)
Branch:  branch-redpitaya-v2024.1
Commit:  3afd5c148a4c80d7bcd37748e39b3f79a4e65ddd
Board:   rp-F0F85A.local
```

The kernel is a **Xilinx-patched build**, not a standard Ubuntu kernel. This matters
because:
- Ubuntu Pro's `linux-realtime` package targets amd64/arm64, not armv7l.
- Installing a stock Ubuntu RT kernel would replace the Xilinx-patched one and break
  all AXI GPIO, AXI I2C, AXI SPI, and FPGA fabric access.
- The only safe path is to apply the PREEMPT_RT patch to the Xilinx 5.15 source tree,
  cross-compile it, and install it alongside the existing boot files.

---

## Risk assessment

| Outcome | Likelihood |
|---------|-----------|
| Boots fine, all AXI peripherals work | ~50% |
| Boots but some AXI/FPGA config needs fixing | ~35% |
| Does not boot — SD card restore required (~10 min) | ~15% |
| Permanent hardware damage | ~0% |

The SD card is the complete safety net. A full image backup before starting means the
absolute worst case is a 10-minute reflash.

---

## Step 0 — Collect exact version info (on Red Pitaya)

Run before doing anything else. The PREEMPT_RT patch version must match the exact
kernel subversion.

```bash
uname -r        # e.g. 5.15.0-1-xilinx or 5.15.36-xilinx
uname -m        # armv7l
uname -v        # build timestamp and config

zcat /proc/config.gz | grep -i preempt   # check current preemption config
cat /sys/kernel/realtime 2>/dev/null || echo "not RT"

cat /proc/cpuinfo | grep -E "model|Hardware|Revision"
df -h /boot     # make sure /boot has space for a second kernel
```

Save all output before continuing.

---

## Step 1 — Full SD card backup (on your PC)

**Do this before touching anything on the board.**

Power off the Red Pitaya. Remove the SD card. Plug it into your PC.

**Linux PC:**
```bash
lsblk   # identify SD card device, e.g. /dev/sdb

sudo dd if=/dev/sdX of=~/redpitaya_backup_$(date +%Y%m%d).img bs=4M status=progress

# Verify — both hashes must match
md5sum /dev/sdX
md5sum ~/redpitaya_backup_$(date +%Y%m%d).img
```

**Windows PC:** Use Win32DiskImager → select drive → Read → save as `redpitaya_backup.img`.

---

## Step 2 — Back up kernel files on the board

SSH in after reinserting the SD card and powering back on.

```bash
ssh root@rp-F0F85A.local

sudo cp /boot/vmlinuz-$(uname -r)    /boot/vmlinuz-$(uname -r).backup
sudo cp /boot/config-$(uname -r)     /boot/config-$(uname -r).backup
sudo cp /boot/initrd.img-$(uname -r) /boot/initrd.img-$(uname -r).backup 2>/dev/null || true

sudo tar czf /home/root/modules_backup_$(uname -r).tar.gz /lib/modules/$(uname -r)/

ls -lh /boot/*.backup
ls -lh /home/root/modules_backup_*.tar.gz
```

Pull backups off the board:
```bash
# On your PC
mkdir -p ~/redpitaya_backup
scp root@rp-F0F85A.local:/home/root/modules_backup_*.tar.gz ~/redpitaya_backup/
```

Also record the current boot configuration:
```bash
# On Red Pitaya
ls /boot/
cat /boot/uEnv.txt 2>/dev/null || true
```

---

## Step 3 — Set up cross-compile toolchain (on Ubuntu PC)

```bash
sudo apt update
sudo apt install -y \
    gcc-arm-linux-gnueabihf \
    g++-arm-linux-gnueabihf \
    bc bison flex libssl-dev make libncurses-dev \
    git wget curl xz-utils

arm-linux-gnueabihf-gcc --version   # verify install
```

---

## Step 4 — Get the Red Pitaya kernel source (on Ubuntu PC)

```bash
mkdir ~/rp-kernel && cd ~/rp-kernel

git clone https://github.com/RedPitaya/linux-xlnx.git
cd linux-xlnx

git checkout branch-redpitaya-v2024.1

# Confirm the top commit matches the board
git log --oneline -3
# Should show 3afd5c148 near the top
```

---

## Step 5 — Download and apply the PREEMPT_RT patch

PREEMPT_RT patches for 5.15.x are published at:
`https://cdn.kernel.org/pub/linux/kernel/projects/rt/5.15/`

The filename format is `patch-5.15.YY-rtZZ.patch.xz` where `YY` is the kernel sub-version
from `uname -r` (collected in Step 0) and `ZZ` is the RT patch revision. Pick the highest
`rtZZ` available for your `5.15.YY`.

```bash
cd ~/rp-kernel

# Download — substitute correct version numbers
wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/5.15/patch-5.15.YY-rtZZ.patch.xz
xz -d patch-5.15.YY-rtZZ.patch.xz

# Apply to kernel source
cd linux-xlnx
patch -p1 < ../patch-5.15.YY-rtZZ.patch

# Check for rejects — Xilinx-specific files may have minor conflicts
find . -name "*.rej"
# If rejects appear, inspect each .rej file and apply the intent manually
```

---

## Step 6 — Configure the kernel

```bash
cd ~/rp-kernel/linux-xlnx

export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

# Use the board's current config as the base
scp root@rp-F0F85A.local:/boot/config-$(ssh root@rp-F0F85A.local uname -r) .config

make menuconfig
```

In the menu, make the following changes:

**Enable full RT preemption:**
```
General Setup
  → Preemption Model
    → [X] Fully Preemptible Kernel (Real-Time)
```

**Enable tickless operation and high-resolution timers:**
```
General Setup
  → Timers subsystem
    → Timer tick handling → Full dynticks system (tickless)
    → [X] High Resolution Timer Support
```

Save and exit.

---

## Step 7 — Compile (on Ubuntu PC)

```bash
cd ~/rp-kernel/linux-xlnx

export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

make -j$(nproc) zImage modules dtbs 2>&1 | tee build.log

# Takes 45–90 minutes depending on the PC
# If it fails, inspect the last 30 lines of build.log
```

---

## Step 8 — Install onto the Red Pitaya

```bash
cd ~/rp-kernel/linux-xlnx

export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

# Stage modules
make INSTALL_MOD_PATH=~/rp-kernel/staging modules_install

NEW_VERSION=$(cat include/config/kernel.release)

# Copy kernel and device tree blobs
scp arch/arm/boot/zImage root@rp-F0F85A.local:/boot/vmlinuz-${NEW_VERSION}-rt
scp arch/arm/boot/dts/*.dtb root@rp-F0F85A.local:/boot/

# Copy modules
rsync -avz ~/rp-kernel/staging/lib/modules/ root@rp-F0F85A.local:/lib/modules/
```

---

## Step 9 — Update bootloader

```bash
# On Red Pitaya
ssh root@rp-F0F85A.local

cp /boot/uEnv.txt /boot/uEnv.txt.backup    # back up first

# Edit uEnv.txt to point at the new kernel image
# The exact key depends on the bootloader config recorded in Step 2.
# Typically: change the uimage= or kernel= line to the new filename.
nano /boot/uEnv.txt
```

The exact bootloader config depends on what `ls /boot/` and `cat /boot/uEnv.txt`
showed in Step 2. Do not guess — confirm the correct key name before editing.

---

## Step 10 — Verify

```bash
sudo reboot

# After reboot
ssh root@rp-F0F85A.local

uname -r                       # should show new version with -rt suffix
cat /sys/kernel/realtime       # should output: 1
```

If the board does not come back up, restore from SD card image (Step 1 backup).

---

## Step 11 — Software RT changes to RedPitaya_justin.c

Installing the RT kernel is necessary but not sufficient. The server process must
opt into RT scheduling to benefit from it. These changes are needed in
`RedPitaya_justin.c` regardless of whether PREEMPT_RT is installed:

### 1. Lock all memory pages at startup

Prevents page faults mid-loop from swapped-out pages. Add to `main()` before
hardware init:

```c
#include <sys/mman.h>

if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    perror("mlockall failed");
    // non-fatal — continue, but jitter may be higher
}
```

### 2. Set SCHED_FIFO on the main thread

Add after `mlockall` in `main()`:

```c
#include <sched.h>

struct sched_param sp = { .sched_priority = 80 };
if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
    perror("sched_setscheduler failed (need root)");
}
```

Priority 80 is high enough to preempt normal work but leaves headroom below
priority 99 for any system-critical RT tasks.

### 3. Set SCHED_FIFO on worker threads

In `sensor_threads_init()`, after each `pthread_create`:

```c
struct sched_param sp = { .sched_priority = 80 };
pthread_setschedparam(g_sensor_threads[i].thread, SCHED_FIFO, &sp);
```

### 4. Replace usleep(100) with clock_nanosleep

The `usleep(100)` in the main timing loop (called when the hardware counter has
not yet reached the next tick) yields to the scheduler with no deadline guarantee.
Replace with an absolute-time sleep so the wakeup is bounded:

```c
// Before:
usleep(100); continue;

// After:
struct timespec ts;
clock_gettime(CLOCK_MONOTONIC, &ts);
ts.tv_nsec += 50000;   // 50 µs
if (ts.tv_nsec >= 1000000000L) {
    ts.tv_nsec -= 1000000000L;
    ts.tv_sec++;
}
clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
continue;
```

---

## Recovery procedure

```bash
# On your PC — full SD card restore (10 minutes)
sudo dd if=~/redpitaya_backup_YYYYMMDD.img of=/dev/sdX bs=4M status=progress
sync
```

This restores the board to exactly the state it was in before starting.

---

## Expected outcomes

| Configuration | Typical worst-case jitter |
|---|---|
| Current (no RT settings, stock kernel) | 500–2000 µs |
| Software RT only (SCHED_FIFO + mlockall + clock_nanosleep) | 100–400 µs |
| PREEMPT_RT kernel + software RT | 10–50 µs |

The 1 ms budget is met by either approach in typical conditions. PREEMPT_RT is the
only option that eliminates the rare worst-case spikes caused by kernel IRQ handlers.
