# FC-CLAMP DShot telemetry repair

This branch targets `hkust_nxt-dual_default` (HKUST NxtPX4, STM32H743, board ID
1013), based on PX4 commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
The connected aircraft reported this exact baseline on 2026-10-24.

Repair commit `497d22e8ac14d6115b4694b247ecf328dd424632` changes two files:

- `platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c`: reject invalid GCR
  symbols before shifting, publish only valid samples, preserve a pending valid
  reading, and consume each sample once under an interrupt critical section.
- `src/drivers/dshot/DShot.cpp`: update valid channels independently when another
  channel has failed telemetry.

Valid stopped-motor reports still produce zero RPM. Failed captures produce no
new sample and do not refresh that motor's published measurement timestamp.

## Build and check

When this repository is the `onboard/HKUST-Nxt-Dual-Valid-Telemetry` submodule of
FC-CLAMP_Real_Drone, run from the parent repository:

```bash
git submodule update --init onboard/HKUST-Nxt-Dual-Valid-Telemetry
git -C onboard/HKUST-Nxt-Dual-Valid-Telemetry submodule update --init --recursive -- \
  platforms/nuttx/NuttX/nuttx platforms/nuttx/NuttX/apps \
  src/modules/mavlink/mavlink src/drivers/gps/devices \
  src/lib/events/libevents src/lib/heatshrink/heatshrink \
  src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client

mkdir -p log tmp/px4-dshot-host-check
python3 onboard/HKUST-Nxt-Dual-Valid-Telemetry/validation/fc_clamp_dshot/check_telemetry.py \
  --source "$PWD/onboard/HKUST-Nxt-Dual-Valid-Telemetry" \
  --build-dir "$PWD/tmp/px4-dshot-host-check" \
  > "$PWD/log/px4-dshot-host-check.log" 2>&1

make -C onboard/HKUST-Nxt-Dual-Valid-Telemetry hkust_nxt-dual_default -j6 \
  > "$PWD/log/px4-dshot-build.log" 2>&1
```

The build requires the standard PX4 Python dependencies and ARM GCC toolchain.
The application is `build/hkust_nxt-dual_default/hkust_nxt-dual_default.px4`
inside this repository. The host check requires Python 3 and `g++`; its source
and output directories can also point to a standalone PX4 checkout and scratch
directory.

The check compiles the actual decoder, sample storage and publisher functions
with host hardware stubs. It covers every 20-bit GCR input, all 4096 valid
payloads, invalid captures, pending and consumed samples, valid stopped motors,
unconfigured channels and per-channel timestamps. The original source fails the
invalid-GCR assertion; the repaired source passes. DMA timing and real interrupt
scheduling still require hardware validation.

## Deployment

The existing PX4 Bootloader supports USB application updates. Match the live
board identity, back up its parameters, and confirm disarmed/landed state and
the physical maintenance conditions before an authorized upload. Keep the
uploader's board-ID and size checks enabled. After upload, verify the new hash,
parameters, sensor startup and localization recovery before motor testing.

Both the baseline and repair commit were built with GCC 13.2.1 on 2026-10-24.
The repair image was 1,807,320 bytes within the 1,835,008-byte application region;
parameter metadata matched the baseline. Preparation records are in the parent
repository's `docs/VALIDATION_20261024_PX4_DSHOT.md` and local
`log/px4-dshot-fix-20261024/` directory. At preparation time the repair had not
been flashed, and ground RPM/flight validation remained outstanding.
