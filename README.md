# Pollen

A granular processor for Electrosmith patch.init (Daisy Patch SM), based on
Emilie Gillet's open source Beads firmware
(https://github.com/pichenettes/eurorack/tree/master/beads).

## Layout

- `beads/`: copy of `eurorack/beads` at upstream commit `f4cfa76` (MIT,
  Emilie Gillet). Only the DSP, gain control, pot controller and lookup
  tables are included. The wavetable synth (`dsp/synth.*`, `dsp/wavetable_*`)
  is left out. Local changes are marked `pollen:`
  - `dsp/processor.h`: `ENABLE_HARDWARE_24K` disabled, libDaisy can't run
    the codec at 24kHz.
  - `gain_control.*`: takes float frames instead of Beads' int32 codec frames.
  - `dsp/processor.*`: `gate()` reports whether the grain gate was high in
    the last block, for the gate out jack.
  - `dsp/delay_granular_player.h`: 24 grains instead of 32
    (`POLLEN_MAX_GRAINS`). With 32 the worst case peaks above 100% CPU.
- `src/pollen.cpp`: replaces Beads' `beads.cc`, `ui.cc`, `cv_reader.cc` and
  `drivers/` with libDaisy.
- `src/buffer_backup.*`: QSPI version of Beads' `background_buffer_backup`.
- `pollen_sram.lds` / `pollen_flash.lds`: libDaisy's linker scripts
  plus sections for the delay banks and reverb buffer.
- stmlib is used from `../eurorack/stmlib`, libDaisy from `../libDaisy`.

## Build

```
make                       # bootloader build (APP_TYPE=BOOT_SRAM, default)
make APP_TYPE=BOOT_NONE    # internal flash, currently ~33K too big
```

`GCC_PATH` defaults to `../arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin`.

## Memory

| Buffer | Beads (H743) | BOOT_SRAM | BOOT_NONE |
|---|---|---|---|
| Code + tables (~161K) | flash | AXI SRAM | flash (doesn't fit) |
| Delay bank 0 (256K) | AXI SRAM | AXI SRAM (after code) | AXI SRAM |
| Delay bank 1 (256K) | AXI SRAM | SDRAM | AXI SRAM |
| Delay bank 2 (256K) | D2 SRAM | D2 SRAM | D2 SRAM |
| Reverb (64K) | D3 SRAM | D3 SRAM | D3 SRAM |
| .data/.bss/heap/stack | DTCM | DTCM | DTCM |

## Flashing (BOOT_SRAM)

Needs the Daisy bootloader on the Patch SM once (`make program-boot` with the
module in ROM DFU mode), then `make program-dfu` while the bootloader is
waiting. Sending `b` over the USB serial port makes the running app jump to
the bootloader (no timeout), so updates don't need the RESET/BOOT buttons.

From WSL, with usbipd-win on the Windows side:

```
make flash                  # build, jump to the bootloader, flash, wait for the app
make monitor                # status lines until Ctrl-C (DURATION=30 for 30 s)
make send CMD=c             # serial commands: b, 1-4, c
```

`tools/flash.sh` and `tools/serial.sh` do the work. The serial port stays on
Windows and is opened through PowerShell. Variables, with their defaults:
`COM_PORT=COM4`, `USBIPD_BUSID=1-2`, `WSL_DISTRO` (the distribution `make`
runs in) and `USBIPD` (the path to `usbipd.exe`).

Setup:

- The Daisy bootloader is its own USB device for usbipd, separate from the
  ROM DFU loader: bind it once with `usbipd bind --busid 1-2` as
  administrator.
- dfu-util needs access to 0483:df11. The udev rule must sort after
  `50-udev-default.rules`, which resets USB device nodes to 0664, e.g.
  `/etc/udev/rules.d/70-stm32-dfu.rules`:
  `SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666"`
- The attach always names the WSL distribution. Without it usbipd starts the
  default one, and when that shuts down again its `systemd-binfmt` clears the
  binfmt entries all distributions share, which breaks running `.exe` files.
  To repair: `sudo sh -c 'echo ":WSLInterop:M::MZ::/init:PF" >
  /proc/sys/fs/binfmt_misc/register'`.

## Panel

| Control | Function |
|---|---|
| Knobs CV_1-CV_4 | Density, Time, Size, Pitch |
| Button held + knobs | Shape, Feedback, Dry/Wet, Reverb |
| Tap, then hold + knobs | Attenuverters of Shape, Time, Size, Pitch (LED blinks) |
| Button | Seed. Starts locked like Beads. Tap to unlock (applied 300ms later, in case it's a tap-then-hold), hold 1.5s to lock |
| Toggle | Freeze |
| Button held + flip toggle | Next quality mode, LED blinks 1-4 times |
| Button held at power-up | V/Oct calibration (see below) |
| CV_5-CV_8 | Time, Size, Pitch (V/Oct), Density |
| Gate in 1 / 2 | Seed / Freeze |
| Gate out | Grain gate, the signal Beads puts on its right output in gate output mode |
| LED (CV_OUT_2) | Freeze (full), seed (half), seed locked (dim) |

The attenuverters only set the random modulation, Beads' behaviour with
nothing patched: clockwise randomizes every grain (Pitch: fast random walk,
up to +/-24 semitones), counter-clockwise randomizes some grains (Pitch: slow
walk). Without jack detection there is no way to know whether a CV is
patched, so the CV inputs always add at full scale instead of being scaled
by the attenuverters. Attenuverter knobs follow Beads' curve, with a dead
zone at noon.

Quality modes, in Beads' order: 1 hi-fi, 2 vintage digital, 3 clean tape,
4 dirty tape. Freeze ignores the toggle while the button is held and follows
its position again on release.

## Settings

The quality mode, the hidden layer (Shape, Feedback, Dry/Wet, Reverb), the
attenuverters and the CV calibration are saved to QSPI at `0x90400000`, after a mode change and when
the button is released after editing a layer. Only changed values cause a write
(one 4K sector erase). The address is clear of the bootloader's reserved first
256K, the app area (from `0x90040000`, at most 480K) and the last 256K, which
is outside the bootloader's DFU map, so app updates leave it alone. Bump
`kSettingsVersion` when the `Settings` struct changes, and add new fields at
the end so `LoadSettings()` can migrate older versions (version 1 had no
attenuverters, version 2 no calibration).

## V/Oct calibration

Like Beads, with two reference voltages into the pitch input (CV_7):

1. Hold the button while powering up (or send `c` over USB serial). The LED
   blinks once per second.
2. Unpatch CV_5, CV_6 and CV_8, patch 1V into CV_7, press the button. The LED
   blinks twice per second.
3. Patch 3V into CV_7, press the button.

The LED is solid for a second when the calibration is saved, and flickers
when it is rejected: a pitch scale more than 10% off nominal (as on Beads),
or a 0V offset above 0.5V on any input, which means something was left
patched. Power-cycle to cancel. The 0V offsets of the other CV inputs are
measured in step 2 as well. The status line shows the pitch CV in cents and
the calibration (semitones per unit x100, pitch offset in mV).

## Mono detection

Beads records in mono, with twice the buffer time, when only one audio input
is patched. patch.init has no jack detection, and it feeds the left input to
the right one when the right jack is empty, so mono is detected from the
input levels instead:

- A signal on one side while the other has been silent for 10 seconds (or
  never had any since power-up): mono, the silent side gets a copy.
- Left and right identical (within 5% for a second): mono. This is the
  left-only case. Back to stereo when they differ by more than 20% for a
  second.
- No signal on either side: the mode doesn't change.

Switching clears the buffer and mutes for 250ms, like Beads. It never
happens while frozen; a pending switch is applied on unfreeze. Detection
pauses during the mute, because a codec restart puts a burst on the inputs.

## Frozen buffer backup

Like Beads, a frozen buffer survives power cycles. After freeze has been on
for 10 seconds, the 768000 byte delay buffer is copied to QSPI at `0x410000`
(12 64K blocks right after the settings), one block erase or 4K write per
pass of the main loop. It takes about 4 seconds; the freeze LED is at half
brightness meanwhile. The header (quality mode, mono, write pointer) is
written last and marks the backup as complete.

At power-up, a complete backup is restored if the toggle is on Freeze.
Unfreezing aborts a backup in progress and invalidates a complete one by
zeroing its tag, so does powering up with the toggle off.

## USB serial

Once per second: quality mode, mono or stereo, audio blocks run, average and
peak CPU load for that second, gate out pulses, input peak levels and the peak difference
between them, the knob positions, the hidden layer values, the attenuverters
(-100 to 100) and the buffer
backup progress (100% when a complete backup is stored). Commands: `b` jumps to the
bootloader, `1`-`4` select the quality mode, `c` starts the V/Oct
calibration.

Worst case measured (density, size and pitch near max), 24 grains:

| Mode | Average | Peak |
|---|---|---|
| hi-fi | 69-73% | 76-82% |
| vintage digital | 54-56% | 61-62% |
| clean tape | 51% | 55-56% |
| dirty tape | 46% | 50-51% |

## License

MIT, see `LICENSE`. The files in `beads/` are Emilie Gillet's Beads code,
also MIT licensed, with the notice in each file.
