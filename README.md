# Pollen

A granular processor for Electrosmith patch.init (Daisy Patch SM), based on
Emilie Gillet's open source Beads firmware
(https://github.com/pichenettes/eurorack/tree/master/beads).

Beads has twelve knobs, four buttons and jack detection. patch.init has four
knobs, one button, a toggle and no jack detection, so some of Beads' controls
are reached through layers, button gestures and settings pages, and a few
behaviours are worked around. The sections below describe them, and
Differences from Beads sums them up.

# Using Pollen

## Controls

| Control | Function |
|---|---|
| Knobs CV_1-CV_4 | Density, Time, Size, Pitch |
| Button held + knobs | Shape, Feedback, Dry/Wet, Reverb (hidden layer) |
| Tap, then hold + knobs | Attenuverters of Shape, Time, Size, Pitch (LED blinks fast) |
| Button | Seed. Starts locked, like Beads. Tap to unlock, hold 1.5s to lock |
| Toggle | Freeze (up) |
| Button held + flip toggle | Next quality mode, LED blinks 1-4 times |
| Button held at power-up | Flip the toggle while holding: settings pages. Release: V/Oct calibration |
| CV_5-CV_8 | Time, Size, Pitch (V/Oct), Density or Shape (see Settings pages) |
| Gate in 1 / 2 | Seed / Freeze (while high) |
| Gate out | Grain gate: a pulse per grain, the delay clock in delay mode |
| LED | Freeze (full, half while the frozen buffer is saved), seed (half), seed locked (dim) |

The Pitch knob covers +/-2 octaves and sticks to musical intervals, like
Beads. Size fully clockwise switches to delay mode.

With the seed locked, gates into gate 1 clock the grains, like Beads with
its seed input patched: below noon Density divides the clock, above noon it
sets the probability of a grain per clock. Gate 1 counts as patched while
gates arrived in the last 10 seconds.

## Layers

Holding the button turns the knobs into Shape, Feedback, Dry/Wet and Reverb.
Tapping it and holding it on the second press (within 300ms) turns them into
the attenuverters of Shape, Time, Size and Pitch. A knob only takes over
once it is moved, and back in the main layer it catches up with the stored
value as you turn it, so nothing jumps. A tap that unlocks the seed takes
effect 300ms later, in case it is the start of a tap-then-hold.

What the attenuverters do depends on the CV mode of their parameter (see
Settings pages). By default they set the random modulation, Beads' behaviour
with nothing patched: clockwise randomizes every grain (Pitch: fast random
walk, up to +/-24 semitones), counter-clockwise randomizes some grains
(Pitch: slow walk), and the CV inputs add at full scale. Attenuverter knobs
follow Beads' curve, which is flat around noon and steep towards the ends.

The layer values are saved when the button is released.

## Quality modes

In Beads' order: 1 hi-fi, 2 vintage digital, 3 clean tape, 4 dirty tape.
Hold the button and flip the toggle to step to the next one; the LED blinks
the mode's number. While the button is held the toggle doesn't change
freeze, it follows the toggle's position again on release. Switching clears
the buffer and mutes the output for 250ms. The mode is saved.

## Freeze

The toggle up, or a high gate on gate in 2, freezes the buffer. Like on
Beads, a frozen buffer survives power cycles: 10 seconds after freezing it
is saved, which takes about 4 seconds (LED at half brightness). Power up
with the toggle on Freeze to get it back, in the quality mode it was
recorded in. Unfreezing, or powering up with the toggle down, drops the
saved buffer.

## Mono and stereo

Beads records in mono, with twice the buffer time, when only one audio input
is patched. patch.init can't tell which jacks are patched, and it feeds the
left input to the right one when the right jack is empty. With the input mode
set to detect, the default (see Settings pages), Pollen decides from the
input levels:

- A signal on one side while the other has been silent for 10 seconds (or
  never had any since power-up): mono, the silent side gets a copy.
- Left and right identical (within 5% for a second): mono. This is the
  left-only case. Back to stereo when they differ by more than 20% for a
  second.
- No signal on either side: no change.

Switching clears the buffer and mutes for 250ms, like Beads. It never
happens while frozen; a pending switch is applied on unfreeze.

With the input mode set to mono or stereo, none of this applies: the mode
stays fixed whatever is patched. Mono then copies the input that has a
signal (left if both have one), stereo uses both inputs as they are.

## Settings pages

Hold the button while powering up, and flip the toggle while still holding
it. The toggle position selects the page, flip it again to change pages. A
knob changes its setting only once it is moved, so opening a page doesn't
overwrite anything. Press the button to save and leave (LED solid for a
second); freeze then follows the toggle again. Power-cycle to leave without
saving. A saved frozen buffer is neither restored nor dropped by the setup.

Three-way settings use the knob in thirds: counter-clockwise, noon,
clockwise.

Toggle up, inputs:

| Knob | Setting |
|---|---|
| 1 | Input mode: detect (see Mono and stereo) / always mono / always stereo |
| 2 | Gain: fully counter-clockwise automatic (Beads' AGC), else fixed, 1x to 40x with Beads' curve |

The LED shows the input level after the gain (Beads' VU meter), or the zone
while knob 1 is set.

Toggle down, CV modes, one per attenuverter. Knob 1 is Shape, which has no
input of its own: any mode other than the first takes CV_8 from Density.

| Knob | Parameter | CCW: CV ignored | Noon: CV added | CW: CV attenuverted |
|---|---|---|---|---|
| 1 | Shape | CV_8 goes to Density | CV_8 added to Shape | CV_8 into Shape's attenuverter |
| 2 | Time (CV_5) | | | |
| 3 | Size (CV_6) | | | |
| 4 | Pitch (CV_7) | | | |

- CV ignored: the attenuverter sets the random modulation.
- CV added: the CV is added to the knob at full scale, the attenuverter sets
  the random modulation. The default for Time, Size and Pitch.
- CV attenuverted: Beads' behaviour with a CV patched. Clockwise the
  attenuverter scales the CV (fully clockwise is 1V/Oct for Pitch),
  counter-clockwise the CV sets the amount of random modulation.

The LED shows the zone of the last knob turned (off, half, full), and blinks
before the first.

## V/Oct calibration

Like Beads, with two reference voltages into the pitch input (CV_7):

1. Hold the button while powering up and release it without flipping the
   toggle. The LED blinks once per second.
2. Unpatch CV_5, CV_6 and CV_8, patch 1V into CV_7, press the button. The LED
   blinks twice per second.
3. Patch 3V into CV_7, press the button.

The LED is solid for a second when the calibration is saved, and flickers
when it is rejected: a pitch scale more than 10% off nominal (as on Beads),
or a 0V offset above 0.5V on any input, which means something was left
patched. Power-cycle to cancel. The 0V offsets of the other CV inputs are
measured in step 2 as well.

## Differences from Beads

- No wavetable synth mode.
- 24 grains instead of 32, to stay within the CPU budget.
- The dirty tape mode runs at 48kHz with Beads' downsampling, not with the
  codec at 24kHz.
- Fewer controls: the layers, the settings pages and the button + toggle
  gesture replace Beads' extra knobs and buttons.
- No jack detection: mono/stereo comes from the input levels, the CV modes
  are set in the settings pages, and gate 1 counts as patched after recent
  gates.
- The grain gate has its own jack instead of replacing the right output.

# Technical details

## Source layout

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
- `tools/`: flashing and USB serial from WSL.
- stmlib is used from `../eurorack/stmlib`, libDaisy from `../libDaisy`.

## Build

This build process is made for running on linux inside WSL, some modifications are 
needed if running on a different system setup.

```
make                       # bootloader build (APP_TYPE=BOOT_SRAM, default)
make APP_TYPE=BOOT_NONE    # internal flash, currently ~33K too big
```

`GCC_PATH` defaults to `../arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin`.

## Flashing

The app runs from the Daisy bootloader (BOOT_SRAM). Install the bootloader
on the Patch SM once (`make program-boot` with the module in ROM DFU mode),
then `make program-dfu` while the bootloader is waiting. Sending `b` over
the USB serial port makes the running app jump to the bootloader (no
timeout), so updates don't need the RESET/BOOT buttons.

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

## USB serial

Once per second: quality mode, mono or stereo, audio blocks run, average and
peak CPU load for that second, gate out pulses, input peak levels and the
peak difference between them, the knob positions, the hidden layer values,
the attenuverters (-100 to 100), the buffer backup progress (100% when a
complete backup is saved), the pitch CV in cents and the calibration
(semitones per unit x100, pitch offset in mV), and the settings pages: input
mode, gain (0 automatic) and CV modes (0 ignored, 1 added, 2 attenuverted).

Commands: `b` jumps to the bootloader, `1`-`4` select the quality mode, `c`
starts the V/Oct calibration.

## Memory

| Buffer | Beads (H743) | BOOT_SRAM | BOOT_NONE |
|---|---|---|---|
| Code + tables (~161K) | flash | AXI SRAM | flash (doesn't fit) |
| Delay bank 0 (256K) | AXI SRAM | AXI SRAM (after code) | AXI SRAM |
| Delay bank 1 (256K) | AXI SRAM | SDRAM | AXI SRAM |
| Delay bank 2 (256K) | D2 SRAM | D2 SRAM | D2 SRAM |
| Reverb (64K) | D3 SRAM | D3 SRAM | D3 SRAM |
| .data/.bss/heap/stack | DTCM | DTCM | DTCM |

## CPU load

Worst case measured (density, size and pitch near max), 24 grains, 480 MHz:

| Mode | Average | Peak |
|---|---|---|
| hi-fi | 69-73% | 76-82% |
| vintage digital | 54-56% | 61-62% |
| clean tape | 51% | 55-56% |
| dirty tape | 46% | 50-51% |

## Mono detection

Levels are followed with a 50ms time constant, so single clicks don't
count. The silence threshold is about -50 dBFS. The two inputs go through
separate ADC channels, so "identical" allows a 5% difference (measured 1-2%,
up to 4% while the ADC settles after a codec restart). Detection pauses
during the 250ms mute after a codec restart, which puts a burst on the
inputs.

## Settings storage

The quality mode, the layers, the settings pages and the CV calibration are
saved to QSPI at `0x90400000`: after a mode change, when leaving the setup,
and when the button is released after editing a layer. Only changed values
cause a write (one 4K sector erase). The address is clear of the
bootloader's reserved first 256K, the app area (from `0x90040000`, at most
480K) and the last 256K, which is outside the bootloader's DFU map, so app
updates leave it alone. Bump `kSettingsVersion` when the `Settings` struct
changes, and add new fields at the end so `LoadSettings()` can migrate older
versions (version 1 had no attenuverters, version 2 no calibration, version 3
no settings pages).

## Frozen buffer backup

The 768000 byte delay buffer is copied to QSPI at `0x410000` (12 64K blocks
right after the settings), one block erase or 4K write per pass of the main
loop, so audio, serial commands and settings saves keep running. The header
(quality mode, mono, write pointer) is written last and marks the backup as
complete, so a power cut halfway leaves nothing to restore. Invalidating it
zeroes the tag, which needs no erase.

# License

MIT, see `LICENSE`. The files in `beads/` are Emilie Gillet's Beads code,
also MIT licensed, with the notice in each file.
