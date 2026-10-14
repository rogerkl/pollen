// Pollen: a granular processor for Electrosmith patch.init, based on
// Emilie Gillet's open source Beads firmware.
//
// The DSP in beads/ is Emilie Gillet's Beads code (MIT license). This file
// replaces the Beads hardware layer (beads.cc, ui.cc, cv_reader.cc and
// drivers/) with libDaisy.
//
// Panel:
//   CV_1..CV_4 knobs   Density, Time, Size, Pitch
//   button + knobs     Shape, Feedback, Dry/Wet, Reverb (hidden layer)
//   tap, hold + knobs  Attenuverters of Shape, Time, Size, Pitch
//   button             Seed. Hold to lock, tap to unlock.
//   toggle             Freeze
//   button + toggle    Each flip selects the next quality mode
//   CV_5..CV_8         Time, Size, Pitch (V/Oct), Density (or Shape)
//   button at power-up flip the toggle while holding it: settings pages,
//                      release it: V/Oct calibration (1V, press, 3V, press)
//   gate in 1 / 2      Seed / Freeze
//   gate out           Grain gate (Beads' gate output mode)
//   CV_OUT_2 (LED)     Freeze, seed activity

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "daisy_patch_sm.h"
#include "util/CpuLoadMeter.h"
#include "util/PersistentStorage.h"

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/hysteresis_filter.h"
#include "stmlib/utils/gate_flags.h"

#include "beads/dsp/processor.h"
#include "beads/gain_control.h"
#include "beads/pot_controller.h"

#include "buffer_backup.h"

using namespace daisy;
using namespace daisy::patch_sm;

using beads::FloatFrame;
using beads::GainControl;
using beads::PotController;
using beads::Processor;
using stmlib::GateFlags;

// Beads uses a 4kHz control rate: the block size is sample rate / 4000.
#ifndef POLLEN_BLOCK_RATE
#define POLLEN_BLOCK_RATE 4000
#endif  // POLLEN_BLOCK_RATE
const int kBlockRate = POLLEN_BLOCK_RATE;
const size_t kMaxBlockSize = 48000 / kBlockRate;
const int kUiRateDivider = kBlockRate / 1000;
const int kTransitionBlocks = kBlockRate / 4;
const size_t kSeedDelayBlocks = kBlockRate / 400;  // 2.5ms, like Beads
const uint32_t kSeedLockTime = 1500;  // ms, Beads uses 4000
// A press within this time after a tap opens the attenuverter layer.
const uint32_t kDoubleTapTime = 300;  // ms
const uint32_t kSeedPatchedTimeout = 10000;  // ms
const uint32_t kBlinkPeriod = 300;  // ms

// Mono detection from input levels, patch.init has no jack detection.
// Switching clears the buffer, so a single click must not count (50ms level
// follower), a stereo source must be silent on one side for a while before
// going mono, and mono only goes back to stereo when both inputs carry
// clearly different signals.
const float kInputThreshold = 0.003f;  // about -50 dBFS
const float kInputFollowerCoefficient = 1.0f / (0.05f * kBlockRate);
const uint32_t kInputSilenceTimeout = 10000;  // ms
// patch.init feeds the left input to the right one when the right jack is
// empty (not the other way), so identical channels also mean mono. They go through separate ADC
// channels, so "identical" allows a small difference (measured 1-2%, up to
// 4% while the ADC settles after a codec restart). Leaving takes a much
// bigger difference, both need to last a second.
const float kIdenticalRatio = 0.05f;  // -26 dB
const float kDifferentRatio = 0.2f;  // -14 dB
const uint32_t kIdenticalTime = 1000;  // ms

// QSPI layout under the Daisy bootloader: 0x90000000-0x9003ffff is reserved
// by the bootloader, the app starts at 0x90040000 (480K max for BOOT_SRAM)
// and the last 256K is outside the bootloader's DFU map. The settings get a
// sector of their own far from all of these.
const uint32_t kSettingsQspiOffset = 0x400000;
const uint32_t kSettingsVersion = 4;

const char* const kMediaName[] = {
  "dirty tape", "clean tape", "vintage digital", "hi-fi"
};

const char* const kInputModeName[] = { "detect", "mono", "stereo" };

// Indexed by Setup.
const char* const kSetupText[] = {
  "",
  " | SETUP: flip the toggle for settings, release for calibration",
  " | CALIBRATING: 1V, press",
  " | CALIBRATING: 3V, press",
  " | SETTINGS inputs: knob 1 mode, knob 2 gain, press to save",
  " | SETTINGS CV modes: knob 1-4 Shape (CV_8), Time, Size, Pitch, "
      "press to save"
};

// Banks 0-1 live in AXI SRAM like on Beads when running from internal flash.
// With the bootloader the code occupies part of AXI SRAM, leaving room for
// bank 0 only, so bank 1 moves to SDRAM (slower, costs CPU with many grains).
#ifdef BOOT_APP
#define DELAY_BANK_1 DSY_SDRAM_BSS
#else
#define DELAY_BANK_1 __attribute__((section(".ramd1")))
#endif

const size_t kDelayBankSize = 256 * 1024;
uint8_t delay_bank_0[kDelayBankSize] __attribute__((section(".ramd1")));
uint8_t delay_bank_1[kDelayBankSize] DELAY_BANK_1;
uint8_t delay_bank_2[kDelayBankSize] __attribute__((section(".ramd2")));
uint16_t reverb_buffer[32768] __attribute__((section(".ramd3")));

enum Knob {
  KNOB_DENSITY,
  KNOB_TIME,
  KNOB_SIZE,
  KNOB_PITCH,
  KNOB_LAST
};

// Edited by holding the button and turning knob 1-4.
enum HiddenParameter {
  HIDDEN_SHAPE,
  HIDDEN_FEEDBACK,
  HIDDEN_DRY_WET,
  HIDDEN_REVERB,
  HIDDEN_LAST
};

// Edited by tapping the button, then holding it and turning knob 1-4. What
// they do depends on the parameter's CV mode, see CvMode.
enum Attenuverter {
  ATTENUVERTER_SHAPE,
  ATTENUVERTER_TIME,
  ATTENUVERTER_SIZE,
  ATTENUVERTER_PITCH,
  ATTENUVERTER_LAST
};

enum Layer {
  LAYER_HIDDEN,
  LAYER_ATTENUVERTERS
};

const int kKnobAdc[KNOB_LAST] = { CV_1, CV_2, CV_3, CV_4 };

enum CvInput {
  CV_INPUT_TIME,
  CV_INPUT_SIZE,
  CV_INPUT_PITCH,
  CV_INPUT_DENSITY,
  CV_INPUT_LAST
};

// CV_INPUT_DENSITY goes to Shape instead when Shape has a CV mode.
const int kCvAdc[CV_INPUT_LAST] = { CV_5, CV_6, CV_7, CV_8 };

// Settings page with the toggle in its Freeze position.
enum InputMode {
  INPUT_MODE_DETECT,  // mono detection from the input levels
  INPUT_MODE_MONO,
  INPUT_MODE_STEREO
};

// Below this the gain knob selects the automatic gain.
const float kGainAutoThreshold = 0.05f;

// Settings page with the toggle in the other position, one per attenuverter.
// patch.init can't tell whether a CV input is patched, Beads can.
enum CvMode {
  // CV ignored, the attenuverter sets the random modulation (Beads with
  // nothing patched). For Shape: CV_8 goes to Density.
  CV_MODE_IGNORE,
  // CV added to the knob at full scale, the attenuverter sets the random
  // modulation.
  CV_MODE_ADD,
  // Beads with a CV patched: clockwise the attenuverter scales the CV,
  // counter-clockwise the CV controls the amount of random modulation.
  CV_MODE_ATTENUVERTER
};

// Knob zones for the three-way settings, and the LED brightness for each.
const float kZoneLed[3] = { 0.0f, 2.0f, 5.0f };

// V/Oct calibration, like Beads: 1V then 3V into the pitch input, with the
// other CV inputs unpatched to measure their 0V offsets. GetAdcValue() gives
// -1..+1 for -5V..+5V, so nominally 60 semitones per unit. Like Beads, a
// scale more than 10% off is rejected.
const float kNominalPitchScale = 60.0f;
const float kMaxPitchScaleError = 0.1f;
const float kMaxCvOffset = 0.1f;  // 0.5V
const float kCalibrationFilterCoefficient = 1.0f / (0.125f * kBlockRate);
const uint32_t kCalibrationResultTime = 1000;  // ms

// Saved in QSPI. Bump kSettingsVersion when the layout changes so that old
// data is replaced by the defaults.
struct Settings {
  uint32_t version;
  uint8_t media;
  float hidden[HIDDEN_LAST];
  // Since version 2. New fields go at the end so older data can be migrated.
  float attenuverter[ATTENUVERTER_LAST];
  // Since version 3. Readings at 0V, and semitones per unit for pitch.
  float cv_offset[CV_INPUT_LAST];
  float pitch_scale;
  // Since version 4, the settings pages.
  uint8_t input_mode;
  uint8_t gain;  // 0: automatic, 1-255 like Beads' manual gain
  uint8_t cv_mode[ATTENUVERTER_LAST];

  bool operator!=(const Settings& other) const {
    if (version != other.version || media != other.media) {
      return true;
    }
    for (int i = 0; i < HIDDEN_LAST; ++i) {
      if (hidden[i] != other.hidden[i]) {
        return true;
      }
    }
    for (int i = 0; i < ATTENUVERTER_LAST; ++i) {
      if (attenuverter[i] != other.attenuverter[i]) {
        return true;
      }
    }
    for (int i = 0; i < CV_INPUT_LAST; ++i) {
      if (cv_offset[i] != other.cv_offset[i]) {
        return true;
      }
    }
    for (int i = 0; i < ATTENUVERTER_LAST; ++i) {
      if (cv_mode[i] != other.cv_mode[i]) {
        return true;
      }
    }
    return pitch_scale != other.pitch_scale || \
        input_mode != other.input_mode || gain != other.gain;
  }
};

const Settings kDefaultSettings = {
  kSettingsVersion,
  Processor::MEDIA_DIGITAL,
  { 0.5f, 0.0f, 0.5f, 0.3f },
  { 0.0f, 0.0f, 0.0f, 0.0f },
  { 0.0f, 0.0f, 0.0f, 0.0f },
  kNominalPitchScale,
  INPUT_MODE_DETECT,
  0,
  // CV_8 on Density, the other CVs added to their knobs.
  { CV_MODE_IGNORE, CV_MODE_ADD, CV_MODE_ADD, CV_MODE_ADD }
};

DaisyPatchSM hw;
Switch button;
Switch toggle;
CpuLoadMeter cpu_load;

PersistentStorage<Settings> storage(hw.qspi);

Processor processor;
GainControl gain_control;
BufferBackup buffer_backup;

PotController pot[KNOB_LAST];
float pot_lp[KNOB_LAST];
float hidden[HIDDEN_LAST];
float attenuverter[ATTENUVERTER_LAST];
stmlib::HysteresisFilter time_hysteresis;
float cv_time_lp;
float cv_size_lp;
float cv_shape_lp;
volatile InputMode input_mode;
uint8_t gain_setting;
CvMode cv_mode[ATTENUVERTER_LAST];

bool seed_pressed;
// Like Beads: grains only play while the seed gate is high, and the module
// starts locked so the density knob works out of the box.
bool seed_locked = true;
bool unlock_on_release;
bool shift_used;
Layer layer;
uint32_t press_time;
// When the last tap ended, 0 if the last press wasn't a tap.
uint32_t tap_release_time;
// A tap's seed unlock waits to see whether it starts a tap-then-hold.
bool pending_unlock;
bool seed_history[kSeedDelayBlocks];
size_t seed_history_ptr;
GateFlags previous_seed;
uint32_t last_seed_time;

// Quality mode ("media" in the Beads code). Changing it restarts the codec,
// so the request is applied by main().
volatile Processor::Media requested_media = Processor::MEDIA_DIGITAL;
Processor::Media current_media;
// Follows the toggle's debounced edges, starting from its raw position at
// power-up, so that a restored frozen buffer isn't unfrozen while the
// debouncer settles.
bool freeze_toggle;
// Shared with main() for the buffer backup.
volatile bool freeze_active;
volatile bool backup_busy;

// Like Beads' normalization flags: the silent input gets a copy of the
// other one. Either flag set means mono.
float input_level[2];
float difference_level;
// Starts true: a stereo source at power-up stays stereo.
bool different = true;
uint32_t different_change_since;
bool input_seen[2];
uint32_t input_last_active[2];
bool left_normalized;
bool right_normalized;
volatile bool requested_mono;
// Peak input levels for the status line, reset every second by main().
volatile float input_peak[2];
volatile float difference_peak;
uint32_t blink_start;
int blink_count;

// Set by the audio callback, the QSPI write happens in main().
volatile bool save_settings;

int transition_blocks;
int ui_divider;

enum Setup {
  SETUP_OFF,
  // Button held at power-up: flipping the toggle opens the settings pages,
  // releasing the button starts the V/Oct calibration.
  SETUP_CHOOSING,
  SETUP_CALIBRATION_1V,
  SETUP_CALIBRATION_3V,
  SETUP_PAGE_INPUTS,  // toggle in the Freeze position
  SETUP_PAGE_CV
};

// The debounced switches need a few ms after power-up to settle, and report
// an edge when they get there, so the power-up choice compares positions.
const uint32_t kSetupSettleTime = 50;  // ms

volatile Setup setup;
uint32_t setup_start_time;
bool setup_toggle_start;
// Knob last turned in a settings page, -1 before the first.
int setup_last_knob;
// The press that ends the setup must not reach ProcessButton().
bool button_owned_by_setup;
uint32_t setup_result_time;
bool setup_result_ok;
// Calibration steps only advance after the button has been released once,
// so the press that starts it doesn't count as the first step.
bool calibration_armed;
float cv_offset[CV_INPUT_LAST];
float pitch_scale = kNominalPitchScale;
// CV readings averaged over about 125ms, used by the calibration.
float cv_average[CV_INPUT_LAST];
float calibration_1v[CV_INPUT_LAST];

// Diagnostics, printed once per second over USB serial.
volatile float pitch_cv_monitor;
volatile uint32_t block_count;
volatile uint32_t gate_count;
bool previous_gate;
// libDaisy's Logger switches to blocking writes once a terminal has read
// from it, which stalls main() (and with it saving and mode changes) as soon
// as the port is closed again. The status line is sent directly instead and
// dropped when nobody reads it. The USB stack reads the buffer
// asynchronously, so a new line goes into the buffer that isn't in flight.
char status_line[2][448];
int status_in_flight = 1;

// Beads' LAW_DETUNE pitch knob: +/-24 semitones with sticky intervals.
float DetuneLaw(float v) {
  float detune = 2.0f * v - 1.0f;
  float sign = detune < 0.0f ? -1.0f : 1.0f;
  detune = detune * sign * 6.0f;
  MAKE_INTEGRAL_FRACTIONAL(detune);

  const float intervals[8] = {
    0.0f, 1.0f, 3.0f, 4.0f, 7.0f, 12.0f, 24.0f, 24.0f
  };

  detune_fractional = std::max((detune_fractional - 0.33333333f) * 1.5f, 0.0f);

  float a = intervals[detune_integral];
  float b = intervals[detune_integral + 1];
  return stmlib::Crossfade(a, b, stmlib::SmoothStep(detune_fractional)) * sign;
}

// The LED blinks 1-4 times: hi-fi, vintage digital, clean tape, dirty tape.
void RequestMedia(Processor::Media media) {
  requested_media = media;
  blink_count = 4 - int(media);
  blink_start = System::GetNow();
}

void DetectMono(AudioHandle::InputBuffer in, size_t size, uint32_t now) {
  float difference = 0.0f;
  for (size_t i = 0; i < size; ++i) {
    difference = std::max(difference, fabsf(in[0][i] - in[1][i]));
  }
  if (difference > difference_peak) {
    difference_peak = difference;
  }
  ONE_POLE(difference_level, difference, kInputFollowerCoefficient);

  bool active[2];
  bool recently_active[2];
  for (int channel = 0; channel < 2; ++channel) {
    float peak = 0.0f;
    for (size_t i = 0; i < size; ++i) {
      peak = std::max(peak, fabsf(in[channel][i]));
    }
    if (peak > input_peak[channel]) {
      input_peak[channel] = peak;
    }
    ONE_POLE(input_level[channel], peak, kInputFollowerCoefficient);
    active[channel] = input_level[channel] > kInputThreshold;
    if (active[channel]) {
      input_seen[channel] = true;
      input_last_active[channel] = now;
    }
    recently_active[channel] = input_seen[channel] && \
        now - input_last_active[channel] < kInputSilenceTimeout;
  }

  // Whether the two inputs carry different signals, with hysteresis. Only
  // decided while both have a signal.
  if (active[0] && active[1]) {
    float ratio = different ? kIdenticalRatio : kDifferentRatio;
    bool different_now = difference_level > \
        ratio * std::max(input_level[0], input_level[1]);
    if (different_now == different) {
      different_change_since = now;
    } else if (now - different_change_since >= kIdenticalTime) {
      different = different_now;
    }
  } else {
    different_change_since = now;
  }

  bool mono = requested_mono;
  if (input_mode == INPUT_MODE_MONO) {
    mono = true;
  } else if (input_mode == INPUT_MODE_STEREO) {
    mono = false;
  } else if (mono) {
    mono = !(active[0] && active[1] && different);
  } else {
    mono = (active[0] && active[1] && !different) || \
        (active[0] && !recently_active[1]) || \
        (active[1] && !recently_active[0]);
  }

  // Mono copies the side that has a signal, left if both have one. With no
  // signal at all the copy direction is kept.
  if (!mono) {
    left_normalized = right_normalized = false;
  } else if (active[0]) {
    left_normalized = false;
    right_normalized = true;
  } else if (active[1]) {
    left_normalized = true;
    right_normalized = false;
  } else if (!left_normalized && !right_normalized) {
    right_normalized = true;
  }
  requested_mono = mono;
}

// Beads' LAW_ATTENUVERTER: -1..+1 with a dead zone at noon and finer
// control around it.
float AttenuverterLaw(float v) {
  v -= 0.5f;
  return (2.0f / 0.47f) * v * std::max(fabsf(v) - 0.03f, 0.0f);
}

void StartCalibration() {
  calibration_armed = false;
  setup = SETUP_CALIBRATION_1V;
}

void FinishSetup(bool ok) {
  setup_result_ok = ok;
  setup_result_time = System::GetNow();
  setup = SETUP_OFF;
  button_owned_by_setup = button.Pressed();
  // The toggle isn't followed during the setup.
  freeze_toggle = toggle.Pressed();
}

void ProcessCalibration() {
  if (!calibration_armed) {
    calibration_armed = !button.RawState();
    return;
  }
  if (!button.RisingEdge()) {
    return;
  }

  if (setup == SETUP_CALIBRATION_1V) {
    std::copy(&cv_average[0], &cv_average[CV_INPUT_LAST], &calibration_1v[0]);
    setup = SETUP_CALIBRATION_3V;
    return;
  }

  float pitch_1v = calibration_1v[CV_INPUT_PITCH];
  float pitch_3v = cv_average[CV_INPUT_PITCH];
  float scale = pitch_3v != pitch_1v ? 24.0f / (pitch_3v - pitch_1v) : 0.0f;
  float offset[CV_INPUT_LAST];
  std::copy(&calibration_1v[0], &calibration_1v[CV_INPUT_LAST], &offset[0]);
  offset[CV_INPUT_PITCH] = scale != 0.0f ? pitch_1v - 12.0f / scale : 0.0f;

  bool ok = fabsf(scale - kNominalPitchScale) <= \
      kMaxPitchScaleError * kNominalPitchScale;
  // A large offset means something was left patched.
  for (int i = 0; i < CV_INPUT_LAST; ++i) {
    ok = ok && fabsf(offset[i]) <= kMaxCvOffset;
  }
  if (ok) {
    pitch_scale = scale;
    std::copy(&offset[0], &offset[CV_INPUT_LAST], &cv_offset[0]);
    save_settings = true;
  }
  FinishSetup(ok);
}

int Zone(float v) {
  return v < 0.333f ? 0 : (v < 0.667f ? 1 : 2);
}

// The knob's position counts only once it has been moved, see
// PotController, so opening a page doesn't overwrite its settings.
void OpenSettingsPage(Setup page) {
  setup = page;
  setup_last_knob = -1;
  for (int i = 0; i < KNOB_LAST; ++i) {
    pot[i].Unlock();
    pot[i].Lock();
  }
}

// Returns false for knobs without a setting on the page.
bool ApplySetting(int knob, float value) {
  if (setup == SETUP_PAGE_CV) {
    cv_mode[knob] = CvMode(Zone(value));
    return true;
  }
  if (knob == 0) {
    input_mode = InputMode(Zone(value));
    return true;
  }
  if (knob == 1) {
    uint8_t gain = value < kGainAutoThreshold
        ? 0
        : uint8_t(1.0f + 254.0f * (value - kGainAutoThreshold) / \
            (1.0f - kGainAutoThreshold));
    if (gain == 0 && gain_setting != 0) {
      // Like Beads: back to automatic gain relearns the input level.
      gain_control.Restart();
    }
    gain_setting = gain;
    return true;
  }
  return false;
}

// Called instead of ProcessButton() during the setup.
void ProcessSetup() {
  Setup toggle_page = toggle.Pressed() ? SETUP_PAGE_INPUTS : SETUP_PAGE_CV;
  switch (setup) {
    case SETUP_CHOOSING:
      if (System::GetNow() - setup_start_time < kSetupSettleTime) {
        break;
      }
      if (toggle.Pressed() != setup_toggle_start) {
        OpenSettingsPage(toggle_page);
      } else if (!button.Pressed()) {
        calibration_armed = true;
        setup = SETUP_CALIBRATION_1V;
      }
      break;

    case SETUP_CALIBRATION_1V:
    case SETUP_CALIBRATION_3V:
      ProcessCalibration();
      break;

    case SETUP_PAGE_INPUTS:
    case SETUP_PAGE_CV:
      if (toggle_page != setup) {
        OpenSettingsPage(toggle_page);
        break;
      }
      for (int i = 0; i < KNOB_LAST; ++i) {
        if (pot[i].editing_hidden_parameter() && \
            ApplySetting(i, pot[i].hidden_parameter())) {
          setup_last_knob = i;
        }
      }
      if (button.RisingEdge()) {
        for (int i = 0; i < KNOB_LAST; ++i) {
          pot[i].Unlock();
        }
        save_settings = true;
        FinishSetup(true);
      }
      break;

    default:
      break;
  }
}

// LED in the setup, or -1 outside of it.
float SetupLed(uint32_t now) {
  if (setup == SETUP_CHOOSING || setup == SETUP_CALIBRATION_1V || \
      setup == SETUP_CALIBRATION_3V) {
    // One or two blinks per second for the 1V and 3V steps.
    uint32_t blinks = setup == SETUP_CALIBRATION_3V ? 2 : 1;
    uint32_t t = now % 1000;
    return t < blinks * 250 && t % 250 < 125 ? 5.0f : 0.0f;
  }
  if (setup == SETUP_PAGE_INPUTS || setup == SETUP_PAGE_CV) {
    // The three-way settings show their zone. Otherwise the input page shows
    // the input level after the gain (Beads' VU meter) on a dim base, the CV
    // page blinks.
    if (setup == SETUP_PAGE_CV && setup_last_knob >= 0) {
      return kZoneLed[cv_mode[setup_last_knob]];
    } else if (setup == SETUP_PAGE_INPUTS && setup_last_knob == 0) {
      return kZoneLed[input_mode];
    } else if (setup == SETUP_PAGE_INPUTS) {
      return 0.5f + 4.5f * float(gain_control.vu_meter()) / 255.0f;
    }
    return now % 1000 < 500 ? 5.0f : 0.0f;
  }
  if (setup_result_time && now - setup_result_time < kCalibrationResultTime) {
    // Solid when saved, flickering when rejected.
    return setup_result_ok || (now & 32) ? 5.0f : 0.0f;
  }
  return -1.0f;
}

// Beads' attenuverter handling for a parameter whose base is set, see
// CvMode.
void ApplyCv(
    beads::RandomizedParameter* parameter,
    CvMode mode,
    float attenuverter,
    float cv) {
  parameter->attenuverter = attenuverter;
  parameter->has_cv = mode == CV_MODE_ATTENUVERTER;
  parameter->cv = parameter->has_cv ? cv : 0.0f;
  if (mode == CV_MODE_ADD) {
    parameter->base += cv;
  }
}

void ProcessButton() {
  uint32_t now = System::GetNow();

  if (button_owned_by_setup) {
    button_owned_by_setup = button.Pressed();
    return;
  }

  if (button.RisingEdge()) {
    for (int i = 0; i < KNOB_LAST; ++i) {
      pot[i].Lock();
    }
    shift_used = false;
    press_time = now;
    if (tap_release_time && now - tap_release_time < kDoubleTapTime) {
      // Second half of tap-then-hold: the tap was not a seed action.
      layer = LAYER_ATTENUVERTERS;
      pending_unlock = false;
      unlock_on_release = false;
      seed_pressed = false;
    } else {
      layer = LAYER_HIDDEN;
      // A tap unlocks a locked seed. Using the button as shift doesn't.
      unlock_on_release = seed_locked;
      seed_pressed = !seed_locked;
    }
  }

  if (button.Pressed()) {
    // Flipping the toggle with the button held steps through the quality
    // modes in Beads' order. Freeze ignores the toggle until the release.
    if (toggle.RisingEdge() || toggle.FallingEdge()) {
      RequestMedia(Processor::Media((requested_media + 3) % 4));
      shift_used = true;
    }
    for (int i = 0; i < KNOB_LAST; ++i) {
      if (pot[i].editing_hidden_parameter()) {
        if (layer == LAYER_HIDDEN) {
          hidden[i] = pot[i].hidden_parameter();
        } else {
          attenuverter[i] = AttenuverterLaw(pot[i].hidden_parameter());
        }
        shift_used = true;
      }
    }
    if (shift_used) {
      seed_pressed = false;
      unlock_on_release = false;
    } else if (layer == LAYER_HIDDEN && !seed_locked && \
        button.TimeHeldMs() >= kSeedLockTime) {
      seed_locked = true;
      seed_pressed = false;
    }
  } else if (toggle.RisingEdge()) {
    freeze_toggle = true;
  } else if (toggle.FallingEdge()) {
    freeze_toggle = false;
  }

  if (button.FallingEdge()) {
    // Flips made with the button held don't count, the position does.
    freeze_toggle = toggle.Pressed();
    for (int i = 0; i < KNOB_LAST; ++i) {
      pot[i].Unlock();
    }
    if (shift_used) {
      save_settings = true;
    }
    bool tap = layer == LAYER_HIDDEN && !shift_used && \
        now - press_time < kDoubleTapTime;
    tap_release_time = tap ? now : 0;
    if (unlock_on_release) {
      pending_unlock = true;
    }
    seed_pressed = false;
  }

  if (pending_unlock && !button.Pressed() && \
      (!tap_release_time || now - tap_release_time >= kDoubleTapTime)) {
    seed_locked = false;
    pending_unlock = false;
  }
}

void AudioCallback(
    AudioHandle::InputBuffer in,
    AudioHandle::OutputBuffer out,
    size_t size) {
  cpu_load.OnBlockStart();
  ++block_count;
  hw.ProcessAllControls();
  button.Debounce();
  toggle.Debounce();
  if (setup != SETUP_OFF) {
    ProcessSetup();
  } else {
    ProcessButton();
  }

  for (int i = 0; i < KNOB_LAST; ++i) {
    float v = hw.GetAdcValue(kKnobAdc[i]);
    CONSTRAIN(v, 0.0f, 1.0f);
    ONE_POLE(pot_lp[i], v, 0.05f);
    pot[i].ProcessControlRate(pot_lp[i]);
  }
  if (++ui_divider >= kUiRateDivider) {
    ui_divider = 0;
    for (int i = 0; i < KNOB_LAST; ++i) {
      pot[i].ProcessUIRate();
    }
  }

  // Scale to the units Beads' CvReader produces with factory calibration.
  // GetAdcValue() returns -1..+1 for -5V..+5V.
  float cv[CV_INPUT_LAST];
  for (int i = 0; i < CV_INPUT_LAST; ++i) {
    cv[i] = hw.GetAdcValue(kCvAdc[i]);
    ONE_POLE(cv_average[i], cv[i], kCalibrationFilterCoefficient);
    cv[i] -= cv_offset[i];
  }
  ONE_POLE(cv_time_lp, cv[CV_INPUT_TIME] * 1.6f, 0.5f);
  ONE_POLE(cv_size_lp, cv[CV_INPUT_SIZE] * 1.6f, 0.5f);
  ONE_POLE(cv_shape_lp, cv[CV_INPUT_DENSITY] * 1.6f, 0.5f);
  float cv_pitch = cv[CV_INPUT_PITCH] * pitch_scale;
  float cv_density = cv[CV_INPUT_DENSITY];
  pitch_cv_monitor = cv_pitch;

  // Gates are read once per block (250us) instead of per sample.
  bool freeze_state = freeze_toggle || hw.gate_in_2.State();
  freeze_active = freeze_state;
  bool seed_gate = hw.gate_in_1.State();
  uint32_t now = System::GetNow();
  if (seed_gate) {
    last_seed_time = now;
  }
  // No jack detection: treat the seed input as patched if it saw a gate
  // recently. A null input matters for the locked (clock divider) mode.
  bool seed_patched = last_seed_time && \
      (now - last_seed_time) < kSeedPatchedTimeout;

  // Delay the seed input by 2.5ms so that CVs patched along with it settle.
  bool delayed_seed = seed_history[seed_history_ptr];
  seed_history[seed_history_ptr] = seed_gate;
  seed_history_ptr = (seed_history_ptr + 1) % kSeedDelayBlocks;

  bool freeze[kMaxBlockSize];
  GateFlags seed[kMaxBlockSize];
  for (size_t i = 0; i < size; ++i) {
    freeze[i] = freeze_state;
    previous_seed = seed[i] = stmlib::ExtractGateFlags(
        previous_seed,
        delayed_seed);
  }
  if (!seed_patched) {
    previous_seed = stmlib::GATE_FLAG_LOW;
  }

  // Freeze dims while the buffer is being saved, like on Beads.
  float led = freeze_state ? (backup_busy ? 2.5f : 5.0f) : 0.0f;
  if (!freeze_state && (seed_gate || seed_pressed)) {
    led = 2.5f;
  } else if (!freeze_state && seed_locked) {
    led = 1.0f;
  }
  if (button.Pressed() && layer == LAYER_ATTENUVERTERS) {
    led = (now & 128) ? 5.0f : 0.0f;
  }
  float setup_led = SetupLed(now);
  if (setup_led >= 0.0f) {
    led = setup_led;
  }
  uint32_t blink_time = now - blink_start;
  if (blink_time < uint32_t(blink_count) * kBlinkPeriod) {
    led = (blink_time % kBlinkPeriod) < kBlinkPeriod / 2 ? 5.0f : 0.0f;
  }
  hw.WriteCvOut(CV_OUT_2, led);

  FloatFrame float_in[kMaxBlockSize];
  FloatFrame float_out[kMaxBlockSize];
  for (size_t i = 0; i < size; ++i) {
    float_in[i].l = in[0][i];
    float_in[i].r = in[1][i];
  }
  // A codec restart puts a burst on the inputs, which would look like a
  // stereo signal and flip back a mono switch that was just made.
  if (!transition_blocks) {
    DetectMono(in, size, now);
  }
  gain_control.Process(
      !transition_blocks,
      gain_setting,
      left_normalized,
      right_normalized,
      float_in,
      float_in,
      size);

  if (transition_blocks) {
    --transition_blocks;
    hw.gate_out_1.Write(false);
    previous_gate = false;
    for (size_t i = 0; i < size; ++i) {
      out[0][i] = out[1][i] = 0.0f;
    }
    cpu_load.OnBlockEnd();
    return;
  }

  beads::Parameters p;
  p.time.base = time_hysteresis.Process(pot[KNOB_TIME].main_parameter());
  ApplyCv(
      &p.time,
      cv_mode[ATTENUVERTER_TIME],
      attenuverter[ATTENUVERTER_TIME],
      cv_time_lp);

  p.size.base = pot[KNOB_SIZE].main_parameter();
  ApplyCv(
      &p.size,
      cv_mode[ATTENUVERTER_SIZE],
      attenuverter[ATTENUVERTER_SIZE],
      cv_size_lp);

  p.shape.base = hidden[HIDDEN_SHAPE];
  ApplyCv(
      &p.shape,
      cv_mode[ATTENUVERTER_SHAPE],
      attenuverter[ATTENUVERTER_SHAPE],
      cv_shape_lp);

  p.pitch.base = DetuneLaw(pot[KNOB_PITCH].main_parameter());
  ApplyCv(
      &p.pitch,
      cv_mode[ATTENUVERTER_PITCH],
      attenuverter[ATTENUVERTER_PITCH],
      cv_pitch);

  p.monophonic_mode = false;

  beads::ModulatedParameter d;
  d.base = pot[KNOB_DENSITY].main_parameter();
  d.cv = cv_mode[ATTENUVERTER_SHAPE] == CV_MODE_IGNORE ? cv_density : 0.0f;

  Processor::MixParameters m;
  m.feedback = std::min(hidden[HIDDEN_FEEDBACK] * 1.005f, 1.0f);
  m.dry_wet = std::min(hidden[HIDDEN_DRY_WET] * 1.005f, 1.0f);
  m.reverb_amount = std::min(hidden[HIDDEN_REVERB] * 1.005f, 1.0f);
  m.output_gate = false;

  beads::SeedButtonState seed_button_state = seed_locked
      ? beads::SEED_BUTTON_STATE_LOCKED
      : (seed_pressed
          ? beads::SEED_BUTTON_STATE_ON
          : beads::SEED_BUTTON_STATE_OFF);

  processor.Process(
      seed_button_state,
      seed_patched ? seed : NULL,
      freeze,
      d, p, m,
      float_in,
      float_out,
      size);

  // Updated once per block. Grains come at 130Hz at most, so a period is
  // around 30 blocks long.
  bool gate = processor.gate();
  hw.gate_out_1.Write(gate);
  if (gate && !previous_gate) {
    ++gate_count;
  }
  previous_gate = gate;

  for (size_t i = 0; i < size; ++i) {
    float l = float_out[i].l;
    float r = float_out[i].r;
    CONSTRAIN(l, -1.0f, 1.0f);
    CONSTRAIN(r, -1.0f, 1.0f);
    out[0][i] = l;
    out[1][i] = r;
  }

  cpu_load.OnBlockEnd();
}

int Decibels(float level) {
  return level > 1e-6f ? int(20.0f * log10f(level)) : -120;
}

// Serial commands, handled in the USB interrupt so they still work when
// audio starves main():
//   b      jump to the Daisy bootloader, for `make program-dfu`
//   1-4    quality mode, numbered like the LED blinks
//   c      start the V/Oct calibration
void UsbReceive(uint8_t* buffer, uint32_t* length) {
  for (uint32_t i = 0; i < *length; ++i) {
    if (buffer[i] == 'b') {
      System::ResetToBootloader(System::DAISY_INFINITE_TIMEOUT);
    } else if (buffer[i] == 'c') {
      StartCalibration();
    } else if (buffer[i] >= '1' && buffer[i] <= '4') {
      RequestMedia(Processor::Media(
          Processor::MEDIA_DIGITAL - (buffer[i] - '1')));
    }
  }
}

void LoadSettings() {
  storage.Init(kDefaultSettings, kSettingsQspiOffset);
  Settings& stored = storage.GetSettings();
  if (stored.version >= 1 && stored.version < kSettingsVersion) {
    // Fill in the fields older versions didn't have, keep the rest.
    if (stored.version < 2) {
      std::copy(
          &kDefaultSettings.attenuverter[0],
          &kDefaultSettings.attenuverter[ATTENUVERTER_LAST],
          &stored.attenuverter[0]);
    }
    if (stored.version < 3) {
      std::copy(
          &kDefaultSettings.cv_offset[0],
          &kDefaultSettings.cv_offset[CV_INPUT_LAST],
          &stored.cv_offset[0]);
      stored.pitch_scale = kDefaultSettings.pitch_scale;
    }
    if (stored.version < 4) {
      stored.input_mode = kDefaultSettings.input_mode;
      stored.gain = kDefaultSettings.gain;
      std::copy(
          &kDefaultSettings.cv_mode[0],
          &kDefaultSettings.cv_mode[ATTENUVERTER_LAST],
          &stored.cv_mode[0]);
    }
    stored.version = kSettingsVersion;
    storage.Save();
  } else if (stored.version != kSettingsVersion) {
    storage.RestoreDefaults();
  }
  const Settings& s = storage.GetSettings();
  requested_media = Processor::Media(s.media <= Processor::MEDIA_DIGITAL
      ? s.media
      : Processor::MEDIA_DIGITAL);
  for (int i = 0; i < HIDDEN_LAST; ++i) {
    float v = s.hidden[i];
    // Also catches NaN from a corrupted sector.
    hidden[i] = v >= 0.0f && v <= 1.0f ? v : kDefaultSettings.hidden[i];
  }
  for (int i = 0; i < ATTENUVERTER_LAST; ++i) {
    float v = s.attenuverter[i];
    attenuverter[i] = v >= -1.0f && v <= 1.0f \
        ? v
        : kDefaultSettings.attenuverter[i];
  }
  for (int i = 0; i < CV_INPUT_LAST; ++i) {
    float v = s.cv_offset[i];
    cv_offset[i] = fabsf(v) <= kMaxCvOffset ? v : kDefaultSettings.cv_offset[i];
  }
  pitch_scale = fabsf(s.pitch_scale - kNominalPitchScale) <= \
      kMaxPitchScaleError * kNominalPitchScale
      ? s.pitch_scale
      : kDefaultSettings.pitch_scale;
  input_mode = InputMode(s.input_mode <= INPUT_MODE_STEREO
      ? s.input_mode
      : kDefaultSettings.input_mode);
  gain_setting = s.gain;
  for (int i = 0; i < ATTENUVERTER_LAST; ++i) {
    cv_mode[i] = CvMode(s.cv_mode[i] <= CV_MODE_ATTENUVERTER
        ? s.cv_mode[i]
        : kDefaultSettings.cv_mode[i]);
  }
}

// Erases and writes a 4K QSPI sector, but only if something changed.
void SaveSettings() {
  Settings& s = storage.GetSettings();
  s.version = kSettingsVersion;
  s.media = current_media;
  for (int i = 0; i < HIDDEN_LAST; ++i) {
    s.hidden[i] = hidden[i];
  }
  for (int i = 0; i < ATTENUVERTER_LAST; ++i) {
    s.attenuverter[i] = attenuverter[i];
  }
  for (int i = 0; i < CV_INPUT_LAST; ++i) {
    s.cv_offset[i] = cv_offset[i];
  }
  s.pitch_scale = pitch_scale;
  s.input_mode = input_mode;
  s.gain = gain_setting;
  for (int i = 0; i < ATTENUVERTER_LAST; ++i) {
    s.cv_mode[i] = cv_mode[i];
  }
  storage.Save();
}

// Starts the codec for the processor's current configuration. Configure()
// isn't called here: it would wipe a buffer restored from QSPI.
void StartAudio(bool reset_gain) {
  Processor::Media media = processor.media();
  current_media = media;
  int sr = int(processor.sample_rate());
  size_t block_size = sr / kBlockRate;

  hw.SetAudioSampleRate(sr == 32000
      ? SaiHandle::Config::SampleRate::SAI_32KHZ
      : SaiHandle::Config::SampleRate::SAI_48KHZ);
  hw.SetAudioBlockSize(block_size);

  gain_control.Init(
      float(sr),
      media == Processor::MEDIA_DIRTY_TAPE
          ? GainControl::SOFT_CLIPPING_MODE_ON
          : (media == Processor::MEDIA_CLEAN_TAPE
                ? GainControl::SOFT_CLIPPING_MODE_DYNAMIC
                : GainControl::SOFT_CLIPPING_MODE_OFF),
      reset_gain);
  cpu_load.Init(float(sr), block_size);

  transition_blocks = kTransitionBlocks;
  hw.StartAudio(AudioCallback);
}

int main(void) {
  hw.Init();
  button.Init(DaisyPatchSM::B7);
  toggle.Init(DaisyPatchSM::B8);
  bool setup_at_power_up = button.RawState();

  for (int i = 0; i < KNOB_LAST; ++i) {
    pot[i].Init(1.0f, 0.0f);
  }
  time_hysteresis.Init(0.001f);

  // Like Beads' System::Init. libDaisy only does this with DATA_IN_D2_SRAM.
  __HAL_RCC_D2SRAM1_CLK_ENABLE();
  __HAL_RCC_D2SRAM2_CLK_ENABLE();
  __HAL_RCC_D2SRAM3_CLK_ENABLE();

  memset(delay_bank_0, 0, kDelayBankSize);
  memset(delay_bank_1, 0, kDelayBankSize);
  memset(delay_bank_2, 0, kDelayBankSize);
  memset(reverb_buffer, 0, sizeof(reverb_buffer));

#ifdef POLLEN_ALIAS_BANKS
  // Profiling aid: all banks share D2 SRAM, to measure what SDRAM costs.
  // The audio is garbage in this mode.
  uint8_t* delay_banks[3] = { delay_bank_2, delay_bank_2, delay_bank_2 };
#else
  uint8_t* delay_banks[3] = { delay_bank_0, delay_bank_1, delay_bank_2 };
#endif  // POLLEN_ALIAS_BANKS
  processor.Init(delay_banks, reverb_buffer);

  LoadSettings();
  processor.Configure(requested_media, false);

  buffer_backup.Init(&hw.qspi, &processor);
  if (setup_at_power_up) {
    // The setup leaves a stored frozen buffer alone: it's neither restored
    // nor dropped, and freeze stays off until the setup is done.
    freeze_toggle = freeze_active = false;
    setup_toggle_start = toggle.RawState();
    setup_start_time = System::GetNow();
    setup = SETUP_CHOOSING;
  } else {
    freeze_toggle = freeze_active = toggle.RawState();
    if (buffer_backup.Load(freeze_toggle)) {
      // The backup knows the quality mode the buffer was recorded in.
      requested_media = processor.media();
    }
  }

  hw.usb.Init(UsbHandle::FS_INTERNAL);
  hw.usb.SetReceiveCallback(UsbReceive, UsbHandle::FS_INTERNAL);
  StartAudio(true);

  uint32_t last_report = System::GetNow();
  while (1) {
    // Mono changes wait for unfreeze: they would wipe the frozen buffer.
    bool mono = freeze_active ? processor.mono() : requested_mono;
    if (requested_media != current_media || mono != processor.mono()) {
      bool media_changed = requested_media != current_media;
      bool mono_changed = mono != processor.mono();
      hw.StopAudio();
      processor.Configure(requested_media, mono);
      // Like Beads, the input gain is relearned for a new input setup.
      StartAudio(mono_changed);
      if (media_changed) {
        save_settings = true;
      }
    }
    if (save_settings) {
      save_settings = false;
      SaveSettings();
    }
    if (setup == SETUP_OFF) {
      buffer_backup.Process(freeze_active);
    }
    backup_busy = buffer_backup.busy();
    uint32_t now = System::GetNow();
    if (now - last_report >= 1000) {
      last_report = now;
      uint32_t blocks = block_count;
      block_count = 0;
      uint32_t gates = gate_count;
      gate_count = 0;
      int avg = int(cpu_load.GetAvgCpuLoad() * 1000.0f);
      int max = int(cpu_load.GetMaxCpuLoad() * 1000.0f);
      cpu_load.Reset();
      char* line = status_line[1 - status_in_flight];
      int length = snprintf(
          line,
          sizeof(status_line[0]),
          "%s %s | blocks %d cpu avg %d.%d%% max %d.%d%% | gates %d | "
          "in %d %d diff %d dB | "
          "knobs %d %d %d %d | hidden %d %d %d %d | atv %d %d %d %d | "
          "backup %d%% | pitch cv %d ct, cal %d %d | "
          "inmode %s gain %d cvmode %d %d %d %d%s%s\r\n",
          kMediaName[current_media],
          processor.mono() ? "mono" : "stereo",
          int(blocks),
          avg / 10, avg % 10,
          max / 10, max % 10,
          int(gates),
          Decibels(input_peak[0]),
          Decibels(input_peak[1]),
          Decibels(difference_peak),
          int(pot_lp[KNOB_DENSITY] * 100.0f),
          int(pot_lp[KNOB_TIME] * 100.0f),
          int(pot_lp[KNOB_SIZE] * 100.0f),
          int(pot_lp[KNOB_PITCH] * 100.0f),
          int(hidden[HIDDEN_SHAPE] * 100.0f),
          int(hidden[HIDDEN_FEEDBACK] * 100.0f),
          int(hidden[HIDDEN_DRY_WET] * 100.0f),
          int(hidden[HIDDEN_REVERB] * 100.0f),
          int(attenuverter[ATTENUVERTER_SHAPE] * 100.0f),
          int(attenuverter[ATTENUVERTER_TIME] * 100.0f),
          int(attenuverter[ATTENUVERTER_SIZE] * 100.0f),
          int(attenuverter[ATTENUVERTER_PITCH] * 100.0f),
          buffer_backup.progress(),
          int(pitch_cv_monitor * 100.0f),
          int(pitch_scale * 100.0f),
          int(cv_offset[CV_INPUT_PITCH] * 5000.0f),
          kInputModeName[input_mode],
          int(gain_setting),
          int(cv_mode[ATTENUVERTER_SHAPE]),
          int(cv_mode[ATTENUVERTER_TIME]),
          int(cv_mode[ATTENUVERTER_SIZE]),
          int(cv_mode[ATTENUVERTER_PITCH]),
          kSetupText[setup],
          pot[KNOB_SIZE].main_parameter() > 0.99f ? " (delay mode)" : "");
      input_peak[0] = input_peak[1] = difference_peak = 0.0f;
      length = std::min(length, int(sizeof(status_line[0])) - 1);
      if (hw.usb.TransmitInternal(reinterpret_cast<uint8_t*>(line), length)
          == UsbHandle::Result::OK) {
        status_in_flight = 1 - status_in_flight;
      }
    }
  }
}
