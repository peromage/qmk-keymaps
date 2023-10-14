#pragma once

#ifdef AUDIO_ENABLE
#define STARTUP_SONG SONG(PLANCK_SOUND)
#endif

#define MIDI_BASIC

#define ENCODER_RESOLUTION 4

/*
  Set any config.h overrides for your specific keymap here.
  See config.h options at https://docs.qmk.fm/#/config_options?id=the-configh-file
*/
#define ORYX_CONFIGURATOR
#define IGNORE_MOD_TAP_INTERRUPT
#define ONESHOT_TAP_TOGGLE 2

#undef ONESHOT_TIMEOUT
#define ONESHOT_TIMEOUT 1000

#undef RGB_DISABLE_TIMEOUT
#define RGB_DISABLE_TIMEOUT 300000

#define USB_SUSPEND_WAKEUP_DELAY 0
#define FIRMWARE_VERSION u8"BPqo0/rROx6"
#define RAW_USAGE_PAGE 0xFF60
#define RAW_USAGE_ID 0x61
#define LAYER_STATE_8BIT

#define RGB_MATRIX_STARTUP_SPD 60

