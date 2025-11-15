#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h>

#define SCL_PIN 6 // Arduion pin A1
#define SCL_PORT PORTF
#define SDA_PIN 7 // Arduion pin A0
#define SDA_PORT PORTF
#define I2C_FASTMODE 1
#define I2C_HARDWARE 0
#define I2C_PULLUP 1

#define RGB_LED_PORT PORTD
#define RGB_LED_BIT  7 // Arduion pin D6

#include <SoftI2CMaster.h>
#include <Waveshare_LCD1602.h>

static const uint8_t ADC_PIN_12V = A2;
static const uint8_t ADC_PIN_5V = A3;
static const uint8_t BUZZER_PIN = 8;
static const uint8_t FAN_PWM_PIN1 = 9;
static const uint8_t FAN_PWM_PIN2 = 10;
static const uint8_t FAN_INT_PIN1 = 7;
static const uint8_t FAN_INT_PIN2 = 2;
static const uint8_t ONEWIRE_PIN = 15;
static const uint8_t HDD_POWER_BUTTON_PIN = 4;
static const uint8_t DISPLAY_BUTTON_RIGHT_PIN = 14;
static const uint8_t DISPLAY_BUTTON_LEFT_PIN = 16;
static const uint8_t RGB_LED_PIN = 6;
// remaining unused pins are: 3, 5

static uint8_t serial_data[64];
static uint8_t serial_write_offset = 0;

typedef enum {
  WAIT_STATUS_MASK_BOOT = 1 << 4,
  WAIT_STATUS_MASK_REBOOT = 1 << 5,
  WAIT_STATUS_MASK_SHUTDOWN = 1 << 6,
  WAIT_STATUS_MASK_SLEEP = 1 << 7,
} WaitStatusMask;

typedef enum {
  WAIT_STATUS_UNDEFINED       =      0,
  WAIT_STATUS_BOOT_PHASE1     = WAIT_STATUS_MASK_BOOT | 1,      // powered up
  WAIT_STATUS_BOOT_PHASE2     = WAIT_STATUS_MASK_BOOT | 2,      // USB connected (kernel loaded, CDC driver connected)
  WAIT_STATUS_BOOT_PHASEN     = WAIT_STATUS_MASK_BOOT | 3,      // user defined boot stages
  WAIT_STATUS_REBOOT_PHASE1   = WAIT_STATUS_MASK_REBOOT | 1,    // reboot intialized (everything is still working)
  WAIT_STATUS_REBOOT_PHASE2   = WAIT_STATUS_MASK_REBOOT | 2,    // final stage of reboot (USB suspended but still connected, we need to wait for USB disconnect)
  WAIT_STATUS_SHUTDOWN_PHASE1 = WAIT_STATUS_MASK_SHUTDOWN | 1,  // shutdown intialized (everything is still working)
  WAIT_STATUS_SHUTDOWN_PHASE2 = WAIT_STATUS_MASK_SHUTDOWN | 2,  // final stage of shutdown (USB suspended)
  WAIT_STATUS_SLEEP_PHASE1    = WAIT_STATUS_MASK_SLEEP | 1,     // prepairing for sleep
  WAIT_STATUS_SLEEP_PHASE2    = WAIT_STATUS_MASK_SLEEP | 2,     // just before entering sleep
  WAIT_STATUS_SLEEP           = WAIT_STATUS_MASK_SLEEP | 3,     // actual sleep status
} WaitStatus;

static WaitStatus _wait_status;
static unsigned long _last_status_change_time = 0;

static const int8_t COMMAND_LEN = 2;
static const uint16_t CMD_RESET = ('~' << 8) | 'R';
static const uint16_t CMD_CANCEL_WAIT_STATUS = ('#' << 8) | '0';
static const uint16_t CMD_PREPARE_FOR_REBOOT = ('#' << 8) | 'R';
static const uint16_t CMD_PREPARE_FOR_SHUTDOWN = ('#' << 8) | 'S';
static const uint16_t CMD_ENTER_HYBERNATE_MODE = ('#' << 8) | 'H';
static const uint16_t CMD_RGB_LED_COLOR_RESET = ('L' << 8) | 'R';
static const uint16_t CMD_RGB_LED_COLOR_SET = ('L' << 8) | 'C';
static const uint16_t CMD_RGB_LED_BREATH_SET = ('L' << 8) | 'B';
static const uint16_t CMD_SOUND_PLAY = ('S' << 8) | 'P';
static const uint16_t CMD_SOUND_ALERT_SET = ('S' << 8) | 'A';
static const uint16_t CMD_FAN_SPEED_SET = ('F' << 8) | 'S';
static const uint16_t CMD_FAN_PWM_SET = ('F' << 8) | 'P';
static const uint16_t CMD_FAN_CARRIER_FREQUENCY_SET = ('F' << 8) | 'F';
static const uint16_t CMD_FAN_HYSTERESYS_SET = ('F' << 8) | 'H';
static const uint16_t CMD_FAN_ALERT_ENABLE = ('F' << 8) | 'A';
static const uint16_t CMD_DISPLAY_PAGE = ('D' << 8) | 'P';
static const uint16_t CMD_DISPLAY_PAGE_INDICES_SET = ('D' << 8) | 'I';
static const uint16_t CMD_DISPLAY_WRITE_PAGE = ('D' << 8) | 'W';
static const uint16_t CMD_DISPLAY_ALERT = ('D' << 8) | 'A';
static const uint16_t CMD_DISPLAY_MESSAGE = ('D' << 8) | 'M';
static const uint16_t CMD_DISPLAY_BRIGHTNESS = ('D' << 8) | 'B';
static const uint16_t CMD_TEMPERATURE_SCALE_FAHRENHEIT_SET = ('T' << 8) | 'F';
static const uint16_t CMD_TEMPERATURE_OFFSET = ('T' << 8) | 'O';
static const uint16_t CMD_TEMPERATURE_ALERT_ENABLE = ('T' << 8) | 'A';
static const uint16_t CMD_EXTERNAL_SENSOR_SET_LOAD = ('E' << 8) | 'L';
static const uint16_t CMD_EXTERNAL_SENSOR_SET_FANS = ('E' << 8) | 'F';
static const uint16_t CMD_EXTERNAL_SENSOR_SET_TEMP = ('E' << 8) | 'T';
static const uint16_t CMD_EXTERNAL_SENSOR_SET_NETWORK_STATUS = ('E' << 8) | 'N';
static const uint16_t CMD_EXTERNAL_SENSOR_SET_ARRAY_STATUS = ('E' << 8) | 'A';
static const uint16_t CMD_DISK_SHUTDOWN_DELAY_SET = ('H' << 8) | 'D';
static const uint16_t CMD_DISK_TEMPERATURE_ALERT_SET = ('H' << 8) | 'A';

// Error return codes
static const int8_t ERR_COMMAND_OVERFLOW = -1;
static const int8_t ERR_UNKNOWN_COMMAND = -2;
static const int8_t ERR_NO_COMMAND_PARAMETERS = -3;
static const int8_t ERR_INVALID_PARAMETERS = -4;

// Display config
static const uint8_t DISPLAY_WIDTH = 16;
static const uint8_t DISPLAY_HEIGHT = 2;

// Sensor config
static const int8_t SENSOR_NOT_PRESENT = -127;

typedef enum {
  DISPLAY_PAGE_TYPE_BLINK_INFO = 0,
  DISPLAY_PAGE_TYPE_CUSTOM = 1,
  DISPLAY_PAGE_TYPE_CUSTOM_PAGE_1 = DISPLAY_PAGE_TYPE_CUSTOM,
  DISPLAY_PAGE_TYPE_CUSTOM_PAGE_2,
  DISPLAY_PAGE_TYPE_CUSTOM_PAGE_3,
  DISPLAY_PAGE_TYPE_CUSTOM_PAGE_4,
  DISPLAY_PAGE_TYPE_CUSTOM_PAGE_LAST,
  DISPLAY_PAGE_TYPE_COOLING_HDD,
  DISPLAY_PAGE_TYPE_COOLING_LSI,
  DISPLAY_PAGE_TYPE_COOLING_CPU,
  DISPLAY_PAGE_TYPE_COOLING_MOTHERBOARD,
  DISPLAY_PAGE_TYPE_COOLING_ALL,
  DISPLAY_PAGE_TYPE_COOLING_ALL_DISKS,
  DISPLAY_PAGE_TYPE_LOAD_CPU,
  DISPLAY_PAGE_TYPE_LOAD_RAM,
  DISPLAY_PAGE_TYPE_LOAD_ALL_DISKS,
  DISPLAY_PAGE_TYPE_UPTIME,
  DISPLAY_PAGE_TYPE__DO_NOT_REMOVE
} DisplayPageTypes;

typedef enum {
  INT_TEMP_SENSOR_INDEX_HDD = 0,
  INT_TEMP_SENSOR_INDEX_LSI,
  INT_TEMP_SENSOR_INDEX__DO_NOT_REMOVE
} IntTempSensorIndex;

typedef enum {
  INT_FAN_SENSOR_INDEX_HDD = 0,
  INT_FAN_SENSOR_INDEX_LSI,
  INT_FAN_SENSOR_INDEX__DO_NOT_REMOVE
} IntFanSensorIndex;

typedef enum {
  EXT_TEMP_SENSOR_INDEX_CPU = 0,
  EXT_TEMP_SENSOR_INDEX_Motherboard,
  EXT_TEMP_SENSOR_INDEX_HDD1,
  EXT_TEMP_SENSOR_INDEX_HDD2,
  EXT_TEMP_SENSOR_INDEX_HDD3,
  EXT_TEMP_SENSOR_INDEX_HDD4,
  EXT_TEMP_SENSOR_INDEX_HDD_LAST = EXT_TEMP_SENSOR_INDEX_HDD4,
  EXT_TEMP_SENSOR_INDEX__DO_NOT_REMOVE
} ExtTempSensorIndex;

typedef enum {
  EXT_FAN_SENSOR_INDEX_CPU = 0,
  EXT_FAN_SENSOR_INDEX_Motherboard,
  EXT_FAN_SENSOR_INDEX__DO_NOT_REMOVE
} ExtFanSensorIndex;

typedef enum {
  EXT_LOAD_SENSOR_INDEX_CPU = 0,
  EXT_LOAD_SENSOR_INDEX_RAM,
  EXT_LOAD_SENSOR_INDEX_HDD1,
  EXT_LOAD_SENSOR_INDEX_HDD2,
  EXT_LOAD_SENSOR_INDEX_HDD3,
  EXT_LOAD_SENSOR_INDEX_HDD4,
  EXT_LOAD_SENSOR_INDEX_HDD_LAST = EXT_LOAD_SENSOR_INDEX_HDD4,
  EXT_LOAD_SENSOR_INDEX__DO_NOT_REMOVE
} ExtLoadSensorIndex;

typedef enum {
  RGB_LED_USER_MODE_NONE = 0,
  RGB_LED_USER_MODE_CONSTANT_COLOR,
  RGB_LED_USER_MODE_BREATHING_COLOR
} RgbLedUserMode;

struct TokenChunk {
  uint16_t tokens[8];
  uint8_t count;
};

struct MelodyNote {
  uint16_t frequency;
  uint16_t length;
};

struct RgbColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint16_t length;
};

struct FanCapture {
  volatile uint16_t rpm_counter;
  uint16_t capture_buffer[8];
  uint8_t capture_buffer_index;
  uint8_t min_duty_cycle;
  uint8_t max_duty_cycle;
  int8_t req_duty_percentage;  // fan speed in 0..100% of PWM between `min_duty_cycle` and `max_duty_cycle` (set negative value to disable fan)
  uint16_t alert_rpm;
  uint8_t alert_threshold;
  uint8_t alert_trigger;
  int16_t current_rpm;
  bool use_hysteresys_curve;
  int8_t hysteresys_curve[8];
};

struct TempCapture {
  DeviceAddress address;
  int8_t correction;
  int8_t current_temp;
  uint8_t alert_temp;
  uint8_t alert_threshold;
  uint8_t alert_trigger;
};

// Some durations
static const int FAN_RPM_CAPTURES_PER_SECOND = 4;
static const int DISPLAY_ALERT_DURATION_IN_MILLISECONDS = 1000;
static const int DISPLAY_INFO_DURATION_IN_MILLISECONDS = 2000;
static const int RGB_LED_BREATH_INCREMENT_IN_MILLISECONDS = 30;

// Time in milliseconds to prevent too frequent HDD power cycles.
static const int HDD_POWER_TOGGLE_COOLING_PERIOD_IN_MILLISECONDS = 30000;

// For how many milliseconds to ignore signal from display buttons after restart.
// This helps touch/capacitive buttons to stabilize. Can be 0 for physical contact buttons.
static const int DISPLAY_BUTTON_IGNORE_PERIOD_IN_MILLISECONDS = 5000;

static bool _use_fahrenheit_temp = false;

static bool _melody_is_playing = false;
static bool _melody_is_playing_error = false;
static MelodyNote _melody_notes[16];
static unsigned long _melody_start_time = 0;
static int8_t _melody_note_playing_index = 0;

static uint16_t _alert_sound_freq = 2000;
static uint16_t _alert_sound_duration = 500;

static uint16_t _fan_carrier_freq = 25000;

static RgbColor _rgb_led_colors[8];
static uint8_t _rgb_led_toggle_counter = 0;
static unsigned long _rgb_led_next_toggle_time = 0;
static int8_t _rgb_led_breath_direction = 0;
static RgbColor _rgb_led_colors_user_set[sizeof(_rgb_led_colors) / sizeof(_rgb_led_colors[0])];
static RgbLedUserMode _rgb_led_colors_user_mode = RGB_LED_USER_MODE_NONE;

static uint8_t _lcd_brightness_pwm = 255;
static uint8_t _lcd_brightness_current = 0;
static uint8_t _lcd_brightness_target = 255;
static uint8_t _lcd_fade_speed_increment = 5;
static unsigned long _last_lcd_fade_update_time = 0;

static uint8_t _display_page_current = 1;
static unsigned long _display_alert_end_time = 0;
static unsigned long _display_info_end_time = 0;
static bool _is_showing_alert = false;
static bool _is_showing_info = false;

static uint8_t _disk_temperature_alert = 0;
static uint8_t _disk_shutdown_delay_in_seconds = 0;
static unsigned long _disk_shutdown_delay_end_time = 0;

static bool _disk_power_state = false;
static unsigned long _disk_power_last_toggle = 0;
static uint8_t _disk_power_button_debouncer = 0;

static uint8_t _display_button_right_debouncer = 0;
static uint8_t _display_button_left_debouncer = 0;
static unsigned long _display_button_screen_page_hold_end_time = 0;

static bool _was_usb_suspended = false;
static bool _was_usb_connected = false;
static bool _was_showing_alert = false;

static int8_t _is_network_connected = SENSOR_NOT_PRESENT;
static int8_t _was_network_connected = SENSOR_NOT_PRESENT;
static int8_t _is_array_started = SENSOR_NOT_PRESENT;
static int8_t _was_array_started = SENSOR_NOT_PRESENT;

static FanCapture fans[INT_FAN_SENSOR_INDEX__DO_NOT_REMOVE];
static TempCapture temps[INT_TEMP_SENSOR_INDEX__DO_NOT_REMOVE];
static int16_t ext_fans[EXT_FAN_SENSOR_INDEX__DO_NOT_REMOVE];
static int8_t ext_temps[EXT_TEMP_SENSOR_INDEX__DO_NOT_REMOVE];
static int8_t ext_loads[EXT_LOAD_SENSOR_INDEX__DO_NOT_REMOVE];

static char *display_page_texts[DISPLAY_PAGE_TYPE_CUSTOM_PAGE_LAST + 1];
static uint8_t display_page_map[DISPLAY_PAGE_TYPE__DO_NOT_REMOVE];

static uint32_t _last_check_time_1o4_second_counter = 0;
static unsigned long _last_check_time_1o4_second = 0;

static OneWire oneWire(ONEWIRE_PIN);
static DallasTemperature tempSensors(&oneWire);
static Waveshare_LCD1602 lcd(16, 2);

void setup() {
  wdt_disable();
  wdt_enable(WDTO_2S);

  // ADC reference voltage
  analogReference(DEFAULT);

  // Configure pins for buttons
  pinMode(HDD_POWER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(DISPLAY_BUTTON_RIGHT_PIN, INPUT_PULLUP);
  pinMode(DISPLAY_BUTTON_LEFT_PIN, INPUT_PULLUP);

  // Configure pin for RGB LED
  pinMode(RGB_LED_PIN, OUTPUT);
  digitalWrite(RGB_LED_PIN, LOW);

  // RGB LED white on power up
  rgb_led_send_data(255, 255, 150);

  i2c_init();
  lcd.init();
  create_custom_lcd_chars();

  init_display_pages();
  init_external_stats();

  // Beep on power up
  tone(BUZZER_PIN, 800, 200);

  // TODO: Check 12v/5v power supply to HDD on reset, and if it's already ON then skip the power button check
  // and also set `_disk_power_state` to reflect current state
  for (uint8_t i = 0; i < 100; i++) {
    if (_disk_power_state) {
      break;
    }

    process_hdd_power_button(i);
    delay(1);
  }

  wdt_reset();

  exit_sleep_mode();

  // Synchronously fade in LCD brightness
  for (uint8_t i = 0; i < (_lcd_brightness_target / _lcd_fade_speed_increment) + 1; i++) {
    fade_lcd_update();
    delay(10);
    wdt_reset();
  }

  // Open serial port
  Serial.begin(9600);

  // Disable TX/RX LEDs
  pinMode(LED_BUILTIN_TX, INPUT);
  pinMode(LED_BUILTIN_RX, INPUT);

  change_wait_status(WAIT_STATUS_BOOT_PHASE1);

  temp_sensors_enable();

  fan_pin_interrupt_enable();
  interrupts();

  // Beep again if HDD power was enabled at startup
  if (_disk_power_state) {
    tone(BUZZER_PIN, 800, 200);
  }

  wdt_disable();
  wdt_enable(WDTO_500MS);
}

void loop() {
  wdt_reset();

  const bool is_usb_suspended = USBDevice.isSuspended();
  const bool is_usb_connected = USBDevice.configured();
  const bool toggle_usb_suspended = !_was_usb_suspended && is_usb_suspended;
  const bool toggle_usb_unsuspended = _was_usb_suspended && !is_usb_suspended;
  const bool toggle_usb_connected = !_was_usb_connected && is_usb_connected;
  const bool toggle_usb_disconnected = _was_usb_connected && !is_usb_connected;

  const unsigned long elapsed_time = millis();
  _is_showing_alert = elapsed_time < _display_alert_end_time;
  _is_showing_info = elapsed_time < _display_info_end_time;

  if (_wait_status)
  {
    if (_wait_status == WAIT_STATUS_BOOT_PHASE1 && toggle_usb_connected) {
      change_wait_status(WAIT_STATUS_BOOT_PHASE2);
    } else if (_wait_status == WAIT_STATUS_BOOT_PHASE2 && Serial) {
      change_wait_status(WAIT_STATUS_BOOT_PHASEN);
    } else if (_wait_status == WAIT_STATUS_REBOOT_PHASE1 && toggle_usb_suspended) {
      change_wait_status(WAIT_STATUS_REBOOT_PHASE2);
    } else if (_wait_status == WAIT_STATUS_REBOOT_PHASE2 && toggle_usb_disconnected) {
      // reset as on power cycle
      while (1) {}
    } else if (_wait_status == WAIT_STATUS_SHUTDOWN_PHASE1 && toggle_usb_suspended) {
      change_wait_status(WAIT_STATUS_SHUTDOWN_PHASE2);
    } else if (_wait_status == WAIT_STATUS_SLEEP_PHASE1) {
      if (_disk_shutdown_delay_end_time) {
        if (elapsed_time > _disk_shutdown_delay_end_time) {
          hdd_power_off();
          change_wait_status(WAIT_STATUS_SLEEP_PHASE2);
        }
      } else {
        change_wait_status(WAIT_STATUS_SLEEP_PHASE2);
      }
    } else if (_wait_status == WAIT_STATUS_SLEEP_PHASE2) {
      change_wait_status(WAIT_STATUS_SLEEP);
      fade_out_lcd_backlight();
    } else if ((_wait_status & WAIT_STATUS_MASK_SLEEP) == WAIT_STATUS_MASK_SLEEP && toggle_usb_unsuspended) {
      exit_sleep_mode();
    }
  } else {
    if (is_usb_connected && toggle_usb_suspended) {
      enter_sleep_mode();
    }
  }

  _was_usb_suspended = is_usb_suspended;
  _was_usb_connected = is_usb_connected;

  if (Serial) {
    if (Serial.available() > 0) {
      uint8_t c = Serial.read();
      if (serial_write_offset && (c == '\r' || c == '\n' || c == '\0')) {
        serial_data[serial_write_offset] = 0;
        parse_command();
        serial_write_offset = 0;
      } else {
        serial_data[serial_write_offset++] = c;
        if (serial_write_offset == sizeof(serial_data)) {
          serial_write_offset = 0;
          play_error_sound(ERR_COMMAND_OVERFLOW);
        }
      }
    }
  }

  // *** HIGH PRIORITY CODE STARTS HERE ***
  process_hdd_power_button(elapsed_time);

  if (elapsed_time > DISPLAY_BUTTON_IGNORE_PERIOD_IN_MILLISECONDS) {
    process_display_button();
  }

  if (_disk_shutdown_delay_end_time && elapsed_time > _disk_shutdown_delay_end_time) {
    hdd_power_off();
  }

  if (elapsed_time - _last_lcd_fade_update_time > 10) {
    fade_lcd_update();
    _last_lcd_fade_update_time = elapsed_time;
  }

  melody_play();

  process_status_changes();

  if (_is_showing_alert != _was_showing_alert) {
    _was_showing_alert = _is_showing_alert;

    rgb_led_restore_color();
  }

  rgb_led_update();

  // Don't run any lower priority code when melody is playing, as it upsets duration timings
  if (_melody_is_playing) {
    return;
  }

  // *** LESS PRIORITY CODE STARTS HERE ***
  const unsigned long elapsed_time_since_1o4_second_check = elapsed_time - _last_check_time_1o4_second;
  if (elapsed_time_since_1o4_second_check >= 250) {
    _last_check_time_1o4_second = elapsed_time - (elapsed_time_since_1o4_second_check - 250);
    _last_check_time_1o4_second_counter++;

    fans_rpm_update();

    //int adcValue = analogRead(ADC_PIN);
    //Serial.println(fan_rpm);
  }

  // *** LOW PRIORITY CODE STARTS HERE ***
  // Every 1 second updates
  if (_last_check_time_1o4_second_counter % 4) {
    fan_pwm_apply_updated_values();

    if (!_is_showing_alert && !_is_showing_info) {
      if (_wait_status) {
        if (elapsed_time > _display_button_screen_page_hold_end_time) {
          display_wait_screen();
        }
      } else {
        display_page(_display_page_current);
      }
    }

    rgb_led_restore_color();
  }

  // Don't run any other code in sleep
  if (_wait_status == WAIT_STATUS_SLEEP) {
    return;
  }

  // Every 2 seconds updates
  if (_last_check_time_1o4_second_counter % 8) {
    temp_sensors_update();
    check_for_alerts();
  }
}

void check_for_alerts() {
  static const uint8_t number_of_fans = sizeof(fans)/sizeof(fans[0]);
  for (uint8_t fan_index = 0; fan_index < number_of_fans; fan_index++) {
    const bool fan_alert = fans[fan_index].alert_rpm && (fans[fan_index].alert_trigger >= fans[fan_index].alert_threshold);
    if (fan_alert) {
      display_alert("*** ALERT! ***", "FAN TOO SLOW");
      break;
    }
  }

  static const uint8_t number_of_sensors = sizeof(temps)/sizeof(temps[0]);
  for (uint8_t sensor_index = 0; sensor_index < number_of_sensors; sensor_index++) {
    const bool temp_alert = temps[sensor_index].alert_temp && (temps[sensor_index].alert_trigger >= temps[sensor_index].alert_threshold);
    if (temp_alert) {
      display_alert("*** ALERT! ***", "HIGH TEMPERATURE");
      break;
    }
  }

  if (_disk_temperature_alert) {
    uint8_t disk_number= 0;
    if (ext_temps[EXT_TEMP_SENSOR_INDEX_HDD1] >= _disk_temperature_alert) {
      disk_number = 1;
    } else if (ext_temps[EXT_TEMP_SENSOR_INDEX_HDD2] >= _disk_temperature_alert) {
      disk_number = 2;
    } else if (ext_temps[EXT_TEMP_SENSOR_INDEX_HDD3] >= _disk_temperature_alert) {
      disk_number = 3;
    } else if (ext_temps[EXT_TEMP_SENSOR_INDEX_HDD4] >= _disk_temperature_alert) {
      disk_number = 4;
    }

    if (disk_number) {
      char line2[] = "HDD #0 TOO HOT";
      line2[5] = '0' + disk_number;
      display_alert("*** ALERT! ***", "line2");
    }
  }
}

void parse_command() {
  if (serial_write_offset == 0) {
    return;
  }

  for (uint8_t i = 0; i < serial_write_offset - 1; i++) {
    if (serial_data[i] <= ' ') {
      continue;
    }

    if (serial_data[i] == '|' || serial_data[i] == '/') {
      continue;
    }

    uint16_t command = (serial_data[i + 0] << 8) | serial_data[i + 1];
    const int8_t error_or_chars_count = process_command(command, &serial_data[i + COMMAND_LEN], serial_write_offset - (i + COMMAND_LEN));
    if (error_or_chars_count < 0) {
      play_error_sound(error_or_chars_count);
      return;
    }
    
    i += (error_or_chars_count - 1) + COMMAND_LEN;
  }

  // if we are still in the boot phase, then switch to user supplied boot messages
  if (_wait_status == WAIT_STATUS_BOOT_PHASE2) {
    change_wait_status(WAIT_STATUS_BOOT_PHASEN);
  }
}

int8_t process_command(const uint16_t cmd, const uint8_t payload[], const uint8_t payload_length) {
  if (cmd == CMD_RESET) {
    while (1) {}
  }

  if (cmd == CMD_CANCEL_WAIT_STATUS) {
    if ((_wait_status & WAIT_STATUS_MASK_SLEEP) == WAIT_STATUS_MASK_SLEEP) {
      exit_sleep_mode();
    } else {
      change_wait_status(WAIT_STATUS_UNDEFINED);
    }

    return 0;
  }

  if (cmd == CMD_PREPARE_FOR_REBOOT) {
    change_wait_status(WAIT_STATUS_REBOOT_PHASE1);
    return 0;
  }

  if (cmd == CMD_PREPARE_FOR_SHUTDOWN) {
    change_wait_status(WAIT_STATUS_SHUTDOWN_PHASE1);
    return 0;
  }

  if (cmd == CMD_ENTER_HYBERNATE_MODE) {
    enter_sleep_mode();
    return 0;
  }

  if (cmd == CMD_RGB_LED_COLOR_RESET) {
    _rgb_led_colors_user_mode = RGB_LED_USER_MODE_NONE;
    memset(_rgb_led_colors_user_set, 0, sizeof(_rgb_led_colors_user_set));
    rgb_led_set_colors(NULL, 0, false);
    return 0;
  }

  if (cmd == CMD_DISPLAY_ALERT || cmd == CMD_DISPLAY_MESSAGE) {
    if (payload[0] != ':') {
      return ERR_INVALID_PARAMETERS;
    }

    write_to_page(0, &payload[1], payload_length - 1);

    char *second_line_start = NULL;

    for (uint8_t i = 0; i <= DISPLAY_WIDTH; i++) {
      if (display_page_texts[0][i] == ';') {
        display_page_texts[0][i] = '\0';
        second_line_start = &display_page_texts[0][i + 1];
        break;
      }
    }

    if (cmd == CMD_DISPLAY_ALERT) {
      display_alert(display_page_texts[0], second_line_start);
    } else {
      display_message(display_page_texts[0], second_line_start);
    }

    return payload_length;
  }

  TokenChunk token_chunks[16];
  uint8_t token_chunks_count = 0;
  const uint8_t chars_count = extract_int_tokens(payload, payload_length, token_chunks, &token_chunks_count);

  if (token_chunks_count == 0) {
    return ERR_NO_COMMAND_PARAMETERS;
  }

  const uint8_t tokens_count_in_first_chunk = token_chunks[0].count;
  if (tokens_count_in_first_chunk == 0) {
    return ERR_INVALID_PARAMETERS;
  }

  if (cmd == CMD_DISPLAY_PAGE) {
    const uint16_t *tokens = token_chunks[0].tokens;
    if (tokens[0] >= DISPLAY_PAGE_TYPE__DO_NOT_REMOVE) {
      return ERR_INVALID_PARAMETERS;
    }
    _display_page_current = tokens[0];
    return chars_count;
  }

  if (cmd == CMD_DISPLAY_PAGE_INDICES_SET) {
    const uint8_t page_map_length = sizeof(display_page_map) / sizeof(display_page_map[0]);
    for (uint8_t i = 0; i < page_map_length; i++) {
      display_page_map[i] = i;
    }

    if (tokens_count_in_first_chunk >= page_map_length)
    {
      return ERR_INVALID_PARAMETERS;
    }

    const uint16_t *tokens = token_chunks[0].tokens;

    // When first page number is magical instead of real, then just don't populate the map after erase
    if (tokens[0] >= 100) {
      return chars_count;
    }
    
    for (uint8_t i = 0; i < tokens_count_in_first_chunk; i++) {
      const uint8_t physical_page_index = tokens[i];
      if (physical_page_index >= DISPLAY_PAGE_TYPE__DO_NOT_REMOVE) {
        return ERR_INVALID_PARAMETERS;
      }
      const uint8_t logical_page_index = i + 1;
      display_page_map[logical_page_index] = physical_page_index;
    }
    
    return chars_count;
  }

  if (cmd == CMD_RGB_LED_COLOR_SET || cmd == CMD_RGB_LED_BREATH_SET) {
    _rgb_led_colors_user_mode = RGB_LED_USER_MODE_NONE;
    memset(_rgb_led_colors_user_set, 0, sizeof(_rgb_led_colors_user_set));
    rgb_led_set_colors(NULL, 0, false);

    for (uint8_t token_chunk_index = 0; token_chunk_index < token_chunks_count; token_chunk_index++) {
      if (token_chunk_index >= sizeof(_rgb_led_colors_user_set) / sizeof(_rgb_led_colors_user_set[0])) {
        return ERR_INVALID_PARAMETERS;
      }

      const uint8_t tokens_count = token_chunks[token_chunk_index].count;
      if (tokens_count < 3) {
        return ERR_INVALID_PARAMETERS;
      }

      RgbColor *color = &_rgb_led_colors_user_set[token_chunk_index];

      const uint16_t *tokens = token_chunks[token_chunk_index].tokens;
      color->r = tokens[0];
      color->g = tokens[1];
      color->b = tokens[2];
      color->length = tokens_count > 3 ? tokens[3] : 0;

      if (cmd == CMD_RGB_LED_BREATH_SET) {
        _rgb_led_colors_user_mode = RGB_LED_USER_MODE_BREATHING_COLOR;
        break;
      }
      
      _rgb_led_colors_user_mode = RGB_LED_USER_MODE_CONSTANT_COLOR;
    }

    rgb_led_set_colors(_rgb_led_colors_user_set, sizeof(_rgb_led_colors_user_set) / sizeof(_rgb_led_colors_user_set[0]), _rgb_led_colors_user_mode == RGB_LED_USER_MODE_BREATHING_COLOR);

    return chars_count;
  }

  if (cmd == CMD_DISPLAY_BRIGHTNESS) {
    const uint16_t *tokens = token_chunks[0].tokens;
    _lcd_brightness_current = _lcd_brightness_pwm;
    _lcd_brightness_pwm = tokens[0] >= 100 ? 255 : (tokens[0] * 2.55);
    _lcd_brightness_target = _lcd_brightness_pwm;
    return chars_count;
  }

  if (cmd == CMD_DISPLAY_WRITE_PAGE) {
    const uint16_t *tokens = token_chunks[0].tokens;
    const uint8_t page_index = tokens[0];
    if (page_index >= sizeof(display_page_texts) / sizeof(display_page_texts[0])) {
      return ERR_INVALID_PARAMETERS;
    }

    if (payload[chars_count] != ':') {
      return ERR_INVALID_PARAMETERS;
    }

    write_to_page(page_index, &payload[chars_count + 1], payload_length - (chars_count + 1));

    if (page_index == DISPLAY_PAGE_TYPE_BLINK_INFO) {
      if (_wait_status) {
        display_wait_screen();
      } else {
        _display_info_end_time = millis() + DISPLAY_INFO_DURATION_IN_MILLISECONDS;
        display_page(DISPLAY_PAGE_TYPE_BLINK_INFO);
      }
    }

    return payload_length;
  }

  if (cmd == CMD_SOUND_PLAY) {
    MelodyNote play_notes[16];
    uint8_t note_index = 0;

    for (uint8_t token_chunk_index = 0; token_chunk_index < token_chunks_count; token_chunk_index++) {
      const uint8_t tokens_count = token_chunks[token_chunk_index].count;
      if (!tokens_count) {
        return ERR_INVALID_PARAMETERS;
      }

      const uint16_t *tokens = token_chunks[token_chunk_index].tokens;
      play_notes[note_index].frequency = tokens[0];
      play_notes[note_index].length = tokens_count > 1 ? tokens[1] : 250;
      note_index++;
    }

    play_notes[note_index].frequency = 0;
    play_notes[note_index].length = 0;

    melody_play(play_notes, note_index);

    return chars_count;
  }

  if (cmd == CMD_SOUND_ALERT_SET) {
    const uint16_t *tokens = token_chunks[0].tokens;
    _alert_sound_freq = tokens[0];
    _alert_sound_duration = tokens_count_in_first_chunk > 1 ? tokens[1] : 500;
    return chars_count;
  }

  if (cmd == CMD_FAN_SPEED_SET) {
    for (uint8_t token_chunk_index = 0; token_chunk_index < token_chunks_count; token_chunk_index++) {
      const uint8_t tokens_count = token_chunks[token_chunk_index].count;
      if (tokens_count <= 1) {
        return ERR_INVALID_PARAMETERS;
      }

      static const uint8_t number_of_fans = sizeof(fans)/sizeof(fans[0]);
      const uint16_t *tokens = token_chunks[token_chunk_index].tokens;
      const uint8_t fan_index = tokens[0] > 0 && tokens[0] <= number_of_fans ? tokens[0] - 1 : 0;
      const int8_t target_pwm = tokens[1] > 255 ? -1 : tokens[1] < 100 ? tokens[1] : 100;

      fans[fan_index].req_duty_percentage = target_pwm;
      fans[fan_index].use_hysteresys_curve = false;
    }

    fan_pwm_apply_updated_values();

    return chars_count;
  }

  if (cmd == CMD_FAN_PWM_SET) {
    for (uint8_t token_chunk_index = 0; token_chunk_index < token_chunks_count; token_chunk_index++) {
      const uint8_t tokens_count = token_chunks[token_chunk_index].count;
      if (tokens_count <= 1) {
        return ERR_INVALID_PARAMETERS;
      }

      static const uint8_t number_of_fans = sizeof(fans)/sizeof(fans[0]);
      const uint16_t *tokens = token_chunks[token_chunk_index].tokens;
      const uint8_t fan_index = tokens[0] > 0 && tokens[0] <= number_of_fans ? tokens[0] - 1 : 0;
      const uint8_t min_pwm = tokens[1] < 100 ? tokens[1] * 2.55F : 255;

      fans[fan_index].min_duty_cycle = min_pwm;

      if (tokens_count > 2) {
        const uint8_t max_pwm = tokens[2] < 100 ? tokens[2] * 2.55F : 255;
        fans[fan_index].max_duty_cycle = max_pwm;
      }

      if (fans[fan_index].min_duty_cycle >= fans[fan_index].max_duty_cycle) {
        fans[fan_index].min_duty_cycle = 0;
        fans[fan_index].max_duty_cycle = 255;
      }
    }

    fan_pwm_apply_updated_values();

    return chars_count;
  }

  if (cmd == CMD_FAN_CARRIER_FREQUENCY_SET) {
    const uint16_t *tokens = token_chunks[0].tokens;
    _fan_carrier_freq = tokens[0];
    fan_pwm_apply_updated_values();
    return chars_count;
  }

  if (cmd == CMD_FAN_HYSTERESYS_SET) {
    for (uint8_t token_chunk_index = 0; token_chunk_index < token_chunks_count; token_chunk_index++) {
      const uint8_t tokens_count = token_chunks[token_chunk_index].count;
      if (tokens_count <= 1) {
        return ERR_INVALID_PARAMETERS;
      }

      static const uint8_t number_of_fans = sizeof(fans)/sizeof(fans[0]);
      const uint16_t *tokens = token_chunks[token_chunk_index].tokens;
      const uint8_t fan_index = tokens[0] > 0 && tokens[0] <= number_of_fans ? tokens[0] - 1 : 0;
      for (uint8_t token_index = 1; token_index < tokens_count; token_index++) {
        const int8_t target_pwm = tokens[token_index] > 255 ? -1 : tokens[token_index] < 100 ? tokens[token_index]
                                                                                             : 100;
        fans[fan_index].hysteresys_curve[token_index - 1] = target_pwm;
        if (token_index == 8) {
          break;
        }
      }

      fans[fan_index].use_hysteresys_curve = true;
    }

    fan_pwm_apply_updated_values();

    return chars_count;
  }

  if (cmd == CMD_FAN_ALERT_ENABLE) {
    for (uint8_t token_chunk_index = 0; token_chunk_index < token_chunks_count; token_chunk_index++) {
      const uint8_t tokens_count = token_chunks[token_chunk_index].count;
      if (tokens_count <= 1) {
        return ERR_INVALID_PARAMETERS;
      }

      static const uint8_t number_of_fans = sizeof(fans)/sizeof(fans[0]);
      const uint16_t *tokens = token_chunks[token_chunk_index].tokens;
      const uint8_t fan_index = tokens[0] > 0 && tokens[0] <= number_of_fans ? tokens[0] - 1 : 0;
      fans[fan_index].alert_rpm = tokens[1];
      fans[fan_index].alert_trigger = 0;

      if (tokens_count > 2) {
        fans[fan_index].alert_threshold = tokens[2];
      } else {
        fans[fan_index].alert_threshold = 4;
      }
    }

    return chars_count;
  }

  if (cmd == CMD_TEMPERATURE_SCALE_FAHRENHEIT_SET) {
    const uint16_t *tokens = token_chunks[0].tokens;
    _use_fahrenheit_temp = tokens[0] != 0;
    return chars_count;
  }

  if (cmd == CMD_TEMPERATURE_OFFSET) {
    for (uint8_t token_chunk_index = 0; token_chunk_index < token_chunks_count; token_chunk_index++) {
      const uint8_t tokens_count = token_chunks[token_chunk_index].count;
      if (tokens_count <= 1) {
        return ERR_INVALID_PARAMETERS;
      }

      const uint8_t max_number_of_sensors = (sizeof(temps) / sizeof(temps[0]));
      const uint16_t *tokens = token_chunks[token_chunk_index].tokens;
      const uint8_t sensor_index = tokens[0] > 0 && tokens[0] <= max_number_of_sensors ? tokens[0] - 1 : 0;

      if (tokens[1] <= 10) {
        temps[sensor_index].correction = tokens[1];
      } else if (tokens[1] > 100 && tokens[1] <= 110) {
        temps[sensor_index].correction = 100 - tokens[1];
      } else {
        return ERR_INVALID_PARAMETERS;
      }
    }

    return chars_count;
  }

  if (cmd == CMD_TEMPERATURE_ALERT_ENABLE) {
    for (uint8_t token_chunk_index = 0; token_chunk_index < token_chunks_count; token_chunk_index++) {
      const uint8_t tokens_count = token_chunks[token_chunk_index].count;
      if (tokens_count <= 1) {
        return ERR_INVALID_PARAMETERS;
      }

      const uint8_t max_number_of_sensors = (sizeof(temps) / sizeof(temps[0]));
      const uint16_t *tokens = token_chunks[token_chunk_index].tokens;
      const uint8_t sensor_index = tokens[0] > 0 && tokens[0] <= max_number_of_sensors ? tokens[0] - 1 : 0;
      temps[sensor_index].alert_temp = tokens[1] < 255 ? tokens[1] : 0;
      temps[sensor_index].alert_trigger = 0;

      if (tokens_count > 2) {
        temps[sensor_index].alert_threshold = tokens[2];
      } else {
        temps[sensor_index].alert_threshold = 4;
      }
    }

    return chars_count;
  }

  if (cmd == CMD_EXTERNAL_SENSOR_SET_LOAD || cmd == CMD_EXTERNAL_SENSOR_SET_FANS || cmd == CMD_EXTERNAL_SENSOR_SET_TEMP) {
    for (uint8_t token_chunk_index = 0; token_chunk_index < token_chunks_count; token_chunk_index++) {
      const uint8_t tokens_count = token_chunks[token_chunk_index].count;
      if (tokens_count <= 1) {
        return ERR_INVALID_PARAMETERS;
      }

      const uint16_t *tokens = token_chunks[token_chunk_index].tokens;

      if (cmd == CMD_EXTERNAL_SENSOR_SET_LOAD) {
        const uint8_t max_number_of_sensors = (sizeof(ext_loads) / sizeof(ext_loads[0]));
        const uint8_t sensor_index = tokens[0] > 0 && tokens[0] <= max_number_of_sensors ? tokens[0] - 1 : 0;
        const uint8_t percentage = tokens[1] < 100 ? tokens[1] : 100;
        ext_loads[sensor_index] = tokens[1]; break;
      } else if (cmd == CMD_EXTERNAL_SENSOR_SET_FANS) {
        const uint8_t max_number_of_sensors = (sizeof(ext_fans) / sizeof(ext_fans[0]));
        const uint8_t sensor_index = tokens[0] > 0 && tokens[0] <= max_number_of_sensors ? tokens[0] - 1 : 0;
        ext_fans[sensor_index] = tokens[1]; break;
      } else if (cmd == CMD_EXTERNAL_SENSOR_SET_TEMP) {
        const uint8_t max_number_of_sensors = (sizeof(ext_temps) / sizeof(ext_temps[0]));
        const uint8_t sensor_index = tokens[0] > 0 && tokens[0] <= max_number_of_sensors ? tokens[0] - 1 : 0;
        ext_temps[sensor_index] = tokens[1]; break;
      }
    }

    return chars_count;
  }

  if (cmd == CMD_EXTERNAL_SENSOR_SET_NETWORK_STATUS || cmd == CMD_EXTERNAL_SENSOR_SET_ARRAY_STATUS) {
    const uint16_t *tokens = token_chunks[0].tokens;

    switch (cmd) {
      case CMD_EXTERNAL_SENSOR_SET_NETWORK_STATUS:
        _is_network_connected = tokens[0] < 2 ? tokens[0] : SENSOR_NOT_PRESENT;
        break;
      case CMD_EXTERNAL_SENSOR_SET_ARRAY_STATUS:
        _is_array_started = tokens[0] < 2 ? tokens[0] : SENSOR_NOT_PRESENT;
        break;
    }

    return chars_count;
  }

  if (cmd == CMD_DISK_SHUTDOWN_DELAY_SET) {
    const uint16_t *tokens = token_chunks[0].tokens;
    _disk_shutdown_delay_in_seconds = tokens[0] < 255 ? tokens[0] : 0;
    return chars_count;
  }

  if (cmd == CMD_DISK_TEMPERATURE_ALERT_SET) {
    const uint16_t *tokens = token_chunks[0].tokens;
    _disk_temperature_alert = tokens[0] < 255 ? tokens[0] : 0;
    return chars_count;
  }

  return ERR_UNKNOWN_COMMAND;
}

uint8_t extract_int_tokens(const char payload[], const uint8_t payload_length, const TokenChunk token_chunks[], uint8_t *token_chunks_count) {
  uint8_t tcc = 0;
  uint8_t i = 0;
  while (i < payload_length) {
    if (payload[i] == '\0') {
      break;
    }

    const uint8_t chars_count = parse_int_array(&payload[i], token_chunks[tcc].tokens, &token_chunks[tcc].count);
    if (chars_count) {
      tcc++;
      i += chars_count;
    } else {
      break;
    }

    if (payload[i] == '|' || payload[i] == ';') {
      i++;
      continue;
    }

    break;
  }

  *token_chunks_count = tcc;
  return i;
}

uint8_t parse_int_array(const char *p, const uint16_t *values, uint8_t *count) {
  uint8_t chars_count = 0;
  uint8_t token_count = 0;

  while (true) {
    const uint8_t len = parse_int(p, &values[token_count]);
    if (!len) {
      break;
    }

    token_count++;
    chars_count += len;
    p += len;

    if (*p == ',') {
      p++;
      chars_count++;
      continue;
    }

    break;
  }

  *count = token_count;
  return chars_count;
}

uint8_t parse_int(const char *p, uint16_t *value) {
  uint16_t result = 0;
  uint8_t chars_count = 0;
  *value = 0xffff;

  while (p[chars_count] >= '0' && p[chars_count] <= '9') {
    result = result * 10 + (p[chars_count] - '0');
    chars_count++;
  }

  *value = result;
  return chars_count;
}

void write_to_page (const uint8_t page_index, const uint8_t payload[], const uint8_t payload_length) {
  if (!display_page_texts[page_index]) {
    display_page_texts[page_index] = malloc(DISPLAY_WIDTH * DISPLAY_HEIGHT);
  }

  char *dst = display_page_texts[page_index];

  uint8_t write_counter = 0;
  for (uint8_t i = 0; i < payload_length; i++) {
    const char c = payload[i];
    if (c == '\0' || c == '\r' || c == '\n') {
      break;
    }

    *dst++ = payload[i];
    
    if (++write_counter == DISPLAY_WIDTH * DISPLAY_HEIGHT) {
      break;
    }
  }

  for (uint8_t i = write_counter; i < DISPLAY_WIDTH * DISPLAY_HEIGHT; i++) {
    *dst++ = '\0';
  }
}

void fan_pwm_apply_updated_values() {
  uint8_t fan_duty_cycle[] = { 0, 0 };

  for (uint8_t fan_index = 0; fan_index < sizeof(fans) / sizeof(fans[0]); fan_index++) {
    if (fans[fan_index].use_hysteresys_curve) {
      const int8_t current_temp = temps[fan_index].current_temp;
      if (current_temp > 0) {
        if (current_temp <= 10) {
          fans[fan_index].req_duty_percentage = fans[fan_index].hysteresys_curve[0];
        } else if (current_temp >= 90) {
          fans[fan_index].req_duty_percentage = 100;
        } else {
          uint8_t lower_index = (current_temp - 10) / 10;
          uint8_t upper_index = lower_index + 1;
          int8_t temp_low = (lower_index * 10) + 10;

          int8_t pwm_low = fans[fan_index].hysteresys_curve[lower_index];
          int8_t pwm_high = upper_index < 8 ? fans[fan_index].hysteresys_curve[upper_index] : 100;

          if (pwm_low < 0) {
            if (pwm_high < 0) {
              fans[fan_index].req_duty_percentage = pwm_low;
            }

            pwm_low = 0;
          }

          fans[fan_index].req_duty_percentage = static_cast<int8_t>(pwm_low + ((pwm_high - pwm_low) * ((current_temp - temp_low) / 10.0f)));
        }
      }
    }

    const uint8_t target_pwm = fans[fan_index].req_duty_percentage < 0
                                 ? 0
                                 : ((fans[fan_index].max_duty_cycle - fans[fan_index].min_duty_cycle) * (fans[fan_index].req_duty_percentage / 100.0F)) + fans[fan_index].min_duty_cycle;

    fan_duty_cycle[fan_index] = target_pwm;
  }

  pwm_start(_fan_carrier_freq, fan_duty_cycle[0], fan_duty_cycle[1]);
}

void pwm_start(uint32_t freq, uint8_t duty1, uint8_t duty2) {
  // Set digital pin 9 (D9) to an output
  pinMode(FAN_PWM_PIN1, OUTPUT);

  // Set digital pin 10 (D10) to an output
  pinMode(FAN_PWM_PIN2, OUTPUT);

  // Table 14-4. Waveform Generation Mode Bit Description
  // Mode:14 - 1 1 1 0 - Fast PWM, ICR, TOP, TOP
  TCCR1A = (1 << WGM11) | (0 << WGM10);
  TCCR1B = (1 << WGM13) | (1 << WGM12);

  // Table 15-7. Compare Output Mode, Phase and Frequency Correct PWM Mode
  // COM4A1..0 = 0b10
  //   Cleared on Compare Match when up-counting.
  //   Set on Compare Match when down-counting.
  TCCR1A |= (1 << COM1A1) | (0 << COM1A0);
  TCCR1A |= (1 << COM1B1) | (0 << COM1B0);

  // Table 14-5. Prescaler Select Bit
  //CS12 CS11 CS10
  //  0    0    1 ..   /1 = 244.14Hz upto 8MHz
  //  0    1    0 ..   /8 =  30.52Hz upto 1MHz
  //  0    1    1 ..  /64 =   3.81Hz upto 125,000kHz
  //  1    0    0 .. /256 =   0.95Hz upto  31,250kHz
  // Set prescaler to 256
  TCCR1B |= ((1 << CS11));
  TCCR1B &= ~((1 << CS12) | (1 << CS10));

  const uint16_t counter_prescaler = 8;
  uint16_t counter_overflow = (F_CPU / freq / counter_prescaler) - 1;

  // Set base frequency (identical for both D9 and D10)
  ICR1 = counter_overflow;

  // Set duty-cycle on D9
  OCR1A = (counter_overflow + 1) * (duty1 < 255 ? ((float)duty1 / 255.0F) : 1);

  // Set duty-cycle on D10
  OCR1B = (counter_overflow + 1) * (duty2 < 255 ? ((float)duty2 / 255.0F) : 1);
}

void pwm_stop() {
  pinMode(FAN_PWM_PIN1, INPUT);
  pinMode(FAN_PWM_PIN2, INPUT);

  // Clear Timer1 Control Register A & B
  TCCR1A = 0;
  TCCR1B = 0;
  ICR1 = 0;
  OCR1A = 0;
}

void fan_pin_interrupt_enable() {
  pinMode(FAN_INT_PIN1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FAN_INT_PIN1), onFan1PinInterrupt, RISING);

  pinMode(FAN_INT_PIN2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FAN_INT_PIN2), onFan2PinInterrupt, RISING);

  for (uint8_t fan_index = 0; fan_index < sizeof(fans) / sizeof(fans[0]); fan_index++) {
    init_fan_capture(fans[fan_index]);
  }

  // Timer4 setup for 1 second interrupt
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4C = 0;
  TCCR4D = 0;
  TCCR4E = 0;

  // Set prescaler to 16384: CS43:CS40 = 1111
  TCCR4B = (1 << CS43) | (1 << CS42) | (1 << CS41) | (1 << CS40);

  // Set TOP for overflow at FAN_RPM_CAPTURES_PER_SECOND Hz
  uint16_t overflow_value = (F_CPU / 16384) / FAN_RPM_CAPTURES_PER_SECOND;
  TC4H = overflow_value >> 8;     // top 2 bits
  OCR4C = overflow_value & 0xFF;  // bottom 8 bits

  // Enable overflow interrupt
  TIMSK4 = (1 << TOIE4);
}

void onFan1PinInterrupt() {
  fans[0].rpm_counter++;
}

void onFan2PinInterrupt() {
  fans[1].rpm_counter++;
}

ISR(TIMER4_OVF_vect) {
  handleFanCapture(fans[0]);
  handleFanCapture(fans[1]);
}

inline void handleFanCapture(FanCapture &fan) {
  const uint8_t capture_buffer_write_index = fan.capture_buffer_index & 0b00000111;
  fan.capture_buffer[capture_buffer_write_index] = fan.rpm_counter;
  fan.capture_buffer_index++;
}

void fans_rpm_update() {
  for (uint8_t fan_index = 0; fan_index < sizeof(fans) / sizeof(fans[0]); fan_index++) {
    const uint16_t fan_rpm = calculate_fan_rpm(fans[fan_index]);
    fans[fan_index].current_rpm = fan_rpm;

    if (fans[fan_index].alert_rpm && fan_rpm < fans[fan_index].alert_rpm) {
      fans[fan_index].alert_trigger++;
      if (fans[fan_index].alert_trigger > fans[fan_index].alert_threshold) {
        fans[fan_index].alert_trigger = fans[fan_index].alert_threshold;
      }
    } else {
      fans[fan_index].alert_trigger = 0;
    }
  }
}

uint16_t calculate_fan_rpm(FanCapture &fan) {
  const uint8_t capture_buffer_size = sizeof(fan.capture_buffer) / sizeof(fan.capture_buffer[0]);
  uint32_t rpm_sum = 0;

  for (uint8_t capture_buffer_index = 0; capture_buffer_index < capture_buffer_size - 1; capture_buffer_index++) {
    if (fan.capture_buffer[capture_buffer_index + 1] >= fan.capture_buffer[capture_buffer_index]) {
      rpm_sum += fan.capture_buffer[capture_buffer_index + 1] - fan.capture_buffer[capture_buffer_index];
    }
  }

  const float avg_rpm = rpm_sum / (float)capture_buffer_size;

  // 2 pulses per revolution
  const float rpm = avg_rpm * FAN_RPM_CAPTURES_PER_SECOND * (60.0F / 2.0F);

  return static_cast<uint16_t>(rpm);
}

void temp_sensors_enable() {
  tempSensors.begin();

  const uint8_t count = tempSensors.getDeviceCount();

  for (uint8_t temp_index = 0; temp_index < sizeof(temps) / sizeof(temps[0]); temp_index++) {
    temps[temp_index].correction = 0;
    temps[temp_index].alert_temp = 0;
    temps[temp_index].alert_trigger = 0;
    temps[temp_index].alert_threshold = 4;

    if (temp_index < count && tempSensors.getAddress(temps[temp_index].address, temp_index)) {
      continue;
    }

    temps[temp_index].address[0] = 0;
    temps[temp_index].current_temp = SENSOR_NOT_PRESENT;
  }

  tempSensors.setWaitForConversion(false);
}

void temp_sensors_update() {
  tempSensors.requestTemperatures();

  for (uint8_t temp_index = 0; temp_index < sizeof(temps) / sizeof(temps[0]); temp_index++) {
    if (temps[temp_index].address[0] == 0) {
      temps[temp_index].current_temp = SENSOR_NOT_PRESENT;
      temps[temp_index].alert_trigger = 0;
      continue;
    }

    const float tempC = tempSensors.getTempC(temps[temp_index].address);
    if (tempC == DEVICE_DISCONNECTED_C) {
      temps[temp_index].current_temp = SENSOR_NOT_PRESENT;
      temps[temp_index].alert_trigger++;
      continue;
    }

    temps[temp_index].current_temp = static_cast<int8_t>(tempC) + temps[temp_index].correction;
    if (temps[temp_index].current_temp >= temps[temp_index].alert_temp) {
      temps[temp_index].alert_trigger++;
    } else {
      temps[temp_index].alert_trigger = 0;
    }
  }
}

void process_hdd_power_button(unsigned long elapsed_time) {
  if (!debounce_button(HDD_POWER_BUTTON_PIN, LOW, &_disk_power_button_debouncer)) {
    return;
  }

  if (_disk_power_last_toggle > 0) {
    const unsigned long ms_since_last_toggle = elapsed_time - _disk_power_last_toggle;
    if (ms_since_last_toggle < 500) {
      return;
    }

    if (ms_since_last_toggle < HDD_POWER_TOGGLE_COOLING_PERIOD_IN_MILLISECONDS - 1000) {
      const uint16_t seconds_left = (HDD_POWER_TOGGLE_COOLING_PERIOD_IN_MILLISECONDS - ms_since_last_toggle) / 1000;
      char line[] = "wait     sec. to";
      itoar(seconds_left, &line[7]);
      display_alert(line, "toggle HDD power");
      return;
    }
  }

  if (!_disk_power_state) {
    display_alert("HDD power ON", "\xFF\xFF");
    _disk_power_state = true;
  } else {
    display_alert("HDD power OFF", "\x01\x05");
    _disk_power_state = false;
  }

  _disk_power_last_toggle = elapsed_time;
}

void process_display_button() {
  const bool is_right_button_pressed = debounce_button(DISPLAY_BUTTON_RIGHT_PIN, LOW, &_display_button_right_debouncer);
  const bool is_left_button_pressed = debounce_button(DISPLAY_BUTTON_LEFT_PIN, LOW, &_display_button_left_debouncer);

  if (!is_right_button_pressed && !is_left_button_pressed) {
    return;
  }

  if (_wait_status == WAIT_STATUS_SLEEP) {
    const MelodyNote play_notes[] = { { 300, 50 }, { 600, 50 }, { 300, 50 } };
    melody_play(play_notes, sizeof(play_notes) / sizeof(play_notes[0]));
    return;
  }

  const MelodyNote play_notes[] = { { 1200, 25 } };
  melody_play(play_notes, sizeof(play_notes) / sizeof(play_notes[0]));

  uint8_t logical_page_index = 0;

  const uint8_t page_map_length = sizeof(display_page_map) / sizeof(display_page_map[0]);
  for (uint8_t i = 0; i < page_map_length; i++) {
    if (display_page_map[i] == _display_page_current) {
      logical_page_index = i;
      break;
    }
  }

  if (is_right_button_pressed) {
    logical_page_index++;
  } else if (is_left_button_pressed) {
    logical_page_index--;
  }

  for (uint8_t i = 0; i < 10; i++) {
    if (logical_page_index >= page_map_length) {
      logical_page_index = 1;
      continue;
    }
    
    uint8_t physical_page_index = display_page_map[logical_page_index];
    if (physical_page_index >= DISPLAY_PAGE_TYPE__DO_NOT_REMOVE) {
      logical_page_index = 1;
      continue;
    }

    if (physical_page_index == 0) {
      logical_page_index++;
      continue;
    }

    if (physical_page_index <= DISPLAY_PAGE_TYPE_CUSTOM_PAGE_LAST && !display_page_texts[physical_page_index]) {
      logical_page_index++;
      continue;
    }

    break;
  }

  _display_button_screen_page_hold_end_time = millis() + 2500;
  _display_page_current = display_page_map[logical_page_index];
  display_page(_display_page_current);
}

bool debounce_button(const uint8_t button_pin, const uint8_t expected_active_state, uint8_t * debouncer) {
  static const int button_debouncer_threshold = 5;
  
  const uint8_t button_state = digitalRead(button_pin);
  if (button_state != expected_active_state) {
    (*debouncer) = 0;
    return false;
  }

  // Check if button press was already detected and now wait for button release 
  if ((*debouncer) > button_debouncer_threshold) {
    return false;
  }

  // Wait some time for button to be pressed to suppress contact bouncing
  if ((*debouncer) < button_debouncer_threshold) {
    (*debouncer)++;
    return false;
  }

  // Indicate that button press was detected
  (*debouncer) = 0xFF;

  return true;
}

void melody_play(MelodyNote *melody_notes, uint8_t count) {
  if (_melody_is_playing_error) {
    return;
  }

  for (uint8_t i = 0; i < count; i++) {
    _melody_notes[i].frequency = melody_notes[i].frequency;
    _melody_notes[i].length = melody_notes[i].length;

    if (melody_notes[i].length == 0) {
      count = i;
      break;
    }
  }

  _melody_notes[count].frequency = 0;
  _melody_notes[count].length = 0;

  _melody_start_time = millis();
  _melody_note_playing_index = -1;
}

void melody_play() {
  if (!_melody_start_time) {
    return;
  }

  const unsigned long elapsed_time = millis();
  uint16_t play_time = elapsed_time - _melody_start_time;
  uint16_t prev_note_end_play_time = 0;

  for (uint8_t i = 0; i < sizeof(_melody_notes) / sizeof(_melody_notes[0]); i++) {
    if (play_time < prev_note_end_play_time) {
      return;
    }

    if (_melody_notes[i].length == 0) {
      noTone(BUZZER_PIN);
      _melody_start_time = 0;
      _melody_note_playing_index = -1;
      _melody_is_playing = false;
      _melody_is_playing_error = false;
      return;
    }

    const uint16_t this_note_end_play_time = prev_note_end_play_time + _melody_notes[i].length;
    if (play_time >= prev_note_end_play_time) {
      if (i == 0 && _melody_note_playing_index < 0) {
        _melody_start_time = millis();
        _melody_is_playing = true;
        play_time = 0;
      }

      if (i > _melody_note_playing_index) {
        const uint16_t frequency = _melody_notes[i].frequency;
        if (frequency > 10) {
          tone(BUZZER_PIN, _melody_notes[i].frequency, _melody_notes[i].length);
        } else {
          noTone(BUZZER_PIN);
        }

        _melody_note_playing_index = i;
      } else {
        prev_note_end_play_time = this_note_end_play_time;
        continue;
      }

      return;
    }

    prev_note_end_play_time = this_note_end_play_time;
  }
}

void play_error_sound(int8_t error_code) {
  const uint16_t none_delay = 100;
  const uint16_t some_delay = 250;
  const uint16_t beep_delay = 100;
  const uint16_t beep_freq = 1200;

  switch (error_code) {
    case ERR_UNKNOWN_COMMAND:
      {
        const MelodyNote play_notes[] = { { 1200, 50 }, { 800, 50 }, { 0, some_delay }, { beep_freq, beep_delay } };
        melody_play(play_notes, sizeof(play_notes) / sizeof(play_notes[0]));
        break;
      }
    case ERR_NO_COMMAND_PARAMETERS:
      {
        const MelodyNote play_notes[] = { { 1200, 50 }, { 800, 50 }, { 0, some_delay }, { beep_freq, beep_delay }, { 0, none_delay }, { beep_freq, beep_delay } };
        melody_play(play_notes, sizeof(play_notes) / sizeof(play_notes[0]));
        break;
      }
    case ERR_INVALID_PARAMETERS:
      {
        const MelodyNote play_notes[] = { { 1200, 50 }, { 800, 50 }, { 0, some_delay }, { beep_freq, beep_delay }, { 0, none_delay }, { beep_freq, beep_delay }, { 0, none_delay }, { beep_freq, beep_delay } };
        melody_play(play_notes, sizeof(play_notes) / sizeof(play_notes[0]));
        break;
      }
    default:
      {
        const MelodyNote play_notes[] = { { 1200, 50 }, { 800, 50 }, { 1200, 50 }, { 800, 50 }, { 1200, 50 }, { 800, 50 } };
        melody_play(play_notes, sizeof(play_notes) / sizeof(play_notes[0]));
        break;
      }
  }

  _melody_is_playing_error = true;
}

void display_page(const uint8_t index) {
  if (_wait_status || _is_showing_alert || _is_showing_info) {
    return;
  }

  if (index <= DISPLAY_PAGE_TYPE_CUSTOM_PAGE_LAST) {
    if (!display_page_texts[index]) {
      char header_text[] = "*** CUSTOM # ***";
      header_text[11] = index + '0';

      lcd.send_line(0, header_text);
      lcd.send_line(1, "  TEXT NOT SET  ");
      return;
    }

    for (uint8_t line_index = 0; line_index < DISPLAY_HEIGHT; line_index++) {
      lcd.send_line(line_index, &display_page_texts[index][line_index * DISPLAY_WIDTH]);
    }

    return;
  }

  switch (index) {
    case DISPLAY_PAGE_TYPE_COOLING_HDD: display_page_sensor("HDD cage cooling", fans[INT_FAN_SENSOR_INDEX_HDD].current_rpm, temps[INT_TEMP_SENSOR_INDEX_HDD].current_temp); return;
    case DISPLAY_PAGE_TYPE_COOLING_LSI: display_page_sensor("LSI card cooling", fans[INT_FAN_SENSOR_INDEX_LSI].current_rpm, temps[INT_TEMP_SENSOR_INDEX_LSI].current_temp); return;
    case DISPLAY_PAGE_TYPE_COOLING_CPU: display_page_sensor("CPU slot cooling", ext_fans[EXT_FAN_SENSOR_INDEX_CPU], ext_temps[EXT_TEMP_SENSOR_INDEX_CPU]); return;
    case DISPLAY_PAGE_TYPE_COOLING_MOTHERBOARD: display_page_sensor("Mother/b cooling", ext_fans[EXT_FAN_SENSOR_INDEX_Motherboard], ext_temps[EXT_TEMP_SENSOR_INDEX_Motherboard]); return;
    case DISPLAY_PAGE_TYPE_COOLING_ALL: display_page_all_temps(); return;
    case DISPLAY_PAGE_TYPE_COOLING_ALL_DISKS: display_page_all_disk_temps(); return;
    case DISPLAY_PAGE_TYPE_LOAD_CPU: display_page_load("CPU load", ext_loads[EXT_LOAD_SENSOR_INDEX_CPU]); return;
    case DISPLAY_PAGE_TYPE_LOAD_RAM: display_page_load("RAM alloc", ext_loads[EXT_LOAD_SENSOR_INDEX_RAM]); return;
    case DISPLAY_PAGE_TYPE_LOAD_ALL_DISKS: display_page_all_disk_loads(); return;
    case DISPLAY_PAGE_TYPE_UPTIME: display_page_uptime(); return;
    default: return;
  }
}

void display_page_sensor(const char *title, const int16_t fan_rpm, const int8_t temp) {
  char line2[17] = "     RPM      \xDF";

  if (fan_rpm >= 0) {
    itoar(fan_rpm, &line2[3]);
  } else {
    line2[0] = '/';
    line2[1] = '/';
    line2[2] = '/';
    line2[3] = '/';
  }

  write_temperature(&line2[12], temp, 4);
  line2[15] = _use_fahrenheit_temp ? 'F' : 'C';

  lcd.send_line(0, title);
  lcd.send_line(1, line2);
}

void display_page_all_temps() {
  char line1[] = "CPU:  \x06  HDD   \x06";
  char line2[] = "M/B:  \x06  LSI   \x06";

  if (_use_fahrenheit_temp) {
    line1[7] = '\x07';
    line2[7] = '\x07';
    line1[8] = 'H';
    line2[8] = 'L';
    line1[9] = 'D';
    line2[9] = 'S';
    line1[10] = 'D';
    line2[10] = 'I';
    line1[11] = ':';
    line2[11] = ':';
    line1[15] = '\x07';
    line2[15] = '\x07';
  } else {
    line1[12] = ':';
    line2[12] = ':';
  }

  const uint8_t number_of_digits = _use_fahrenheit_temp ? 3 : 2;

  write_temperature(&line1[_use_fahrenheit_temp ? 6 : 5], ext_temps[EXT_TEMP_SENSOR_INDEX_CPU], number_of_digits);
  write_temperature(&line2[_use_fahrenheit_temp ? 6 : 5], ext_temps[EXT_TEMP_SENSOR_INDEX_Motherboard], number_of_digits);
  write_temperature(&line1[14], temps[0].current_temp, number_of_digits);
  write_temperature(&line2[14], temps[1].current_temp, number_of_digits);

  lcd.send_line(0, line1);
  lcd.send_line(1, line2);
}

void display_page_all_disk_temps() {
  char line1[] = "HDD1  #2  #3  #4";
  char line2[] = "                ";

  const uint8_t number_of_digits = _use_fahrenheit_temp ? 3 : 2;

  for (uint8_t disk_index = 0; disk_index <= EXT_TEMP_SENSOR_INDEX_HDD_LAST - EXT_TEMP_SENSOR_INDEX_HDD1; disk_index++) {
    const uint8_t char_offset = (disk_index * 4);
    line2[char_offset + 3] = _use_fahrenheit_temp ? '\x07' : '\x06';

    if (ext_temps[disk_index + EXT_TEMP_SENSOR_INDEX_HDD1] == SENSOR_NOT_PRESENT) {
      line2[char_offset + 1] = '-';
      line2[char_offset + 2] = '-';
      continue;
    }

    write_temperature(&line2[char_offset + 2], ext_temps[disk_index + EXT_TEMP_SENSOR_INDEX_HDD1], number_of_digits);
  }

  lcd.send_line(0, line1);
  lcd.send_line(1, line2);
}

void display_page_load(const char *caption, const int8_t percentage) {
  char line1[] = "               %";
  char line2[] = "\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x05";

  strcopy(line1, caption, DISPLAY_WIDTH);

  if (percentage >= 0) {
    itoar(percentage, &line1[13]);

    if (percentage > 0) {
      const float percentage_per_one_bar = 100.0F / 16.0F;
      const uint8_t full_bars = percentage / percentage_per_one_bar;
      for (uint8_t i = 0; i < full_bars; i++) {
        line2[i] = '\xFF';
      }

      const uint8_t remainder = percentage - (full_bars * percentage_per_one_bar);

      if (remainder > 0) {
        line2[full_bars] = remainder >= 5 ? '\xFF' : remainder;
      }
    }
  } else {
    line1[11] = '/';
    line1[12] = '/';
    line1[13] = '/';
  }

  lcd.send_line(0, line1);

  lcd.setCursor(0, 1);
  for (uint8_t i = 0; i < 16; i++) {
    lcd.data(line2[i]);
  }
}

void display_page_all_disk_loads() {
  char line1[] = "HDD1  #2  #3  #4";
  char line2[] = "                ";

  for (uint8_t disk_index = 0; disk_index <= EXT_TEMP_SENSOR_INDEX_HDD_LAST - EXT_TEMP_SENSOR_INDEX_HDD1; disk_index++) {
    const uint8_t char_offset = (disk_index * 4);
    line2[char_offset + 3] = '%';

    if (ext_temps[disk_index + EXT_LOAD_SENSOR_INDEX_HDD1] == SENSOR_NOT_PRESENT) {
      line2[char_offset + 1] = '-';
      line2[char_offset + 2] = '-';
      continue;
    }

    itoar(&line2[char_offset + 2], ext_loads[disk_index + EXT_LOAD_SENSOR_INDEX_HDD1]);
  }

  lcd.send_line(0, line1);
  lcd.send_line(1, line2);
}

void display_page_uptime() {
  char line1[] = "  POWER UPTIME  ";
  char line2[] = "00 days 00:00:00";

  const unsigned long totalSeconds = millis() / 1000;

  const uint8_t days = totalSeconds / 86400;
  const uint8_t hours = (totalSeconds % 86400) / 3600;
  const uint8_t minutes = (totalSeconds % 3600) / 60;
  const uint8_t seconds = totalSeconds % 60;

  itoar(days, &line2[1]);
  itoar(hours, &line2[9]);
  itoar(minutes, &line2[12]);
  itoar(seconds, &line2[15]);

  lcd.send_line(0, line1);
  lcd.send_line(1, line2);
}

void display_alert(const char *s1, const char *s2) {
  display_text_centered(s1, s2);
  _is_showing_alert = true;
  _is_showing_info = !_is_showing_alert;
  _display_alert_end_time = millis() + DISPLAY_ALERT_DURATION_IN_MILLISECONDS;

  // Play alert sound
  const MelodyNote play_notes[] = { { _alert_sound_freq, _alert_sound_duration } };
  melody_play(play_notes, sizeof(play_notes) / sizeof(play_notes[0]));

  // Turn RGB LED Red
  const uint16_t blink_period = DISPLAY_ALERT_DURATION_IN_MILLISECONDS / 5;
  RgbColor rgb_color[] = { { 255, 0, 0, blink_period },  { 0, 0, 0, blink_period } };
  rgb_led_set_colors(rgb_color, sizeof(rgb_color) / sizeof(rgb_color[1]), false);
}

void display_message(const char *s1, const char *s2) {
  display_text_centered(s1, s2);
  _is_showing_info = true;
  _is_showing_alert = !_is_showing_info;
  _display_info_end_time = millis() + DISPLAY_INFO_DURATION_IN_MILLISECONDS;
}

void display_text_centered(const char *s1, const char *s2) {
  char line1[] = "                ";
  char line2[] = "                ";

  uint8_t sl1 = strl(s1);
  uint8_t offset1 = sl1 > (DISPLAY_WIDTH - 2) ? 0 : ((DISPLAY_WIDTH - sl1) / 2);
  uint8_t sl1p = (sl1 < DISPLAY_WIDTH) ? sl1 : DISPLAY_WIDTH;
  for (uint8_t i = 0; i < sl1p; i++) {
    line1[i + offset1] = s1[i];
  }

  if (s2) {
    uint8_t sl2 = strl(s2);
    uint8_t offset2 = sl2 > (DISPLAY_WIDTH - 2) ? 0 : ((DISPLAY_WIDTH - sl2) / 2);
    uint8_t sl2p = (sl2 < DISPLAY_WIDTH) ? sl2 : DISPLAY_WIDTH;
    for (uint8_t i = 0; i < sl2p; i++) {
      line2[i + offset2] = s2[i];
    }
  } else if (sl1 > DISPLAY_WIDTH) {
    uint8_t sl2 = sl1 - DISPLAY_WIDTH;
    uint8_t offset2 = sl2 > (DISPLAY_WIDTH - 2) ? 0 : ((DISPLAY_WIDTH - sl2) / 2);
    uint8_t sl2p = (sl2 < DISPLAY_WIDTH) ? sl2 : DISPLAY_WIDTH;
    for (uint8_t i = 0; i < sl2p; i++) {
      line2[i + offset2] = s1[i + DISPLAY_WIDTH];
    }
  }
  
  lcd.send_line(0, line1);
  lcd.send_line(1, line2);
}

void display_wait_screen() {
  if (!_wait_status || _is_showing_alert || _is_showing_info) {
    return;
  }

  if (_wait_status == WAIT_STATUS_SLEEP) {
    lcd.send_line(0, "   SLEEP MODE   ");
    lcd.send_line(1, "POWER to wake-up");
    return;
  }

  const unsigned long elapsed_time = millis();

  char line1[] = "Booting...      ";
  char line2[] = "                ";

  if (_wait_status == WAIT_STATUS_SLEEP_PHASE1) {
    if (_disk_shutdown_delay_end_time) {
      if (_disk_shutdown_delay_end_time > elapsed_time) {
        const unsigned long seconds_to_hdd_power_off = (_disk_shutdown_delay_end_time - elapsed_time) / 1000;

        if (seconds_to_hdd_power_off > 99) {
          strcopy(&line2[1], "in a few minutes", DISPLAY_WIDTH);
        } else {
          strcopy(&line2[3], "in 00 sec.", DISPLAY_WIDTH);
          itoar(seconds_to_hdd_power_off, &line2[7]);
        }
      } else {
        strcopy(&line2[6], "NOW!", DISPLAY_WIDTH);
      }

      lcd.send_line(0, "HDD powering OFF");
      lcd.send_line(1, line2);
    }

    return;
  }

  const uint16_t elapsed_seconds_in_current_status = (elapsed_time - _last_status_change_time) / 1000;

  if ((_wait_status & WAIT_STATUS_MASK_REBOOT) == WAIT_STATUS_MASK_REBOOT) {
    strcopy(line1, "Rebooting...", DISPLAY_WIDTH);
  } else if ((_wait_status & WAIT_STATUS_MASK_SHUTDOWN) == WAIT_STATUS_MASK_SHUTDOWN) {
    strcopy(line1, "Shutdown...", DISPLAY_WIDTH);
  }

  itoar(elapsed_seconds_in_current_status, &line1[15]);

  if (_wait_status == WAIT_STATUS_BOOT_PHASE1) {
    strcopy(line2, "Loading Kernel", DISPLAY_WIDTH);
  } else if (_wait_status == WAIT_STATUS_BOOT_PHASE2) {
    strcopy(line2, "Loading System", DISPLAY_WIDTH);
  } else if (_wait_status == WAIT_STATUS_BOOT_PHASEN || _wait_status == WAIT_STATUS_REBOOT_PHASE1 || _wait_status == WAIT_STATUS_SHUTDOWN_PHASE1) {
    if (display_page_texts[0] && display_page_texts[0][0]) {
      strcopy(line2, &display_page_texts[0][0], DISPLAY_WIDTH);
    } else {
      strcopy(line2, "Getting Ready", DISPLAY_WIDTH);
    }
  } else if (_wait_status == WAIT_STATUS_REBOOT_PHASE2) {
    strcopy(line2, "Reloading System", DISPLAY_WIDTH);
  } else if (_wait_status == WAIT_STATUS_SHUTDOWN_PHASE2) {
    strcopy(line2, "Bye-bye", DISPLAY_WIDTH);
  }

  lcd.send_line(0, line1);
  lcd.send_line(1, line2);
}

void change_wait_status(WaitStatus new_status) {
  const uint8_t old_status = _wait_status;

  if (new_status == old_status) {
    return;
  }

  _wait_status = new_status;

  if (new_status == WAIT_STATUS_UNDEFINED) {
    _last_status_change_time = 0;
  } else {
    if (old_status == WAIT_STATUS_UNDEFINED || (new_status ^ old_status) < 7) {
      _last_status_change_time = millis();
    }
  }
  
  if (display_page_texts[0]) {
    display_page_texts[0][0] = '\0';
  }

  if (new_status != WAIT_STATUS_UNDEFINED) {
    display_wait_screen();
  }

  switch (new_status) {
    case WAIT_STATUS_BOOT_PHASE1:
      {
        RgbColor rgb_color[] = { { 255, 255, 150, 800 } };
        rgb_led_set_colors(rgb_color, sizeof(rgb_color) / sizeof(rgb_color[1]), true);
        break;
      }
    case WAIT_STATUS_BOOT_PHASE2:
      {
        RgbColor rgb_color[] = { { 0, 255, 0, 400 } };
        rgb_led_set_colors(rgb_color, sizeof(rgb_color) / sizeof(rgb_color[1]), true);
        break;
      }
    case WAIT_STATUS_BOOT_PHASEN:
      {
        RgbColor rgb_color[] = { { 0, 255, 0, 150 } };
        rgb_led_set_colors(rgb_color, sizeof(rgb_color) / sizeof(rgb_color[1]), true);
        break;
      }
    case WAIT_STATUS_SHUTDOWN_PHASE1:
    case WAIT_STATUS_REBOOT_PHASE1:
      {
        RgbColor rgb_color[] = { { 255, 70, 0, 400 } };
        rgb_led_set_colors(rgb_color, sizeof(rgb_color) / sizeof(rgb_color[1]), true);
        break;
      }
    case WAIT_STATUS_REBOOT_PHASE2:
    case WAIT_STATUS_SLEEP_PHASE1:
      {
        RgbColor rgb_color[] = { { 255, 70, 0, 150 } };
        rgb_led_set_colors(rgb_color, sizeof(rgb_color) / sizeof(rgb_color[1]), true);
        break;
      }
    case WAIT_STATUS_SHUTDOWN_PHASE2:
    case WAIT_STATUS_SLEEP_PHASE2:
      {
        RgbColor rgb_color[] = { { 255, 70, 0, 0 } };
        rgb_led_set_colors(rgb_color, sizeof(rgb_color) / sizeof(rgb_color[1]), false);
        break;
      }
    case WAIT_STATUS_UNDEFINED:
      {
        if (old_status != WAIT_STATUS_UNDEFINED) {
          rgb_led_restore_color();
        }
        break;
      }  
    case WAIT_STATUS_SLEEP:
      {
        rgb_led_set_colors(NULL, 0, false);
        break;
      }
    default:
      {
        rgb_led_set_colors(NULL, 0, false);
        break;
      }
  }
}

void init_display_pages() {
  const uint8_t custom_pages_count = sizeof(display_page_texts) / sizeof(display_page_texts[0]);
  const uint8_t page_map_length = sizeof(display_page_map) / sizeof(display_page_map[0]);

  for (uint8_t i = 0; i < custom_pages_count; i++) {
    display_page_texts[i] = NULL;
  }

  for (uint8_t i = 0; i < page_map_length; i++) {
    display_page_map[i] = i;
  }
}

void enter_sleep_mode() {
  // TODO: check 12v/5v power supply to HDD to see if disks are already down
  // if already down, then enter WAIT_STATUS_SLEEP_PHASE2 immediately

  if (_disk_shutdown_delay_in_seconds > 0) {
    _disk_shutdown_delay_end_time = millis() + ((unsigned long)_disk_shutdown_delay_in_seconds * 1000);
    change_wait_status(WAIT_STATUS_SLEEP_PHASE1);
  } else {
    _disk_shutdown_delay_end_time = 0;
     change_wait_status(WAIT_STATUS_SLEEP_PHASE2);
  }
}

void exit_sleep_mode() {
  _disk_shutdown_delay_end_time = 0;
  change_wait_status(WAIT_STATUS_UNDEFINED);
  fade_in_lcd_backlight();
}

void hdd_power_off() {
  if (!_disk_shutdown_delay_end_time) {
    return;
  }

  _disk_shutdown_delay_end_time = 0;

  display_message("Powering OFF", "all HDDs now");
  // TODO: change HDD power pin here
}

void process_status_changes() {
  bool status_toggle_0_1 = false;
  bool status_toggle_1_0 = false;

  if (_is_network_connected != _was_network_connected) {
    if (_was_network_connected > 0 && _is_network_connected == 0) {
      display_message("Network", "connection lost");
      status_toggle_1_0 = true;
    } else if (_was_network_connected <= 0 && _is_network_connected > 0) {
      display_message("Network", "connected");
      status_toggle_0_1 = true;
    }

    _was_network_connected = _is_network_connected;
  }

  if (_is_array_started != _was_array_started) {
    if (_was_array_started > 0 && _is_array_started == 0) {
      display_message("Array", "stopped");
      status_toggle_1_0 = true;
    } else if (_was_array_started <= 0 && _is_array_started > 0) {
      display_message("Array", "started");
      status_toggle_0_1 = true;
    }

    _was_array_started = _is_array_started;
  }

  if (status_toggle_1_0) {
    const MelodyNote play_notes[] = { { 1400, 300 }, { 800, 300 }, { 600, 300 }, { 400, 300 } };
    melody_play(play_notes, sizeof(play_notes) / sizeof(play_notes[0]));
  } else if (status_toggle_0_1) {
    const MelodyNote play_notes[] = { { 400, 150 }, { 600, 150 }, { 800, 150 } };
    melody_play(play_notes, sizeof(play_notes) / sizeof(play_notes[0]));
  }
}

void fade_out_lcd_backlight() {
  _lcd_brightness_target = 0;
}

void fade_in_lcd_backlight() {
  _lcd_brightness_target = _lcd_brightness_pwm;
}

void fade_lcd_update() {
  if (_lcd_brightness_current > _lcd_brightness_target) {
    if (_lcd_brightness_current > _lcd_brightness_target + _lcd_fade_speed_increment) {
      _lcd_brightness_current -= _lcd_fade_speed_increment;
    } else {
      _lcd_brightness_current = _lcd_brightness_target;
    }

    lcd.set_brightness(_lcd_brightness_current);

    return;
  }
  
  if (_lcd_brightness_current < _lcd_brightness_target) {
    if (_lcd_brightness_current < _lcd_brightness_target - _lcd_fade_speed_increment) {
      _lcd_brightness_current += _lcd_fade_speed_increment;
    } else {
      _lcd_brightness_current = _lcd_brightness_target;
    }

    lcd.set_brightness(_lcd_brightness_current);

    return;
  }
}

void rgb_led_restore_color() {
  if (_wait_status || _is_showing_alert) {
    return;
  }

  if (_rgb_led_colors_user_mode == RGB_LED_USER_MODE_NONE) {
    if (_is_network_connected) {
      if (_is_array_started) {
        RgbColor rgb_color[] = { { 0, 255, 0, 0 } };
        rgb_led_set_colors(rgb_color, sizeof(rgb_color) / sizeof(rgb_color[1]), false);
      } else {
        RgbColor rgb_color[] = { { 255, 70, 0, 0 } };
        rgb_led_set_colors(rgb_color, sizeof(rgb_color) / sizeof(rgb_color[1]), false);
      }
      return;
    } else {
      if (_is_array_started) {
        RgbColor rgb_color[] = { { 0, 255, 0, 1000 }, { 0, 0, 0, 100 }, { 0, 255, 0, 100 }, { 0, 0, 0, 100 }, { 0, 255, 0, 100 }, { 0, 0, 0, 500 } };
        rgb_led_set_colors(rgb_color, sizeof(rgb_color) / sizeof(rgb_color[1]), false);
      } else {
        RgbColor rgb_color[] = { { 255, 70, 0, 1000 }, { 0, 0, 0, 100 }, { 255, 70, 0, 100 }, { 0, 0, 0, 100 }, { 255, 70, 0, 100 }, { 0, 0, 0, 500 } };
        rgb_led_set_colors(rgb_color, sizeof(rgb_color) / sizeof(rgb_color[1]), false);
      }
      return;
    }
    return;
  }

  rgb_led_set_colors(_rgb_led_colors_user_set, sizeof(_rgb_led_colors_user_set) / sizeof(_rgb_led_colors_user_set[0]), _rgb_led_colors_user_mode == RGB_LED_USER_MODE_BREATHING_COLOR);
}

uint32_t compute_rgb_config_hash(const struct RgbColor* colors, size_t num_colors, bool breathing_mode) {
    uint32_t hash = 5381;
    hash = ((hash << 5) + hash) + (breathing_mode ? 1 : 0);
    
    for (uint8_t i = 0; i < num_colors; i++) {
      hash = ((hash << 5) + hash) + colors[i].r;
      hash = ((hash << 5) + hash) + colors[i].g;
      hash = ((hash << 5) + hash) + colors[i].b;
      hash = ((hash << 5) + hash) + (colors[i].length & 0xFF);
      hash = ((hash << 5) + hash) + (colors[i].length >> 8);
    }
    
    return hash;
}

void rgb_led_set_colors(RgbColor *colors, uint8_t count, bool breathing_mode) {
  if (colors && count > 0) {
    const uint32_t oldColorHash = compute_rgb_config_hash(_rgb_led_colors, count, RGB_LED_USER_MODE_NONE);
    const uint32_t newColorHash = compute_rgb_config_hash(colors, count, RGB_LED_USER_MODE_NONE);

    if (newColorHash == oldColorHash) {
      return;
    }
  }

  for (uint8_t i = 0; i < count; i++) {
    _rgb_led_colors[i].r = colors[i].r;
    _rgb_led_colors[i].g = colors[i].g;
    _rgb_led_colors[i].b = colors[i].b;
    _rgb_led_colors[i].length = colors[i].length;
  }

  for (uint8_t i = count; i < sizeof(_rgb_led_colors) / sizeof(_rgb_led_colors[0]); i++) {
    _rgb_led_colors[i].r = 0;
    _rgb_led_colors[i].g = 0;
    _rgb_led_colors[i].b = 0;
    _rgb_led_colors[i].length = 0;
  }

  _rgb_led_toggle_counter = 0;
  _rgb_led_next_toggle_time = 1;

  if (!breathing_mode || !colors || count == 0) {
    _rgb_led_breath_direction = 0;
    return;
  }

  float divider = (float)_rgb_led_colors[0].length / (float)RGB_LED_BREATH_INCREMENT_IN_MILLISECONDS;
  if (divider > 64) { divider = 64; }
  if (divider < 4) { divider = 4; }
  float r, g, b;
  for (uint8_t i = 0; i < 10; i++) {
    r = (float)_rgb_led_colors[0].r / divider;
    g = (float)_rgb_led_colors[0].g / divider;
    b = (float)_rgb_led_colors[0].b / divider;
    if (_rgb_led_colors[0].r != 0 && r < 1) { divider -= 0.1; continue; }
    if (_rgb_led_colors[0].g != 0 && g < 1) { divider -= 0.1; continue; }
    if (_rgb_led_colors[0].b != 0 && b < 1) { divider -= 0.1; continue; }
  }
  _rgb_led_colors[1].r = r;
  _rgb_led_colors[1].g = g;
  _rgb_led_colors[1].b = b;
  _rgb_led_colors[2].r = 0;
  _rgb_led_colors[2].g = 0;
  _rgb_led_colors[2].b = 0;
  _rgb_led_breath_direction = +1;
  _rgb_led_next_toggle_time = 1;
}

void rgb_led_update() {
  if (_rgb_led_next_toggle_time == 0) {
    return;
  }

  const unsigned long elapsed_time = millis();

  if (elapsed_time < _rgb_led_next_toggle_time) {
    return;
  }

  if (_rgb_led_breath_direction != 0) {
    _rgb_led_next_toggle_time = elapsed_time;
    int16_t r, g, b;
    if (_rgb_led_breath_direction > 0) {
      r = _rgb_led_colors[2].r + _rgb_led_colors[1].r;
      g = _rgb_led_colors[2].g + _rgb_led_colors[1].g;
      b = _rgb_led_colors[2].b + _rgb_led_colors[1].b;

      uint8_t threshold_counter = 0;
      if (r >= _rgb_led_colors[0].r) { r = _rgb_led_colors[0].r; threshold_counter++; }
      if (g >= _rgb_led_colors[0].g) { g = _rgb_led_colors[0].g; threshold_counter++; }
      if (b >= _rgb_led_colors[0].b) { b = _rgb_led_colors[0].b; threshold_counter++; }
      if (threshold_counter == 3) {
        _rgb_led_breath_direction = -1;
        _rgb_led_next_toggle_time += RGB_LED_BREATH_INCREMENT_IN_MILLISECONDS * 4;
      }
    } else {
      r = _rgb_led_colors[2].r - _rgb_led_colors[1].r;
      g = _rgb_led_colors[2].g - _rgb_led_colors[1].g;
      b = _rgb_led_colors[2].b - _rgb_led_colors[1].b;

      uint8_t threshold_counter = 0;
      if (r <= 0) { r = 0; threshold_counter++; }
      if (g <= 0) { g = 0; threshold_counter++; }
      if (b <= 0) { b = 0; threshold_counter++; }
      if (threshold_counter == 3) {
        _rgb_led_breath_direction = +1;
        _rgb_led_next_toggle_time += RGB_LED_BREATH_INCREMENT_IN_MILLISECONDS * 4;
      }
    }

    _rgb_led_colors[2].r = r;
    _rgb_led_colors[2].g = g;
    _rgb_led_colors[2].b = b;

    rgb_led_send_data(_rgb_led_colors[2].r, _rgb_led_colors[2].g, _rgb_led_colors[2].b);
    _rgb_led_next_toggle_time += RGB_LED_BREATH_INCREMENT_IN_MILLISECONDS;
    return;
  }

  const RgbColor *rgb_color = &_rgb_led_colors[_rgb_led_toggle_counter];
  rgb_led_send_data(rgb_color->r, rgb_color->g, rgb_color->b);

  if (rgb_color->length == 0) {
    _rgb_led_next_toggle_time = 0;
    _rgb_led_toggle_counter = 0;
    return;
  }

  _rgb_led_next_toggle_time = elapsed_time + rgb_color->length;
  _rgb_led_toggle_counter++;

  if (_rgb_led_toggle_counter == 1 && _rgb_led_colors[1].length == 0) {
    return;
  }

  if (_rgb_led_toggle_counter == (sizeof(_rgb_led_colors) / sizeof(_rgb_led_colors[0])) || _rgb_led_colors[_rgb_led_toggle_counter].length == 0) {
    _rgb_led_toggle_counter = 0;
  }
}

void rgb_led_send_data(uint8_t r, uint8_t g, uint8_t b) {
  noInterrupts();

  for (uint8_t c = 0; c < 3; c++) {
    const uint8_t byte = c == 0 ? g : c == 1 ? r : b;
    for (uint8_t i = 0; i < 8; i++) {
      if ((byte << i) & 0x80) {
        RGB_LED_PORT |= (1 << RGB_LED_BIT);
        __asm__ __volatile__ ("nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t");
        RGB_LED_PORT &= ~(1 << RGB_LED_BIT);
        __asm__ __volatile__ ("nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t");
      } else {
        RGB_LED_PORT |= (1 << RGB_LED_BIT);
        __asm__ __volatile__ ("nop\n\t nop\n\t");
        RGB_LED_PORT &= ~(1 << RGB_LED_BIT);
        __asm__ __volatile__ ("nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t");
      }
    }
  }

  interrupts();
}

void init_fan_capture(FanCapture &fan) {
  fan.rpm_counter = 0;
  memset(fan.capture_buffer, 0, sizeof(fan.capture_buffer));
  fan.capture_buffer_index = 0;
  fan.min_duty_cycle = (20.0F / 100.0F) * 255.0F;  // min fan speed is 40% (* 255 as we use full byte value)
  fan.max_duty_cycle = 255;                        // max fan speed is 100% (= 255 as we use full byte value)
  fan.req_duty_percentage = 100;                   // default fan speed is 100% of `max_duty_cycle` (= 100 as we use 0..100 values only)
  fan.alert_rpm = 0;
  fan.alert_threshold = 4;  // alert after 4 consecutive check failures (it's = 1 second as we check every 250ms)
  fan.alert_trigger = 0;
  fan.current_rpm = SENSOR_NOT_PRESENT;
  fan.use_hysteresys_curve = true;
  fan.hysteresys_curve[0] = -1;   // fan speed in % of PWM between `min_duty_cycle` and `max_duty_cycle` at 10C
  fan.hysteresys_curve[1] = 0;    // fan speed in % of PWM between `min_duty_cycle` and `max_duty_cycle` at 20C
  fan.hysteresys_curve[2] = 10;   // fan speed in % of PWM between `min_duty_cycle` and `max_duty_cycle` at 30C
  fan.hysteresys_curve[3] = 20;   // fan speed in % of PWM between `min_duty_cycle` and `max_duty_cycle` at 40C
  fan.hysteresys_curve[4] = 40;   // fan speed in % of PWM between `min_duty_cycle` and `max_duty_cycle` at 50C
  fan.hysteresys_curve[5] = 60;   // fan speed in % of PWM between `min_duty_cycle` and `max_duty_cycle` at 60C
  fan.hysteresys_curve[6] = 80;   // fan speed in % of PWM between `min_duty_cycle` and `max_duty_cycle` at 70C
  fan.hysteresys_curve[7] = 100;  // fan speed in % of PWM between `min_duty_cycle` and `max_duty_cycle` at 80C
}

void init_external_stats() {
  for (uint8_t i = 0; i < (sizeof(ext_fans) / sizeof(ext_fans[0])); i++) {
    ext_fans[i] = SENSOR_NOT_PRESENT;
  }

  for (uint8_t i = 0; i < (sizeof(ext_temps) / sizeof(ext_temps[0])); i++) {
    ext_temps[i] = SENSOR_NOT_PRESENT;
  }
  
  for (uint8_t i = 0; i < (sizeof(ext_loads) / sizeof(ext_loads[0])); i++) {
    ext_loads[i] = SENSOR_NOT_PRESENT;
  }
}

uint8_t strl(const char *s) {
  if (!s) {
    return 0;
  }
  uint8_t len = 0;
  while (*s++) len++;
  return len;
}

void strcopy(char *dst, const char *src, const uint8_t max_length) {
  for (uint8_t i = 0; i < max_length; i++) {
    if (src[i] == 0) {
      break;
    }

    dst[i] = src[i];
  }
}

void itoar(int val, char *end) {
  bool neg = val < 0;
  uint16_t uval = neg ? -val : val;

  do {
    *end-- = '0' + (uval % 10);
    uval /= 10;
  } while (uval);

  if (neg) {
    *end-- = '-';
  }
}

void write_temperature(char *target, int8_t value, uint8_t max_len) {
  if (value == SENSOR_NOT_PRESENT) {
    for (uint8_t i = 0; i < max_len; i++) {
      *(target - i) = '/';
    }
    return;
  }

  if (_use_fahrenheit_temp) {
    int16_t fahrenheit = static_cast<int16_t>(value * 9.0F / 5.0F) + 32;
    itoar(fahrenheit, target);
  } else {
    itoar(value, target);
  }
}

void create_custom_lcd_chars() {
  uint8_t char0[8] = {
    B11111,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B11111,
  };
  lcd.createChar(0, char0);

  uint8_t char1[8] = {
    B11111,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B10000,
    B11111,
  };
  lcd.createChar(1, char1);

  uint8_t char2[8] = {
    B11111,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11111,
  };
  lcd.createChar(2, char2);

  uint8_t char3[8] = {
    B11111,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11100,
    B11111,
  };
  lcd.createChar(3, char3);

  uint8_t char4[8] = {
    B11111,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11110,
    B11111,
  };
  lcd.createChar(4, char4);

  uint8_t char5[8] = {
    B11111,
    B00001,
    B00001,
    B00001,
    B00001,
    B00001,
    B00001,
    B11111,
  };
  lcd.createChar(5, char5);

  uint8_t char6[8] = {
    B11100,
    B10100,
    B11100,
    B00000,
    B00011,
    B00100,
    B00100,
    B00011,
  };
  lcd.createChar(6, char6);

  uint8_t char7[8] = {
    B11100,
    B10100,
    B11100,
    B00000,
    B00111,
    B00100,
    B00111,
    B00100,
  };
  lcd.createChar(7, char7);
}
