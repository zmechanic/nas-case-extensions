# Wiring

https://app.cirkitdesigner.com/project/63c4cfeb-4659-44b3-8fcc-0739deb1d74b

# Supervisor Firmware - Serial Command Reference

## Overview

Commands are sent over serial (9600 baud) as plain text terminated by `\n`, `\r`, or `\0`. Each command consists of a **2-character command code** followed by parameters. Multiple commands can be chained in a single line separated by `|` or `/`. Numeric parameters are unsigned 16-bit integers separated by commas (`,`). Multiple parameter groups are separated by `|` or `;`.

## Error Codes

| Code | Meaning |
|------|---------|
| `-2` | Unknown command |
| `-3` | No command parameters provided |
| `-4` | Invalid parameters |

Errors produce an audible error sound on the buzzer.

---

## System Commands

### `~F` — Enter flash (programming) mode

Stops all supervisor code. Microcontroller enters flashing mode. In most cases it can be successfully flashed after this command without the need for manual hardware reset.

| Parameters | None |
|---|---|

---

### `~R` — Reset

Triggers a hardware watchdog reset (reboots the microcontroller).

| Parameters | None |
|---|---|

---

### `#R` — Prepare for Reboot

Notifies the supervisor that the host OS is rebooting. Transitions to `REBOOT_PHASE1` wait status.

| Parameters | None |
|---|---|

---

### `#S` — Prepare for Shutdown

Notifies the supervisor that the host OS is shutting down. Transitions to `SHUTDOWN_PHASE1` wait status.

| Parameters | None |
|---|---|

---

### `#H` — Enter Hibernate / Sleep Mode

Puts the supervisor into low-power sleep mode.

| Parameters | None |
|---|---|

---

### `#0` — Cancel Wait Status

Cancels any active wait status. If in sleep mode, exits sleep. Otherwise transitions to `READY`.

| Parameters | None |
|---|---|

---

### `#1` — Cancel Wait Status (Boot Only)

Cancels wait status only if the supervisor is currently in a boot phase. Transitions to `READY`. No effect if not booting.

| Parameters | None |
|---|---|

---

## RGB LED Commands

### `LR` — LED Color Reset

Resets all user-defined LED colors and status colors back to defaults.

| Parameters | None |
|---|---|

---

### `LC` — LED Set Constant Color

Sets one or more RGB LED segments to constant colors.

| Parameter | Type | Description |
|---|---|---|
| `R` | 0-255 | Red channel |
| `G` | 0-255 | Green channel |
| `B` | 0-255 | Blue channel |
| `length` | 0-65535 | *(Optional)* Duration in ms, 0 = permanent |

Supports up to 8 color groups separated by `|` or `;` for multi-segment LEDs.

**Example:** `LC255,0,0|0,255,0` — First segment red, second segment green.

---

### `LB` — LED Set Breathing Color

Sets the RGB LED to a breathing (pulsing) animation with the given color.

| Parameter | Type | Description |
|---|---|---|
| `R` | 0-255 | Red channel |
| `G` | 0-255 | Green channel |
| `B` | 0-255 | Blue channel |
| `length` | 0-65535 | *(Optional)* Duration in ms |

Only the first color group is used for breathing mode.

**Example:** `LB0,0,255` — Blue breathing effect.

---

### `LS` — LED Set Status Color

Overrides the automatic status indicator LED colors.

| Parameter | Type | Description |
|---|---|---|
| `R` | 0-255 | Red channel |
| `G` | 0-255 | Green channel |
| `B` | 0-255 | Blue channel |

Supports up to 2 groups (index 0 = array status, index 1 = network status). Pass `0` as a single value in a group to skip that status index.

**Example:** `LS255,0,0|0,255,0` — Array status = red, network status = green.

---

### `LD` — LED Set Dimming Level

Sets the dimming level for status LEDs and user LEDs.

| Parameter | Position | Type | Description |
|---|---|---|---|
| `status_dimming` | 1st | 0-5 | Dimming level for status LEDs (0 = off, >5 resets to 0) |
| `user_dimming` | 2nd | 0-5 | *(Optional)* Dimming level for user-set LEDs |

**Example:** `LD3,2` — Status LEDs at level 3, user LEDs at level 2.

---

## Sound Commands

### `SP` — Play Sound / Melody

Plays a sequence of tones through the buzzer.

| Parameter | Type | Description |
|---|---|---|
| `frequency` | 0-65535 | Tone frequency in Hz |
| `duration` | 0-65535 | *(Optional, default: 250)* Note duration in ms |

Supports up to 16 notes separated by `|` or `;`.

**Example:** `SP800,200|1000,200|1200,400` — Three-note melody.

---

### `SA` — Set Alert Sound

Configures the frequency and duration for the alert buzzer sound.

| Parameter | Position | Type | Description |
|---|---|---|---|
| `frequency` | 1st | 0-65535 | Alert tone frequency in Hz (default: 2000) |
| `duration` | 2nd | 0-65535 | *(Optional, default: 500)* Alert duration in ms |

**Example:** `SA3000,1000` — Alert at 3 kHz for 1 second.

---

## Fan Commands

### `FS` — Set Fan Speed

Sets the target speed (duty cycle percentage) for a fan.

| Parameter | Position | Type | Description |
|---|---|---|---|
| `fan_index` | 1st | 1-2 | Fan number (1 = HDD fan, 2 = LSI fan) |
| `speed` | 2nd | 0-100 | Target speed as percentage (=0 = fan off) |

Supports multiple groups for setting several fans at once. Disables hysteresis curve for the specified fan.

**Example:** `FS1,75|2,50` — Fan 1 at 75%, fan 2 at 50%.

---

### `FP` — Set Fan PWM Range

Configures the minimum and maximum PWM duty cycle limits for a fan.

| Parameter | Position | Type | Description |
|---|---|---|---|
| `fan_index` | 1st | 1-2 | Fan number |
| `min_pwm` | 2nd | 0-100 | Minimum duty cycle percentage |
| `max_pwm` | 3rd | 0-100 | *(Optional)* Maximum duty cycle percentage |

If min >= max, both are reset to defaults (0-255).

**Example:** `FP1,20,90` — Fan 1 operates between 20% and 90% PWM.

---

### `FF` — Set Fan Carrier Frequency

Sets the PWM carrier frequency for all fans.

| Parameter | Type | Description |
|---|---|---|
| `frequency` | 0-65535 | PWM carrier frequency in Hz (default: 25000) |

**Example:** `FF25000` — 25 kHz PWM frequency.

---

### `FH` — Set Fan Hysteresis Curve

Defines a temperature-to-speed hysteresis curve for a fan. The curve maps temperature ranges (10-80 C in 10 C steps) to fan speed percentages.

| Parameter | Position | Type | Description |
|---|---|---|---|
| `fan_index` | 1st | 1-2 | Fan number |
| `speed_at_10C` | 2nd | 0-100 | Speed at <=10 C (=0 = fan off) |
| `speed_at_20C` | 3rd | 0-100 | Speed at 20 C |
| `speed_at_30C` | 4th | 0-100 | Speed at 30 C |
| `speed_at_40C` | 5th | 0-100 | Speed at 40 C |
| `speed_at_50C` | 6th | 0-100 | Speed at 50 C |
| `speed_at_60C` | 7th | 0-100 | Speed at 60 C |
| `speed_at_70C` | 8th | 0-100 | Speed at 70 C |
| `speed_at_80C` | 9th | 0-100 | Speed at 80 C |

**Note:** Temperature >=90 C will always force 100% speed no matter of the curve set.

**Example:** `FH1,0,10,30,50,70,85,95,100` — Fan 1 curve from off at 10 C to full at 80 C.

---

### `FA` — Set Fan Alert

Enables an alert when a fan's RPM drops below a threshold.

| Parameter | Position | Type | Description |
|---|---|---|---|
| `fan_index` | 1st | 1-2 | Fan number |
| `min_rpm` | 2nd | 0-65535 | Minimum acceptable RPM (0 = disable alert) |
| `threshold` | 3rd | 1-255 | *(Optional, default: 4)* Number of consecutive low readings before alert |

**Example:** `FA1,500,3` — Alert if fan 1 drops below 500 RPM for 3 consecutive checks.

---

## Display Commands

### `DP` — Set Display Page

Switches the LCD display to show a specific page.

| Parameter | Type | Description |
|---|---|---|
| `page_id` | 0-20 | Display page index (see page types below) |

**Page Types:**

| ID | Page |
|---|---|
| 0 | Blink Info (temporary) |
| 1-4 | Custom Pages 1-4 |
| 5 | Cooling - CPU |
| 6 | Cooling - Motherboard |
| 7 | Cooling - HDD |
| 8 | Cooling - LSI |
| 9 | Cooling - All |
| 10 | Cooling - All Disks |
| 11 | Load - CPU |
| 12 | Load - RAM |
| 13 | Load - LAN Upload |
| 14 | Load - LAN Download |
| 15 | Load - All |
| 16 | Load - All Disks |
| 17 | Uptime |
| 18 | HDD Cage Voltage |

**Example:** `DP5` — Show CPU cooling page.

---

### `DI` — Set Display Page Indices (Page Map)

Remaps the order of display pages for button navigation. Resets the page map, then assigns pages to logical positions.

| Parameter | Type | Description |
|---|---|---|
| `page_id, ...` | comma-separated | Ordered list of physical page IDs for button navigation |

If the first value is >=100, the page map is cleared without repopulating (hides all pages from navigation).

**Example:** `DI1,5,7,17` — Navigation order: Custom 1, CPU Cooling, HDD Cooling, Uptime.

---

### `DW` — Write to Display Page

Writes custom text to a display page buffer.

**Format:** `DW<page_id>:<text>`

| Parameter | Type | Description |
|---|---|---|
| `page_id` | 0-4 | Target page index |
| `text` | string | Up to 32 characters (16 cols x 2 rows). Use `;` to separate lines |

Writing to page 0 (Blink Info) triggers a 2-second temporary display if in `READY` state.

**Example:** `DW1:Hello World!;Second Line` — Write to custom page 1.

---

### `DA` — Display Alert

Shows a temporary alert message on the display with an alert sound.

**Format:** `DA:<line1>;<line2>`

| Parameter | Type | Description |
|---|---|---|
| `line1` | string | First display line (up to 16 chars) |
| `line2` | string | *(Optional)* Second display line, separated by `;` |

**Example:** `DA:DISK FAILURE;Check Drive 3` — Show alert with two lines.

---

### `DM` — Display Message

Shows a temporary informational message on the display (no alert sound).

**Format:** `DM:<line1>;<line2>`

| Parameter | Type | Description |
|---|---|---|
| `line1` | string | First display line (up to 16 chars) |
| `line2` | string | *(Optional)* Second display line, separated by `;` |

**Example:** `DM:Backup Done;100% Complete` — Show info message.

---

### `DB` — Set Display Brightness

Sets the LCD backlight brightness.

| Parameter | Type | Description |
|---|---|---|
| `brightness` | 0-100 | Brightness percentage (>=100 = full brightness) |

**Example:** `DB50` — Set display to 50% brightness.

---

## Temperature Commands

### `TF` — Set Temperature Scale

Switches between Celsius and Fahrenheit display.

| Parameter | Type | Description |
|---|---|---|
| `use_fahrenheit` | 0 or 1 | 0 = Celsius, 1 = Fahrenheit |

**Example:** `TF1` — Switch to Fahrenheit.

---

### `TR` — Set Temperature Reference (Calibration)

Calibrates an internal temperature sensor by providing a known reference temperature.

| Parameter | Position | Type | Description |
|---|---|---|---|
| `sensor_index` | 1st | 1-2 | Sensor number (1 = HDD, 2 = LSI) |
| `reference_temp` | 2nd | 0-50 | Known reference temperature in C |

The firmware calculates a correction offset from the difference between the reference and current reading.

**Example:** `TR1,25` — Calibrate HDD sensor to 25 C.

---

### `TA` — Set Temperature Alert

Enables a high-temperature alert for an internal sensor.

| Parameter | Position | Type | Description |
|---|---|---|---|
| `sensor_index` | 1st | 1-2 | Sensor number (1 = HDD, 2 = LSI) |
| `alert_temp` | 2nd | 0-254 | Temperature threshold in C (>=255 = disable, 0 = disable) |
| `threshold` | 3rd | 1-255 | *(Optional, default: 4)* Consecutive readings before alert |

**Example:** `TA1,55,3` — Alert if HDD sensor exceeds 55 C for 3 consecutive checks.

---

## Voltage Commands

### `VR` — Set Voltage Reference (Calibration)

Calibrates the ADC voltage readings for 12V and 5V rails.

| Parameter | Position | Type | Description |
|---|---|---|---|
| `ref_12v` | 1st | 0-65535 | 12V reference value (auto-scaled: <20 = x1, <200 = x0.1, >=200 = x0.01) |
| `ref_5v` | 2nd | 0-65535 | 5V reference value (same scaling) |

**Example:** `VR120,50` — Set 12V reference to 12.0V and 5V reference to 5.0V.

---

## External Sensor Commands

These commands feed host-system sensor data into the supervisor for display on the LCD.

### `EL` — Set External Load Sensor

| Parameter | Position | Type | Description |
|---|---|---|---|
| `sensor_index` | 1st | 1-10 | Sensor (1=CPU, 2=MB, 3=RAM, 4=LAN up, 5=LAN down, 6-9=HDD1-4) |
| `load` | 2nd | 0-100 | Load percentage (>=255 = not present) |

Supports multiple groups.

**Example:** `EL1,45|3,72` — CPU at 45%, RAM at 72%.

---

### `EF` — Set External Fan Speed Sensor

| Parameter | Position | Type | Description |
|---|---|---|---|
| `sensor_index` | 1st | 1-2 | Sensor (1=CPU fan, 2=Motherboard fan) |
| `rpm` | 2nd | 0-8000 | Fan speed in RPM (>=8000 = not present) |

**Example:** `EF1,1200` — CPU fan at 1200 RPM.

---

### `ET` — Set External Temperature Sensor

| Parameter | Position | Type | Description |
|---|---|---|---|
| `sensor_index` | 1st | 1-10 | Sensor (1=CPU, 2=MB, 3=RAM, 4=LAN up, 5=LAN down, 6-9=HDD1-4, 101=override HDD cage sensor value, 101=override LSI card sensor value) |
| `temperature` | 2nd | 0-127 | Temperature in C (>=255 = not present) |

**Example:** `ET1,62|6,38|7,40` — CPU 62 C, HDD1 38 C, HDD2 40 C.

---

### `EN` — Set Network Status

Sets the network connection status for the RGB LED indicator.

| Parameter | Type | Description |
|---|---|---|
| `status_code` | 0-7 | Network status code (>=8 = not present) |

**Example:** `EN1` — Set network status indicator.

---

### `EA` — Set Array Status

Sets the disk array status for the RGB LED indicator.

| Parameter | Type | Description |
|---|---|---|
| `status_code` | 0-7 | Array status code (>=8 = not present) |

**Example:** `EA1` — Set array status indicator.

---

## HDD / Disk Commands

### `HD` — Set Disk Shutdown Delay

Configures a delay before powering off disks (used during sleep/shutdown).

| Parameter | Type | Description |
|---|---|---|
| `delay` | 0-254 | Delay in seconds (0 = disable, >=255 = disable) |

**Example:** `HD30` — Wait 30 seconds before disk power-off.

---

### `HA` — Set Disk Temperature Alert

Sets a temperature threshold for disk overheat alerts (uses external HDD temperature sensors).

| Parameter | Type | Description |
|---|---|---|
| `temperature` | 0-254 | Alert threshold in C (0 = disable, >=255 = disable) |

**Example:** `HA55` — Alert when any disk exceeds 55 C.

---

### `HF` — Enable Disk Temperature to Fan Speed Link

Enables or disables automatic fan speed adjustment based on the highest disk temperature (uses the hysteresis curve of fan 1).

| Parameter | Type | Description |
|---|---|---|
| `enable` | 0 or 1 | 0 = disable, 1 = enable |

**Example:** `HF1` — Enable automatic disk-temperature-based fan control.

---

## Command Chaining

Multiple commands can be sent in a single serial line using `|` or `/` as separators between command groups.

**Example:**

```
FS1,75|DP5|DB80
```

Sets fan 1 to 75%, switches to CPU cooling display page, and sets brightness to 80%.
