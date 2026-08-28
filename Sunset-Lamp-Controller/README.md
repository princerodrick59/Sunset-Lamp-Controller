# Sunset — sunset-lamp controller for ESP32-S3-Touch-LCD-4.3

A touch-screen remake of the Windows Python controller. It talks to a Zengge /
LEDnetWF "sunset lamp" over Bluetooth LE and gives you a wheel for colour, a bar
for brightness, a power switch and four presets — one calm, Apple-ish screen.

Same BLE protocol as the Python app (`set_rgb` = HSV frame, plus the two power
packets). The board is the BLE **central**, so nothing is wired to the lamp —
it's all radio.

---

## 1. What you need

| Thing | Notes |
|---|---|
| Waveshare **ESP32-S3-Touch-LCD-4.3** | The plain 4.3 or 4.3B. 800×480 RGB LCD, GT911 touch, CH422G expander. |
| USB-C cable | Data cable, not charge-only. |
| The lamp's BLE address | Already filled in (`08:65:F0:28:44:30`). Change `LAMP_MAC` in the `.ino` if yours differs — or just let it match any name starting with `LEDnetWF`. |

---

## 2. Arduino IDE setup (do this once)

### 2a. ESP32 board package — **use core 2.0.x**

Arduino IDE → **File ▸ Preferences ▸ Additional boards manager URLs**, add:

```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

Then **Tools ▸ Board ▸ Boards Manager**, search **esp32**, and install any
**2.0.x** version — **2.0.18** (the last 2.0.x release) is recommended.
Pick it from the version dropdown; do **not** install 3.x.

> The RGB-panel library and the CH422G library below do not work on core 3.x
> (it switched to a new I2C driver). Stay on 2.0.x — 2.0.14 through 2.0.18 are
> all fine.

### 2b. Libraries

**Library Manager** (`Tools ▸ Manage Libraries`), install:

| Library | Author | Version |
|---|---|---|
| `GFX Library for Arduino` | Moononournation | **1.4.9** (any 1.4.x) |
| `lvgl` | kisvegabor / LVGL | **8.3.x** (e.g. 8.3.11 — *not* 9.x) |
| `bb_captouch` | Larry Bank | **1.2.2** (not 1.3.x — it probes I²C 0x24, which is the CH422G, and mis-detects touch) |
| `NimBLE-Arduino` | h2zero | **1.4.x** (e.g. 1.4.3 — *not* 2.x) |
| `ESP32_IO_Expander` | espressif | **0.1.0**  (CH422G — *not* 1.x, the API changed) |

> `ESP32_IO_Expander` **0.1.0** is the version that provides
> `#include <ESP_IOExpander_Library.h>` and the `ESP_IOExpander_CH422G` class the
> sketch uses. Pick 0.1.0 from the version dropdown; 1.x won't compile here.
> If Library Manager only offers 1.x, install the ZIP instead:
> `https://github.com/esp-arduino-libs/ESP32_IO_Expander/archive/refs/tags/v0.1.0.zip`
> (Sketch ▸ Include Library ▸ Add .ZIP Library).

### 2c. `lv_conf.h` — LVGL's config file

LVGL needs `lv_conf.h` sitting **next to** the `lvgl` folder (not inside it).
Copy the `lv_conf.h` from this project:

```
from:  Sunset-Lamp-Controller/lv_conf.h
to:    Documents/Arduino/libraries/lv_conf.h
```

Result:

```
Documents/Arduino/libraries/
├── lv_conf.h                 ← the file you just copied
├── lvgl/
├── GFX Library for Arduino/
├── bb_captouch/
├── NimBLE-Arduino/
└── ESP32_IO_Expander/
```

This `lv_conf.h` is the stock LVGL 8.3 template with four changes already made:
content enabled, 64 KB LVGL heap, `LV_TICK_CUSTOM` on (uses `millis()`), and the
Montserrat 14/16/20/28 fonts + the colour wheel enabled.

---

## 3. Board settings (Tools menu)

Select **Board: "ESP32S3 Dev Module"**, then set:

| Setting | Value |
|---|---|
| USB CDC On Boot | **Enabled** |
| CPU Frequency | 240 MHz (WiFi) |
| Flash Mode | QIO 80 MHz |
| Flash Size | **8 MB (64 Mb)**  *(use 16 MB if that's your board)* |
| PSRAM | **OPI PSRAM**  ← required, screen stays black without it |
| Partition Scheme | **Huge APP (3 MB No OTA / 1 MB SPIFFS)** |
| Arduino Runs On | Core 1 |
| Events Run On | Core 1 |
| Upload Speed | 921600 |
| USB Mode | Hardware CDC and JTAG |

Serial Monitor baud rate: **115200**.

---

## 4. Upload

1. Plug USB-C into the board (the USB port, not the debug UART).
2. **Tools ▸ Port** → pick the COM port that appears.
3. Press **Upload** (→). It normally resets into download mode on its own.
4. **If it says "Failed to connect / No serial data received":**
   hold **BOOT**, tap **RESET**, release **BOOT**, then Upload again.
   Tap **RESET** once when the upload finishes.
5. Open **Serial Monitor @ 115200** — you should see:

   ```
   [Sunset] boot
   [Sunset] ready
   ```

---

## 5. Using it

- **Connect** (top-right) — scans ~6 s, then connects. Dot goes green.
  Close any phone / PC app first: these lamps allow only one connection.
  If the link drops unexpectedly it auto-reconnects (up to 6 tries; "Stop"
  cancels); a first-attempt failure still shows "Retry".
- **Power** switch — sends the on/off packet, and follows the lamp's own status
  reports (queried on connect, then polled) so it stays in sync.
- **Wheel** — a custom canvas widget (not `lv_colorwheel`). Outer ring = hue
  (red at the top, clockwise); inner disc = the selected hue fading to white at
  the centre = saturation. The ring is painted once; the disc is repainted only
  when the hue changes. Two knobs: hue on the ring, saturation on the disc.
- **Brightness bar** — vertical slider on the right; the % sits in a dark pill
  *inside* the bar so it reads over the fill or the track.
- **Presets** — a vertical stack beside the bar. Tap one to jump to that colour
  (also powers the lamp on); the active one keeps a white outline until you
  touch the wheel.
- When disconnected the controls still work — a hint says changes apply once you
  connect, and they're pushed the moment the link is up.

Colour + brightness changes are rate-limited to ~30/s so dragging stays smooth
and the lamp keeps up.

---

## 6. Troubleshooting

| Symptom | Fix |
|---|---|
| Screen stays black | PSRAM must be **OPI PSRAM**; ESP32 core must be **2.0.x**. |
| Compile error on `Arduino_RGB_Display(...)` | Arduino_GFX is too new — install **1.4.9**. |
| `lv_conf.h` errors / `LV_FONT_MONTSERRAT_*` undefined | `lv_conf.h` isn't at `Arduino/libraries/lv_conf.h` (beside the `lvgl` folder). |
| Compile error in NimBLE (`NimBLEScanResults` etc.) | You're on NimBLE 2.x — downgrade to **1.4.3**. |
| Touch does nothing / is offset | Check Serial for `touch type=2` (GT911). If it says `0`, the GT911 wasn't on the bus when `bbct.init()` ran — power-cycle the board (not just RESET) so the CH422G releases EXIO1. |
| Reds and blues look swapped | In `lv_conf.h` set `LV_COLOR_16_SWAP` to `1`. |
| "Couldn't Connect" | Wrong `LAMP_MAC`, or another device is already connected to the lamp. |
| Image tears / slides down (esp. while BLE scans) | RGB DMA starving. `pclk` is set to 14 MHz; drop it to `12000000` if it persists. |
| `gpio_set_level(... ): GPIO output gpio_num error` at boot | Harmless — `bb_captouch` pinging the touch INT pin as a reset line. Touch still works. |
| Power switch doesn't track the lamp | Watch Serial for `[notify N] …` after connecting (first 12 frames are dumped). The lamp's state frame is JSON-wrapped — `{"code":0,"payload":"81…"}` — and `notifyCB` decodes byte 2 for power. No `[notify]` lines at all = this firmware doesn't push status. |

---

## 7. How it's built

```
core 1  (loop)     LVGL draw + GT911 touch + every UI callback
core 0  (bleTask)  every BLE scan / connect / GATT write
```

The UI thread only ever sets a few `volatile`s and pushes small events onto a
queue; the BLE thread drains them. So Bluetooth work never blocks a frame, and
LVGL is only touched from one thread.

The lamp protocol lives in section 1 of the `.ino` and is byte-identical to
`LampController` in the original Python file — `PKT_POWER_ON/OFF` and the
`PKT_HSV_TEMPLATE` frame whose bytes 10/11/12 are hue (deg ÷ 2), saturation and
value.
