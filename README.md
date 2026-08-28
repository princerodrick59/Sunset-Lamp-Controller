# Sunset Lamp

A touch-screen remote for a Zengge / LEDnetWF "sunset lamp". Runs on a
**Waveshare ESP32-S3-Touch-LCD-4.3** and talks to the lamp over Bluetooth —
nothing is wired to the lamp.

Colour wheel, brightness slider, power switch, four presets. After 5 minutes
untouched the screen sleeps; tap to wake.

---

## Setup (once)

**1. ESP32 board package — must be core 2.0.x** (3.x breaks the screen).
Add this URL in *Preferences ▸ Additional boards manager URLs*, then install
**esp32 2.0.18** from *Boards Manager*:

```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

**2. Libraries** (*Tools ▸ Manage Libraries*) — versions matter:

| Library | Version |
|---|---|
| GFX Library for Arduino (Moononournation) | 1.4.9 |
| lvgl | 8.3.x |
| bb_captouch | **1.2.2** (not 1.3+) |
| NimBLE-Arduino | 1.4.x |
| ESP32_IO_Expander (espressif) | **0.1.0** |

**3. lv_conf.h** — copy this project's `lv_conf.h` to
`Documents/Arduino/libraries/lv_conf.h` (next to the `lvgl` folder, not inside).

**4. Board settings** — Board: *ESP32S3 Dev Module*, then:

- PSRAM: **OPI PSRAM** (screen is black without it)
- Flash Size: 8 MB (or 16 MB if that's your board)
- Partition Scheme: Huge APP
- USB CDC On Boot: Enabled

---

## Upload

Plug in USB-C, pick the port, hit Upload. If it won't connect: hold **BOOT**,
tap **RESET**, release **BOOT**, upload again, then tap **RESET** when done.

Set `LAMP_MAC` near the top of the `.ino` to your lamp's address if it isn't
`08:65:f0:28:44:30` (or leave it — it also matches any `LEDnetWF…` name).

---

## Using it

- **Connect** (top right) — close any phone/PC app first; the lamp allows one
  connection at a time. Drops reconnect automatically.
- **Wheel** — outer ring picks hue, inner disc picks saturation.
- **Slider** — brightness.
- **Presets** — tap to jump to a colour (also powers the lamp on).

Controls still work while disconnected; changes apply once you connect.

---

## If something's wrong

| Problem | Fix |
|---|---|
| Screen black | PSRAM not set to OPI PSRAM, or wrong ESP32 core |
| Touch dead | bb_captouch must be 1.2.2; power-cycle (not just RESET) |
| Won't compile | Check every library version above |
| Reds/blues swapped | Set `LV_COLOR_16_SWAP` to `1` in `lv_conf.h` |
| Image slides/tears | Lower `pclk` in the `.ino` from `14000000` to `12000000` |
| "Couldn't connect" | Wrong `LAMP_MAC`, or another device has the lamp |
