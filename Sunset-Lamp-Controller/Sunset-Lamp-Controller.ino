/* ============================================================================
 *  Sunset  —  Zengge / LEDnetWF sunset-lamp controller
 *  Board   :  Waveshare ESP32-S3-Touch-LCD-4.3   (800x480 RGB LCD, GT911 touch)
 *
 *  A touch-panel port of the Windows Python controller.  Same Bluetooth
 *  protocol, same idea: turn the lamp on, drag the HSV wheel for a colour,
 *  drag the vertical bar for brightness, tap a preset.  The UI is LVGL,
 *  styled to feel calm and Apple-like — one screen, big soft controls.
 *
 *  Two cores, cleanly split
 *  -----------------------
 *    core 1  (Arduino loop)  : LVGL drawing + touch + every UI callback
 *    core 0  (bleTask)       : every BLE scan / connect / GATT write
 *  They share three volatiles and one queue, so the screen never stutters
 *  while Bluetooth is busy and LVGL is only ever touched from one thread.
 *
 *  Libraries  (exact versions + IDE settings are in README.md)
 *  ---------
 *    GFX Library for Arduino   (moononournation)   1.4.x
 *    lvgl                      8.3.x   + the bundled lv_conf.h
 *    bb_captouch               1.2.2  (GT911 driver; NOT 1.3.x — see README)
 *    ESP32_IO_Expander        0.1.0  (CH422G; header ESP_IOExpander_Library.h)
 *    NimBLE-Arduino            1.4.x
 *  Build with ESP32 Arduino core 2.0.x  (see README — 3.x breaks the RGB panel).
 * ==========================================================================*/

#include <Arduino.h>
#include <Wire.h>
#include <string>
#include <math.h>
#include "esp_heap_caps.h"

#include <Arduino_GFX_Library.h>
#include <ESP_IOExpander_Library.h>
#include <bb_captouch.h>
#include <lvgl.h>
#include <NimBLEDevice.h>


/* ============================================================================
 *  1.  LAMP  —  BLE protocol   (byte-for-byte identical to the Python version)
 * ==========================================================================*/

// Your lamp's Bluetooth address — lower case, colon separated.
// Same value as DEVICE_ADDRESS in the Python script.
static const char *LAMP_MAC = "08:65:f0:28:44:30";

static NimBLEUUID SVC_UUID   ("0000ffff-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CHR_WRITE  ("0000ff01-0000-1000-8000-00805f9b34fb");
static NimBLEUUID CHR_NOTIFY ("0000ff02-0000-1000-8000-00805f9b34fb");

// Power packets, straight from LampController.power().
static const uint8_t PKT_POWER_ON[21] = {
  0x00,0x04,0x80,0x00,0x00,0x0d,0x0e,0x0b,0x3b,0x23,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x32,0x00,0x00,0x90
};
static const uint8_t PKT_POWER_OFF[21] = {
  0x00,0x5b,0x80,0x00,0x00,0x0d,0x0e,0x0b,0x3b,0x24,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x32,0x00,0x00,0x91
};
// set_rgb() frame — bytes [10],[11],[12] carry hue / saturation / value,
// everything else is the fixed LEDnetWF "HSV colour" command.
static const uint8_t PKT_HSV_TEMPLATE[21] = {
  0x00,0x05,0x80,0x00,0x00,0x0d,0x0e,0x0b,0x3b,0xa1,0x00,
  0x64,0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};
// State query (0x81) wrapped in the transport layer.  cmdId 0x0a = "expects a
// response": the lamp replies on the notify characteristic with a 14-byte
// 0x81 frame whose byte 2 is the power state (0x23 = on).
static const uint8_t PKT_STATE_QUERY[13] = {
  0x00,0x06,0x80,0x00,0x00,0x00,0x04,0x05,0x0a,0x81,0x8a,0x8b,0x40
};


/* ---- shared state : core 1 writes intent, core 0 acts on it -------------- */

enum BleState : int {
  BLE_IDLE = 0, BLE_SCANNING, BLE_CONNECTING, BLE_CONNECTED,
  BLE_DISCONNECTED, BLE_FAILED, BLE_RECONNECTING
};
volatile int  g_bleState = BLE_IDLE;

// Current colour intent.  h 0..359, s 0..100, v (brightness) 1..100.
volatile int  g_h = 16, g_s = 86, g_v = 80;
volatile bool g_colorDirty = false;

// The lamp's real power state, learned from its status notifications.
// -1 = unknown / not connected, 0 = off, 1 = on.  The UI mirrors this.
volatile int      g_lampOn    = -1;
volatile uint32_t g_powerCmdAt = 0;    // millis() of our last power command

// ---- animated effects (rendered on core 0, one BLE frame at a time) --------
enum FxId : int {
  FX_NONE = 0, FX_7FADE, FX_7STROBE, FX_7JUMP, FX_BREATHE, FX_1BLINK, FX_2FADE
};
volatile int  g_fx      = FX_NONE;
volatile int  g_fxSpeed = 50;          // 0..100  (50 = normal)
volatile int  g_fxColA  = 0;           // index into FX_RGB (single colour / colour A)
volatile int  g_fxColB  = 2;           // colour B for the two-colour fade
volatile bool g_fxRestart = false;     // UI sets this when the effect or its inputs change

// Red, Green, Blue, Yellow, Cyan, Purple, White — the lamp's own effect palette.
static const uint8_t FX_RGB[7][3] = {
  {255,0,0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {160,0,255}, {255,255,255}
};
static const char *FX_COLOR_NAME[7] = {
  "Red", "Green", "Blue", "Yellow", "Cyan", "Purple", "White"
};

enum BleEvt : uint8_t { EVT_CONNECT = 1, EVT_DISCONNECT, EVT_POWER_ON, EVT_POWER_OFF };
static QueueHandle_t g_evtQ = nullptr;

static NimBLEClient                *g_client   = nullptr;
static NimBLERemoteCharacteristic  *g_writeChr = nullptr;


class ClientCB : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient *c) override {
    g_writeChr = nullptr;
    g_bleState = BLE_DISCONNECTED;
    g_lampOn   = -1;
    g_fx       = FX_NONE;
  }
};
static ClientCB g_clientCB;

static volatile int g_dumpNotify = 3;       // dump this many frames, then stop

static inline int hexVal(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static void notifyCB(NimBLERemoteCharacteristic *c, uint8_t *d, size_t n, bool isNotify) {
  if (g_dumpNotify > 0) {
    g_dumpNotify--;
    Serial.printf("[notify %u]", (unsigned)n);
    for (size_t i = 0; i < n && i < 72; i++) Serial.printf(" %02x", d[i]);
    Serial.println();
  }

  // The lamp wraps its 0x81 state frame as JSON after an 8-byte transport
  // header:  {"code":0,"payload":"81....<hex>...."}.  Pull the hex string out,
  // decode it, and read byte 2 (0x23 = on).  Fall back to a bare 0x81 frame.
  uint8_t frame[20];
  int flen = 0;
  static const char KEY[] = "\"payload\":\"";
  const int KLEN = sizeof(KEY) - 1;

  int start = -1;
  for (size_t i = 0; i + KLEN <= n; i++)
    if (memcmp(d + i, KEY, KLEN) == 0) { start = (int)i + KLEN; break; }

  if (start >= 0) {
    for (size_t i = start; i + 1 < n && flen < (int)sizeof(frame); i += 2) {
      int hi = hexVal(d[i]), lo = hexVal(d[i + 1]);
      if (hi < 0 || lo < 0) break;
      frame[flen++] = (uint8_t)((hi << 4) | lo);
    }
  } else {
    for (size_t i = 0; i + 14 <= n && i <= 10; i++)
      if (d[i] == 0x81) { memcpy(frame, d + i, 14); flen = 14; break; }
  }

  // ignore a stale frame that's still in flight from just before our command
  if (millis() - g_powerCmdAt < 1500) return;
  if (flen >= 3 && frame[0] == 0x81)
    g_lampOn = (frame[2] == 0x23) ? 1 : 0;
}

static bool bleWrite(const uint8_t *data, size_t len) {
  if (g_bleState != BLE_CONNECTED || g_writeChr == nullptr) return false;
  if (!g_writeChr->writeValue(data, len, true)) {
    g_bleState = BLE_DISCONNECTED;
    g_writeChr = nullptr;
    return false;
  }
  return true;
}

static void bleSendColor() {
  uint8_t pkt[21];
  memcpy(pkt, PKT_HSV_TEMPLATE, 21);
  pkt[10] = (uint8_t)(g_h / 2);                     // hue   : degrees / 2  (0..179)
  pkt[11] = (uint8_t)constrain((int)g_s, 0, 100);   // sat   : 0..100
  pkt[12] = (uint8_t)constrain((int)g_v, 0, 100);   // value : brightness 0..100
  bleWrite(pkt, 21);
}

static void bleSendRGB(uint8_t R, uint8_t G, uint8_t B, int bri) {
  float r = R / 255.0f, g = G / 255.0f, b = B / 255.0f;
  float mx = fmaxf(r, fmaxf(g, b)), mn = fminf(r, fminf(g, b)), d = mx - mn;
  float h = 0;
  if (d > 1e-4f) {
    if      (mx == r) h = 60.0f * fmodf((g - b) / d, 6.0f);
    else if (mx == g) h = 60.0f * ((b - r) / d + 2.0f);
    else              h = 60.0f * ((r - g) / d + 4.0f);
  }
  if (h < 0) h += 360.0f;
  int s = (mx > 1e-4f) ? (int)(d / mx * 100.0f + 0.5f) : 0;
  uint8_t pkt[21];
  memcpy(pkt, PKT_HSV_TEMPLATE, 21);
  pkt[10] = (uint8_t)(((int)(h + 0.5f) % 360) / 2);
  pkt[11] = (uint8_t)constrain(s, 0, 100);
  pkt[12] = (uint8_t)constrain(bri, 0, 100);
  bleWrite(pkt, 21);
}

/* ---- effect engine : one frame per call, own pacing ---------------------- */

// speed 0..100 -> 0.05x .. 5x, like the Python app
static uint32_t fxScaledMs(float baseSec) {
  float f = fmaxf(0.05f, g_fxSpeed / 100.0f * 5.0f);
  return (uint32_t)(fmaxf(0.01f, baseSec / f) * 1000.0f);
}

// sleep in small slices so Stop / a param change is picked up promptly
static void fxDelay(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (g_fx == FX_NONE || g_fxRestart || g_bleState != BLE_CONNECTED) return;
    vTaskDelay(pdMS_TO_TICKS(12));
  }
}

static inline void fxLerp(const uint8_t *a, const uint8_t *b, float t, int bri) {
  bleSendRGB((uint8_t)(a[0] + (b[0] - a[0]) * t),
             (uint8_t)(a[1] + (b[1] - a[1]) * t),
             (uint8_t)(a[2] + (b[2] - a[2]) * t), bri);
}

static void fxStep() {
  static int phase = 0, sub = 0, lastFx = -1;
  if (g_fxRestart || g_fx != lastFx) { phase = sub = 0; lastFx = g_fx; g_fxRestart = false; }

  int fx  = g_fx;
  int bri = constrain((int)g_v, 0, 100);
  int a   = constrain((int)g_fxColA, 0, 6);
  int b   = constrain((int)g_fxColB, 0, 6);

  switch (fx) {
    case FX_7JUMP:
      bleSendRGB(FX_RGB[phase % 7][0], FX_RGB[phase % 7][1], FX_RGB[phase % 7][2], bri);
      phase++;
      fxDelay(fxScaledMs(0.5f));
      break;

    case FX_7STROBE: {
      const uint8_t *c = FX_RGB[phase % 7];
      bleSendRGB(c[0], c[1], c[2], sub == 0 ? bri : 0);
      if (sub) phase++;
      sub ^= 1;
      fxDelay(fxScaledMs(0.09f));
      break;
    }

    case FX_7FADE:
      fxLerp(FX_RGB[phase % 7], FX_RGB[(phase + 1) % 7], sub / 40.0f, bri);
      if (++sub > 40) { sub = 0; phase++; }
      fxDelay(fxScaledMs(0.05f));
      break;

    case FX_2FADE: {
      const uint8_t *from = (phase & 1) ? FX_RGB[b] : FX_RGB[a];
      const uint8_t *to   = (phase & 1) ? FX_RGB[a] : FX_RGB[b];
      fxLerp(from, to, sub / 50.0f, bri);
      if (++sub > 50) { sub = 0; phase++; }
      fxDelay(fxScaledMs(0.05f));
      break;
    }

    case FX_BREATHE: {
      int step = (phase & 1) ? sub : (50 - sub);        // peak -> 0 -> peak
      bleSendRGB(FX_RGB[a][0], FX_RGB[a][1], FX_RGB[a][2], bri * step / 50);
      if (++sub > 50) { sub = 0; phase++; }
      fxDelay(fxScaledMs(0.035f));
      break;
    }

    case FX_1BLINK:
      bleSendRGB(FX_RGB[a][0], FX_RGB[a][1], FX_RGB[a][2], sub == 0 ? bri : 0);
      sub ^= 1;
      fxDelay(fxScaledMs(0.18f));
      break;
  }
  if (phase >= 5040) phase -= 5040;      // keep it bounded (5040 is even & /7)
}

static void bleDoConnect() {
  g_bleState = BLE_SCANNING;

  if (!g_client) {
    g_client = NimBLEDevice::createClient();
    g_client->setClientCallbacks(&g_clientCB, false);
    // 30 ms interval + slave latency 4: when nothing is happening the lamp may
    // skip up to 4 events (~150 ms of radio quiet), but it still wakes on the
    // very next event when we send.  Fewer radio bursts = fewer LCD line drops.
    g_client->setConnectionParams(24, 24, 4, 600);
    g_client->setConnectTimeout(10);
  }

  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setActiveScan(false);       // passive: less radio churn during the scan
  scan->setInterval(80);
  scan->setWindow(40);
  NimBLEScanResults res = scan->start(4, false);

  NimBLEAdvertisedDevice *target = nullptr;
  for (auto *d : res) {
    String addr = d->getAddress().toString().c_str();
    addr.toLowerCase();
    bool nameMatch = d->haveName() &&
                     String(d->getName().c_str()).startsWith("LEDnetWF");
    if (addr == LAMP_MAC || nameMatch) { target = d; break; }
  }

  g_bleState = BLE_CONNECTING;
  bool ok = target ? g_client->connect(target)
                   : g_client->connect(NimBLEAddress(std::string(LAMP_MAC)));
  NimBLEDevice::getScan()->clearResults();
  if (!ok) { g_bleState = BLE_FAILED; return; }

  NimBLERemoteService *svc = g_client->getService(SVC_UUID);
  if (!svc) { g_client->disconnect(); g_bleState = BLE_FAILED; return; }

  g_writeChr = svc->getCharacteristic(CHR_WRITE);
  NimBLERemoteCharacteristic *nc = svc->getCharacteristic(CHR_NOTIFY);
  if (!g_writeChr) { g_client->disconnect(); g_bleState = BLE_FAILED; return; }

  bool sub = false;
  if (nc && nc->canNotify()) sub = nc->subscribe(true, notifyCB);
  Serial.printf("[Sunset] notify char %s, subscribe=%d\n", nc ? "found" : "MISSING", sub);
  delay(120);

  g_bleState   = BLE_CONNECTED;
  g_colorDirty = true;            // push whatever the UI is currently showing
  delay(80);
  bool q = bleWrite(PKT_STATE_QUERY, sizeof(PKT_STATE_QUERY));   // ask for power state
  Serial.printf("[Sunset] state query sent=%d\n", q);
}

static void bleDoDisconnect() {
  if (g_client && g_client->isConnected()) g_client->disconnect();
  g_writeChr = nullptr;
  g_bleState = BLE_IDLE;
  g_lampOn   = -1;
  g_fx       = FX_NONE;
}

static void bleTask(void *) {
  NimBLEDevice::init("Sunset");
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);   // 0 dBm is plenty for a lamp in the room;
                                            // max power spikes noise into the LCD
  NimBLEDevice::setMTU(247);         // so the lamp's status frame fits one notification

  bool autoReconnect  = false;        // keep trying after an unexpected drop
  int  reconnectTries = 0;

  // --- auto-connect on boot : one try + two retries ---
  autoReconnect = true;
  for (int t = 0; t < 3 && g_bleState != BLE_CONNECTED; t++) {
    if (t) vTaskDelay(pdMS_TO_TICKS(1500));
    bleDoConnect();
  }

  for (;;) {
    uint8_t evt;
    if (xQueueReceive(g_evtQ, &evt, pdMS_TO_TICKS(30)) == pdTRUE) {
      switch (evt) {
        case EVT_CONNECT:    autoReconnect = true; reconnectTries = 0;
                             if (g_bleState != BLE_CONNECTED) bleDoConnect(); break;
        case EVT_DISCONNECT: autoReconnect = false; bleDoDisconnect();       break;
        case EVT_POWER_ON:   if (bleWrite(PKT_POWER_ON,  21)) g_lampOn = 1;
                             g_powerCmdAt = millis();                        break;
        case EVT_POWER_OFF:  if (bleWrite(PKT_POWER_OFF, 21)) g_lampOn = 0;
                             g_powerCmdAt = millis();                        break;
      }
    }
    if (g_bleState == BLE_CONNECTED && g_fx != FX_NONE) {
      fxStep();                       // one animation frame, self-paced
      continue;
    }
    if (g_bleState == BLE_CONNECTED && g_colorDirty) {
      g_colorDirty = false;
      bleSendColor();
      vTaskDelay(pdMS_TO_TICKS(40));   // rate-limit while a control is dragged
    }
    // No periodic state poll: the lamp already pushes its 0x81 frame ~1/s, and
    // we set g_lampOn optimistically, so polling would just add radio traffic.
    // --- auto-reconnect after an unexpected drop --------------------------
    if (autoReconnect &&
        (g_bleState == BLE_DISCONNECTED || g_bleState == BLE_RECONNECTING)) {
      reconnectTries++;
      g_bleState = BLE_RECONNECTING;
      for (int w = 0; w < 20 && autoReconnect; w++) vTaskDelay(pdMS_TO_TICKS(100));
      // drain a cancel that arrived during the wait
      uint8_t e2;
      while (xQueueReceive(g_evtQ, &e2, 0) == pdTRUE)
        if (e2 == EVT_DISCONNECT) { autoReconnect = false; }
      if (!autoReconnect) { bleDoDisconnect(); continue; }
      bleDoConnect();
      if      (g_bleState == BLE_CONNECTED) reconnectTries = 0;
      else if (reconnectTries < 6)          g_bleState = BLE_RECONNECTING;
      else { g_bleState = BLE_FAILED; autoReconnect = false; }
    }
  }
}

static inline void bleQueue(uint8_t evt) { xQueueSend(g_evtQ, &evt, 0); }


/* ============================================================================
 *  2.  DISPLAY  —  Arduino_GFX RGB panel + GT911 touch + CH422G expander
 *  Pin map / timings are the published values for ESP32-S3-Touch-LCD-4.3.
 * ==========================================================================*/

#define TFT_W 800
#define TFT_H 480

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
  5 /*DE*/, 3 /*VSYNC*/, 46 /*HSYNC*/, 7 /*PCLK*/,
  1, 2, 42, 41, 40,                 // R0..R4
  39, 0, 45, 48, 47, 21,            // G0..G5
  14, 38, 18, 17, 10,               // B0..B4
  0, 40, 48, 88,                    // hsync  polarity / front / pulse / back
  0, 13,  3, 32,                    // vsync  polarity / front / pulse / back
  1, 14000000                       // pclk active-neg, 14 MHz (known-good on this panel)
);
Arduino_RGB_Display *gfx =
  new Arduino_RGB_Display(TFT_W, TFT_H, rgbpanel, 0 /*rotation*/, true /*auto_flush*/);

// CH422G I/O expander — holds the LCD + touch resets and the backlight line.
#define EXP_TP_RST   1
#define EXP_LCD_BL   2
#define EXP_LCD_RST  3
#define EXP_SD_CS    4
#define EXP_USB_SEL  5
static ESP_IOExpander *expander = nullptr;

static void backlight(bool on) {
  if (expander) expander->digitalWrite(EXP_LCD_BL, on ? HIGH : LOW);
}

// --- screen sleep : blank the backlight after a spell with no touch, wake on
//     the next touch (that touch is swallowed so it can't hit a control). ---
#define SLEEP_AFTER_MS  (1UL * 60 * 1000)     // 5 minutes untouched
static uint32_t s_lastTouchMs = 0;
static bool     s_asleep      = false;

static void screenSleep() {
  if (s_asleep) return;
  s_asleep = true;
  backlight(false);
}
static void screenWake() {
  s_lastTouchMs = millis();
  if (!s_asleep) return;
  s_asleep = false;
  backlight(true);
}

// GT911 capacitive touch.
#define TOUCH_SDA 8
#define TOUCH_SCL 9
#define TOUCH_INT 4
// The GT911 reset line is wired to CH422G EXIO1, NOT to an ESP32 GPIO — the
// expander releases it high at power-on, so the GT911 is already alive when
// bbct.init() probes.  bb_captouch 1.2.x still pulses whatever pin it's handed
// during GT911 setup: -1 makes it spam "gpio_num error", and GPIO0 is panel
// signal G1.  Handing it the INT pin (GPIO4) is harmless — bb_captouch drives
// INT low during init anyway and restores it to INPUT at the end.
#define TOUCH_RST TOUCH_INT
static BBCapTouch bbct;

// LVGL plumbing (these MUST stay in static storage).
static lv_disp_draw_buf_t  s_drawBuf;
static lv_color_t         *s_buf = nullptr;
static lv_disp_drv_t       s_dispDrv;
static lv_indev_drv_t      s_indevDrv;

static void lvglFlush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *px) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px, w, h);
  lv_disp_flush_ready(drv);
}

static void lvglTouch(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  TOUCHINFO ti;
  if (bbct.getSamples(&ti) && ti.count > 0) {
    if (s_asleep) {                 // first touch just wakes the screen
      screenWake();
      data->state = LV_INDEV_STATE_RELEASED;
      return;
    }
    s_lastTouchMs = millis();
    data->state   = LV_INDEV_STATE_PRESSED;
    data->point.x = constrain(ti.x[0], 0, TFT_W - 1);
    data->point.y = constrain(ti.y[0], 0, TFT_H - 1);
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}


/* ============================================================================
 *  3.  UI  —  the "Sunset" screen
 * ==========================================================================*/

namespace C {                        // palette — mirrors the Python Theme class
  const lv_color_t bg     = LV_COLOR_MAKE(0x00, 0x00, 0x00);
  const lv_color_t card   = LV_COLOR_MAKE(0x1c, 0x1c, 0x1e);
  const lv_color_t card2  = LV_COLOR_MAKE(0x2c, 0x2c, 0x2e);
  const lv_color_t card3  = LV_COLOR_MAKE(0x3a, 0x3a, 0x3c);
  const lv_color_t text   = LV_COLOR_MAKE(0xf5, 0xf5, 0xf7);
  const lv_color_t muted  = LV_COLOR_MAKE(0x8e, 0x8e, 0x93);
  const lv_color_t faint  = LV_COLOR_MAKE(0x63, 0x63, 0x66);
  const lv_color_t white  = LV_COLOR_MAKE(0xff, 0xff, 0xff);
  const lv_color_t ink    = LV_COLOR_MAKE(0x0b, 0x0b, 0x0d);
  const lv_color_t green  = LV_COLOR_MAKE(0x32, 0xd5, 0x4b);
  const lv_color_t red    = LV_COLOR_MAKE(0xff, 0x45, 0x3a);
  const lv_color_t yellow = LV_COLOR_MAKE(0xff, 0xd6, 0x0a);
  const lv_color_t blue   = LV_COLOR_MAKE(0x0a, 0x84, 0xff);
}

struct Preset { const char *name; uint32_t hex; };
static const Preset PRESETS[] = {
  {"Sunset", 0xff5900}, {"Rose",  0xff4f8b},
  {"Ocean",  0x0a84ff}, {"White", 0xffffff},
};
static const int PRESET_COUNT = sizeof(PRESETS) / sizeof(PRESETS[0]);

// ---- custom HSV wheel : static hue ring + static hue/saturation disc -------
#define WHEEL_SZ    340
#define WHEEL_C     170                // centre
#define WHEEL_ROUT  168                // hue-ring outer radius
#define WHEEL_RIN   124                // hue-ring inner radius
#define WHEEL_RDISC 112                // saturation-disc radius
#define WHEEL_RHIT  118                // press >= this radius = ring, else disc
#define WHEEL_HUE_R 146                // hue knob orbit radius

static lv_obj_t  *ui_wheel;
static lv_obj_t  *ui_hueDot;
static lv_obj_t  *ui_satDot;
static lv_obj_t  *ui_bright;
static lv_obj_t  *ui_pct;
static lv_obj_t  *ui_hex;
static lv_obj_t  *ui_statusDot;
static lv_obj_t  *ui_statusText;
static lv_obj_t  *ui_connectLbl;
static lv_obj_t  *ui_power;
static lv_obj_t  *ui_chips[PRESET_COUNT];
static lv_color_t *s_wheelBuf = nullptr;

static lv_obj_t  *scr_main;
static lv_obj_t  *scr_fx;
static lv_obj_t  *ui_fxBtn[6];
static lv_obj_t  *ui_fxSwA[7];
static lv_obj_t  *ui_fxSwB[7];
static lv_obj_t  *ui_fxSpeedLbl;
static lv_obj_t  *ui_fxStatus;

static int  s_lastUiState   = -1;
static int  s_lastFxShown   = -99;    // FxId last reflected in the effects UI
static int  s_activePreset  = -1;      // which preset chip is highlighted, -1 = none
static int  s_wheelZone     = -1;      // during a drag: 0 = disc, 1 = ring, -1 = none
static bool s_suppressPower  = false;  // stop programmatic toggles re-firing the cb


static void markActivePreset(int idx) {
  if (idx == s_activePreset) return;
  s_activePreset = idx;
  for (int i = 0; i < PRESET_COUNT; i++) {
    bool on = (i == idx);
    lv_obj_set_style_border_color(ui_chips[i], on ? C::white : C::card3, 0);
    lv_obj_set_style_border_width(ui_chips[i], on ? 3 : 2, 0);
  }
}

// Screen angle (deg, atan2 convention) <-> hue.  Red sits at the top of the
// ring and hue increases clockwise, so hue = screenAngle + 90.
static inline int screenAngToHue(float deg)  { int h = (int)(deg + 90.0f + 0.5f); h %= 360; return h < 0 ? h + 360 : h; }
static inline float hueToScreenRad(int hue)  { return (hue - 90) * 0.01745329f; }

// The hue ring is fixed art — paint it (and the dark moat/corners) once.
static void wheelDrawRing() {
  for (int y = 0; y < WHEEL_SZ; y++) {
    for (int x = 0; x < WHEEL_SZ; x++) {
      int dx = x - WHEEL_C, dy = y - WHEEL_C;
      int d2 = dx * dx + dy * dy;
      if (d2 <= WHEEL_RDISC * WHEEL_RDISC) continue;   // disc: left to wheelDrawDisc
      lv_color_t px;
      if (d2 <= WHEEL_ROUT * WHEEL_ROUT && d2 >= WHEEL_RIN * WHEEL_RIN) {
        float ang = atan2f((float)dy, (float)dx) * 57.2957795f;
        px = lv_color_hsv_to_rgb((uint16_t)screenAngToHue(ang), 100, 100);
      } else {
        px = C::card;                                  // moat + corners
      }
      s_wheelBuf[y * WHEEL_SZ + x] = px;
    }
  }
  lv_obj_invalidate(ui_wheel);
}

// The inner disc is a vertical gradient of the selected hue: full colour at the
// top, white at the bottom (top->bottom == saturation 100->0).  Repaint on hue
// change.  y maps linearly, so no trig at all.
static inline int discSatForDy(int dy) {           // dy: -RDISC (top) .. +RDISC (bottom)
  int s = (WHEEL_RDISC - dy) * 100 / (2 * WHEEL_RDISC);
  return s < 0 ? 0 : (s > 100 ? 100 : s);
}
static void wheelDrawDisc() {
  int hue = g_h;
  int lo = WHEEL_C - WHEEL_RDISC, hi = WHEEL_C + WHEEL_RDISC;
  for (int y = lo; y <= hi; y++) {
    int dy = y - WHEEL_C;
    lv_color_t row = lv_color_hsv_to_rgb((uint16_t)hue, (uint8_t)discSatForDy(dy), 100);
    for (int x = lo; x <= hi; x++) {
      int dx = x - WHEEL_C;
      if (dx * dx + dy * dy > WHEEL_RDISC * WHEEL_RDISC) continue;
      s_wheelBuf[y * WHEEL_SZ + x] = row;           // whole row is one colour
    }
  }
  // invalidate only the disc, not the whole 340x340 canvas — a full re-blit is a
  // ~230 KB burst to the framebuffer and that alone can make the panel slip.
  lv_area_t wc; lv_obj_get_coords(ui_wheel, &wc);
  int cx = (wc.x1 + wc.x2) / 2, cy = (wc.y1 + wc.y2) / 2;
  lv_area_t da = { (lv_coord_t)(cx - WHEEL_RDISC), (lv_coord_t)(cy - WHEEL_RDISC),
                   (lv_coord_t)(cx + WHEEL_RDISC), (lv_coord_t)(cy + WHEEL_RDISC) };
  lv_obj_invalidate_area(ui_wheel, &da);
}

static void wheelUpdateDots() {
  float a = hueToScreenRad(g_h);
  lv_obj_align_to(ui_hueDot, ui_wheel, LV_ALIGN_CENTER,
                  (int)(cosf(a) * WHEEL_HUE_R), (int)(sinf(a) * WHEEL_HUE_R));
  int dy = WHEEL_RDISC * (100 - 2 * (int)g_s) / 100;   // top = full colour, bottom = white
  lv_obj_align_to(ui_satDot, ui_wheel, LV_ALIGN_CENTER, 0, dy);
}


static void rgbToHsv(uint32_t hex, int &h, int &s, int &v) {
  lv_color_hsv_t hsv = lv_color_rgb_to_hsv((hex >> 16) & 0xff,
                                           (hex >> 8) & 0xff,
                                           hex & 0xff);
  h = hsv.h; s = hsv.s; v = hsv.v;
}

static void refreshColor() {            // brightness bar + hex track the live colour
  lv_color_t c = lv_color_hsv_to_rgb((uint16_t)g_h, (uint8_t)g_s, 100);
  lv_obj_set_style_bg_color(ui_bright, c, LV_PART_INDICATOR);
  lv_label_set_text_fmt(ui_pct, "%d%%", (int)g_v);
  lv_obj_align(ui_pct, LV_ALIGN_CENTER, 0, 0);   // re-centre after the digits change
  uint32_t v = lv_color_to32(c);
  lv_label_set_text_fmt(ui_hex, "#%02X%02X%02X",
                        (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v);
}

static void pushColor() {               // hand the current intent to core 0
  refreshColor();
  g_colorDirty = true;
}


/* ---- callbacks --------------------------------------------------------- */

static void onWheelTouch(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_indev_t *indev = lv_indev_get_act();
  if (!indev) return;
  lv_point_t p; lv_indev_get_point(indev, &p);
  lv_area_t a; lv_obj_get_coords(ui_wheel, &a);
  float dx = p.x - (a.x1 + a.x2) * 0.5f;
  float dy = p.y - (a.y1 + a.y2) * 0.5f;
  float d  = sqrtf(dx * dx + dy * dy);

  if (code == LV_EVENT_PRESSED)
    s_wheelZone = (d > WHEEL_ROUT + 12) ? -1 : (d >= WHEEL_RHIT ? 1 : 0);
  if (s_wheelZone < 0) return;

  g_fx = FX_NONE;                         // manual control cancels any effect
  if (s_wheelZone == 1) {                 // outer ring: angle = hue
    g_h = screenAngToHue(atan2f(dy, dx) * 57.2957795f);
  } else {                                // inner disc: vertical = saturation
    g_s = discSatForDy((int)dy);
  }
  markActivePreset(-1);
  wheelUpdateDots();
  pushColor();

  if (code == LV_EVENT_RELEASED && s_wheelZone == 1)
    wheelDrawDisc();                      // catch the disc up to the final hue
}

static void onBright(lv_event_t *e) {
  g_v = lv_slider_get_value(ui_bright);
  pushColor();                            // effects read g_v live, so no need to stop them
}

static void onPreset(lv_event_t *e) {
  g_fx = FX_NONE;
  int idx = (int)(intptr_t)lv_event_get_user_data(e);
  uint32_t hex = PRESETS[idx].hex;
  int h, s, v; rgbToHsv(hex, h, s, v);
  g_h = h; g_s = s;
  markActivePreset(idx);
  wheelDrawDisc();
  wheelUpdateDots();
  if (g_bleState == BLE_CONNECTED && !lv_obj_has_state(ui_power, LV_STATE_CHECKED)) {
    s_suppressPower = true;
    lv_obj_add_state(ui_power, LV_STATE_CHECKED);
    s_suppressPower = false;
    g_lampOn = 1;
    g_powerCmdAt = millis();
    bleQueue(EVT_POWER_ON);
  }
  pushColor();
}

static void onConnect(lv_event_t *e) {
  int st = g_bleState;
  if (st == BLE_SCANNING || st == BLE_CONNECTING) return;
  bool active = (st == BLE_CONNECTED || st == BLE_RECONNECTING);
  if (active) g_fx = FX_NONE;
  bleQueue(active ? EVT_DISCONNECT : EVT_CONNECT);
}

static void onPower(lv_event_t *e) {
  if (s_suppressPower) return;
  bool on = lv_obj_has_state(ui_power, LV_STATE_CHECKED);
  if (g_bleState != BLE_CONNECTED) {          // nothing to talk to yet — snap back
    s_suppressPower = true;
    lv_obj_clear_state(ui_power, LV_STATE_CHECKED);
    s_suppressPower = false;
    return;
  }
  g_lampOn = on ? 1 : 0;                      // optimistic; the lamp's notify confirms
  g_powerCmdAt = millis();
  bleQueue(on ? EVT_POWER_ON : EVT_POWER_OFF);
}


/* ---- status poll (runs in the LVGL thread) --------------------------- */

// mirror g_fx into the effects screen (called from statusTimer + the fx buttons)
static void fxRefresh() {
  int fx = g_fx;
  if (fx == s_lastFxShown) return;
  s_lastFxShown = fx;
  for (int i = 0; i < 6; i++)
    lv_obj_set_style_bg_color(ui_fxBtn[i], (fx == i + 1) ? C::blue : C::card2, 0);
  const char *n = "No effect";
  switch (fx) {
    case FX_7FADE:   n = "Playing: Seven-Colour Fade";   break;
    case FX_7STROBE: n = "Playing: Seven-Colour Strobe"; break;
    case FX_7JUMP:   n = "Playing: Seven-Colour Jump";   break;
    case FX_BREATHE: n = "Playing: Breathe";             break;
    case FX_1BLINK:  n = "Playing: Blink";               break;
    case FX_2FADE:   n = "Playing: Two-Colour Fade";     break;
  }
  lv_label_set_text(ui_fxStatus, n);
}

static void statusTimer(lv_timer_t *) {
  // Keep the Power switch mirroring the lamp's real state (g_lampOn is fed by
  // the lamp's status notifications).  -1 (unknown / disconnected) shows as off.
  bool want  = (g_lampOn == 1);
  bool shown = lv_obj_has_state(ui_power, LV_STATE_CHECKED);
  if (want != shown) {
    s_suppressPower = true;
    if (want) lv_obj_add_state(ui_power, LV_STATE_CHECKED);
    else      lv_obj_clear_state(ui_power, LV_STATE_CHECKED);
    s_suppressPower = false;
  }

  fxRefresh();

  int st = g_bleState;
  if (st == s_lastUiState) return;
  s_lastUiState = st;

  const char *txt; lv_color_t dot; const char *btn;
  switch (st) {
    case BLE_CONNECTED:    txt = "Connected";       dot = C::green;  btn = "Disconnect"; break;
    case BLE_SCANNING:     txt = "Scanning...";     dot = C::yellow; btn = "Connect";    break;
    case BLE_CONNECTING:   txt = "Connecting...";   dot = C::yellow; btn = "Connect";    break;
    case BLE_RECONNECTING: txt = "Reconnecting..."; dot = C::yellow; btn = "Stop";       break;
    case BLE_FAILED:       txt = "Couldn't connect"; dot = C::red;   btn = "Retry";   break;
    default:              txt = "Not connected";    dot = C::faint;  btn = "Connect"; break;
  }
  lv_label_set_text(ui_statusText, txt);
  lv_label_set_text(ui_connectLbl, btn);
  lv_obj_set_style_bg_color(ui_statusDot, dot, 0);
}


/* ---- little style helpers ------------------------------------------- */

static lv_obj_t *card(lv_obj_t *parent, int w, int h) {
  lv_obj_t *o = lv_obj_create(parent);
  lv_obj_set_size(o, w, h);
  lv_obj_set_style_bg_color(o, C::card, 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(o, 0, 0);
  lv_obj_set_style_radius(o, 28, 0);
  lv_obj_set_style_pad_all(o, 22, 0);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  return o;
}

static lv_obj_t *label(lv_obj_t *parent, const char *txt,
                       const lv_font_t *font, lv_color_t color) {
  lv_obj_t *l = lv_label_create(parent);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, color, 0);
  return l;
}


static void onEffectsBtn(lv_event_t *e) { lv_scr_load(scr_fx); }
static void onFxBack(lv_event_t *e)     { lv_scr_load(scr_main); }

static void buildUI() {
  Serial.println("[Sunset] buildUI: enter"); Serial.flush();
  lv_obj_t *scr = lv_scr_act();
  scr_main = scr;
  lv_obj_set_style_bg_color(scr, C::bg, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_set_style_text_font(scr, &lv_font_montserrat_16, 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  /* ---- header (one compact row) -------------------------------- */
  lv_obj_t *title = label(scr, "Sunset Lamp", &lv_font_montserrat_28, C::text);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 28, 14);

  ui_statusDot = lv_obj_create(scr);
  lv_obj_set_size(ui_statusDot, 14, 14);
  lv_obj_set_style_radius(ui_statusDot, 7, 0);
  lv_obj_set_style_border_width(ui_statusDot, 0, 0);
  lv_obj_set_style_bg_color(ui_statusDot, C::faint, 0);
  lv_obj_align(ui_statusDot, LV_ALIGN_TOP_LEFT, 30, 56);
  ui_statusText = label(scr, "Not connected", &lv_font_montserrat_16, C::muted);
  lv_obj_align(ui_statusText, LV_ALIGN_TOP_LEFT, 54, 51);

  lv_obj_t *connectBtn = lv_btn_create(scr);
  lv_obj_set_size(connectBtn, 210, 56);
  lv_obj_align(connectBtn, LV_ALIGN_TOP_RIGHT, -22, 12);
  lv_obj_set_style_radius(connectBtn, 28, 0);
  lv_obj_set_style_bg_color(connectBtn, C::white, 0);
  lv_obj_set_style_shadow_width(connectBtn, 0, 0);
  lv_obj_add_event_cb(connectBtn, onConnect, LV_EVENT_CLICKED, nullptr);
  ui_connectLbl = label(connectBtn, "Connect", &lv_font_montserrat_20, C::ink);
  lv_obj_center(ui_connectLbl);

  ui_power = lv_switch_create(scr);
  lv_obj_set_size(ui_power, 100, 48);
  lv_obj_align(ui_power, LV_ALIGN_TOP_RIGHT, -246, 16);   // just left of the Connect button
  lv_obj_set_style_bg_color(ui_power, C::card3, LV_PART_MAIN);
  lv_obj_set_style_bg_color(ui_power, C::green, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_set_style_bg_color(ui_power, C::white, LV_PART_KNOB);
  lv_obj_add_event_cb(ui_power, onPower, LV_EVENT_VALUE_CHANGED, nullptr);

  lv_obj_t *fxBtn = lv_btn_create(scr);
  lv_obj_set_size(fxBtn, 130, 48);
  lv_obj_align(fxBtn, LV_ALIGN_TOP_RIGHT, -360, 16);
  lv_obj_set_style_radius(fxBtn, 24, 0);
  lv_obj_set_style_bg_color(fxBtn, C::card2, 0);
  lv_obj_set_style_shadow_width(fxBtn, 0, 0);
  lv_obj_add_event_cb(fxBtn, onEffectsBtn, LV_EVENT_CLICKED, nullptr);
  lv_obj_center(label(fxBtn, "Effects", &lv_font_montserrat_16, C::text));

  /* ---- left card : the big HSV wheel --------------------------- */
  lv_obj_t *left = card(scr, 400, 384);
  lv_obj_align(left, LV_ALIGN_TOP_LEFT, 22, 88);
  lv_obj_set_style_pad_all(left, 19, 0);

  Serial.println("[Sunset] buildUI: wheel"); Serial.flush();
  ui_wheel = lv_canvas_create(left);
  lv_canvas_set_buffer(ui_wheel, s_wheelBuf, WHEEL_SZ, WHEEL_SZ, LV_IMG_CF_TRUE_COLOR);
  lv_obj_align(ui_wheel, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_flag(ui_wheel, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(ui_wheel, onWheelTouch, LV_EVENT_PRESSED,  nullptr);
  lv_obj_add_event_cb(ui_wheel, onWheelTouch, LV_EVENT_PRESSING, nullptr);
  lv_obj_add_event_cb(ui_wheel, onWheelTouch, LV_EVENT_RELEASED, nullptr);

  for (int k = 0; k < 2; k++) {                 // hue knob + saturation knob
    lv_obj_t *dot = lv_obj_create(left);
    lv_obj_set_size(dot, 34, 34);
    lv_obj_set_style_radius(dot, 17, 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(dot, C::white, 0);
    lv_obj_set_style_border_width(dot, 4, 0);
    lv_obj_set_style_shadow_width(dot, 6, 0);
    lv_obj_set_style_shadow_color(dot, C::ink, 0);
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    if (k == 0) ui_hueDot = dot; else ui_satDot = dot;
  }

  // hex readout in the top-left corner of the wheel's square
  ui_hex = label(left, "#FFFFFF", &lv_font_montserrat_14, C::muted);
  lv_obj_align_to(ui_hex, ui_wheel, LV_ALIGN_TOP_LEFT, 2, 2);

  Serial.println("[Sunset] buildUI: wheel paint"); Serial.flush();
  wheelDrawRing();
  wheelDrawDisc();
  Serial.println("[Sunset] buildUI: wheel done"); Serial.flush();

  /* ---- right card : brightness slider + preset stack ---------- */
  lv_obj_t *right = card(scr, 340, 384);
  lv_obj_align(right, LV_ALIGN_TOP_RIGHT, -22, 88);
  lv_obj_set_style_pad_all(right, 19, 0);

  ui_bright = lv_slider_create(right);
  lv_obj_set_size(ui_bright, 84, 346);
  lv_obj_align(ui_bright, LV_ALIGN_LEFT_MID, 0, 0);
  lv_slider_set_range(ui_bright, 1, 100);
  lv_slider_set_value(ui_bright, g_v, LV_ANIM_OFF);
  lv_obj_set_style_radius(ui_bright, 26, LV_PART_MAIN);
  lv_obj_set_style_radius(ui_bright, 26, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(ui_bright, C::card3, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ui_bright, LV_OPA_TRANSP, LV_PART_KNOB);   // fill colour set live by refreshColor()
  lv_obj_set_style_pad_all(ui_bright, 0, LV_PART_KNOB);
  lv_obj_add_event_cb(ui_bright, onBright, LV_EVENT_VALUE_CHANGED, nullptr);

  // percentage sits *inside* the slider in a dark pill so it reads over any fill
  ui_pct = lv_label_create(ui_bright);
  lv_label_set_text(ui_pct, "80%");
  lv_obj_set_style_text_font(ui_pct, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(ui_pct, C::white, 0);
  lv_obj_set_style_bg_color(ui_pct, C::ink, 0);
  lv_obj_set_style_bg_opa(ui_pct, LV_OPA_70, 0);
  lv_obj_set_style_radius(ui_pct, 11, 0);
  lv_obj_set_style_pad_hor(ui_pct, 9, 0);
  lv_obj_set_style_pad_ver(ui_pct, 5, 0);
  lv_obj_align(ui_pct, LV_ALIGN_CENTER, 0, 0);

  for (int i = 0; i < PRESET_COUNT; i++) {
    uint32_t hx = PRESETS[i].hex;
    int lum = ((int)((hx >> 16) & 0xff) * 30 +
               (int)((hx >> 8) & 0xff) * 59 +
               (int)(hx & 0xff) * 11) / 100;
    lv_obj_t *chip = lv_btn_create(right);
    lv_obj_set_size(chip, 200, 78);
    lv_obj_align(chip, LV_ALIGN_TOP_RIGHT, 0, 2 + i * 88);
    lv_obj_set_style_radius(chip, 14, 0);
    lv_obj_set_style_bg_color(chip, lv_color_hex(hx), 0);
    lv_obj_set_style_border_width(chip, 2, 0);
    lv_obj_set_style_border_color(chip, C::card3, 0);
    lv_obj_set_style_shadow_width(chip, 0, 0);
    lv_obj_add_event_cb(chip, onPreset, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    lv_obj_t *cl = label(chip, PRESETS[i].name, &lv_font_montserrat_20,
                         lum > 150 ? C::ink : C::white);
    lv_obj_center(cl);
    ui_chips[i] = chip;
  }

  wheelUpdateDots();
  refreshColor();
  markActivePreset(-1);
  Serial.println("[Sunset] buildUI: done"); Serial.flush();
}


/* ============================================================================
 *  3b.  EFFECTS SCREEN
 * ==========================================================================*/

static void onFxPick(lv_event_t *e) {
  if (g_bleState != BLE_CONNECTED) return;
  g_fx = (int)(intptr_t)lv_event_get_user_data(e);   // FxId 1..6
  g_fxRestart = true;
  fxRefresh();
}
static void onFxStop(lv_event_t *e) { g_fx = FX_NONE; fxRefresh(); }

static void onFxSpeed(lv_event_t *e) {
  g_fxSpeed = lv_slider_get_value(lv_event_get_target(e));
  lv_label_set_text_fmt(ui_fxSpeedLbl, "Speed  %d%%", (int)g_fxSpeed);
}
static void onFxBright(lv_event_t *e) {
  g_v = lv_slider_get_value(lv_event_get_target(e));
  lv_slider_set_value(ui_bright, g_v, LV_ANIM_OFF);
  refreshColor();
}
static void onFxColA(lv_event_t *e) {
  int idx = (int)(intptr_t)lv_event_get_user_data(e);
  g_fxColA = idx;
  for (int i = 0; i < 7; i++)
    lv_obj_set_style_border_color(ui_fxSwA[i], i == idx ? C::white : C::card3, 0);
  if (g_fx == FX_BREATHE || g_fx == FX_1BLINK || g_fx == FX_2FADE) g_fxRestart = true;
}
static void onFxColB(lv_event_t *e) {
  int idx = (int)(intptr_t)lv_event_get_user_data(e);
  g_fxColB = idx;
  for (int i = 0; i < 7; i++)
    lv_obj_set_style_border_color(ui_fxSwB[i], i == idx ? C::white : C::card3, 0);
  if (g_fx == FX_2FADE) g_fxRestart = true;
}

static void fxSwatchRow(lv_obj_t **arr, lv_event_cb_t cb, int y, int sel) {
  for (int i = 0; i < 7; i++) {
    lv_obj_t *s = lv_obj_create(scr_fx);
    lv_obj_set_size(s, 46, 46);
    lv_obj_align(s, LV_ALIGN_TOP_LEFT, 28 + i * 58, y);
    lv_obj_set_style_radius(s, 23, 0);
    lv_obj_set_style_bg_color(s, lv_color_make(FX_RGB[i][0], FX_RGB[i][1], FX_RGB[i][2]), 0);
    lv_obj_set_style_border_width(s, 3, 0);
    lv_obj_set_style_border_color(s, i == sel ? C::white : C::card3, 0);
    lv_obj_clear_flag(s, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s, cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    arr[i] = s;
  }
}

static void buildFxUI() {
  scr_fx = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr_fx, C::bg, 0);
  lv_obj_set_style_bg_opa(scr_fx, LV_OPA_COVER, 0);
  lv_obj_set_style_text_font(scr_fx, &lv_font_montserrat_16, 0);
  lv_obj_clear_flag(scr_fx, LV_OBJ_FLAG_SCROLLABLE);

  /* header */
  lv_obj_align(label(scr_fx, "Effects", &lv_font_montserrat_28, C::text),
               LV_ALIGN_TOP_LEFT, 28, 14);

  lv_obj_t *back = lv_btn_create(scr_fx);
  lv_obj_set_size(back, 120, 48);
  lv_obj_align(back, LV_ALIGN_TOP_RIGHT, -22, 14);
  lv_obj_set_style_radius(back, 24, 0);
  lv_obj_set_style_bg_color(back, C::white, 0);
  lv_obj_set_style_shadow_width(back, 0, 0);
  lv_obj_add_event_cb(back, onFxBack, LV_EVENT_CLICKED, nullptr);
  lv_obj_center(label(back, "Back", &lv_font_montserrat_20, C::ink));

  lv_obj_t *stop = lv_btn_create(scr_fx);
  lv_obj_set_size(stop, 120, 48);
  lv_obj_align(stop, LV_ALIGN_TOP_RIGHT, -156, 14);
  lv_obj_set_style_radius(stop, 24, 0);
  lv_obj_set_style_bg_color(stop, C::card2, 0);
  lv_obj_set_style_shadow_width(stop, 0, 0);
  lv_obj_add_event_cb(stop, onFxStop, LV_EVENT_CLICKED, nullptr);
  lv_obj_center(label(stop, "Stop", &lv_font_montserrat_20, C::text));

  /* speed + brightness */
  ui_fxSpeedLbl = label(scr_fx, "Speed  50%", &lv_font_montserrat_16, C::muted);
  lv_obj_align(ui_fxSpeedLbl, LV_ALIGN_TOP_LEFT, 28, 74);
  lv_obj_t *spd = lv_slider_create(scr_fx);
  lv_obj_set_size(spd, 500, 16);
  lv_obj_align(spd, LV_ALIGN_TOP_LEFT, 28, 100);
  lv_slider_set_range(spd, 0, 100);
  lv_slider_set_value(spd, g_fxSpeed, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(spd, C::card3, LV_PART_MAIN);
  lv_obj_set_style_bg_color(spd, C::blue, LV_PART_INDICATOR);
  lv_obj_add_event_cb(spd, onFxSpeed, LV_EVENT_VALUE_CHANGED, nullptr);

  lv_obj_align(label(scr_fx, "Brightness", &lv_font_montserrat_16, C::muted),
               LV_ALIGN_TOP_LEFT, 560, 74);
  lv_obj_t *br = lv_slider_create(scr_fx);
  lv_obj_set_size(br, 200, 16);
  lv_obj_align(br, LV_ALIGN_TOP_LEFT, 560, 100);
  lv_slider_set_range(br, 1, 100);
  lv_slider_set_value(br, g_v, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(br, C::card3, LV_PART_MAIN);
  lv_obj_set_style_bg_color(br, C::text, LV_PART_INDICATOR);
  lv_obj_add_event_cb(br, onFxBright, LV_EVENT_VALUE_CHANGED, nullptr);

  /* effect buttons (2 rows x 3) */
  static const char *NM[6] = {
    "Seven-Colour Fade", "Seven-Colour Strobe", "Seven-Colour Jump",
    "Breathe", "Blink", "Two-Colour Fade"
  };
  for (int i = 0; i < 6; i++) {
    lv_obj_t *b = lv_btn_create(scr_fx);
    lv_obj_set_size(b, 240, 66);
    lv_obj_align(b, LV_ALIGN_TOP_LEFT, 28 + (i % 3) * 252, 136 + (i / 3) * 78);
    lv_obj_set_style_radius(b, 16, 0);
    lv_obj_set_style_bg_color(b, C::card2, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_add_event_cb(b, onFxPick, LV_EVENT_CLICKED, (void *)(intptr_t)(i + 1));
    lv_obj_center(label(b, NM[i], &lv_font_montserrat_16, C::text));
    ui_fxBtn[i] = b;
  }

  /* colour swatches */
  lv_obj_align(label(scr_fx, "COLOUR  (Breathe / Blink / Fade A)",
                     &lv_font_montserrat_16, C::faint), LV_ALIGN_TOP_LEFT, 28, 296);
  fxSwatchRow(ui_fxSwA, onFxColA, 320, g_fxColA);

  lv_obj_align(label(scr_fx, "COLOUR B  (Two-Colour Fade)",
                     &lv_font_montserrat_16, C::faint), LV_ALIGN_TOP_LEFT, 28, 376);
  fxSwatchRow(ui_fxSwB, onFxColB, 400, g_fxColB);

  ui_fxStatus = label(scr_fx, "No effect", &lv_font_montserrat_16, C::muted);
  lv_obj_align(ui_fxStatus, LV_ALIGN_TOP_LEFT, 28, 452);
}


/* ============================================================================
 *  4.  setup / loop
 * ==========================================================================*/

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 800) { }
  Serial.println("\n[Sunset] boot");

  // --- Touch FIRST.  bbct.init() also brings up the I2C bus (Wire on pins
  //     8/9) that the CH422G expander then shares.  This order — touch, then
  //     expander, then display — matches the proven Waveshare ESP32-S3-Touch-
  //     LCD-4.3 Arduino example.  The GT911 is already out of reset at power-on
  //     (CH422G leaves EXIO1 high), so it answers here.  Retry a couple of
  //     times in case it is still booting. ---
  int touchType = CT_TYPE_UNKNOWN;
  for (int i = 0; i < 3 && touchType != CT_TYPE_GT911; i++) {
    if (i) delay(120);
    bbct.init(TOUCH_SDA, TOUCH_SCL, TOUCH_RST, TOUCH_INT);
    touchType = bbct.sensorType();
  }
  Serial.printf("[Sunset] touch type=%d (%d = GT911)\n", touchType, CT_TYPE_GT911);
  if (touchType != CT_TYPE_GT911)
    Serial.println("[Sunset] !! GT911 not detected — touch will not work");

  // --- CH422G expander : owns the backlight + reset lines.  begin() already
  //     drives IO0..7 output-high; multiPinMode makes the intent explicit and
  //     matches the reference driver. ---
  expander = new ESP_IOExpander_CH422G((i2c_port_t)0, 0x24);   // addr is ignored by the CH422G driver
  expander->init();
  expander->begin();
  expander->multiPinMode(EXP_TP_RST | EXP_LCD_BL | EXP_LCD_RST | EXP_SD_CS | EXP_USB_SEL, OUTPUT);
  Serial.println("[Sunset] .. expander");
  backlight(false);                 // dark until the first frame is drawn
  delay(120);

  // --- RGB panel ---
  if (!gfx->begin()) {
    Serial.println("[Sunset] gfx->begin() FAILED — check board / core / PSRAM");
    while (true) delay(1000);
  }
  gfx->fillScreen(BLACK);
  Serial.println("[Sunset] .. gfx"); Serial.flush();

  // --- LVGL ---
  lv_init();
  Serial.println("[Sunset] .. lv_init"); Serial.flush();
  const uint32_t bufPx = TFT_W * 32;
  s_buf = (lv_color_t *)heap_caps_malloc(bufPx * sizeof(lv_color_t),
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!s_buf)
    s_buf = (lv_color_t *)heap_caps_malloc(bufPx * sizeof(lv_color_t), MALLOC_CAP_8BIT);
  if (!s_buf) {
    Serial.println("[Sunset] LVGL draw-buffer alloc FAILED — need PSRAM / free heap");
    while (true) delay(1000);
  }
  Serial.println("[Sunset] .. draw buf"); Serial.flush();
  lv_disp_draw_buf_init(&s_drawBuf, s_buf, nullptr, bufPx);

  lv_disp_drv_init(&s_dispDrv);
  s_dispDrv.hor_res  = TFT_W;
  s_dispDrv.ver_res  = TFT_H;
  s_dispDrv.flush_cb = lvglFlush;
  s_dispDrv.draw_buf = &s_drawBuf;
  lv_disp_drv_register(&s_dispDrv);

  lv_indev_drv_init(&s_indevDrv);
  s_indevDrv.type    = LV_INDEV_TYPE_POINTER;
  s_indevDrv.read_cb = lvglTouch;
  lv_indev_drv_register(&s_indevDrv);
  Serial.println("[Sunset] .. drv reg"); Serial.flush();

  // colour-wheel canvas buffer (static art, lives in PSRAM)
  size_t wheelBytes = (size_t)WHEEL_SZ * WHEEL_SZ * sizeof(lv_color_t);
  s_wheelBuf = (lv_color_t *)heap_caps_malloc(wheelBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!s_wheelBuf)
    s_wheelBuf = (lv_color_t *)heap_caps_malloc(wheelBytes, MALLOC_CAP_8BIT);
  Serial.printf("[Sunset] wheel buf %u -> %p\n", (unsigned)wheelBytes, s_wheelBuf);
  Serial.flush();
  if (!s_wheelBuf) { while (true) delay(1000); }

  buildUI();
  buildFxUI();
  lv_timer_create(statusTimer, 200, nullptr);
  Serial.println("[Sunset] .. refr"); Serial.flush();

  lv_refr_now(nullptr);
  backlight(true);
  s_lastTouchMs = millis();          // start the sleep countdown from boot
  Serial.println("[Sunset] .. ui up"); Serial.flush();

  // --- Bluetooth on the other core ---
  g_evtQ = xQueueCreate(8, sizeof(uint8_t));
  xTaskCreatePinnedToCore(bleTask, "ble", 12288, nullptr, 1, nullptr, 0);

  Serial.println("[Sunset] ready");
}

void loop() {
  lv_timer_handler();
  if (!s_asleep && millis() - s_lastTouchMs > SLEEP_AFTER_MS)
    screenSleep();
  delay(5);
}
