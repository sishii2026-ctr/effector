#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "FspTimer.h"
#include "arduinoFFT.h"
#include <math.h>
#include <WiFiS3.h>
#include <EEPROM.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_ADDR     0x3C

#define FFT_SAMPLES   256
#define FFT_BANDS     16
#define SAMPLE_RATE   20000.0f

// ── 機能有効化スイッチ ─────────────────────────────
// チューナー機能を使わない場合は、下の行をコメントアウトしてください。
// コメントアウトすると、チューナー関連のコード・UI・処理が
// コンパイル時にまるごと除外されます（コードサイズ・処理負荷も削減されます）。
//#define ENABLE_TUNER

// ── プリセット定義 ─────────────────────────────────
#define PRESET_COUNT    5
#define EEPROM_MAGIC    0xA5
#define EEPROM_BASE     0

struct Preset {
  bool  effectOn;
  bool  bitMode;
  bool  tremoloOn;
  int   distPct;
  int   trmSpdPct;
  char  name[12];
};

void savePreset(int slot, const Preset& p) {
  int addr = EEPROM_BASE + 1 + slot * sizeof(Preset);
  EEPROM.put(addr, p);
}

bool loadPreset(int slot, Preset& p) {
  byte magic;
  EEPROM.get(EEPROM_BASE, magic);
  if (magic != EEPROM_MAGIC) return false;
  int addr = EEPROM_BASE + 1 + slot * sizeof(Preset);
  EEPROM.get(addr, p);
  return true;
}

void initEEPROM() {
  byte magic;
  EEPROM.get(EEPROM_BASE, magic);
  if (magic == EEPROM_MAGIC) return;

  Preset def;
  def.effectOn  = false;
  def.bitMode   = false;
  def.tremoloOn = false;
  def.distPct   = 50;
  def.trmSpdPct = 30;

  const char* names[PRESET_COUNT] = { "Preset 1", "Preset 2", "Preset 3", "Preset 4", "Preset 5" };
  for (int i = 0; i < PRESET_COUNT; i++) {
    strncpy(def.name, names[i], sizeof(def.name) - 1);
    def.name[sizeof(def.name) - 1] = '\0';
    savePreset(i, def);
  }
  EEPROM.put(EEPROM_BASE, (byte)EEPROM_MAGIC);
}

// ── Wi-Fi設定 ──────────────────────────────────────
#include "mywifi.h"
WiFiServer server(80);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
ArduinoFFT<float> FFT;

const int DAC_OUT  = A0;
const int POT_PIN  = A1;
const int POT_TRM  = A2;
const int ADC_IN   = A3;
const int BTN_PIN2 = 13;
const int BTN_PIN  = 12;
const int BTN_PIN3 = 11;
const int DC_BIAS  = 2048;

const unsigned long INTERVAL = 100;
unsigned long prevTime = 0;

volatile bool effectOn  = false;
volatile bool bitMode   = false;
volatile bool tremoloOn = false;
volatile bool tunerMode = false;   // ★ チューナーモード（ENABLE_TUNER無効時は常にfalseのまま）

bool lastBtnState  = HIGH;
bool lastBtnState2 = HIGH;
bool lastBtnState3 = HIGH;
unsigned long lastDebounceTime  = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;

// ★ ボタン3 長押し検出用（短押し=トレモロ, 長押し=チューナー）
unsigned long btn3PressStart   = 0;
bool          btn3LongPressFired = false;
const unsigned long LONG_PRESS_MS = 600;

volatile int potVal  = 0;
volatile int potTrm  = 0;
volatile int outVal  = DC_BIAS;

volatile int distParam  = 0;
int          webDistVal = -1;
bool         webControl = false;
int          lastPotVal = 0;

bool  webTrmControl = false;
int   webTrmVal     = -1;

volatile float lfoPhase     = 0.0f;
volatile float lfoPhaseStep = 0.0f;

int activePresetSlot = -1;

float vReal[FFT_SAMPLES];
float vImag[FFT_SAMPLES];

volatile int fftBuf[FFT_SAMPLES];
volatile int fftIndex = 0;
volatile bool fftReady = false;

FspTimer audioTimer;

// ── ★ チューナー関連（ENABLE_TUNER時のみコンパイル） ──────
#ifdef ENABLE_TUNER

struct GuitarString {
  const char* name;
  float freq;
};

const int STRING_COUNT = 6;
GuitarString GUITAR_STRINGS[STRING_COUNT] = {
  {"6E", 82.41f},   // 6弦 E2
  {"5A", 110.00f},  // 5弦 A2
  {"4D", 146.83f},  // 4弦 D3
  {"3G", 196.00f},  // 3弦 G3
  {"2B", 246.94f},  // 2弦 B3
  {"1E", 329.63f},  // 1弦 E4
};

float detectedFreq     = 0;
int   matchedStringIdx = -1;
int   centsOffset      = 0;
bool  tunerSignalOK    = false;

// ★ チューナーのノイズゲート閾値（小さいほど弱い音でも検出する）
//    実機でシリアルモニタのRMS値を見ながら調整してください
float TUNER_RMS_THRESHOLD = 3.0f;

float autocorrAt(float* buf, int lag) {
  float c = 0;
  for (int i = 0; i < FFT_SAMPLES - lag; i++) c += buf[i] * buf[i + lag];
  return c;
}

float detectPitchAutocorr() {
  static float buf[FFT_SAMPLES];
  float mean = 0;
  for (int i = 0; i < FFT_SAMPLES; i++) mean += fftBuf[i];
  mean /= FFT_SAMPLES;
  for (int i = 0; i < FFT_SAMPLES; i++) buf[i] = fftBuf[i] - mean;

  float rms = 0;
  for (int i = 0; i < FFT_SAMPLES; i++) rms += buf[i] * buf[i];
  rms = sqrtf(rms / FFT_SAMPLES);

  // ★ デバッグ: 実際のRMS値を確認する（原因切り分け後は削除してOK）
  Serial.print("RMS: "); Serial.println(rms);

  if (rms < TUNER_RMS_THRESHOLD) {
    tunerSignalOK = false;
    return 0;
  }
  tunerSignalOK = true;

  int minLag = (int)(SAMPLE_RATE / 400.0f);  // 高音側上限 ~400Hz
  int maxLag = (int)(SAMPLE_RATE / 70.0f);   // 低音側下限 ~70Hz
  maxLag = min(maxLag, FFT_SAMPLES - 2);
  if (minLag < 1) minLag = 1;

  float bestCorr = -1e9;
  int   bestLag  = 0;

  for (int lag = minLag; lag <= maxLag; lag++) {
    float corr = autocorrAt(buf, lag);
    if (corr > bestCorr) {
      bestCorr = corr;
      bestLag = lag;
    }
  }

  if (bestLag <= 1) return 0;

  float c0 = autocorrAt(buf, bestLag - 1);
  float c1 = autocorrAt(buf, bestLag);
  float c2 = autocorrAt(buf, bestLag + 1);
  float denom = (c0 - 2.0f * c1 + c2);
  float delta = (denom != 0) ? 0.5f * (c0 - c2) / denom : 0.0f;
  float refinedLag = bestLag + delta;

  if (refinedLag <= 0) return 0;
  return SAMPLE_RATE / refinedLag;
}

void matchStringAndCents() {
  float freq = detectPitchAutocorr();
  detectedFreq = freq;

  if (!tunerSignalOK || freq <= 0) {
    matchedStringIdx = -1;
    centsOffset = 0;
    return;
  }

  int   bestIdx = -1;
  float bestDiff = 1e9;
  for (int i = 0; i < STRING_COUNT; i++) {
    float diff = fabsf(log2f(freq / GUITAR_STRINGS[i].freq));
    if (diff < bestDiff) {
      bestDiff = diff;
      bestIdx = i;
    }
  }

  matchedStringIdx = bestIdx;

  float target = GUITAR_STRINGS[bestIdx].freq;
  centsOffset = (int)roundf(1200.0f * log2f(freq / target));
  centsOffset = constrain(centsOffset, -99, 99);
}

#endif // ENABLE_TUNER

// ── 音声割り込み ──────────────────────────────────
void audioISR(timer_callback_args_t *args) {
  int raw = analogRead(ADC_IN);
  int POT = distParam;
  int out = DC_BIAS;

#ifdef ENABLE_TUNER
  if (tunerMode) {
    // チューナー中はドライ音をそのまま出力（エフェクト無効）
    int centered = raw - DC_BIAS;
    out = constrain(centered + DC_BIAS, 0, 4095);
  } else if (effectOn) {
#else
  if (effectOn) {
#endif
    if (!bitMode) {
      float distortion = map(POT, 0, 4095, 1, 100);
      int   centered   = raw - DC_BIAS;
      float amp        = centered * distortion;
      float clipped    = constrain(amp, -2000, 2000);
      out = constrain((int)clipped + DC_BIAS, 0, 4095);

    } else {
      static float smoothed = DC_BIAS;
      smoothed = smoothed * 0.6f + raw * 0.4f;
      int smoothedRaw = (int)smoothed;

      static int holdVal   = DC_BIAS;
      static int holdCount = 0;
      if (holdCount++ >= 6) {
        holdVal   = smoothedRaw;
        holdCount = 0;
      }

      int centered  = holdVal - DC_BIAS;
      int amplified = centered * 10;
      out = constrain(amplified + DC_BIAS, 0, 4095);
      raw = holdVal;
    }
  } else {
    int centered  = raw - DC_BIAS;
    int amplified = centered * 10;
    out = constrain(amplified + DC_BIAS, 0, 4095);
  }

#ifdef ENABLE_TUNER
  if (tremoloOn && !tunerMode) {
#else
  if (tremoloOn) {
#endif
    int centered = out - DC_BIAS;
    float depth = 0.7f;
    float lfo = 1.0f - depth + depth * (sinf(lfoPhase) + 1.0f) * 0.5f;
    int modulated = (int)(centered * lfo);
    out = constrain(modulated + DC_BIAS, 0, 4095);

    lfoPhase += lfoPhaseStep;
    if (lfoPhase >= 2.0f * (float)M_PI) {
      lfoPhase -= 2.0f * (float)M_PI;
    }
  }

  analogWrite(DAC_OUT, out);
  outVal = out;

  if (!fftReady) {
    fftBuf[fftIndex++] = raw;
    if (fftIndex >= FFT_SAMPLES) {
      fftIndex = 0;
      fftReady = true;
    }
  }
}

// ── プリセット操作をHTTPリクエスト内で実行 ────────
void handlePreset(const String& req) {
  if (req.indexOf("/preset/save") >= 0) {
    int si = req.indexOf("slot=");
    if (si < 0) return;
    int slot = constrain(req.substring(si + 5).toInt(), 0, PRESET_COUNT - 1);

    char pname[12] = "";
    int ni = req.indexOf("name=");
    if (ni >= 0) {
      String raw = req.substring(ni + 5);
      int amp = raw.indexOf('&');
      if (amp >= 0) raw = raw.substring(0, amp);
      int spaceEnd = raw.indexOf(' ');
      if (spaceEnd >= 0) raw = raw.substring(0, spaceEnd);
      raw.replace("+", " ");
      raw.toCharArray(pname, sizeof(pname));
    }
    if (strlen(pname) == 0) {
      snprintf(pname, sizeof(pname), "Preset %d", slot + 1);
    }

    Preset p;
    p.effectOn  = effectOn;
    p.bitMode   = bitMode;
    p.tremoloOn = tremoloOn;
    p.distPct   = map(distParam, 0, 4095, 0, 100);
    p.trmSpdPct = webTrmControl ? webTrmVal : map(potTrm, 0, 4095, 0, 100);
    strncpy(p.name, pname, sizeof(p.name) - 1);
    p.name[sizeof(p.name) - 1] = '\0';

    savePreset(slot, p);
    activePresetSlot = slot;
    Serial.print("Preset saved: slot="); Serial.print(slot);
    Serial.print(" name="); Serial.println(p.name);

  } else if (req.indexOf("/preset/load") >= 0) {
    int si = req.indexOf("slot=");
    if (si < 0) return;
    int slot = constrain(req.substring(si + 5).toInt(), 0, PRESET_COUNT - 1);

    Preset p;
    if (loadPreset(slot, p)) {
      effectOn  = p.effectOn;
      bitMode   = p.bitMode;
      tremoloOn = p.tremoloOn;

      webDistVal = p.distPct;
      webControl = true;
      distParam  = map(p.distPct, 0, 100, 0, 4095);

      webTrmVal     = p.trmSpdPct;
      webTrmControl = true;
      float hz      = 0.5f + (p.trmSpdPct / 100.0f) * 9.5f;
      lfoPhaseStep  = 2.0f * (float)M_PI * hz / SAMPLE_RATE;

      Serial.print("Preset loaded: slot="); Serial.print(slot);
      Serial.print(" name="); Serial.println(p.name);
      activePresetSlot = slot;
    }
  }
}

// ── JSON（モニター用） ────────────────────────────
void sendJSON(WiFiClient& client) {
  int distPct = map(distParam, 0, 4095, 0, 100);
  int trmPct  = map(potTrm,   0, 4095, 0, 100);

  client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n");

  String json = "{";
  json += "\"effect\":"  + String(effectOn      ? "true" : "false") + ",";
  json += "\"bitMode\":" + String(bitMode       ? "true" : "false") + ",";
  json += "\"tremolo\":" + String(tremoloOn     ? "true" : "false") + ",";
  json += "\"webCtrl\":" + String(webControl    ? "true" : "false") + ",";
  json += "\"webTrm\":"  + String(webTrmControl ? "true" : "false") + ",";
  json += "\"dist\":"    + String(distPct) + ",";
  json += "\"trm\":"     + String(trmPct) + ",";
#ifdef ENABLE_TUNER
  // ★ チューナー情報
  json += "\"tuner\":"   + String(tunerMode ? "true" : "false") + ",";
  json += "\"note\":\""  + String(matchedStringIdx >= 0 ? GUITAR_STRINGS[matchedStringIdx].name : "--") + "\",";
  json += "\"cents\":"   + String(centsOffset) + ",";
  json += "\"freq\":"    + String((int)detectedFreq);
#else
  json += "\"tuner\":false,\"note\":\"--\",\"cents\":0,\"freq\":0";
#endif
  json += "}";
  client.print(json);
}

// ── プリセット一覧JSONを返す ──────────────────────
void sendPresetsJSON(WiFiClient& client) {
  client.print("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n");
  client.print("{\"activeSlot\":");
  client.print(activePresetSlot);
  client.print(",\"presets\":[");
  for (int i = 0; i < PRESET_COUNT; i++) {
    Preset p;
    bool ok = loadPreset(i, p);
    if (i > 0) client.print(",");
    client.print("{\"slot\":");
    client.print(i);
    client.print(",\"name\":\"");
    client.print(ok ? p.name : "---");
    client.print("\",\"effectOn\":");
    client.print((ok && p.effectOn) ? "true" : "false");
    client.print(",\"bitMode\":");
    client.print((ok && p.bitMode)  ? "true" : "false");
    client.print(",\"tremoloOn\":");
    client.print((ok && p.tremoloOn)? "true" : "false");
    client.print(",\"dist\":");
    client.print(ok ? p.distPct : 0);
    client.print(",\"trm\":");
    client.print(ok ? p.trmSpdPct : 0);
    client.print("}");
  }
  client.print("]}");
}

// ── HTML送信 ─────────────────────────────────────
void sendHTML(WiFiClient& client) {
  int distPct = map(distParam, 0, 4095, 0, 100);
  int trmPct  = webTrmControl ? webTrmVal : map(potTrm, 0, 4095, 0, 100);

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html; charset=utf-8");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE html><html><head>");
  client.println("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  client.println("<title>Effect Controller</title>");
  client.println("<style>");
  client.println("body{font-family:sans-serif;background:#111;color:#eee;text-align:center;padding:20px;}");
  client.println("h1{color:#0f0;letter-spacing:2px;}");
  client.println(".card{background:#222;border-radius:12px;padding:16px;margin:12px auto;max-width:340px;}");
  client.println(".row{display:flex;justify-content:center;gap:10px;margin:8px 0;flex-wrap:wrap;}");
  client.println("button{padding:10px 20px;border:none;border-radius:8px;font-size:15px;cursor:pointer;}");
  client.println(".on{background:#0a0;color:#fff;} .off{background:#a00;color:#fff;}");
  client.println(".mode{background:#036;color:#fff;}");
  client.println(".tab{background:#333;color:#aaa;font-size:14px;padding:10px 24px;border-radius:8px;}");
  client.println(".tab.active{background:#0f0;color:#111;font-weight:bold;}");
  client.println("input[type=range]{width:90%;accent-color:#0f0;}");
  client.println("input[type=text]{background:#333;color:#eee;border:1px solid #555;border-radius:6px;padding:6px 10px;font-size:14px;width:60%;}");
  client.println(".label{font-size:12px;color:#888;margin-top:4px;}");
  client.println(".status{font-size:13px;color:#ff0;margin-top:6px;}");
  client.println(".monitor{font-size:28px;font-weight:bold;color:#0f0;margin:8px 0;}");
  client.println(".monitor-label{font-size:11px;color:#666;margin-bottom:4px;}");
  client.println(".badge{display:inline-block;padding:4px 12px;border-radius:20px;font-size:13px;font-weight:bold;margin:4px;}");
  client.println(".badge-on{background:#0a0;color:#fff;} .badge-off{background:#333;color:#888;}");
  client.println(".preset-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px;}");
  client.println(".preset-btn{background:#1a1a2e;border:1px solid #444;border-radius:8px;padding:8px;text-align:left;cursor:pointer;transition:border-color .2s,box-shadow .2s;}");
  client.println(".preset-btn:hover{border-color:#0f0;}");
  client.println(".preset-btn.selected{border-color:#0f0;border-width:2px;box-shadow:0 0 8px rgba(0,255,0,.5);background:#16291a;}");
  client.println(".preset-btn .pname{font-size:13px;font-weight:bold;color:#0f0;margin-bottom:4px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;}");
  client.println(".preset-btn .pinfo{font-size:10px;color:#666;line-height:1.5;}");
  client.println(".preset-save-area{display:flex;align-items:center;gap:6px;margin-top:10px;justify-content:center;}");
  client.println(".save-btn{background:#550;color:#ff0;padding:8px 14px;font-size:13px;border-radius:8px;border:none;cursor:pointer;}");
  client.println(".save-btn:hover{background:#770;}");
  client.println(".slot-sel{background:#333;color:#eee;border:1px solid #555;border-radius:6px;padding:6px 8px;font-size:13px;}");
#ifdef ENABLE_TUNER
  // ★ チューナーUI用スタイル
  client.println(".tuner-note{font-size:64px;font-weight:bold;color:#0f0;margin:10px 0;}");
  client.println(".tuner-note.off{color:#666;}");
  client.println(".tuner-meter{position:relative;height:24px;background:#333;border-radius:4px;margin:10px 0;overflow:hidden;}");
  client.println(".tuner-meter .center-line{position:absolute;left:50%;top:0;bottom:0;width:2px;background:#555;}");
  client.println(".tuner-meter .needle{position:absolute;top:2px;bottom:2px;width:6px;border-radius:3px;background:#ff0;left:calc(50% - 3px);transition:left .1s;}");
  client.println(".tuner-meter .needle.intune{background:#0f0;}");
  client.println(".tuner-freq{font-size:14px;color:#aaa;}");
#endif
  client.println("</style></head><body>");

  client.println("<h1>🎸 Effect Controller</h1>");

  client.println("<div class='row' style='margin-bottom:16px;'>");
  client.println("<button class='tab active' id='tab-ctrl'  onclick='switchTab(\"ctrl\")'>⚙️ 操作</button>");
  client.println("<button class='tab'        id='tab-pre'   onclick='switchTab(\"pre\")' >🎛️ プリセット</button>");
#ifdef ENABLE_TUNER
  client.println("<button class='tab'        id='tab-tuner' onclick='switchTab(\"tuner\")'>🎵 チューナー</button>");
#endif
  client.println("<button class='tab'        id='tab-mon'   onclick='switchTab(\"mon\")' >👁️ 表示</button>");
  client.println("</div>");

  // ════════════════════════════════
  // 操作モード
  // ════════════════════════════════
  client.println("<div id='ctrl-panel'>");

  client.println("<div class='card'>");
  client.println("<div class='label'>EFFECT</div><div class='row'>");
  client.println("<button class='on'  onclick=\"go('/set?effect=1')\">ON</button>");
  client.println("<button class='off' onclick=\"go('/set?effect=0')\">OFF</button>");
  client.println("</div>");
  client.print("<div class='status' id='st-effect'>現在: "); client.print(effectOn ? "ON" : "OFF"); client.println("</div></div>");

  client.println("<div class='card'>");
  client.println("<div class='label'>MODE</div><div class='row'>");
  client.println("<button class='mode' onclick=\"go('/set?mode=dist')\">DIST</button>");
  client.println("<button class='mode' onclick=\"go('/set?mode=bit')\">BIT</button>");
  client.println("</div>");
  client.print("<div class='status' id='st-mode'>現在: "); client.print(bitMode ? "BIT" : "DIST"); client.println("</div></div>");

  client.println("<div class='card'>");
  client.println("<div class='label'>TREMOLO</div><div class='row'>");
  client.println("<button class='on'  onclick=\"go('/set?trm=1')\">ON</button>");
  client.println("<button class='off' onclick=\"go('/set?trm=0')\">OFF</button>");
  client.println("</div>");
  client.print("<div class='status' id='st-trm'>現在: "); client.print(tremoloOn ? "ON" : "OFF"); client.println("</div>");
  client.println("<div class='label'>TREMOLO SPEED</div>");
  client.print("<input type='range' min='0' max='100' value='"); client.print(trmPct);
  client.println("' oninput='sendTrm(this.value)'>");
  client.print("<div class='status' id='tv'>"); client.print(trmPct); client.println("%</div>");
  client.print("<div class='label'>"); client.print(webTrmControl ? "⚡ WEB制御中" : "🎛️ POT制御中"); client.println("</div></div>");

  client.println("<div class='card'>");
  client.println("<div class='label'>DIST AMOUNT</div>");
  client.print("<input type='range' min='0' max='100' value='"); client.print(distPct);
  client.println("' oninput='sendDist(this.value)'>");
  client.print("<div class='status' id='dv'>"); client.print(distPct); client.println("%</div>");
  client.print("<div class='label'>"); client.print(webControl ? "⚡ WEB制御中" : "🎛️ POT制御中"); client.println("</div></div>");

#ifdef ENABLE_TUNER
  // ★ チューナー起動ボタンも操作パネルに置いておく
  client.println("<div class='card'>");
  client.println("<div class='label'>TUNER</div><div class='row'>");
  client.println("<button class='on'  onclick=\"go('/set?tuner=1')\">起動</button>");
  client.println("<button class='off' onclick=\"go('/set?tuner=0')\">停止</button>");
  client.println("</div>");
  client.print("<div class='status' id='st-tuner'>現在: "); client.print(tunerMode ? "ON" : "OFF"); client.println("</div></div>");
#endif

  client.println("</div>"); // ctrl-panel終了

  // ════════════════════════════════
  // プリセットモード
  // ════════════════════════════════
  client.println("<div id='pre-panel' style='display:none;'>");

  client.println("<div class='card'>");
  client.println("<div class='label'>📂 プリセットを呼び出す</div>");
  client.println("<div class='preset-grid' id='preset-grid'>読み込み中...</div>");
  client.println("</div>");

  client.println("<div class='card'>");
  client.println("<div class='label'>💾 現在の設定を保存する</div>");
  client.println("<div class='preset-save-area'>");
  client.println("<input type='text' id='pname' maxlength='11' placeholder='プリセット名'>");
  client.println("<select class='slot-sel' id='pslot'>");
  for (int i = 0; i < PRESET_COUNT; i++) {
    client.print("<option value='"); client.print(i); client.print("'>スロット "); client.print(i + 1); client.println("</option>");
  }
  client.println("</select>");
  client.println("<button class='save-btn' onclick='savePreset()'>保存</button>");
  client.println("</div>");
  client.println("<div class='status' id='save-msg'></div>");
  client.println("</div>");

  client.println("</div>"); // pre-panel終了

#ifdef ENABLE_TUNER
  // ════════════════════════════════
  // ★ チューナーモード
  // ════════════════════════════════
  client.println("<div id='tuner-panel' style='display:none;'>");
  client.println("<div class='card'>");
  client.println("<div class='label'>🎵 GUITAR TUNER</div>");
  client.println("<div class='tuner-note off' id='tuner-note'>--</div>");
  client.println("<div class='tuner-meter'><div class='center-line'></div><div class='needle' id='tuner-needle'></div></div>");
  client.println("<div class='tuner-freq' id='tuner-freq'>-- Hz / -- cent</div>");
  client.println("<div class='row' style='margin-top:14px;'>");
  client.println("<button class='on'  onclick=\"go('/set?tuner=1')\">チューナーON</button>");
  client.println("<button class='off' onclick=\"go('/set?tuner=0')\">チューナーOFF</button>");
  client.println("</div>");
  client.println("</div>");
  client.println("</div>"); // tuner-panel終了
#endif

  // ════════════════════════════════
  // 表示モード
  // ════════════════════════════════
  client.println("<div id='mon-panel' style='display:none;'>");

  client.println("<div class='card'>");
  client.println("<div class='label'>ボタン状態</div><div style='margin:10px 0;'>");
  client.print("<span class='badge "); client.print(effectOn ? "badge-on" : "badge-off");
  client.println("' id='b-effect'>EFFECT: "); client.print(effectOn ? "ON" : "OFF"); client.println("</span>");
  client.print("<span class='badge "); client.print(!bitMode ? "badge-on" : "badge-off");
  client.println("' id='b-dist'>DIST</span>");
  client.print("<span class='badge "); client.print(bitMode ? "badge-on" : "badge-off");
  client.println("' id='b-bit'>BIT</span>");
  client.print("<span class='badge "); client.print(tremoloOn ? "badge-on" : "badge-off");
  client.println("' id='b-trm'>TREMOLO: "); client.print(tremoloOn ? "ON" : "OFF"); client.println("</span>");
#ifdef ENABLE_TUNER
  client.print("<span class='badge "); client.print(tunerMode ? "badge-on" : "badge-off");
  client.println("' id='b-tuner'>TUNER: "); client.print(tunerMode ? "ON" : "OFF"); client.println("</span>");
#endif
  client.println("</div></div>");

  client.println("<div class='card'>");
  client.println("<div class='label'>ポテンショメーター</div>");
  client.println("<div class='monitor-label'>DIST (POT)</div>");
  client.print("<div class='monitor' id='m-dist'>"); client.print(distPct); client.println("%</div>");
  client.println("<div class='monitor-label'>TREMOLO SPEED (POT)</div>");
  client.print("<div class='monitor' id='m-trm'>"); client.print(map(potTrm, 0, 4095, 0, 100)); client.println("%</div>");
  client.println("</div>");

  client.println("<div class='card'>");
  client.println("<div class='label'>制御元</div><div style='margin:8px 0;'>");
  client.print("<span class='badge "); client.print(webControl ? "badge-on" : "badge-off");
  client.println("' id='b-wctrl'>DIST: "); client.print(webControl ? "WEB" : "POT"); client.println("</span>");
  client.print("<span class='badge "); client.print(webTrmControl ? "badge-on" : "badge-off");
  client.println("' id='b-wtrm'>TRM SPD: "); client.print(webTrmControl ? "WEB" : "POT"); client.println("</span>");
  client.println("</div></div>");

  client.println("</div>"); // mon-panel終了

  // ── JavaScript ──────────────────────────────────
  client.println("<script>");
  client.println("function go(url){fetch(url).then(()=>location.reload());}");

  client.println("let t=null,t2=null;");
  client.println("function sendDist(v){document.getElementById('dv').innerText=v+'%';clearTimeout(t);t=setTimeout(()=>fetch('/set?dist='+v),500);}");
  client.println("function sendTrm(v){document.getElementById('tv').innerText=v+'%';clearTimeout(t2);t2=setTimeout(()=>fetch('/set?trmspd='+v),500);}");

  client.println("function loadPresetList(){");
  client.println("  fetch('/presets')");
  client.println("  .then(r=>r.json())");
  client.println("  .then(data=>{");
  client.println("    var g=document.getElementById('preset-grid');");
  client.println("    g.innerHTML='';");
  client.println("    data.presets.forEach(function(p){");
  client.println("      var modes=[];");
  client.println("      if(p.effectOn) modes.push('EFFECT ON');");
  client.println("      modes.push(p.bitMode?'BIT':'DIST');");
  client.println("      if(!p.bitMode) modes.push('DIST '+p.dist+'%');");
  client.println("      if(p.tremoloOn) modes.push('TRM ON SPD '+p.trm+'%');");
  client.println("      var info=modes.join(' / ');");
  client.println("      var btn=document.createElement('div');");
  client.println("      btn.className='preset-btn'+(data.activeSlot===p.slot?' selected':'');");
  client.println("      btn.id='preset-card-'+p.slot;");
  client.println("      btn.innerHTML='<div class=\\'pname\\'>'+p.name+'</div><div class=\\'pinfo\\'>'+info+'</div>';");
  client.println("      btn.onclick=function(){loadSlot(p.slot);};");
  client.println("      g.appendChild(btn);");
  client.println("    });");
  client.println("  })");
  client.println("  .catch(function(){document.getElementById('preset-grid').innerText='取得失敗';});");
  client.println("}");

  client.println("function loadSlot(slot){");
  client.println("  document.querySelectorAll('.preset-btn').forEach(function(el){el.classList.remove('selected');});");
  client.println("  var card=document.getElementById('preset-card-'+slot);");
  client.println("  if(card) card.classList.add('selected');");
  client.println("  fetch('/preset/load?slot='+slot)");
  client.println("  .then(()=>{");
  client.println("    refreshUIFromDevice();");
  client.println("    loadPresetList();");
  client.println("  });");
  client.println("}");

  client.println("function refreshUIFromDevice(){");
  client.println("  fetch('/monitor').then(r=>r.json()).then(d=>{");
  client.println("    document.getElementById('dv').innerText=d.dist+'%';");
  client.println("    document.getElementById('tv').innerText=d.trm+'%';");
  client.println("    document.getElementById('st-effect').innerText='現在: '+(d.effect?'ON':'OFF');");
  client.println("    document.getElementById('st-mode').innerText='現在: '+(d.bitMode?'BIT':'DIST');");
  client.println("    document.getElementById('st-trm').innerText='現在: '+(d.tremolo?'ON':'OFF');");
#ifdef ENABLE_TUNER
  client.println("    document.getElementById('st-tuner').innerText='現在: '+(d.tuner?'ON':'OFF');");
#endif
  client.println("  }).catch(()=>{});");
  client.println("}");

  client.println("function savePreset(){");
  client.println("  var name=encodeURIComponent(document.getElementById('pname').value||'Preset');");
  client.println("  var slot=document.getElementById('pslot').value;");
  client.println("  fetch('/preset/save?slot='+slot+'&name='+name)");
  client.println("  .then(r=>r.text())");
  client.println("  .then(()=>{");
  client.println("    var msg=document.getElementById('save-msg');");
  client.println("    msg.innerText='✅ スロット'+(parseInt(slot)+1)+'に保存しました';");
  client.println("    setTimeout(()=>{msg.innerText='';},3000);");
  client.println("    loadPresetList();");
  client.println("  });");
  client.println("}");

#ifdef ENABLE_TUNER
  // ★ チューナー表示更新
  client.println("function updateTuner(){");
  client.println("  fetch('/monitor').then(r=>r.json()).then(d=>{");
  client.println("    var noteEl=document.getElementById('tuner-note');");
  client.println("    var needleEl=document.getElementById('tuner-needle');");
  client.println("    var freqEl=document.getElementById('tuner-freq');");
  client.println("    if(d.tuner && d.note!=='--'){");
  client.println("      noteEl.innerText=d.note;");
  client.println("      noteEl.className='tuner-note';");
  client.println("      var pct=Math.max(-50,Math.min(50,d.cents));");
  client.println("      needleEl.style.left='calc('+(50+pct/2)+'% - 3px)';");
  client.println("      needleEl.className='needle'+(Math.abs(d.cents)<=5?' intune':'');");
  client.println("      freqEl.innerText=d.freq+' Hz / '+(d.cents>=0?'+':'')+d.cents+' cent';");
  client.println("    } else {");
  client.println("      noteEl.innerText='--';");
  client.println("      noteEl.className='tuner-note off';");
  client.println("      needleEl.style.left='calc(50% - 3px)';");
  client.println("      needleEl.className='needle';");
  client.println("      freqEl.innerText=d.tuner?'弦を弾いてください':'チューナー停止中';");
  client.println("    }");
  client.println("  }).catch(()=>{});");
  client.println("}");
#endif

  client.println("let pollTimer=null,tunerTimer=null;");
  client.println("function switchTab(tab){");
  client.println("  document.getElementById('ctrl-panel').style.display=tab==='ctrl'?'':'none';");
  client.println("  document.getElementById('pre-panel').style.display=tab==='pre'?'':'none';");
#ifdef ENABLE_TUNER
  client.println("  document.getElementById('tuner-panel').style.display=tab==='tuner'?'':'none';");
#endif
  client.println("  document.getElementById('mon-panel').style.display=tab==='mon'?'':'none';");
#ifdef ENABLE_TUNER
  client.println("  ['ctrl','pre','tuner','mon'].forEach(function(t){");
#else
  client.println("  ['ctrl','pre','mon'].forEach(function(t){");
#endif
  client.println("    document.getElementById('tab-'+t).className='tab'+(tab===t?' active':'');");
  client.println("  });");
  client.println("  clearInterval(pollTimer); clearInterval(tunerTimer);");
  client.println("  if(tab==='mon'){pollTimer=setInterval(updateMonitor,1000);updateMonitor();}");
  client.println("  if(tab==='pre'){loadPresetList();}");
#ifdef ENABLE_TUNER
  client.println("  if(tab==='tuner'){fetch('/set?tuner=1'); tunerTimer=setInterval(updateTuner,150);updateTuner();}");
  client.println("  else { fetch('/set?tuner=0'); }"); // ★ タブを離れたらチューナー自動OFF
#endif
  client.println("}");

  client.println("function updateMonitor(){");
  client.println("  fetch('/monitor').then(r=>r.json()).then(d=>{");
  client.println("    setBadge('b-effect',d.effect,'EFFECT: '+(d.effect?'ON':'OFF'));");
  client.println("    setBadge('b-dist',!d.bitMode,'DIST');");
  client.println("    setBadge('b-bit',d.bitMode,'BIT');");
  client.println("    setBadge('b-trm',d.tremolo,'TREMOLO: '+(d.tremolo?'ON':'OFF'));");
#ifdef ENABLE_TUNER
  client.println("    setBadge('b-tuner',d.tuner,'TUNER: '+(d.tuner?'ON':'OFF'));");
#endif
  client.println("    setBadge('b-wctrl',d.webCtrl,'DIST: '+(d.webCtrl?'WEB':'POT'));");
  client.println("    setBadge('b-wtrm',d.webTrm,'TRM SPD: '+(d.webTrm?'WEB':'POT'));");
  client.println("    document.getElementById('m-dist').innerText=d.dist+'%';");
  client.println("    document.getElementById('m-trm').innerText=d.trm+'%';");
  client.println("  }).catch(()=>{});");
  client.println("}");

  client.println("function setBadge(id,on,text){var e=document.getElementById(id);e.className='badge '+(on?'badge-on':'badge-off');e.innerText=text;}");

  client.println("</script>");
  client.println("</body></html>");
}

// ── Webサーバー：リクエスト解析 ──────────────────
void handleWeb() {
  WiFiClient client = server.available();
  if (!client) return;
  if (!client.available()) { client.stop(); return; }

  String req = "";
  while (client.available()) {
    char c = client.read();
    if (c == '\n') break;
    req += c;
  }
  while (client.available()) client.read();

  Serial.print("REQ: "); Serial.println(req);

  if (req.indexOf("GET /monitor") >= 0) {
    sendJSON(client); client.stop(); return;
  }

  if (req.indexOf("GET /presets") >= 0) {
    sendPresetsJSON(client); client.stop(); return;
  }

  if (req.indexOf("GET /preset/") >= 0) {
    handlePreset(req);
    client.print("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nOK");
    client.stop(); return;
  }

  if (req.indexOf("GET /set") >= 0) {
#ifdef ENABLE_TUNER
    // ★ チューナーON/OFFはプリセット選択解除の対象外にする
    if (req.indexOf("tuner=") < 0) {
      activePresetSlot = -1;
    }
#else
    activePresetSlot = -1;
#endif

    if      (req.indexOf("effect=1")  >= 0) { effectOn  = true;  }
    else if (req.indexOf("effect=0")  >= 0) { effectOn  = false; }
    if      (req.indexOf("mode=dist") >= 0) { bitMode   = false; }
    else if (req.indexOf("mode=bit")  >= 0) { bitMode   = true;  }
    if      (req.indexOf("trm=1")     >= 0) { tremoloOn = true;  }
    else if (req.indexOf("trm=0")     >= 0) { tremoloOn = false; }
#ifdef ENABLE_TUNER
    // ★ チューナー切り替え
    if      (req.indexOf("tuner=1")   >= 0) { tunerMode = true;  }
    else if (req.indexOf("tuner=0")   >= 0) { tunerMode = false; }
#endif

    int di = req.indexOf("dist=");
    if (di >= 0) {
      int val = constrain(req.substring(di + 5).toInt(), 0, 100);
      webDistVal = val; webControl = true;
      distParam  = map(val, 0, 100, 0, 4095);
    }

    int ti = req.indexOf("trmspd=");
    if (ti >= 0) {
      int val = constrain(req.substring(ti + 7).toInt(), 0, 100);
      webTrmVal = val; webTrmControl = true;
      float hz  = 0.5f + (val / 100.0f) * 9.5f;
      lfoPhaseStep = 2.0f * (float)M_PI * hz / SAMPLE_RATE;
    }
  }

  sendHTML(client);
  client.stop();
}

// ── setup ─────────────────────────────────────────
void setup() {
  analogReadResolution(12);
  analogWriteResolution(12);
  analogWrite(DAC_OUT, DC_BIAS);

  pinMode(BTN_PIN,  INPUT_PULLUP);
  pinMode(BTN_PIN2, INPUT_PULLUP);
  pinMode(BTN_PIN3, INPUT_PULLUP);
  Serial.begin(9600);
  delay(1000);
  Serial.println("=== BOOT ===");

  initEEPROM();

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFiモジュールとの通信に失敗しました！");
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("ファームウェアをアップグレードしてください");
  }

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.print("接続を試みています: "); Serial.println(WIFI_SSID);
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(10000);
  }
  server.begin();
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.setTextColor(SSD1306_WHITE);

  audioTimer.begin(TIMER_MODE_PERIODIC, GPT_TIMER, 0, SAMPLE_RATE, 0.0f, audioISR);
  audioTimer.setup_overflow_irq();
  audioTimer.open();
  audioTimer.start();

  lastPotVal = analogRead(POT_PIN);
  distParam  = lastPotVal;
}

float bandPeak[FFT_BANDS] = {0};

// ── loop ──────────────────────────────────────────
void loop() {
  int currentPot = analogRead(POT_PIN);
  int currentTrm = analogRead(POT_TRM);
  potTrm = currentTrm;

  // DISTキャッチアップ
  if (webControl) {
    int potPct = map(currentPot, 0, 4095, 0, 100);
    if (abs(potPct - webDistVal) <= 5) { webControl = false; webDistVal = -1; }
  } else {
    distParam = currentPot;
  }
  lastPotVal = currentPot;
  potVal     = currentPot;

  // TREMOLOスピードキャッチアップ
  if (webTrmControl) {
    int trmPotPct = map(currentTrm, 0, 4095, 0, 100);
    if (abs(trmPotPct - webTrmVal) <= 5) { webTrmControl = false; webTrmVal = -1; }
  } else {
    float tremoloHz = 0.5f + (currentTrm / 4095.0f) * 9.5f;
    lfoPhaseStep    = 2.0f * (float)M_PI * tremoloHz / SAMPLE_RATE;
  }

  // ボタン1：エフェクトON/OFF
  bool currentBtn = digitalRead(BTN_PIN);
  if (lastBtnState == HIGH && currentBtn == LOW) {
    if (millis() - lastDebounceTime > 50) {
      effectOn = !effectOn; lastDebounceTime = millis();
      Serial.println(effectOn ? ">>> Effect: ON" : ">>> Effect: OFF");
    }
  }
  lastBtnState = currentBtn;

  // ボタン2：DIST / BIT 切り替え
  bool currentBtn2 = digitalRead(BTN_PIN2);
  if (lastBtnState2 == HIGH && currentBtn2 == LOW) {
    if (millis() - lastDebounceTime2 > 50) {
      bitMode = !bitMode; lastDebounceTime2 = millis();
      Serial.println(bitMode ? ">>> Mode: BIT" : ">>> Mode: DIST");
    }
  }
  lastBtnState2 = currentBtn2;

  // ★ ボタン3：短押し=トレモロON/OFF、長押し=チューナーON/OFF（ENABLE_TUNER無効時は長押しは何もしない）
  bool currentBtn3 = digitalRead(BTN_PIN3);
  if (currentBtn3 == LOW) {
    if (lastBtnState3 == HIGH) {
      // 押し始め
      btn3PressStart = millis();
      btn3LongPressFired = false;
    } else if (!btn3LongPressFired && millis() - btn3PressStart > LONG_PRESS_MS) {
      // 長押し確定
#ifdef ENABLE_TUNER
      tunerMode = !tunerMode;
      Serial.println(tunerMode ? ">>> TUNER MODE: ON" : ">>> TUNER MODE: OFF");
#endif
      btn3LongPressFired = true;
    }
  } else {
    if (lastBtnState3 == LOW && !btn3LongPressFired) {
      // 短押し（離した時点で長押しが発火していなければ）
      if (millis() - lastDebounceTime3 > 50) {
        tremoloOn = !tremoloOn; lastDebounceTime3 = millis();
        Serial.println(tremoloOn ? ">>> Tremolo: ON" : ">>> Tremolo: OFF");
      }
    }
  }
  lastBtnState3 = currentBtn3;

  handleWeb();

  // FFT / チューナー処理
  if (fftReady) {
    fftReady = false;

#ifdef ENABLE_TUNER
    if (tunerMode) {
      // ★ チューナーモード中はピッチ検出のみ実行（スペクトラム計算はスキップ）
      matchStringAndCents();
    } else {
#endif
      for (int i = 0; i < FFT_SAMPLES; i++) {
        vReal[i] = (float)(fftBuf[i] - DC_BIAS);
        vImag[i] = 0.0f;
      }
      FFT.windowing(vReal, FFT_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(vReal, vImag, FFT_SAMPLES, FFT_FORWARD);
      FFT.complexToMagnitude(vReal, vImag, FFT_SAMPLES);

      int binsPerBand = (FFT_SAMPLES / 2) / FFT_BANDS;
      for (int b = 0; b < FFT_BANDS; b++) {
        float maxVal = 0;
        for (int i = 0; i < binsPerBand; i++) {
          int idx = b * binsPerBand + i + 1;
          if (vReal[idx] > maxVal) maxVal = vReal[idx];
        }
        float db = log10(max(maxVal, 1.0f)) / log10(50000.0f);
        db = constrain(db, 0.0f, 1.0f);
        bandPeak[b] = max(db, bandPeak[b] * 0.75f);
      }
#ifdef ENABLE_TUNER
    }
#endif
  }

  // OLED表示（100msごと）
  unsigned long now = millis();
  if (now - prevTime >= INTERVAL) {
    prevTime = now;

    display.clearDisplay();

#ifdef ENABLE_TUNER
    if (tunerMode) {
      // ★ チューナー専用画面
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print("TUNER MODE");

      display.setTextSize(4);
      display.setCursor(40, 16);
      if (matchedStringIdx >= 0) {
        display.print(GUITAR_STRINGS[matchedStringIdx].name);
      } else {
        display.print("--");
      }

      // セントメーター
      int meterCenter = 64;
      int meterY = 50;
      display.drawFastVLine(meterCenter, meterY, 14, SSD1306_WHITE);
      if (matchedStringIdx >= 0) {
        int meterX = meterCenter + constrain(centsOffset, -50, 50);
        if (abs(centsOffset) <= 5) {
          display.fillRect(meterCenter - 3, meterY - 2, 6, 18, SSD1306_WHITE);
        } else {
          display.fillRect(meterX - 2, meterY, 4, 14, SSD1306_WHITE);
        }
      }

      display.setTextSize(1);
      display.setCursor(0, 56);
      if (matchedStringIdx >= 0) {
        display.print((int)detectedFreq);
        display.print("Hz ");
        display.print(centsOffset >= 0 ? "+" : "");
        display.print(centsOffset);
      } else {
        display.print("play a string");
      }

    } else {
#endif
      // ── 通常画面（既存表示） ──
      int distPct  = map(distParam, 0, 4095, 0, 100);
      int trmPct   = webTrmControl ? webTrmVal : map(potTrm, 0, 4095, 0, 100);
      int level    = abs(outVal - DC_BIAS);
      int barWidth = map(level, 0, 2048, 0, 118);

      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print(bitMode ? "BIT " : "DIST");
      display.print(effectOn ? ":ON " : ":OFF");
      display.print(" TRM:");
      display.print(tremoloOn ? "ON" : "OFF");

      display.setCursor(0, 10);
      if (effectOn && !bitMode) {
        display.print("DIST:");
        display.print(distPct);
        display.print("%");
        display.print(webControl ? " W" : " P");
      }
      if (tremoloOn) {
        display.print(" SPD:");
        display.print(trmPct);
        display.print("%");
        display.print(webTrmControl ? " W" : " P");
      }

      display.drawRect(0, 18, 128, 8, SSD1306_WHITE);
      if (barWidth > 0) {
        display.fillRect(2, 20, barWidth, 4, SSD1306_WHITE);
      }

      display.setTextSize(1);
      display.setCursor(0, 27);
      display.print("SPECTRUM");

      int barAreaTop    = 34;
      int barAreaHeight = 28;
      int bandWidth     = 128 / FFT_BANDS;

      for (int b = 0; b < FFT_BANDS; b++) {
        int bh = (int)(bandPeak[b] * barAreaHeight);
        int bx = b * bandWidth;
        int by = barAreaTop + barAreaHeight - bh;
        if (bh > 0) {
          display.fillRect(bx + 1, by, bandWidth - 2, bh, SSD1306_WHITE);
        }
      }
#ifdef ENABLE_TUNER
    }
#endif

    display.display();
  }
}
