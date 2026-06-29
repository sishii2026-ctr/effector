#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "FspTimer.h"
#include "arduinoFFT.h"
#include <math.h>
#include <WiFiS3.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_ADDR     0x3C

#define FFT_SAMPLES   256
#define FFT_BANDS     16
#define SAMPLE_RATE   20000.0f

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

bool lastBtnState  = HIGH;
bool lastBtnState2 = HIGH;
bool lastBtnState3 = HIGH;
unsigned long lastDebounceTime  = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;

volatile int potVal  = 0;
volatile int potTrm  = 0;
volatile int outVal  = DC_BIAS;

// ── キャッチアップ方式の制御変数 ──────────────────
volatile int distParam  = 0;
int          webDistVal = -1;
bool         webControl = false;
int          lastPotVal = 0;

// トレモロスピードのWeb制御
bool  webTrmControl = false;
int   webTrmVal     = -1;   // 0〜100%

volatile float lfoPhase     = 0.0f;
volatile float lfoPhaseStep = 0.0f;

float vReal[FFT_SAMPLES];
float vImag[FFT_SAMPLES];

volatile int fftBuf[FFT_SAMPLES];
volatile int fftIndex = 0;
volatile bool fftReady = false;

FspTimer audioTimer;

// ── 音声割り込み ──────────────────────────────────
void audioISR(timer_callback_args_t *args) {
  int raw = analogRead(ADC_IN);
  int POT = distParam;
  int out = DC_BIAS;

  if (effectOn) {
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

  if (tremoloOn) {
    int centered  = out - DC_BIAS;
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

// ── モニター用JSONを返す ──────────────────────────
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
  json += "\"trm\":"     + String(trmPct);
  json += "}";
  client.print(json);
}

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
  client.println(".row{display:flex;justify-content:center;gap:10px;margin:8px 0;}");
  client.println("button{padding:10px 20px;border:none;border-radius:8px;font-size:15px;cursor:pointer;}");
  client.println(".on{background:#0a0;color:#fff;} .off{background:#a00;color:#fff;}");
  client.println(".mode{background:#036;color:#fff;}");
  client.println(".tab{background:#333;color:#aaa;font-size:14px;padding:10px 24px;border-radius:8px;}");
  client.println(".tab.active{background:#0f0;color:#111;font-weight:bold;}");
  client.println("input[type=range]{width:90%;accent-color:#0f0;}");
  client.println(".label{font-size:12px;color:#888;margin-top:4px;}");
  client.println(".status{font-size:13px;color:#ff0;margin-top:6px;}");
  client.println(".monitor{font-size:28px;font-weight:bold;color:#0f0;margin:8px 0;}");
  client.println(".monitor-label{font-size:11px;color:#666;margin-bottom:4px;}");
  client.println(".badge{display:inline-block;padding:4px 12px;border-radius:20px;font-size:13px;font-weight:bold;margin:4px;}");
  client.println(".badge-on{background:#0a0;color:#fff;} .badge-off{background:#333;color:#888;}");
  client.println("</style></head><body>");

  client.println("<h1>🎸 Effect Controller</h1>");

  // ── モード切り替えタブ ──
  client.println("<div class='row' style='margin-bottom:16px;'>");
  client.println("<button class='tab active' id='tab-ctrl' onclick='switchTab(\"ctrl\")'>⚙️ 操作モード</button>");
  client.println("<button class='tab'        id='tab-mon'  onclick='switchTab(\"mon\")' >👁️ 表示モード</button>");
  client.println("</div>");

  // ════════════════════════════════
  // 操作モード
  // ════════════════════════════════
  client.println("<div id='ctrl-panel'>");

  client.println("<div class='card'>");
  client.println("<div class='label'>EFFECT</div>");
  client.println("<div class='row'>");
  client.println("<button class='on'  onclick=\"go('/set?effect=1')\">ON</button>");
  client.println("<button class='off' onclick=\"go('/set?effect=0')\">OFF</button>");
  client.println("</div>");
  client.print("<div class='status'>現在: ");
  client.print(effectOn ? "ON" : "OFF");
  client.println("</div></div>");

  client.println("<div class='card'>");
  client.println("<div class='label'>MODE</div>");
  client.println("<div class='row'>");
  client.println("<button class='mode' onclick=\"go('/set?mode=dist')\">DIST</button>");
  client.println("<button class='mode' onclick=\"go('/set?mode=bit')\">BIT</button>");
  client.println("</div>");
  client.print("<div class='status'>現在: ");
  client.print(bitMode ? "BIT" : "DIST");
  client.println("</div></div>");

  client.println("<div class='card'>");
  client.println("<div class='label'>TREMOLO</div>");
  client.println("<div class='row'>");
  client.println("<button class='on'  onclick=\"go('/set?trm=1')\">ON</button>");
  client.println("<button class='off' onclick=\"go('/set?trm=0')\">OFF</button>");
  client.println("</div>");
  client.print("<div class='status'>現在: ");
  client.print(tremoloOn ? "ON" : "OFF");
  client.println("</div>");
  client.println("<div class='label'>TREMOLO SPEED</div>");
  client.print("<input type='range' min='0' max='100' value='");
  client.print(trmPct);
  client.println("' oninput='sendTrm(this.value)'>");
  client.print("<div class='status' id='tv'>");
  client.print(trmPct);
  client.println("%</div>");
  client.print("<div class='label'>");
  client.print(webTrmControl ? "⚡ WEB制御中" : "🎛️ POT制御中");
  client.println("</div></div>");

  client.println("<div class='card'>");
  client.println("<div class='label'>DIST AMOUNT</div>");
  client.print("<input type='range' min='0' max='100' value='");
  client.print(distPct);
  client.println("' oninput='sendDist(this.value)'>");
  client.print("<div class='status' id='dv'>");
  client.print(distPct);
  client.println("%</div>");
  client.print("<div class='label'>");
  client.print(webControl ? "⚡ WEB制御中" : "🎛️ POT制御中");
  client.println("</div></div>");

  client.println("</div>"); // ctrl-panel終了

  // ════════════════════════════════
  // 表示モード（リアルタイム更新）
  // ════════════════════════════════
  client.println("<div id='mon-panel' style='display:none;'>");

  // ボタン状態カード
  client.println("<div class='card'>");
  client.println("<div class='label'>ボタン状態</div>");
  client.println("<div style='margin:10px 0;'>");
  client.print("<span class='badge "); client.print(effectOn ? "badge-on" : "badge-off");
  client.println("' id='b-effect'>EFFECT: "); client.print(effectOn ? "ON" : "OFF"); client.println("</span>");
  // DISTとBITを別々のバッジに（片方が光ればもう片方が消える）
  client.print("<span class='badge "); client.print(!bitMode ? "badge-on" : "badge-off");
  client.println("' id='b-dist'>DIST</span>");
  client.print("<span class='badge "); client.print(bitMode ? "badge-on" : "badge-off");
  client.println("' id='b-bit'>BIT</span>");
  client.print("<span class='badge "); client.print(tremoloOn ? "badge-on" : "badge-off");
  client.println("' id='b-trm'>TREMOLO: "); client.print(tremoloOn ? "ON" : "OFF"); client.println("</span>");
  client.println("</div></div>");

  // POT値カード
  client.println("<div class='card'>");
  client.println("<div class='label'>ポテンショメーター</div>");
  client.println("<div class='monitor-label'>DIST (POT)</div>");
  client.print("<div class='monitor' id='m-dist'>"); client.print(distPct); client.println("%</div>");
  client.println("<div class='monitor-label'>TREMOLO SPEED (POT)</div>");
  client.print("<div class='monitor' id='m-trm'>"); client.print(map(potTrm, 0, 4095, 0, 100)); client.println("%</div>");
  client.println("</div>");

  // 制御元カード
  client.println("<div class='card'>");
  client.println("<div class='label'>制御元</div><div style='margin:8px 0;'>");
  client.print("<span class='badge "); client.print(webControl ? "badge-on" : "badge-off");
  client.println("' id='b-wctrl'>DIST: "); client.print(webControl ? "WEB" : "POT"); client.println("</span>");
  client.print("<span class='badge "); client.print(webTrmControl ? "badge-on" : "badge-off");
  client.println("' id='b-wtrm'>TRM SPD: "); client.print(webTrmControl ? "WEB" : "POT"); client.println("</span>");
  client.println("</div></div>");

  client.println("</div>"); // mon-panel終了

  // ── JavaScript ──
  client.println("<script>");
  client.println("function go(url){fetch(url).then(()=>location.reload());}");
  client.println("let t=null,t2=null;");
  client.println("function sendDist(v){");
  client.println("  document.getElementById('dv').innerText=v+'%';");
  client.println("  clearTimeout(t);");
  client.println("  t=setTimeout(()=>fetch('/set?dist='+v),500);");
  client.println("}");
  client.println("function sendTrm(v){");
  client.println("  document.getElementById('tv').innerText=v+'%';");
  client.println("  clearTimeout(t2);");
  client.println("  t2=setTimeout(()=>fetch('/set?trmspd='+v),500);");
  client.println("}");

  // タブ切り替え：表示モードに入ったらポーリング開始、出たら止める
  client.println("let pollTimer=null;");
  client.println("function switchTab(tab){");
  client.println("  document.getElementById('ctrl-panel').style.display=tab==='ctrl'?'':'none';");
  client.println("  document.getElementById('mon-panel').style.display=tab==='mon'?'':'none';");
  client.println("  document.getElementById('tab-ctrl').className='tab'+(tab==='ctrl'?' active':'');");
  client.println("  document.getElementById('tab-mon').className='tab'+(tab==='mon'?' active':'');");
  client.println("  if(tab==='mon'){");
  client.println("    pollTimer=setInterval(updateMonitor,1000);"); // 500msごとに更新
  client.println("    updateMonitor();");
  client.println("  } else {");
  client.println("    clearInterval(pollTimer);");
  client.println("  }");
  client.println("}");

  // /monitorにfetchしてDOMを更新
  client.println("function updateMonitor(){");
  client.println("  fetch('/monitor')");
  client.println("  .then(r=>r.json())");
  client.println("  .then(d=>{");
  client.println("    setBadge('b-effect', d.effect, 'EFFECT: '+(d.effect?'ON':'OFF'));");
  client.println("    setBadge('b-dist',  !d.bitMode, 'DIST');");  // DISTはbitModeがfalseのとき光る
  client.println("    setBadge('b-bit',    d.bitMode, 'BIT');");   // BITはbitModeがtrueのとき光る
  client.println("    setBadge('b-trm',   d.tremolo,  'TREMOLO: '+(d.tremolo?'ON':'OFF'));");
  client.println("    setBadge('b-wctrl', d.webCtrl,  'DIST: '+(d.webCtrl?'WEB':'POT'));");
  client.println("    setBadge('b-wtrm',  d.webTrm,   'TRM SPD: '+(d.webTrm?'WEB':'POT'));");
  client.println("    document.getElementById('m-dist').innerText=d.dist+'%';");
  client.println("    document.getElementById('m-trm').innerText=d.trm+'%';");
  client.println("  })");
  client.println("  .catch(()=>{});");  // 通信エラーは無視
  client.println("}");

  client.println("function setBadge(id,on,text){");
  client.println("  var e=document.getElementById(id);");
  client.println("  e.className='badge '+(on?'badge-on':'badge-off');");
  client.println("  e.innerText=text;");
  client.println("}");
  client.println("</script>");
  client.println("</body></html>");
}

// ── Webサーバー：リクエスト解析 ──────────────────
void handleWeb() {
  WiFiClient client = server.available();
  if (!client) return;

  if (!client.available()) {
    client.stop();
    return;
  }

  String req = "";
  while (client.available()) {
    char c = client.read();
    if (c == '\n') break;
    req += c;
  }
  while (client.available()) client.read();

  Serial.print("REQ: ");
  Serial.println(req);

  // /monitor エンドポイント：JSONを返すだけ
  if (req.indexOf("GET /monitor") >= 0) {
    sendJSON(client);
    client.stop();
    return;
  }

  if (req.indexOf("GET /set") >= 0) {
    if      (req.indexOf("effect=1")  >= 0) { effectOn  = true;  Serial.println("effect ON"); }
    else if (req.indexOf("effect=0")  >= 0) { effectOn  = false; Serial.println("effect OFF"); }
    if      (req.indexOf("mode=dist") >= 0) { bitMode   = false; Serial.println("mode DIST"); }
    else if (req.indexOf("mode=bit")  >= 0) { bitMode   = true;  Serial.println("mode BIT"); }
    if      (req.indexOf("trm=1")     >= 0) { tremoloOn = true;  Serial.println("tremolo ON"); }
    else if (req.indexOf("trm=0")     >= 0) { tremoloOn = false; Serial.println("tremolo OFF"); }

    int di = req.indexOf("dist=");
    if (di >= 0) {
      int val = constrain(req.substring(di + 5).toInt(), 0, 100);
      webDistVal = val;
      webControl = true;
      distParam  = map(val, 0, 100, 0, 4095);
      Serial.print("dist="); Serial.println(val);
    }

    int ti = req.indexOf("trmspd=");
    if (ti >= 0) {
      int val = constrain(req.substring(ti + 7).toInt(), 0, 100);
      webTrmVal     = val;
      webTrmControl = true;
      float hz      = 0.5f + (val / 100.0f) * 9.5f;
      lfoPhaseStep  = 2.0f * (float)M_PI * hz / SAMPLE_RATE;
      Serial.print("trmspd="); Serial.println(val);
    }
  }

  sendHTML(client);
  client.stop();
}

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

  // ── WiFiモジュール確認 ──
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFiモジュールとの通信に失敗しました！");
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("ファームウェアをアップグレードしてください");
  }

  // ── Wi-Fi接続 ──
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.print("接続を試みています: ");
    Serial.println(WIFI_SSID);
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(10000);
  }
  server.begin();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // ── OLED・タイマー初期化 ──
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

void loop() {
  int currentPot = analogRead(POT_PIN);
  int currentTrm = analogRead(POT_TRM);
  potTrm = currentTrm;

  // ── DISTキャッチアップ判定 ──
  if (webControl) {
    int potPct = map(currentPot, 0, 4095, 0, 100);
    if (abs(potPct - webDistVal) <= 5) {
      webControl = false;
      webDistVal = -1;
      Serial.println(">>> DIST Control: POT");
    }
  } else {
    distParam = currentPot;
  }
  lastPotVal = currentPot;
  potVal     = currentPot;

  // ── TREMOLOスピードキャッチアップ判定 ──
  if (webTrmControl) {
    int trmPotPct = map(currentTrm, 0, 4095, 0, 100);
    if (abs(trmPotPct - webTrmVal) <= 5) {
      webTrmControl = false;
      webTrmVal     = -1;
      Serial.println(">>> TRM Control: POT");
    }
    // Web制御中はlfoPhaseStepをそのまま維持（handleWebで更新済み）
  } else {
    // POT制御中
    float tremoloHz = 0.5f + (currentTrm / 4095.0f) * 9.5f;
    lfoPhaseStep    = 2.0f * (float)M_PI * tremoloHz / SAMPLE_RATE;
  }

  // ── ボタン1：エフェクトON/OFF ──
  bool currentBtn = digitalRead(BTN_PIN);
  if (lastBtnState == HIGH && currentBtn == LOW) {
    if (millis() - lastDebounceTime > 50) {
      effectOn = !effectOn;
      lastDebounceTime = millis();
      Serial.println(effectOn ? ">>> Effect: ON" : ">>> Effect: OFF");
    }
  }
  lastBtnState = currentBtn;

  // ── ボタン2：DIST / BIT 切り替え ──
  bool currentBtn2 = digitalRead(BTN_PIN2);
  if (lastBtnState2 == HIGH && currentBtn2 == LOW) {
    if (millis() - lastDebounceTime2 > 50) {
      bitMode = !bitMode;
      lastDebounceTime2 = millis();
      Serial.println(bitMode ? ">>> Mode: BIT" : ">>> Mode: DIST");
    }
  }
  lastBtnState2 = currentBtn2;

  // ── ボタン3：トレモロON/OFF ──
  bool currentBtn3 = digitalRead(BTN_PIN3);
  if (lastBtnState3 == HIGH && currentBtn3 == LOW) {
    if (millis() - lastDebounceTime3 > 50) {
      tremoloOn = !tremoloOn;
      lastDebounceTime3 = millis();
      Serial.println(tremoloOn ? ">>> Tremolo: ON" : ">>> Tremolo: OFF");
    }
  }
  lastBtnState3 = currentBtn3;

  // ── Webサーバー処理（ブロックしない） ──
  handleWeb();

  // ── FFT処理 ──
  if (fftReady) {
    fftReady = false;
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
  }

  // ── OLED表示（100msごと） ──
  unsigned long now = millis();
  if (now - prevTime >= INTERVAL) {
    prevTime = now;

    int distPct  = map(distParam, 0, 4095, 0, 100);
    int trmPct   = webTrmControl ? webTrmVal : map(potTrm, 0, 4095, 0, 100);
    int level    = abs(outVal - DC_BIAS);
    int barWidth = map(level, 0, 2048, 0, 118);

    display.clearDisplay();

    // 上段：モード・エフェクト量・トレモロ
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(bitMode ? "BIT " : "DIST");
    display.print(effectOn ? ":ON " : ":OFF");
    display.print(" TRM:");
    display.print(tremoloOn ? "ON" : "OFF");

    // DIST量・SPD表示 ＋ 制御元インジケーター
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

    display.display();
  }
}
