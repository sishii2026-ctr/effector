#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "FspTimer.h"
#include "arduinoFFT.h"
#include <math.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_ADDR     0x3C

#define FFT_SAMPLES   256
#define FFT_BANDS     16
#define SAMPLE_RATE   20000.0f

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

volatile bool effectOn  = false;  // DIST/BIT ON/OFF
volatile bool bitMode   = false;  // false=DIST, true=BIT
volatile bool tremoloOn = false;  // トレモロ独立ON/OFF

bool lastBtnState  = HIGH;
bool lastBtnState2 = HIGH;
bool lastBtnState3 = HIGH;
unsigned long lastDebounceTime  = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;

volatile int potVal  = 0;
volatile int potTrm  = 0;
volatile int outVal  = DC_BIAS;

// トレモロ用 LFO
volatile float lfoPhase     = 0.0f;
volatile float lfoPhaseStep = 0.0f;

float vReal[FFT_SAMPLES];
float vImag[FFT_SAMPLES];

volatile int fftBuf[FFT_SAMPLES];
volatile int fftIndex = 0;
volatile bool fftReady = false;

FspTimer audioTimer;

void audioISR(timer_callback_args_t *args) {
  int raw = analogRead(ADC_IN);
  int POT = potVal;
  int out = DC_BIAS;

  // ── DIST / BIT 処理 ──
  if (effectOn) {
    if (!bitMode) {
      // DISTモード
      float distortion = map(POT, 0, 4095, 1, 100);
      int   centered   = raw - DC_BIAS;
      float amp        = centered * distortion;
      float clipped    = constrain(amp, -2000, 2000);
      out = constrain((int)clipped + DC_BIAS, 0, 4095);
    } else {
      // BITモード
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
    // エフェクトOFF：クリーン（×10増幅）
    int centered  = raw - DC_BIAS;
    int amplified = centered * 10;
    out = constrain(amplified + DC_BIAS, 0, 4095);
  }

  // ── トレモロ処理（独立して上乗せ）──
  if (tremoloOn) {
    int centered  = out - DC_BIAS;
    float depth = 0.7f;  // 0.0〜1.0 で深さ調整（小さいほど揺れが浅い）
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

void setup() {
  analogReadResolution(12);
  analogWriteResolution(12);
  analogWrite(DAC_OUT, DC_BIAS);

  pinMode(BTN_PIN,  INPUT_PULLUP);
  pinMode(BTN_PIN2, INPUT_PULLUP);
  pinMode(BTN_PIN3, INPUT_PULLUP);
  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.setTextColor(SSD1306_WHITE);

  audioTimer.begin(TIMER_MODE_PERIODIC, GPT_TIMER, 0, SAMPLE_RATE, 0.0f, audioISR);
  audioTimer.setup_overflow_irq();
  audioTimer.open();
  audioTimer.start();
}

float bandPeak[FFT_BANDS] = {0};

void loop() {
  potVal = analogRead(POT_PIN);
  potTrm = analogRead(POT_TRM);

  // TREMOLOスピード計算：0.5Hz〜10Hz
  float tremoloHz = 0.5f + (potTrm / 4095.0f) * 9.5f;
  lfoPhaseStep    = 2.0f * (float)M_PI * tremoloHz / SAMPLE_RATE;

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

  // ── OLED表示（100msごと）──
  unsigned long now = millis();
  if (now - prevTime >= INTERVAL) {
    prevTime = now;

    int distPct  = map(potVal, 0, 4095, 0, 100);
    int trmPct   = map(potTrm, 0, 4095, 0, 100);
    int level    = abs(outVal - DC_BIAS);
    int barWidth = map(level, 0, 2048, 0, 118);

    display.clearDisplay();

    // ── 上段：モード名・エフェクト量・トレモロ状態 ──
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(bitMode ? "BIT " : "DIST");
    display.print(effectOn ? ":ON " : ":OFF");
    display.print(" TRM:");
    display.print(tremoloOn ? "ON" : "OFF");

    // DIST量 or TREMOLOスピード
    display.setCursor(0, 10);
    if (effectOn && !bitMode) {
      display.print("DIST:");
      display.print(distPct);
      display.print("%  ");
    }
    if (tremoloOn) {
      display.print("SPD:");
      display.print(trmPct);
      display.print("%");
    }

    display.drawRect(0, 18, 128, 8, SSD1306_WHITE);
    if (barWidth > 0) {
      display.fillRect(2, 20, barWidth, 4, SSD1306_WHITE);
    }

    // ── 下段：スペクトラム ──
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