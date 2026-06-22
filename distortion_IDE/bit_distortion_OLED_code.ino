#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "FspTimer.h"
#include "arduinoFFT.h"

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_ADDR     0x3C

#define FFT_SAMPLES   256
#define FFT_BANDS     16
#define SAMPLE_RATE   20000.0f

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
ArduinoFFT<float> FFT;

const int DAC_OUT  = A0;
const int ADC_IN   = A3;
const int POT_PIN  = A2;
const int BTN_PIN  = 13;  // エフェクトON/OFF
const int BTN_PIN2 = 12;  // エフェクト切り替え
const int DC_BIAS  = 2048;

const unsigned long INTERVAL = 100;
unsigned long prevTime = 0;

volatile bool effectOn   = false;
volatile bool bitMode    = false;  // false=DIST, true=BIT

bool lastBtnState  = HIGH;
bool lastBtnState2 = HIGH;
unsigned long lastDebounceTime  = 0;
unsigned long lastDebounceTime2 = 0;

volatile int potVal = 0;
volatile int outVal = DC_BIAS;

float vReal[FFT_SAMPLES];
float vImag[FFT_SAMPLES];

volatile int fftBuf[FFT_SAMPLES];
volatile int fftIndex = 0;
volatile bool fftReady = false;

FspTimer audioTimer;

void audioISR(timer_callback_args_t *args) {
  int raw = analogRead(ADC_IN);

  if (bitMode) {
    static float smoothed = DC_BIAS;
    smoothed = smoothed * 0.6f + raw * 0.4f;
    raw = (int)smoothed;

    static int holdVal = DC_BIAS;
    static int holdCount = 0;
    if (holdCount++ >= 8) {
      holdVal = raw;
      holdCount = 0;
    }
    raw = holdVal;
  }

  int POT = potVal;
  int out = DC_BIAS;

  if (effectOn) {
    float distortion = map(POT, 0, 4095, 1, 100);
    int   centered   = raw - DC_BIAS;
    float amp        = centered * distortion;
    float clipped    = constrain(amp, -2000, 2000);
    out = constrain((int)clipped + DC_BIAS, 0, 4095);
  } else {
    int centered  = raw - DC_BIAS;
    int amplified = centered * 10;
    out = constrain(amplified + DC_BIAS, 0, 4095);
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

  // ボタン1：エフェクトON/OFF
  bool currentBtn = digitalRead(BTN_PIN);
  if (lastBtnState == HIGH && currentBtn == LOW) {
    if (millis() - lastDebounceTime > 50) {
      effectOn = !effectOn;
      lastDebounceTime = millis();
      Serial.println(effectOn ? ">>> Effect: ON" : ">>> Effect: OFF");
    }
  }
  lastBtnState = currentBtn;

  // ボタン2：DIST/BIT切り替え
  bool currentBtn2 = digitalRead(BTN_PIN2);
  if (lastBtnState2 == HIGH && currentBtn2 == LOW) {
    if (millis() - lastDebounceTime2 > 50) {
      bitMode = !bitMode;
      lastDebounceTime2 = millis();
      Serial.println(bitMode ? ">>> Mode: BIT" : ">>> Mode: DIST");
    }
  }
  lastBtnState2 = currentBtn2;

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

  unsigned long now = millis();
  if (now - prevTime >= INTERVAL) {
    prevTime = now;

    int distPct  = map(potVal, 0, 4095, 0, 100);
    int level    = abs(outVal - DC_BIAS);
    int barWidth = map(level, 0, 2048, 0, 118);

    display.clearDisplay();

    // ── 上段：モード名とエフェクト量 ──
    display.setTextSize(1);
    display.setCursor(0, 2);
    display.print(bitMode ? "BIT: " : "DIST: ");
    display.setTextSize(2);
    display.setCursor(42, 0);

    if (effectOn) {
      if (bitMode) {
        display.print("ON");
      } else {
        display.print(distPct);
        display.print("%");
      }
    } else {
      display.print("OFF");
    }

    // ── 中段：レベルメーター ──
    display.setTextSize(1);
    display.setCursor(98, 8);
    display.print("LEVEL");
    display.drawRect(0, 16, 128, 8, SSD1306_WHITE);
    if (barWidth > 0) {
      display.fillRect(2, 18, barWidth, 4, SSD1306_WHITE);
    }

    // ── 下段：スペクトラム ──
    display.setTextSize(1);
    display.setCursor(0, 26);
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