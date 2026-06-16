const int DAC_OUT = A0;
const int ADC_IN = A1;
const int POT_PIN = A2;
const int BTN_PIN = 13;
const int DC_BIAS = 2048;

bool effectOn = false;
bool lastBtnState = HIGH; 

unsigned long lastSerialTime = 0;
unsigned long lastDebounceTime = 0;

void setup() {
  analogReadResolution(12);
  analogWriteResolution(12);
  analogWrite(DAC_OUT, DC_BIAS);
  
  pinMode(BTN_PIN, INPUT_PULLUP);

  Serial.begin(9600);
 
}

void loop() {

  bool currentBtn = digitalRead(BTN_PIN);

  int POT = analogRead(POT_PIN);
  int raw = analogRead(ADC_IN);
  int out = DC_BIAS;

  if (lastBtnState == HIGH && currentBtn == LOW) {
    if (millis() - lastDebounceTime > 50) {
      effectOn = !effectOn;
      lastDebounceTime = millis(); 
      Serial.println(effectOn ? ">>> Effect: ON" : ">>> Effect: OFF");
    }
  }
  lastBtnState = currentBtn;

  if(effectOn) {
    float distortion = map(POT, 0, 4095, 1, 100);
    int centered = raw - DC_BIAS;
    float amp = centered * distortion;
    float clipped = constrain(amp, -2000, 2000);
    out = constrain((int)clipped + DC_BIAS, 0, 4095);
  } 
  
  else {
    int centered = raw - DC_BIAS;
    int amplified = centered * 10;  // ← 増幅率、好みで調整
    out = constrain(amplified + DC_BIAS, 0, 4095);
  }

  analogWrite(DAC_OUT, out);

  //if (millis() - lastSerialTime > 500) { 
  //  lastSerialTime = millis();
    
  //  String msg = "Input(A1): " + String(raw) + 
  //               " | Pot(A2): " + String(POT) + 
  //               " | Output: " + String(out) + 
  //               " | Effect: " + String(effectOn ? "ON" : "OFF");
  //  Serial.println(msg);
  //}
}