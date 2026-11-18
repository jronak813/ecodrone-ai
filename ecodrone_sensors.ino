/*
  EcoDrone AI – RGB colour sensor + light sensor for the web app
  -------------------------------------------------------------
  Sends one line every half second over USB, for example:
      R:120,G:80,B:60,L:55
  R, G, B = colour of the ground (0–255)   L = light level (0–100 %)

  Parts (common school-kit parts):
    • TCS3200 / TCS230 colour sensor module
    • LDR (light dependent resistor) + 10k resistor  (voltage divider to A0)

  Wiring (Arduino Uno / Nano):
    TCS3200  S0 → D4   S1 → D5   S2 → D6   S3 → D7   OUT → D8   VCC → 5V   GND → GND
    LDR      one leg → 5V, other leg → A0 and → 10k resistor → GND

  How to use with the app:
    1. Upload this sketch.
    2. Open the EcoDrone page in Chrome or Edge on the computer.
    3. Step 2 → “Connect Arduino (USB)” → choose the Arduino port.
    4. Hold the colour sensor 1–2 cm above the ground at different spots.

  CALIBRATION (do this once, it makes colours correct):
    Hold the sensor over WHITE paper → note the raw R,G,B numbers in Serial Monitor → put them in WHITE_*.
    Hold it over BLACK paper        → note the raw numbers                         → put them in BLACK_*.
    (Raw = pulse length; smaller number = more of that colour.)
    Set CALIBRATE to true to print raw numbers, then back to false.
*/

const int S0 = 4, S1 = 5, S2 = 6, S3 = 7, OUT = 8, LDR = A0;
const bool CALIBRATE = false;

// Replace these with your own readings from white and black paper
int WHITE_R = 25,  WHITE_G = 27,  WHITE_B = 22;
int BLACK_R = 180, BLACK_G = 190, BLACK_B = 150;

int readPulse(bool s2, bool s3) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  delay(20);
  return pulseIn(OUT, LOW, 100000);      // time of one pulse (µs)
}

int toByte(int raw, int white, int black) {
  // white paper → 255, black paper → 0
  return constrain(map(raw, black, white, 0, 255), 0, 255);
}

void setup() {
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT); pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  digitalWrite(S0, HIGH); digitalWrite(S1, LOW);   // 20% frequency scaling
  Serial.begin(9600);
}

void loop() {
  int rawR = readPulse(LOW, LOW);    // red filter
  int rawG = readPulse(HIGH, HIGH);  // green filter
  int rawB = readPulse(LOW, HIGH);   // blue filter
  int light = map(analogRead(LDR), 0, 1023, 0, 100);

  if (CALIBRATE) {
    Serial.print("RAW R="); Serial.print(rawR);
    Serial.print(" G=");    Serial.print(rawG);
    Serial.print(" B=");    Serial.println(rawB);
  } else {
    Serial.print("R:"); Serial.print(toByte(rawR, WHITE_R, BLACK_R));
    Serial.print(",G:"); Serial.print(toByte(rawG, WHITE_G, BLACK_G));
    Serial.print(",B:"); Serial.print(toByte(rawB, WHITE_B, BLACK_B));
    Serial.print(",L:"); Serial.println(light);
  }
  delay(500);
}
