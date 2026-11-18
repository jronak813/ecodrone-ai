/*
  EcoDrone AI – ESP32 Ground Station
  ==================================
  1. DRONE SCAN DISPLAY: when the EcoDrone app scans a drone photo, it sends the result
     here by Bluetooth or USB. The OLED shows the zone, macro %, micro % and location,
     and the red / orange / green LED lights up for that zone.
  2. SENSORS: RGB colour sensor + light sensor + LiDAR distance + GPS, sent to the app.

  Every half second it sends one line, for example:
      R:120,G:80,B:60,L:55,D:3.2,LAT:24.585400,LON:73.712500,SAT:7,SAVE:0
  Press the BOOT button on the ESP32 to SAVE a reading for the spot you are measuring
  (that line ends with SAVE:1). The app only counts saved readings.

  PARTS
    ESP32 DevKit (30 or 38 pin)
    TCS34725 RGB colour sensor (I2C)
    VL53L0X LiDAR / time-of-flight distance sensor (I2C)
    NEO-6M GPS module with its antenna
    0.96" SSD1306 OLED display, 128x64, I2C
    LDR + 10k resistor (light sensor)
    Power bank or 18650 battery holder with 5V output

  WIRING
    TCS34725   VIN->3V3  GND->GND  SDA->GPIO21  SCL->GPIO22
    OLED       VCC->3V3  GND->GND  SDA->GPIO21  SCL->GPIO22   (shares the bus with TCS34725)
    VL53L0X    VIN->3V3  GND->GND  SDA->GPIO25  SCL->GPIO26   (its own bus: it has the same
                                                               I2C address 0x29 as the TCS34725)
    NEO-6M     VCC->VIN (5V)  GND->GND  TX->GPIO16  RX->GPIO17
    LDR        3V3 -> LDR -> GPIO34 -> 10k resistor -> GND
    Button     the ESP32's own BOOT button (GPIO0) – nothing to wire
    Zone LEDs  GPIO13 -> 220 ohm -> RED LED    -> GND
               GPIO27 -> 220 ohm -> ORANGE/YELLOW LED -> GND
               GPIO14 -> 220 ohm -> GREEN LED  -> GND

  LIBRARIES (Arduino IDE > Library Manager)
    Adafruit TCS34725, Adafruit_VL53L0X, Adafruit SSD1306, Adafruit GFX Library, TinyGPSPlus
    Board: "ESP32 Dev Module" (Boards Manager: esp32 by Espressif). Bluetooth is built in.

  CALIBRATION (do once, makes colours correct)
    1. Set CALIBRATE to true and upload. Open Serial Monitor at 115200.
    2. Hold the sensor 3 cm above WHITE paper. Write the RAW R, G, B numbers into WHITE_R/G/B.
    3. Set CALIBRATE back to false and upload again.

  USING IT
    - Hold the RGB sensor 2–8 cm above the ground. The OLED says "Dist OK" when it is right.
    - Wait for "GPS" to show numbers (first fix outdoors can take 1–2 minutes).
    - Press BOOT to save each spot. Take 5 or more spots per site.
*/

#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------- settings ----------
const bool CALIBRATE = false;
float WHITE_R = 2600, WHITE_G = 3000, WHITE_B = 2700;   // replace with your white-paper RAW values
const float D_MIN = 2.0, D_MAX = 8.0;                   // good distance band in cm (same as the app)
const int PIN_LDR = 34, PIN_BTN = 0, GPS_RX = 16, GPS_TX = 17;
const int LED_RED = 13, LED_ORANGE = 27, LED_GREEN = 14;
const unsigned long DRONE_PAGE_MS = 15000;              // show a new drone scan for 15 s, then switch pages

// ---------- Bluetooth (Nordic UART service – the app looks for this) ----------
#define NUS_SERVICE "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
BLEServer* bleServer = nullptr;
BLECharacteristic* txChar = nullptr;
bool bleConnected = false;
String rxBuf = "";            // text arriving from the app (Bluetooth or USB)
volatile bool lineReady = false;
String rxLine = "";
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) { bleConnected = true; }
  void onDisconnect(BLEServer* s) { bleConnected = false; s->startAdvertising(); }
};
class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    String v = c->getValue().c_str();
    for (unsigned int i = 0; i < v.length(); i++) {
      char ch = v[i];
      if (ch == '\n') { rxLine = rxBuf; rxBuf = ""; lineReady = true; }
      else if (rxBuf.length() < 200) rxBuf += ch;
    }
  }
};

// ---------- last drone scan received from the app ----------
struct DroneScan { bool has = false; String zone, mac, mic, items, red, conf, lat, lon, site; int count = 0; unsigned long at = 0; } scan;

String field(const String& line, const String& key) {   // value of KEY:value in a comma line
  int i = line.indexOf(key + ":");
  if (i < 0) return "";
  int a = i + key.length() + 1, b = line.indexOf(',', a);
  return line.substring(a, b < 0 ? line.length() : b);
}
void setZoneLeds(const String& z) {
  digitalWrite(LED_RED, z == "HIGH");
  digitalWrite(LED_ORANGE, z == "MEDIUM");
  digitalWrite(LED_GREEN, z == "LOW");
}
void handleAppLine(String line) {
  line.trim();
  if (field(line, "SCAN") != "1") return;
  scan.has = true; scan.count++; scan.at = millis();
  scan.zone = field(line, "ZONE"); scan.mac = field(line, "MAC"); scan.mic = field(line, "MIC");
  scan.items = field(line, "ITEMS"); scan.red = field(line, "RED"); scan.conf = field(line, "CONF");
  scan.lat = field(line, "LAT"); scan.lon = field(line, "LON"); scan.site = field(line, "SITE");
  setZoneLeds(scan.zone);
}

// ---------- sensors ----------
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_VL53L0X lidar;
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
bool tcsOK = false, lidarOK = false, oledOK = false;

int savedCount = 0;
unsigned long lastSend = 0, savedFlashUntil = 0;
bool lastBtn = HIGH;
String lastMsg = "";

// Is this colour plastic-like? (simple version of the app's rule, for the OLED only)
String colourName(int r, int g, int b) {
  int mx = max(r, max(g, b)), mn = min(r, min(g, b));
  float v = mx / 255.0, s = mx ? (mx - mn) / (float)mx : 0, h = 0;
  if (mx != mn) {
    float k = mx - mn;
    if (mx == r) h = fmod((g - b) / k + 6, 6);
    else if (mx == g) h = (b - r) / k + 2;
    else h = (r - g) / k + 4;
    h *= 60;
  }
  if (v > 0.85 && s < 0.14) return "PLASTIC? white";
  if (s > 0.55 && v > 0.45) {
    if (h >= 180 && h <= 270) return "PLASTIC? blue";
    if (h >= 320 || h <= 12)  return "PLASTIC? red";
    if (h >= 48 && h <= 65)   return "PLASTIC? yellow";
  }
  if (h >= 70 && h <= 170 && s > 0.2) return "plants";
  return "soil / natural";
}

// Send one line by USB and Bluetooth (Bluetooth in 20-byte pieces; the app joins them)
void sendLine(const String& line) {
  Serial.print(line);
  if (bleConnected && txChar) {
    for (int i = 0; i < (int)line.length(); i += 20) {
      String part = line.substring(i, min((int)line.length(), i + 20));
      txChar->setValue((uint8_t*)part.c_str(), part.length());
      txChar->notify();
      delay(8);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT); pinMode(LED_ORANGE, OUTPUT); pinMode(LED_GREEN, OUTPUT);
  // quick LED test at start-up
  setZoneLeds("HIGH"); delay(250); setZoneLeds("MEDIUM"); delay(250); setZoneLeds("LOW"); delay(250); setZoneLeds("");
  analogReadResolution(12);

  Wire.begin(21, 22);
  Wire1.begin(25, 26);
  tcsOK = tcs.begin(TCS34725_ADDRESS, &Wire);
  lidarOK = lidar.begin(VL53L0X_I2C_ADDR, false, &Wire1);
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  BLEDevice::init("EcoDrone");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCB());
  BLEService* svc = bleServer->createService(NUS_SERVICE);
  txChar = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());
  BLECharacteristic* rxChar = svc->createCharacteristic(NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new RxCB());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  if (oledOK) {
    oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
    oled.setCursor(0, 0);  oled.println("EcoDrone Station");
    oled.println(tcsOK ? "RGB ok" : "RGB MISSING");
    oled.println(lidarOK ? "LiDAR ok" : "LiDAR MISSING");
    oled.println("GPS starting...");
    oled.display();
  }
  delay(1200);
}

void loop() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  // results sent by the app: by Bluetooth ...
  if (lineReady) { lineReady = false; handleAppLine(rxLine); }
  // ... or by USB
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') { handleAppLine(rxBuf); rxBuf = ""; }
    else if (rxBuf.length() < 200) rxBuf += ch;
  }

  // BOOT button pressed = save this spot
  bool btn = digitalRead(PIN_BTN);
  bool savePressed = (lastBtn == HIGH && btn == LOW);
  lastBtn = btn;
  if (savePressed) delay(30);  // simple debounce

  if (!savePressed && millis() - lastSend < 500) return;
  lastSend = millis();

  // --- read colour ---
  uint16_t rr = 0, gg = 0, bb = 0, cc = 0;
  if (tcsOK) tcs.getRawData(&rr, &gg, &bb, &cc);
  if (CALIBRATE) {
    Serial.printf("RAW R=%u G=%u B=%u C=%u\n", rr, gg, bb, cc);
    return;
  }
  int R = constrain((int)(rr * 255.0 / WHITE_R), 0, 255);
  int G = constrain((int)(gg * 255.0 / WHITE_G), 0, 255);
  int B = constrain((int)(bb * 255.0 / WHITE_B), 0, 255);

  // --- read light (0–100 %) ---
  int L = map(analogRead(PIN_LDR), 0, 4095, 0, 100);

  // --- read LiDAR distance in cm (-1 = out of range) ---
  float D = -1;
  if (lidarOK) {
    VL53L0X_RangingMeasurementData_t m;
    lidar.rangingTest(&m, false);
    if (m.RangeStatus != 4) D = m.RangeMilliMeter / 10.0;
  }
  bool distOK = !lidarOK || (D >= D_MIN && D <= D_MAX);

  // --- GPS ---
  bool fix = gps.location.isValid() && gps.location.age() < 5000;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

  // --- save only when the distance is right ---
  bool save = false;
  if (savePressed) {
    if (distOK) { save = true; savedCount++; lastMsg = "SAVED #" + String(savedCount); }
    else        { lastMsg = "Move to 2-8 cm!"; }
    savedFlashUntil = millis() + 1200;
  }

  // --- build and send the line ---
  String line = "R:" + String(R) + ",G:" + String(G) + ",B:" + String(B) + ",L:" + String(L);
  if (D > 0) line += ",D:" + String(D, 1);
  if (fix) line += ",LAT:" + String(gps.location.lat(), 6) + ",LON:" + String(gps.location.lng(), 6);
  else     line += ",GPS:0";
  line += ",SAT:" + String(sats) + ",SAVE:" + String(save ? 1 : 0) + "\n";
  sendLine(line);

  // --- OLED: drone scan page (new scan, or every other 4 s) or sensor page ---
  bool showDrone = scan.has && !savePressed && millis() > savedFlashUntil &&
                   (millis() - scan.at < DRONE_PAGE_MS || (millis() / 4000) % 2 == 0);
  if (oledOK && showDrone) {
    oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1); oled.setCursor(0, 0); oled.print("DRONE SCAN #"); oled.print(scan.count);
    oled.setTextSize(2); oled.setCursor(0, 11);
    oled.print(scan.zone == "HIGH" ? "RED ZONE" : scan.zone == "MEDIUM" ? "ORANGE" : "GREEN");
    oled.setTextSize(1);
    oled.setCursor(0, 30); oled.print("Macro "); oled.print(scan.mac); oled.print("% "); oled.print(scan.items); oled.print("pcs");
    oled.setCursor(0, 39); oled.print("Micro "); oled.print(scan.mic); oled.print("%  C"); oled.print(scan.conf); oled.print("%");
    oled.setCursor(0, 48); if (scan.site.length()) oled.print(scan.site); else { oled.print("Red land "); oled.print(scan.red); oled.print("%"); }
    oled.setCursor(0, 57); if (scan.lat.length()) { oled.print(scan.lat); oled.print(","); oled.print(scan.lon); }
    oled.display();
  } else if (oledOK) {
    oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);  oled.print("EcoDrone  "); oled.print(bleConnected ? "BT on" : "BT --");
    oled.setCursor(0, 9);  oled.printf("RGB %3d %3d %3d", R, G, B);
    oled.setCursor(0, 18); oled.print("> "); oled.print(colourName(R, G, B));
    oled.setCursor(0, 27); oled.printf("Light %d%%", L);
    oled.setCursor(0, 36);
    if (!lidarOK) oled.print("Dist: no LiDAR");
    else if (D < 0) oled.print("Dist: --  too far");
    else oled.printf("Dist %.1fcm %s", D, distOK ? "OK" : (D < D_MIN ? "UP" : "DOWN"));
    oled.setCursor(0, 45);
    if (fix) oled.printf("%.5f,%.5f", gps.location.lat(), gps.location.lng());
    else     oled.printf("GPS searching s%d", sats);
    if (millis() < savedFlashUntil) {
      oled.fillRect(0, 54, 128, 10, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK);
      oled.setCursor(2, 55); oled.print(lastMsg);
    } else {
      oled.setCursor(0, 55); oled.printf("BOOT=save  saved %d", savedCount);
    }
    oled.display();
  }
}
