#define ST7789_DRIVER
#define TFT_WIDTH  240
#define TFT_HEIGHT 320

#define TFT_MISO -1          // nicht benutzt
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  4

#define TFT_BL   -1          // Backlight direkt an VCC oder separat steuern
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define SPI_FREQUENCY  40000000   // 40 MHz läuft problemlos

#include <Wire.h>
#include <TFT_eSPI.h>        // <-- Hauptbibliothek


TFT_eSPI tft = TFT_eSPI();

// --- LIDAR-Teil bleibt fast unverändert -----------------
const uint8_t LIDAR_ADDR = 0x62;

void lidarWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void lidarRead(uint8_t reg, uint8_t* buf, uint8_t num) {
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LIDAR_ADDR, num);
  for (uint8_t i = 0; i < num; i++) buf[i] = Wire.read();
}

bool lidarReady() {
  uint8_t s; 
  lidarRead(0x01, &s, 1); 
  return !(s & 0x01);
}
// --------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // LCD starten
  tft.init();
  tft.setRotation(1);           // Landscape (320×240 wird zur Breite)
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Startmeldung
  tft.setFreeFont(&FreeSans12pt7b);
  tft.drawString("Starte LIDAR...", 20, 100);
  
  // Velocity-Modus aktivieren
  lidarWrite(0x04, 0x08);       // Bit 3 = Velocity-Messung einschalten
  delay(100);
}

void loop() {
  // --- dein LIDAR-Code bleibt unverändert ---
  int distance = 0;
  int8_t velocity = 0;

  lidarWrite(0x00, 0x04);
  unsigned long t = millis();
  while (!lidarReady() && (millis() - t < 50));

  if (lidarReady()) {
    uint8_t buf[2];
    lidarRead(0x8F, buf, 2);
    distance = (buf[0] << 8) | buf[1];
    lidarRead(0x09, (uint8_t*)&velocity, 1);
  }

  // --- Anzeige (komplett ohne getTextBounds) ---
  tft.fillScreen(TFT_BLACK);

  float kmh = velocity * 0.036f;                 // cm/s → km/h
  int kmh_int = (int)abs(kmh + 0.5f);             // gerundet, positiv

  // Große Zahl zentriert (manuell gerechnet – funktioniert immer)
  String s = String(kmh_int);
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextColor(velocity < 0 ? TFT_CYAN : TFT_YELLOW, TFT_BLACK);

  // 1- bis 3-stellige Zahl zentrieren
  if (kmh_int < 10)       tft.drawString(s, 110, 90);      // einstellige Zahl
  else if (kmh_int < 100) tft.drawString(s, 80, 90);       // zweistellig
  else                    tft.drawString(s, 50, 90);       // dreistellig 

  // km/h kleiner darunter
  tft.setFreeFont(&FreeSans12pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("km/h", 130, 140);

  // Richtungspfeile links/rechts
  tft.setFreeFont(&FreeSansBold24pt7b);
  if (velocity < -30) tft.drawString("<<", 20, 90);    // kommt auf dich zu
  if (velocity >  30) tft.drawString(">>", 260, 90);   // läuft weg

  // Distanz unten
  tft.setFreeFont(NULL);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Distanz: " + String(distance) + " cm", 5, 220);

  delay(100);
}
