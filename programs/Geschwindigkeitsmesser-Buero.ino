#include <HardwareSerial.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

HardwareSerial lidarSerial(2);

#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17

// --- Mittelwert-Variablen ---
long sum = 0;
int validCount = 0;
unsigned long lastOutputTime = 0;
const unsigned long interval = 250;  // 250 ms = 4 Ausgaben pro Sekunde

// --- Variablen für Geschwindigkeitsberechnung ---
float lastDistance = 0;
unsigned long lastValidTime = 0;
bool firstValid = true;

// --- Glättung für Geschwindigkeit (Puffer) ---
const int smoothWindow = 4;  // Mittelwert über 4 Werte (1 Sekunde)
float speedHistory[smoothWindow] = {0};  // Array für letzte Werte
int speedIndex = 0;  // Aktueller Index

// --- MAX7219 Display-Variablen ---
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW  // Für 4-in-1-Module
#define MAX_DEVICES 4                      // 4 Module = 32x8
#define CS_PIN 5                           // Chip Select (beliebig, z. B. GPIO5)
MD_Parola myDisplay = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

// --- Neu: Letzte angezeigte Geschwindigkeit speichern + Toleranz ---
float last_speed_kmh = -1.0;  // Initial ungleich jedem Wert
const float speed_tolerance = 0.1;  // Mindeständerung für Update (gegen Rauschen)

// --- Smiley als 8x8 Bitmap ---
const uint8_t angrySmiley[8] PROGMEM = {
  B00111100,  // Oberseite Kopf
  B01000010,  // Augenbrauen runter
  B10100101,  // Augen
  B10000001,  // 
  B10100101,  // Mund
  B10011001,  //
  B01000010,  // 
  B00111100   // Unterseite Kopf
};

// --- Neu: Variablen für Smiley-Blinken und 3-Sekunden-Timer ---
bool showSmiley = false;
unsigned long smileyStartTime = 0;
unsigned long lastBlinkTime = 0;
bool smileyVisible = false;
const unsigned long smileyDuration = 3000;  // 3 Sekunden
const unsigned long blinkInterval = 500;    // Blinken alle 500 ms

void setup() {
  Serial.begin(115200);
  lidarSerial.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  
  // Display initialisieren
  myDisplay.begin();
  myDisplay.setIntensity(8);         // Helligkeit (0–15, anpassen)
  myDisplay.displayClear();          // Display löschen
  myDisplay.setCharSpacing(1);       // Standard-Abstand → schöne Darstellung
  
  Serial.println("\nTFmini-S: Abstand + Geschwindigkeit (km/h) auf Display – Smiley blinkt 3s bei >5 km/h");
  delay(100);
}

int getDistance() {
  if (lidarSerial.available() >= 9) {
    if (lidarSerial.read() != 0x59) return -1;
    if (lidarSerial.read() != 0x59) return -1;

    uint8_t Dist_L  = lidarSerial.read();
    uint8_t Dist_H  = lidarSerial.read();
    uint8_t Str_L   = lidarSerial.read();
    uint8_t Str_H   = lidarSerial.read();
    uint8_t Temp_L  = lidarSerial.read();
    uint8_t Temp_H  = lidarSerial.read();
    uint8_t CheckSum = lidarSerial.read();

    uint8_t calculatedCheck = (0x59 + 0x59 + Dist_L + Dist_H + Str_L + Str_H + Temp_L + Temp_H) & 0xFF;

    if (CheckSum == calculatedCheck) {
      int distance = Dist_L + (Dist_H << 8);
      int strength = Str_L + (Str_H << 8);

      if (strength < 50) return -1;
      if (distance < 10 || distance > 1200) return -1;

      return distance;  // in cm
    }
  }
  return -1;
}

// Funktion zur Glättung: Mittelwert der letzten smoothWindow Werte
float getSmoothedSpeed(float newSpeed) {
  speedHistory[speedIndex] = newSpeed;
  speedIndex = (speedIndex + 1) % smoothWindow;

  float total = 0.0;
  for (int i = 0; i < smoothWindow; i++) {
    total += speedHistory[i];
  }
  return total / smoothWindow;
}

// Funktion zum Zeigen des Smileys (Bitmap setzen)
void drawSmiley(bool visible) {
  myDisplay.displayClear();  // Display leeren
  if (visible) {
    for (uint8_t row = 0; row < 8; row++) {
      uint8_t data = pgm_read_byte_near(&angrySmiley[row]);
      for (uint8_t col = 0; col < 8; col++) {
        bool point = (data >> (7 - col)) & 1;
        myDisplay.getGraphicObject()->setPoint(row, 12 + col, point);  // Zentriert (anpassen, z. B. 12 für 32x8)
      }
    }
  }
}

void loop() {
  int dist = getDistance();

  if (dist != -1) {
    sum += dist;
    validCount++;
  }

  unsigned long now = millis();

  // Alle 250 ms → genau 4 Hz (Geschwindigkeit aktualisieren)
  if (now - lastOutputTime >= interval) {
    if (validCount > 0) {
      float average = (float)sum / validCount;

      // --- Geschwindigkeit berechnen NUR bei Annäherung ---
      float raw_speed_kmh = 0.0;
      if (!firstValid && lastDistance > average) {
        float deltaDistance_cm = lastDistance - average;
        float deltaTime_s = (now - lastValidTime) / 1000.0;
        if (deltaTime_s > 0) {
          float speed_cm_per_s = deltaDistance_cm / deltaTime_s;
          raw_speed_kmh = speed_cm_per_s * 0.036;
        }
      }
      float speed_kmh = round(getSmoothedSpeed(raw_speed_kmh));

      // --- Serial-Ausgabe ---
      Serial.print("Abstand: ");
      Serial.print(average, 1);
      Serial.print(" cm  (");
      Serial.print(validCount);
      Serial.print(" Werte)  →  ");
      Serial.print(speed_kmh, 0);
      Serial.println(" km/h");

      // --- Prüfen, ob Smiley starten (bei >9 km/h) ---
      if (speed_kmh > 9 && !showSmiley) {
        showSmiley = true;
        smileyStartTime = now;
        smileyVisible = true;
        drawSmiley(true);  // Sofort anzeigen
      }

      // --- Normale Text-Anzeige, wenn kein Smiley aktiv ---
      if (!showSmiley && abs(speed_kmh - last_speed_kmh) > speed_tolerance) {
        char speedText[20];
        sprintf(speedText, "%.0f km/h", speed_kmh);
        myDisplay.displayReset();
        myDisplay.displayText(speedText, PA_CENTER, 0, 0, PA_PRINT, PA_PRINT);
        myDisplay.displayAnimate();
        last_speed_kmh = speed_kmh;
      }

      lastDistance = average;
      lastValidTime = now;
      firstValid = false;
    }
    else {
      Serial.println("Keine gültigen Messungen im letzten Intervall");
      if (last_speed_kmh != -1.0) {
        myDisplay.displayClear();
        last_speed_kmh = -1.0;
      }
    }

    sum = 0;
    validCount = 0;
    lastOutputTime = now;
  }

  // --- Smiley-Handling (unabhängig von Update-Intervall, für flüssiges Blinken) ---
  if (showSmiley) {
    // Blinken: Ein/Aus wechseln
    if (now - lastBlinkTime >= blinkInterval) {
      smileyVisible = !smileyVisible;
      drawSmiley(smileyVisible);
      lastBlinkTime = now;
    }

    // Timer prüfen: Nach 3 Sekunden Smiley deaktivieren
    if (now - smileyStartTime >= smileyDuration) {
      showSmiley = false;
      myDisplay.displayClear();  // Zurück zur normalen Anzeige
    }
  }

  delay(20);  // ca. 50 Messungen/Sekunde
}
