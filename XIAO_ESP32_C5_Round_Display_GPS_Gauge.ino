// ============================================================
// XIAO ESP32-C5 + XIAO Round Display (GC9A01A) + GPS (ATGM336H) + SD Logger + WiFi Download
// Pin-Zuordnung:
// - Display: CS=D1, DC=D3, BL=D6, SCK=D8, MISO=D9, MOSI=D10
// - GPS: RX=D4 (SoftwareSerial), TX=D5 (SoftwareSerial)
// - SD-Karte: CS=D2, SCK=D8, MISO=D9, MOSI=D10 (SPI geteilt)
// - Zeit: Deutsche Zeit (MEZ/MESZ) mit DST-Berechnung
// - GPS-Optimierungen: SBAS, NMEA-Filterung, Warm Start
// - NEU: Automatische Datei pro Fahrt
// - NEU: WiFi Access Point zum Download der Log-Dateien
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>

// ============================================================
// PIN-DEFINITIONEN (Ihre Konfiguration)
// ============================================================
#define TFT_CS    D1
#define TFT_DC    D3
#define TFT_RST  -1
#define TFT_BL    D6
#define TFT_SCLK  D8
#define TFT_MISO  D9
#define TFT_MOSI D10

#define GPS_RX_PIN D4
#define GPS_TX_PIN D5
#define GPS_BAUD  9600

#define SD_CS    D2

// ============================================================
// WIFI KONFIGURATION (Access Point Modus)
// ============================================================
const char* AP_SSID = "XIAO-GPS-Logger";
const char* AP_PASSWORD = "gps12345";

WebServer server(80);
bool wifiConnected = false;

// ============================================================
// KONFIGURATION
// ============================================================
#define DISPLAY_INTERVAL_MS 250
#define LOG_INTERVAL_MS 10000
#define LOG_MIN_DISTANCE_M 10.0
#define MAX_LOG_ENTRIES 10000
#define LOG_FILE_PREFIX "GPS_TRACK"
#define INACTIVITY_TIMEOUT_MS 600000  // 10 Minuten Inaktivitat

// ============================================================
// GLOBALE VARIABLEN
// ============================================================
TinyGPSPlus gps;
SoftwareSerial GPSSerial(GPS_RX_PIN, GPS_TX_PIN);
Adafruit_GC9A01A tft(TFT_CS, TFT_DC, TFT_RST);

double odometerKm = 0.0;
bool havePreviousPosition = false;
double previousLat = 0.0;
double previousLng = 0.0;

const double MIN_SPEED_KMPH = 3.0;
const double MIN_VALID_STEP_METERS = 0.10;
const double MAX_VALID_STEP_METERS = 50.0;
const byte MAX_JUMP_REJECTS = 5;
byte consecutiveJumpRejects = 0;

unsigned long lastDisplayUpdate = 0;
unsigned long lastMovementTime = 0;
unsigned long lastLogTime = 0;
unsigned long logEntryCount = 0;

File logFile;
bool sdCardReady = false;
bool tripActive = false;
char currentLogFilename[32] = "";
double lastLoggedLat = 0.0;
double lastLoggedLng = 0.0;
bool sdStatusNeedsRedraw = true;
int tripNumber = 0;

// ============================================================
// DEUTSCHE ZEIT (MEZ/MESZ)
// ============================================================
int dayOfWeek(int year, int month, int day) {
  if (month < 3) { month += 12; year--; }
  int K = year % 100; int J = year / 100;
  int h = (day + (13 * (month + 1)) / 5 + K + K / 4 + J / 4 + 5 * J) % 7;
  return (h + 6) % 7;
}

int lastSunday(int year, int month, int lastDay) {
  return lastDay - dayOfWeek(year, month, lastDay);
}

bool getGermanTime(int &hour, int &minute, int &second) {
  if (!gps.time.isValid() || !gps.date.isValid()) return false;

  int year = gps.date.year();
  int month = gps.date.month();
  int day = gps.date.day();
  int utcHour = gps.time.hour();
  int utcMinute = gps.time.minute();
  int utcSecond = gps.time.second();

  int offset = 1; // MEZ
  int marchLastSunday = lastSunday(year, 3, 31);
  int octoberLastSunday = lastSunday(year, 10, 31);
  bool daylightSaving = false;

  if (month > 3 && month < 10) daylightSaving = true;
  else if (month == 3) {
    if (day > marchLastSunday) daylightSaving = true;
    else if (day == marchLastSunday && utcHour >= 1) daylightSaving = true;
  }
  else if (month == 10) {
    if (day < octoberLastSunday) daylightSaving = true;
    else if (day == octoberLastSunday && utcHour < 2) daylightSaving = true;
  }

  if (daylightSaving) offset = 2;
  hour = utcHour + offset;
  minute = utcMinute;
  second = utcSecond;
  if (hour >= 24) hour -= 24;
  return true;
}

// ============================================================
// WIFI FUNKTIONEN
// ============================================================

void initWiFi() {
  Serial.println("\n=== WIFI INITIALIZATION ===");
  
  // Access Point Modus starten
  Serial.print("Creating Access Point: ");
  Serial.println(AP_SSID);
  
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  delay(100);
  
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  
  // Server Routen einrichten
  server.on("/", handleRoot);
  server.on("/list", handleFileList);
  server.on("/download", handleFileDownload);
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("HTTP server started");
  
  wifiConnected = true;
}

void handleRoot() {
  server.sendHeader("Location", "/list");
  server.send(302, "text/plain", "Redirecting to file list...");
}

void handleFileList() {
  if (!sdCardReady) {
    server.send(500, "text/plain", "SD card not ready");
    return;
  }
  
  String html = "<html><head><title>XIAO GPS Logger - Dateien</title><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'></head><body>"
               "<h1>GPS Log-Dateien</h1>"
               "<p>Verbunden mit: " + String(AP_SSID) + "</p>"
               "<p><a href='/list'>Aktualisieren</a></p>"
               "<ul>";
  
  File root = SD.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      if (!entry.isDirectory()) {
        String filename = entry.name();
        if (filename.endsWith(".CSV") || filename.endsWith(".csv")) {
          size_t size = entry.size();
          html += "<li><a href='/download?file=" + String(filename) + "'>" + String(filename) + "</a> (" + String(size/1024.0, 1) + " KB)</li>";
        }
      }
      entry = root.openNextFile();
    }
    root.close();
  }
  
  html += "</ul><p>Hinweis: Dateien werden automatisch pro Fahrt erstellt.</p></body></html>";
  server.send(200, "text/html", html);
}

void handleFileDownload() {
  if (!sdCardReady) {
    server.send(500, "text/plain", "SD card not ready");
    return;
  }
  
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "No file specified");
    return;
  }
  
  String filename = server.arg("file");
  if (filename.indexOf("/") >= 0 || filename.indexOf("..") >= 0) {
    server.send(400, "text/plain", "Invalid filename");
    return;
  }
  
  File file = SD.open("/" + filename);
  if (!file) {
    server.send(404, "text/plain", "File not found: " + filename);
    return;
  }
  
  server.sendHeader("Content-Type", "text/csv");
  server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
  server.sendHeader("Content-Length", String(file.size()));
  server.send(200);
  
  // Datei in Chunks senden
  uint8_t buffer[1024];
  size_t bytesRead;
  while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0) {
    server.sendContent((const char*)buffer, bytesRead);
  }
  
  file.close();
}

void handleNotFound() {
  String message = "File Not Found\n\nURI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

// ============================================================
// SD-KARTEN FUNKTIONEN
// ============================================================
void initSDCard() {
  Serial.println("=== SD CARD INITIALIZATION ===");
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  delay(10);

  if (!SD.begin(SD_CS)) {
    Serial.println("[FAILED] SD card initialization failed!");
    sdCardReady = false;
    return;
  }
  Serial.println("[OK] SD card ready");
  sdCardReady = true;
  tripNumber = 0;
  createNewLogFile();
}

void createNewLogFile() {
  if (!sdCardReady) return;
  if (logFile) logFile.close();

  tripNumber++;
  char timestamp[25];
  if (gps.date.isValid() && gps.time.isValid()) {
    int hour, minute, second;
    if (getGermanTime(hour, minute, second)) {
      snprintf(timestamp, sizeof(timestamp), "%04d%02d%02d_T%02d%02d%02d_T%03d",
               gps.date.year(), gps.date.month(), gps.date.day(), hour, minute, second, tripNumber);
    } else {
      snprintf(timestamp, sizeof(timestamp), "%04d%02d%02d_T%02d%02d%02d_T%03d",
               gps.date.year(), gps.date.month(), gps.date.day(),
               gps.time.hour(), gps.time.minute(), gps.time.second(), tripNumber);
    }
  } else {
    snprintf(timestamp, sizeof(timestamp), "%04d%02d%02d_T000000_T%03d", 2024, 1, 1, tripNumber);
  }

  snprintf(currentLogFilename, sizeof(currentLogFilename), "/%s_%s.CSV", LOG_FILE_PREFIX, timestamp);
  logFile = SD.open(currentLogFilename, FILE_WRITE);
  if (!logFile) {
    Serial.print("[ERROR] Failed to create: "); Serial.println(currentLogFilename);
    sdCardReady = false;
    return;
  }

  logFile.seek(0, SeekEnd);
  if (logFile.size() == 0) {
    logFile.println("Timestamp,Latitude,Longitude,Speed_Kmph,Altitude_M,Satellites,Odometer_Km");
  }
  logFile.flush();
  logEntryCount = 0;
  Serial.print("[OK] Log file: "); Serial.println(currentLogFilename);
}

void writeLogEntry() {
  if (!sdCardReady || !gps.location.isValid() || !gps.time.isValid()) return;
  if (millis() - lastLogTime < LOG_INTERVAL_MS) return;
  if (LOG_MIN_DISTANCE_M > 0 && havePreviousPosition) {
    double distanceMeters = TinyGPSPlus::distanceBetween(
      lastLoggedLat, lastLoggedLng, gps.location.lat(), gps.location.lng());
    if (distanceMeters < LOG_MIN_DISTANCE_M) return;
  }

  if (!logFile) {
    logFile = SD.open(currentLogFilename, FILE_WRITE);
    if (!logFile) {
      Serial.println("[SD ERROR] Failed to reopen log file");
      sdCardReady = false;
      return;
    }
    logFile.seek(0, SeekEnd);
  }

  char buffer[128];
  int year, month, day, hour, minute, second;
  float speed = gps.speed.isValid() ? gps.speed.kmph() : 0;
  float altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0;
  int satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;

  if (getGermanTime(hour, minute, second)) {
    year = gps.date.year(); month = gps.date.month(); day = gps.date.day();
  } else {
    year = gps.date.year(); month = gps.date.month(); day = gps.date.day();
    hour = gps.time.hour(); minute = gps.time.minute(); second = gps.time.second();
  }

  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d,%.6f,%.6f,%.1f,%.1f,%d,%.2f",
           year, month, day, hour, minute, second,
           gps.location.lat(), gps.location.lng(), speed, altitude, satellites, odometerKm);

  logFile.println(buffer);
  logFile.flush();

  lastLogTime = millis();
  lastLoggedLat = gps.location.lat();
  lastLoggedLng = gps.location.lng();
  logEntryCount++;

  if (logEntryCount >= MAX_LOG_ENTRIES) {
    logFile.close();
    createNewLogFile();
  }
}

// ============================================================
// DISPLAY FUNKTIONEN
// ============================================================
void drawWiFiStatus() {
  tft.setTextSize(1);
  tft.setCursor(155, 210);
  if (wifiConnected) {
    tft.setTextColor(0x07E0, 0x0000); // Gruen
    tft.print("WiFi");
  } else {
    tft.setTextColor(0xF800, 0x0000); // Rot
    tft.print("WiFi");
  }
}

void drawSDStatus() {
  tft.setTextSize(1);
  tft.setCursor(185, 210);
  if (sdCardReady) {
    tft.setTextColor(0x07E0, 0x0000); // Gruen
    tft.print("SD");
  } else {
    tft.setTextColor(0xF800, 0x0000); // Rot
    tft.print("SD");
  }
}

void drawTime() {
  char timeBuffer[16];
  int hour, minute, second;
  if (getGermanTime(hour, minute, second)) {
    snprintf(timeBuffer, 16, "%02d:%02d:%02d", hour, minute, second);
  } else {
    strcpy(timeBuffer, "--:--:--");
  }
  tft.setTextSize(2);
  tft.setCursor(75, 15);
  tft.setTextColor(0xFFFF, 0x0000);
  tft.print(timeBuffer);
}

void drawSpeed() {
  char speedBuffer[8];
  if (gps.speed.isValid()) {
    int speed = (int)(gps.speed.kmph() + 0.5);
    snprintf(speedBuffer, sizeof(speedBuffer), "%03d", speed);
  } else {
    strcpy(speedBuffer, "---");
  }
  tft.setTextSize(5);
  tft.setCursor(78, 90);
  tft.setTextColor(0xFFFF, 0x0000);
  tft.print(speedBuffer);
  tft.setTextSize(2);
  tft.setCursor(95, 135);
  tft.print("km/h");
}

void drawOdometer() {
  char odometerBuffer[16];
  int roundedValue = (int)((odometerKm * 100.0) + 0.5);
  snprintf(odometerBuffer, 16, "%03d.%02d", roundedValue / 100, roundedValue % 100);
  tft.setTextSize(3);
  tft.setCursor(55, 160);
  tft.setTextColor(0xFFFF, 0x0000);
  tft.print(odometerBuffer);
  tft.setTextSize(1);
  tft.setCursor(180, 165);
  tft.print("km");
}

void drawSatellites() {
  tft.setTextSize(1);
  tft.setCursor(60, 195);
  tft.setTextColor(0xFFFF, 0x0000);
  if (gps.satellites.isValid()) {
    tft.print("Sats:");
    tft.print(gps.satellites.value());
  } else {
    tft.print("Sats:--");
  }
}

void drawLatitude() {
  tft.setTextSize(1);
  tft.setCursor(30, 55);
  tft.setTextColor(0xFFFF, 0x0000);
  if (gps.location.isValid()) {
    tft.print("LAT ");
    tft.print(gps.location.lat(), 6);
  } else {
    tft.print("LAT --.------");
  }
}

void drawLongitude() {
  tft.setTextSize(1);
  tft.setCursor(125, 55);
  tft.setTextColor(0xFFFF, 0x0000);
  if (gps.location.isValid()) {
    tft.print("LON ");
    tft.print(gps.location.lng(), 6);
  } else {
    tft.print("LON --.------");
  }
}

void drawStaticScreen() {
  tft.fillScreen(0x0000);
  tft.drawLine(25, 40, 215, 40, 0xFFFF);
  tft.drawLine(25, 155, 215, 155, 0xFFFF);
  drawSatellites();
  drawSDStatus();
  drawWiFiStatus();
  drawTime();
  drawLatitude();
  drawLongitude();
  drawSpeed();
  drawOdometer();
}

// ============================================================
// GPS FUNKTIONEN
// ============================================================
void readGPS() {
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    gps.encode(c);
    Serial.write(c); // Debug: NMEA-Daten an Serial ausgeben
  }
}

void updateGPSOdometer() {
  if (!gps.location.isUpdated() || !gps.location.isValid()) return;
  if (!gps.satellites.isValid() || gps.satellites.value() < 4) {
    havePreviousPosition = false;
    consecutiveJumpRejects = 0;
    return;
  }

  bool isMoving = (gps.speed.isValid() && gps.speed.kmph() >= MIN_SPEED_KMPH);
  if (isMoving) {
    lastMovementTime = millis();
    if (!tripActive) {
      tripActive = true;
      odometerKm = 0.0;
      havePreviousPosition = false;
      createNewLogFile();
      Serial.println("[TRIP] Neue Fahrt gestartet");
    }
  }

  if (!gps.speed.isValid() || gps.speed.kmph() < MIN_SPEED_KMPH) return;

  double lat = gps.location.lat();
  double lng = gps.location.lng();

  if (!havePreviousPosition) {
    previousLat = lat;
    previousLng = lng;
    havePreviousPosition = true;
    consecutiveJumpRejects = 0;
    return;
  }

  double distanceMeters = TinyGPSPlus::distanceBetween(
    previousLat, previousLng, lat, lng);

  if (distanceMeters >= MIN_VALID_STEP_METERS && distanceMeters <= MAX_VALID_STEP_METERS) {
    odometerKm += distanceMeters / 1000.0;
    previousLat = lat;
    previousLng = lng;
    consecutiveJumpRejects = 0;
  } else if (distanceMeters > MAX_VALID_STEP_METERS) {
    consecutiveJumpRejects++;
    if (consecutiveJumpRejects >= MAX_JUMP_REJECTS) {
      previousLat = lat;
      previousLng = lng;
      consecutiveJumpRejects = 0;
    }
  }
}

// ============================================================
// SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=======================");
  Serial.println("\nXIAO ESP32-C5 GPS Logger");
  Serial.println("=======================");

  // SPI initialisieren
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);

  // Display
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.begin();
  delay(500);
  tft.invertDisplay(true);
  tft.setRotation(3);
  tft.fillScreen(0x0000);
  drawStaticScreen();

  // SD-Karte
  initSDCard();

  // GPS
  GPSSerial.begin(GPS_BAUD);
  delay(1000);
  GPSSerial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // only GGA + RMC aktiv
  GPSSerial.println("$PMTK220,200*2F"); // 5 Hz Update-Rate
  GPSSerial.println("$PMTK301,2*2E"); // SBAS (EGNOS/WAAS) aktivieren

  // WiFi initialisieren
  initWiFi();

  // Trip Detection
  lastMovementTime = millis();
  tripActive = false;

  Serial.println("\n=== SYSTEM READY ===");
  Serial.println("Connect to WiFi: " + String(AP_SSID));
  Serial.println("Password: " + String(AP_PASSWORD));
  Serial.println("Then open: http://" + WiFi.softAPIP().toString());
}

void loop() {
  readGPS();
  updateGPSOdometer();

  // Inaktivitats-Erkennung
  if (tripActive && millis() - lastMovementTime > INACTIVITY_TIMEOUT_MS) {
    tripActive = false;
    if (logFile) logFile.close();
    Serial.println("[TRIP] Fahrt beendet (10 Min. Inaktivitat)");
  }

  writeLogEntry();

  // WiFi Server bedienen
  server.handleClient();

  // Display aktualisieren
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = millis();
    drawSatellites();
    drawTime();
    drawLatitude();
    drawLongitude();
    drawSpeed();
    drawOdometer();
    drawSDStatus();
    drawWiFiStatus();
  }
}
