#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <TinyGPS++.h>

// ============================================================
// XIAO ESP32-C5 + Round Display (GC9A01A) + GPS (ATGM336H) + SD Logger
// Pin-Zuordnung:
//   - Display: CS=D1, DC=D3, BL=D6, SCK=D8, MISO=D9, MOSI=D10
//   - GPS:     RX=19 (GPIO19, LP_UART_RX), TX=20 (GPIO20, LP_UART_TX)
//   - SD-Karte: CS=D2, SCK=D8, MISO=D9, MOSI=D10 (SPI geteilt)
// ============================================================

// ============================================================
// DISPLAY PINS (GC9A01A Round Display)
// ============================================================
#define TFT_CS      D1    // Chip Select
#define TFT_DC      D3    // Data/Command (wie gewünscht!)
#define TFT_RST     -1   // Reset (nicht verbunden)
#define TFT_BL      D6    // Backlight

// SPI Pins (geteilt mit SD-Karte)
#define TFT_SCLK    D8    // Shared SCK
#define TFT_MISO    D9    // Shared MISO
#define TFT_MOSI    D10   // Shared MOSI

// ============================================================
// GPS PINS (ATGM336H auf LP_UART - GPIO19/20)
// ============================================================
#define GPS_RX_PIN  19    // LP_UART_RX (GPS TX → ESP RX, GPIO19)
#define GPS_TX_PIN  20    // LP_UART_TX (GPS RX → ESP TX, GPIO20)
#define GPS_BAUD    9600

// ============================================================
// SD CARD PINS (SPI Mode, geteilt mit Display)
// ============================================================
#define SD_CS       D2    // Chip Select (GEÄNDERT VON D4 AUF D2!)

#define SD_SCK      D8    // Geteilt mit Display
#define SD_MISO     D9    // Geteilt mit Display
#define SD_MOSI     D10   // Geteilt mit Display

// ============================================================
// COLORS
// ============================================================
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0

// ============================================================
// SCREEN SIZE (240x240 Round Display)
// ============================================================
#define SCREEN_W 240
#define SCREEN_H 240

// ============================================================
// LAYOUT CONFIGURATION (Optimized for Round Display)
// ============================================================
// SATELLITES
#define SAT_X       70
#define SAT_Y       195

// SD STATUS
#define SD_STATUS_X 170
#define SD_STATUS_Y 195

// TIME
#define TIME_X      75
#define TIME_Y      15

// LATITUDE
#define LAT_X       30
#define LAT_Y       55

// LONGITUDE
#define LON_X      125
#define LON_Y       55

// SPEED
#define SPEED_X     78
#define SPEED_Y     90
#define SPEED_TEXT_SIZE 5
#define SPEED_UNIT_X   95
#define SPEED_UNIT_Y  135

// ODOMETER
#define ODO_X       55
#define ODO_Y       160
#define ODO_TEXT_SIZE 3
#define ODO_UNIT_X  180
#define ODO_UNIT_Y  165

// ============================================================
// DISPLAY UPDATE RATE
// ============================================================
#define DISPLAY_INTERVAL_MS 250

// ============================================================
// LOGGING CONFIGURATION
// ============================================================
#define LOG_INTERVAL_MS 1000          // Log every 1 second
#define LOG_MIN_DISTANCE_M 5.0       // Log only if moved >5m
#define MAX_LOG_ENTRIES 10000        // Max entries per file
#define LOG_FILE_PREFIX "GPS_TRACK"

// ============================================================
// GPS
// ============================================================
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);  // LP_UART = UART2 auf ESP32-C5

// ============================================================
// DISPLAY (Seeed Studio XIAO Round Display)
// ============================================================
Adafruit_GC9A01A tft(TFT_CS, TFT_DC, TFT_RST);

// ============================================================
// ODOMETER
// ============================================================
double odometerKm = 0.0;
bool havePreviousPosition = false;
double previousLat = 0.0;
double previousLng = 0.0;

// ============================================================
// ODOMETER FILTER
// ============================================================
const double MIN_SPEED_KMPH = 3.0;
const double MIN_VALID_STEP_METERS = 0.10;
const double MAX_VALID_STEP_METERS = 50.0;
const byte MAX_JUMP_REJECTS = 5;
byte consecutiveJumpRejects = 0;

// ============================================================
// DISPLAY TIMER
// ============================================================
unsigned long lastDisplayUpdate = 0;

// ============================================================
// SD CARD & LOGGING
// ============================================================
File logFile;
unsigned long lastLogTime = 0;
unsigned long logEntryCount = 0;
bool sdCardReady = false;
char currentLogFilename[32] = "";
double lastLoggedLat = 0.0;
double lastLoggedLng = 0.0;
bool sdStatusNeedsRedraw = true;

// ============================================================
// BULGARIAN TIME
// ============================================================
int dayOfWeek(int year, int month, int day) {
    if (month < 3) { month += 12; year--; }
    int K = year % 100;
    int J = year / 100;
    int h = (day + (13 * (month + 1)) / 5 + K + K / 4 + J / 4 + 5 * J) % 7;
    return (h + 6) % 7;
}

int lastSunday(int year, int month, int lastDay) {
    int weekday = dayOfWeek(year, month, lastDay);
    return lastDay - weekday;
}

bool getBulgarianTime(int &hour, int &minute, int &second) {
    if (!gps.time.isValid() || !gps.date.isValid()) return false;

    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();
    int utcHour = gps.time.hour();
    int utcMinute = gps.time.minute();
    int utcSecond = gps.time.second();

    int offset = 2;

    int marchLastSunday = lastSunday(year, 3, 31);
    int octoberLastSunday = lastSunday(year, 10, 31);
    bool daylightSaving = false;

    if (month > 3 && month < 10) {
        daylightSaving = true;
    } else if (month == 3) {
        if (day > marchLastSunday || (day == marchLastSunday && utcHour >= 1)) {
            daylightSaving = true;
        }
    } else if (month == 10) {
        if (day < octoberLastSunday || (day == octoberLastSunday && utcHour < 1)) {
            daylightSaving = true;
        }
    }

    if (daylightSaving) offset = 3;
    hour = utcHour + offset;
    minute = utcMinute;
    second = utcSecond;

    if (hour >= 24) hour -= 24;
    return true;
}

// ============================================================
// SD CARD FUNCTIONS (MIT DEBUG-CODE)
// ============================================================
void initSDCard() {
    Serial.println("=== SD CARD INITIALIZATION ===");

    // Setze CS-Pin auf HIGH (SD-Karte deaktivieren)
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    delay(10);  // Kurze Wartezeit für Stabilität

    Serial.print("Initializing SD card on CS Pin D2... ");
    if (!SD.begin(SD_CS)) {
        Serial.println("[FAILED]");
        Serial.println("  - Check if SD card is inserted correctly");
        Serial.println("  - Check if SD card is formatted as FAT32");
        Serial.println("  - Check if CS pin (D2) is connected properly");
        Serial.println("  - Try a different SD card (max 32GB)");
        sdCardReady = false;
        sdStatusNeedsRedraw = true;
        return;
    }

    Serial.println("[OK]");

    // SD-Karten-Informationen anzeigen
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("  - No SD card detected!");
        sdCardReady = false;
        sdStatusNeedsRedraw = true;
        return;
    }

    Serial.print("  - Card Type: ");
    if (cardType == CARD_MMC) {
        Serial.println("MMC");
    } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    // SD-Karten-Größe anzeigen
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.print("  - Card Size: ");
    Serial.print(cardSize);
    Serial.println(" MB");

    // Dateisystem-Informationen
    uint64_t totalBytes = SD.totalBytes();
    uint64_t usedBytes = SD.usedBytes();
    Serial.print("  - Total Space: ");
    Serial.print(totalBytes / (1024 * 1024));
    Serial.println(" MB");
    Serial.print("  - Used Space: ");
    Serial.print(usedBytes / (1024 * 1024));
    Serial.println(" MB");
    Serial.print("  - Free Space: ");
    Serial.print((totalBytes - usedBytes) / (1024 * 1024));
    Serial.println(" MB");

    // Dateiliste anzeigen
    listSDFiles();

    sdCardReady = true;
    createNewLogFile();
    sdStatusNeedsRedraw = true;
    Serial.println("=== SD CARD READY ===\n");
}

// Liste aller Dateien auf der SD-Karte anzeigen
void listSDFiles() {
    Serial.println("  - Listing files on SD card:");
    File root = SD.open("/");
    if (!root) {
        Serial.println("    [ERROR] Failed to open root directory!");
        return;
    }

    int fileCount = 0;
    int dirCount = 0;
    File entry;
    while (entry = root.openNextFile()) {
        if (entry.isDirectory()) {
            Serial.print("    [DIR]  ");
            Serial.println(entry.name());
            dirCount++;
        } else {
            Serial.print("    [FILE] ");
            Serial.print(entry.name());
            Serial.print(" (");
            Serial.print(entry.size() / 1024);
            Serial.println(" KB)");
            fileCount++;
        }
        entry.close();
    }
    root.close();

    if (fileCount == 0 && dirCount == 0) {
        Serial.println("    (No files or directories found)");
    } else {
        Serial.print("  - Total: ");
        Serial.print(fileCount);
        Serial.print(" files, ");
        Serial.print(dirCount);
        Serial.println(" directories");
    }
}

void createNewLogFile() {
    if (!sdCardReady) return;

    Serial.println("Creating new log file...");

    char timestamp[20];
    if (gps.date.isValid() && gps.time.isValid()) {
        snprintf(timestamp, sizeof(timestamp), "%04d%02d%02d_%02d%02d%02d",
                gps.date.year(), gps.date.month(), gps.date.day(),
                gps.time.hour(), gps.time.minute(), gps.time.second());
    } else {
        snprintf(timestamp, sizeof(timestamp), "%04d%02d%02d_%02d%02d%02d",
                2024, 1, 1, 0, 0, 0);
    }

    snprintf(currentLogFilename, sizeof(currentLogFilename), "/%s_%s.CSV",
            LOG_FILE_PREFIX, timestamp);

    logFile = SD.open(currentLogFilename, FILE_WRITE);
    if (!logFile) {
        Serial.print("  [ERROR] Failed to create log file: ");
        Serial.println(currentLogFilename);
        sdCardReady = false;
        sdStatusNeedsRedraw = true;
        return;
    }

    Serial.print("  [OK] Created: ");
    Serial.println(currentLogFilename);

    // CSV-Header schreiben
    logFile.println("Timestamp,Latitude,Longitude,Speed_Kmph,Altitude_M,Satellites,Odometer_Km");
    logFile.flush();
    logFile.close();
    logEntryCount = 0;
    sdStatusNeedsRedraw = true;
}

void writeLogEntry() {
    if (!sdCardReady) return;
    if (!gps.location.isValid() || !gps.time.isValid()) return;
    if (millis() - lastLogTime < LOG_INTERVAL_MS) return;

    if (LOG_MIN_DISTANCE_M > 0 && havePreviousPosition) {
        double distanceMeters = TinyGPSPlus::distanceBetween(
            lastLoggedLat, lastLoggedLng,
            gps.location.lat(), gps.location.lng()
        );
        if (distanceMeters < LOG_MIN_DISTANCE_M) return;
    }

    logFile = SD.open(currentLogFilename, FILE_WRITE);
    if (!logFile) {
        Serial.print("[SD ERROR] Failed to open log file: ");
        Serial.println(currentLogFilename);
        sdCardReady = false;
        sdStatusNeedsRedraw = true;
        return;
    }

    char buffer[128];
    int year, month, day, hour, minute, second;
    float speed = gps.speed.isValid() ? gps.speed.kmph() : 0;
    float altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0;
    int satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;

    if (getBulgarianTime(hour, minute, second)) {
        year = gps.date.year();
        month = gps.date.month();
        day = gps.date.day();
    } else {
        year = gps.date.year();
        month = gps.date.month();
        day = gps.date.day();
        hour = gps.time.hour();
        minute = gps.time.minute();
        second = gps.time.second();
    }

    snprintf(buffer, sizeof(buffer),
            "%04d-%02d-%02d %02d:%02d:%02d,%.6f,%.6f,%.1f,%.1f,%d,%.2f",
            year, month, day, hour, minute, second,
            gps.location.lat(), gps.location.lng(),
            speed, altitude, satellites, odometerKm);

    logFile.println(buffer);
    logFile.flush();
    logFile.close();

    // Debug-Output für erfolgreiches Logging
    Serial.print("[LOG] Entry ");
    Serial.print(logEntryCount + 1);
    Serial.print(" written to ");
    Serial.println(currentLogFilename);

    lastLogTime = millis();
    lastLoggedLat = gps.location.lat();
    lastLoggedLng = gps.location.lng();
    logEntryCount++;

    if (logEntryCount >= MAX_LOG_ENTRIES) {
        Serial.println("[LOG] Max entries reached, creating new file...");
        createNewLogFile();
    }
}

// ============================================================
// DISPLAY FUNCTIONS
// ============================================================
void drawSDStatus() {
    tft.setTextSize(1);
    tft.setCursor(SD_STATUS_X, SD_STATUS_Y);

    if (sdCardReady) {
        tft.setTextColor(COLOR_GREEN, COLOR_BLACK);
        tft.print("SD:OK ");
    } else {
        tft.setTextColor(COLOR_RED, COLOR_BLACK);
        tft.print("SD:ERR");
    }
    sdStatusNeedsRedraw = false;
}

void drawSatellites() {
    tft.setTextSize(1);
    tft.setTextColor(COLOR_WHITE, COLOR_BLACK);
    tft.setCursor(SAT_X, SAT_Y);
    char satBuffer[8];
    if (gps.satellites.isValid()) {
        snprintf(satBuffer, sizeof(satBuffer), "Sats:%02d", gps.satellites.value());
    } else {
        strcpy(satBuffer, "Sats:--");
    }
    tft.print(satBuffer);
}

void drawTime() {
    char timeBuffer[16];
    getGPSTime(timeBuffer);
    tft.setTextSize(2);
    tft.setTextColor(COLOR_WHITE, COLOR_BLACK);
    tft.setCursor(TIME_X, TIME_Y);
    tft.print(timeBuffer);
}

void getGPSTime(char *buffer) {
    int hour, minute, second;
    if (getBulgarianTime(hour, minute, second)) {
        snprintf(buffer, 16, "%02d:%02d:%02d", hour, minute, second);
    } else {
        strcpy(buffer, "--:--:--");
    }
}

void drawLatitude() {
    tft.setTextSize(1);
    tft.setTextColor(COLOR_WHITE, COLOR_BLACK);
    tft.setCursor(LAT_X, LAT_Y);
    if (gps.location.isValid()) {
        tft.print("LAT ");
        tft.print(gps.location.lat(), 6);
    } else {
        tft.print("LAT --.------");
    }
}

void drawLongitude() {
    tft.setTextSize(1);
    tft.setTextColor(COLOR_WHITE, COLOR_BLACK);
    tft.setCursor(LON_X, LON_Y);
    if (gps.location.isValid()) {
        tft.print("LON ");
        tft.print(gps.location.lng(), 6);
    } else {
        tft.print("LON --.------");
    }
}

void drawSpeed() {
    char speedBuffer[8];
    if (gps.speed.isValid()) {
        int speed = (int)(gps.speed.kmph() + 0.5);
        snprintf(speedBuffer, sizeof(speedBuffer), "%03d", speed);
    } else {
        strcpy(speedBuffer, "---");
    }
    tft.setTextSize(SPEED_TEXT_SIZE);
    tft.setTextColor(COLOR_WHITE, COLOR_BLACK);
    tft.setCursor(SPEED_X, SPEED_Y);
    tft.print(speedBuffer);

    tft.setTextSize(2);
    tft.setTextColor(COLOR_WHITE, COLOR_BLACK);
    tft.setCursor(SPEED_UNIT_X, SPEED_UNIT_Y);
    tft.print("km/h");
}

void drawOdometer() {
    char odometerBuffer[16];
    formatOdometer(odometerKm, odometerBuffer);
    tft.setTextSize(ODO_TEXT_SIZE);
    tft.setTextColor(COLOR_WHITE, COLOR_BLACK);
    tft.setCursor(ODO_X, ODO_Y);
    tft.print(odometerBuffer);

    tft.setTextSize(1);
    tft.setTextColor(COLOR_WHITE, COLOR_BLACK);
    tft.setCursor(ODO_UNIT_X, ODO_UNIT_Y);
    tft.print("km");
}

void formatOdometer(double value, char *buffer) {
    if (value < 0.0) value = 0.0;
    if (value > 999.99) value = 999.99;
    int roundedValue = (int)((value * 100.0) + 0.5);
    snprintf(buffer, 16, "%03d.%02d", roundedValue / 100, roundedValue % 100);
}

// ============================================================
// DRAW STATIC SCREEN
// ============================================================
void drawStaticScreen() {
    tft.fillScreen(COLOR_BLACK);

    // Draw dividers
    tft.drawLine(25, 40, 215, 40, COLOR_WHITE);
    tft.drawLine(25, 155, 215, 155, COLOR_WHITE);

    // Draw all elements
    drawSatellites();
    drawSDStatus();
    drawTime();
    drawLatitude();
    drawLongitude();
    drawSpeed();
    drawOdometer();
}

// ============================================================
// READ GPS
// ============================================================
void readGPS() {
    while (GPSSerial.available()) {
        char c = GPSSerial.read();
        gps.encode(c);
        Serial.write(c);  // Debug: NMEA-Daten an Serial ausgeben
    }
}

// ============================================================
// UPDATE ODOMETER
// ============================================================
void updateGPSOdometer() {
    if (!gps.location.isUpdated() || !gps.location.isValid()) return;

    if (!gps.satellites.isValid() || gps.satellites.value() < 4) {
        havePreviousPosition = false;
        consecutiveJumpRejects = 0;
        return;
    }

    if (!gps.speed.isValid() || gps.speed.kmph() < MIN_SPEED_KMPH) {
        return;
    }

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

    if (distanceMeters >= MIN_VALID_STEP_METERS &&
        distanceMeters <= MAX_VALID_STEP_METERS) {
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
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("======================================");
    Serial.println("XIAO ESP32-C5 + Round Display");
    Serial.println("GPS Speedometer + Odometer + SD Logger");
    Serial.println("TFT_DC on D3, GPS on LP_UART (GPIO19/20)");
    Serial.println("SD_CS on D2 (changed from D4)");
    Serial.println("======================================\n");

    // --- 1. SPI-Bus initialisieren (für Display UND SD-Karte) ---
    Serial.println("Initializing SPI...");
    SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);  // Nur EINMAL aufrufen!
    Serial.println("SPI OK\n");

    // --- 2. Display ---
    Serial.println("Initializing Round Display (DC on D3)...");
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    tft.begin();
    delay(500);
    tft.invertDisplay(true);
    tft.setRotation(3);
    tft.fillScreen(COLOR_BLACK);
    drawStaticScreen();
    Serial.println("Display OK\n");

    // --- 3. SD-Karte ---
    initSDCard();

    // --- 4. GPS (LP_UART auf GPIO19/20) ---
    Serial.println("Initializing GPS (LP_UART on GPIO19/20)...");
    GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("GPS OK\n");

    Serial.println("Waiting for GPS data...\n");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    // Read GPS data
    readGPS();

    // Update odometer
    updateGPSOdometer();

    // Log to SD card
    writeLogEntry();

    // Update display
    if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
        lastDisplayUpdate = millis();
        drawSatellites();
        drawTime();
        drawLatitude();
        drawLongitude();
        drawSpeed();
        drawOdometer();

        if (sdStatusNeedsRedraw) {
            drawSDStatus();
        }
    }
}
