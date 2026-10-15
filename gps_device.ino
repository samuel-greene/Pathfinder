#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

// Hiking GPS telemetry display for an ESP32 + UART GPS + SSD1306 OLED.
// The loop intentionally never blocks so serial GPS ingestion is prioritized.

namespace Config {
constexpr uint8_t kScreenWidth = 128;
constexpr uint8_t kScreenHeight = 64;
constexpr uint8_t kOledAddress = 0x3C;
constexpr int8_t kOledReset = -1;
constexpr int kGpsRxPin = D7;
constexpr int kGpsTxPin = D6;
constexpr uint32_t kGpsBaud = 9600;

constexpr uint32_t kDisplayIntervalMs = 250;  // 4 Hz UI refresh
constexpr uint32_t kPageIntervalMs = 4000;
constexpr uint32_t kDiagnosticsIntervalMs = 2000;
constexpr uint32_t kTrackSampleIntervalMs = 2000;
constexpr uint32_t kLiveFixMaxAgeMs = 5000;
constexpr double kMaxPlausibleStepMeters = 100.0;

// Set true and replace coordinates before a hike to enable guidance.
constexpr bool kWaypointEnabled = false;
constexpr double kWaypointLatitude = 0.0;
constexpr double kWaypointLongitude = 0.0;
constexpr char kWaypointName[] = "TRAILHEAD";
}  // namespace Config

Adafruit_SSD1306 display(Config::kScreenWidth, Config::kScreenHeight, &Wire,
                         Config::kOledReset);
HardwareSerial gpsSerial(UART_NUM_1);
TinyGPSPlus gps;

enum class GpsState : uint8_t { kNoData, kSearching, kStale, kFixed };
enum class Page : uint8_t { kNavigate, kTelemetry, kHikeStats, kCount };

struct Scheduler {
  uint32_t lastDisplayMs = 0;
  uint32_t lastPageMs = 0;
  uint32_t lastDiagnosticsMs = 0;
  uint32_t lastTrackSampleMs = 0;
};

struct HikeStats {
  bool hasPreviousPoint = false;
  double previousLatitude = 0.0;
  double previousLongitude = 0.0;
  double distanceMeters = 0.0;
  uint32_t acceptedSamples = 0;
  uint32_t rejectedSamples = 0;
};

Scheduler scheduler;
HikeStats hike;
Page activePage = Page::kNavigate;

bool elapsed(uint32_t now, uint32_t &last, uint32_t interval) {
  if (now - last < interval) return false;
  last = now;
  return true;
}

GpsState gpsState() {
  if (gps.charsProcessed() < 10) return GpsState::kNoData;
  if (!gps.location.isValid()) return GpsState::kSearching;
  if (gps.location.age() > Config::kLiveFixMaxAgeMs) return GpsState::kStale;
  return GpsState::kFixed;
}

bool hasLiveFix() { return gpsState() == GpsState::kFixed; }

const char *gpsStateLabel(GpsState state) {
  switch (state) {
    case GpsState::kNoData: return "NO GPS";
    case GpsState::kSearching: return "SEARCH";
    case GpsState::kStale: return "STALE";
    case GpsState::kFixed: return "FIX OK";
  }
  return "UNKNOWN";
}

const char *cardinalDirection(double degrees) {
  static const char *kDirections[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  const uint8_t index = static_cast<uint8_t>((degrees + 22.5) / 45.0) % 8;
  return kDirections[index];
}

void printText(const char text[], int16_t x, int16_t y, uint8_t size = 1) {
  display.setTextSize(size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.print(text);
}

void drawFrame() {
  display.drawFastHLine(0, 0, 128, SSD1306_WHITE);
  display.drawFastHLine(0, 15, 128, SSD1306_WHITE);
  display.drawRect(0, 16, 128, 48, SSD1306_WHITE);
}

void drawHeader() {
  char freshness[12];
  if (gps.location.isValid()) {
    const uint32_t ageSeconds = gps.location.age() / 1000;
    snprintf(freshness, sizeof(freshness), ageSeconds > 99 ? ">99s" : "%lus", ageSeconds);
  } else {
    snprintf(freshness, sizeof(freshness), "WAIT");
  }
  printText(freshness, 4, 4);
  printText(gpsStateLabel(gpsState()), 85, 4);
}

void drawPageDots() {
  const uint8_t pageIndex = static_cast<uint8_t>(activePage);
  for (uint8_t i = 0; i < static_cast<uint8_t>(Page::kCount); ++i) {
    const int16_t x = 104 + (i * 8);
    if (i == pageIndex) display.fillCircle(x, 58, 2, SSD1306_WHITE);
    else display.drawCircle(x, 58, 2, SSD1306_WHITE);
  }
}

void drawAcquiringScreen() {
  char satellites[20];
  snprintf(satellites, sizeof(satellites), "SATS: %d", gps.satellites.value());
  printText("ACQUIRING GPS", 13, 25);
  printText(satellites, 13, 40);
  printText(gps.charsProcessed() < 10 ? "CHECK UART / BAUD" : "WAITING FOR FIX", 13, 51);
}

void drawNavigationPage() {
  if (!hasLiveFix()) {
    drawAcquiringScreen();
    return;
  }

  char line[24];
  if (!Config::kWaypointEnabled) {
    snprintf(line, sizeof(line), "LAT %+.4f", gps.location.lat());
    printText(line, 5, 23);
    snprintf(line, sizeof(line), "LON %+.4f", gps.location.lng());
    printText(line, 5, 36);
    printText("SET WAYPOINT IN CODE", 5, 49);
    return;
  }

  const double distance = TinyGPSPlus::distanceBetween(
      gps.location.lat(), gps.location.lng(), Config::kWaypointLatitude,
      Config::kWaypointLongitude);
  const double bearing = TinyGPSPlus::courseTo(
      gps.location.lat(), gps.location.lng(), Config::kWaypointLatitude,
      Config::kWaypointLongitude);
  snprintf(line, sizeof(line), "%s", Config::kWaypointName);
  printText(line, 5, 23);
  snprintf(line, sizeof(line), "%.2f km %s", distance / 1000.0, cardinalDirection(bearing));
  printText(line, 5, 36);
  snprintf(line, sizeof(line), "BRG %03.0f deg", bearing);
  printText(line, 5, 49);
}

void drawTelemetryPage() {
  char line[24];
  if (gps.speed.isValid()) snprintf(line, sizeof(line), "SPEED %4.1f km/h", gps.speed.kmph());
  else snprintf(line, sizeof(line), "SPEED --.- km/h");
  printText(line, 5, 23);

  if (gps.altitude.isValid()) snprintf(line, sizeof(line), "ALT %5.0f m", gps.altitude.meters());
  else snprintf(line, sizeof(line), "ALT ----- m");
  printText(line, 5, 36);

  if (gps.hdop.isValid()) snprintf(line, sizeof(line), "SATS %02d HDOP %.1f", gps.satellites.value(), gps.hdop.hdop());
  else snprintf(line, sizeof(line), "SATS %02d HDOP --", gps.satellites.value());
  printText(line, 5, 49);
}

void drawHikeStatsPage(uint32_t now) {
  char line[24];
  snprintf(line, sizeof(line), "TRACK %5.2f km", hike.distanceMeters / 1000.0);
  printText(line, 5, 23);
  snprintf(line, sizeof(line), "UPTIME %02lu:%02lu", (now / 3600000UL) % 100,
           (now / 60000UL) % 60);
  printText(line, 5, 36);
  snprintf(line, sizeof(line), "PTS %lu  DROP %lu", hike.acceptedSamples, hike.rejectedSamples);
  printText(line, 5, 49);
}

void renderDisplay(uint32_t now) {
  display.clearDisplay();
  drawFrame();
  drawHeader();
  switch (activePage) {
    case Page::kNavigate: drawNavigationPage(); break;
    case Page::kTelemetry: drawTelemetryPage(); break;
    case Page::kHikeStats: drawHikeStatsPage(now); break;
    case Page::kCount: break;
  }
  drawPageDots();
  display.display();
}

void updateTrackDistance(uint32_t now) {
  if (!hasLiveFix() || !elapsed(now, scheduler.lastTrackSampleMs,
                                Config::kTrackSampleIntervalMs)) return;

  const double latitude = gps.location.lat();
  const double longitude = gps.location.lng();
  if (!hike.hasPreviousPoint) {
    hike = {true, latitude, longitude, 0.0, 1, 0};
    return;
  }

  const double stepMeters = TinyGPSPlus::distanceBetween(
      hike.previousLatitude, hike.previousLongitude, latitude, longitude);
  // Reject discontinuities caused by poor GPS fixes rather than inflating distance.
  if (stepMeters <= Config::kMaxPlausibleStepMeters) {
    hike.distanceMeters += stepMeters;
    hike.previousLatitude = latitude;
    hike.previousLongitude = longitude;
    ++hike.acceptedSamples;
  } else {
    ++hike.rejectedSamples;
  }
}

void printDiagnostics() {
  Serial.println("--- HIKING GPS DIAGNOSTICS ---");
  Serial.print("State: "); Serial.println(gpsStateLabel(gpsState()));
  Serial.print("NMEA chars: "); Serial.println(gps.charsProcessed());
  Serial.print("Satellites: "); Serial.println(gps.satellites.value());
  Serial.print("HDOP: "); Serial.println(gps.hdop.isValid() ? gps.hdop.hdop() : -1.0);
  Serial.print("Track meters: "); Serial.println(hike.distanceMeters, 1);
  Serial.print("GPS failed checksum: "); Serial.println(gps.failedChecksum());
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(Config::kGpsBaud, SERIAL_8N1, Config::kGpsRxPin, Config::kGpsTxPin);

  if (!display.begin(SSD1306_SWITCHCAPVCC, Config::kOledAddress)) {
    Serial.println("SSD1306 allocation failed");
    for (;;) delay(1000);
  }
  display.clearDisplay();
  printText("HIKER GPS", 28, 24, 2);
  printText("INITIALIZING", 22, 48);
  display.display();
}

void loop() {
  // Consume every available NMEA byte before doing scheduled work.
  while (gpsSerial.available() > 0) gps.encode(static_cast<char>(gpsSerial.read()));

  const uint32_t now = millis();
  updateTrackDistance(now);
  if (elapsed(now, scheduler.lastPageMs, Config::kPageIntervalMs)) {
    activePage = static_cast<Page>((static_cast<uint8_t>(activePage) + 1) %
                                   static_cast<uint8_t>(Page::kCount));
  }
  if (elapsed(now, scheduler.lastDiagnosticsMs, Config::kDiagnosticsIntervalMs)) {
    printDiagnostics();
  }
  if (elapsed(now, scheduler.lastDisplayMs, Config::kDisplayIntervalMs)) {
    renderDisplay(now);
  }
}
