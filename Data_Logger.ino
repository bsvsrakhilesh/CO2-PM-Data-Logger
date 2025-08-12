/***************************************************************
 *   Includes
 ***************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>         // Core graphics library for the OLED
#include <Adafruit_SSD1306.h>     // OLED display library
#include <RTClib.h>               // Library for DS3231 RTC
#include <sps30.h>                // Library for SPS30 particulate matter sensor
#include <FS.h>
#include <SD.h>

// ========== Wi-Fi and Web Server ==========
#include <WiFi.h>
#include <Preferences.h>          // For storing Wi-Fi credentials in flash memory
#include <WebServer.h>            // Lightweight web server library

// ========== SCD30 Includes ==========
#include <SensirionI2cScd30.h>

/***************************************************************
 *   Pin Definitions and Constants
 ***************************************************************/

#define SCREEN_WIDTH  128         // OLED display width in pixels
#define SCREEN_HEIGHT 64          // OLED display height in pixels

// Define OLED pins (update these based on your wiring)
#define OLED_MOSI   D10
#define OLED_CLK    D8
#define OLED_DC     D2
#define OLED_CS     D3
#define OLED_RESET  D1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);

// ---------------------------------------------------
RTC_DS3231 rtc;

// ---------------------------------------------------
// SD Card Module SPI Config (for XIAO ESP32S3)
#define SD_CS_PIN  D7
#define SD_MOSI    D10
#define SD_MISO    D9
#define SD_SCK     D8

File logFile;  // File object for logging data to SD card

// ---------------------------------------------------
const unsigned long PM_INTERVAL      = 3000UL;
const unsigned long NC_INTERVAL      = 3000UL;
const unsigned long SCD30_INTERVAL   = 3000UL;
const unsigned long IP_INTERVAL      = 3000UL;

enum DisplayState {
  SHOW_PM = 0,
  SHOW_NC,
  SHOW_SCD30,
  SHOW_IP
};

static DisplayState currentDisplay = SHOW_PM;
static unsigned long lastToggleTime = 0;

// SD card logging interval
const unsigned long LOG_INTERVAL = 20000UL;
const unsigned long SPS30_INTERVAL   = 20000UL;
unsigned long lastLogTime = 0;

/*  ---------  run-time-changeable copies (initialised with the same defaults)  --------- */
unsigned long pmInterval          = PM_INTERVAL;      // OLED page: PM values
unsigned long ncInterval          = NC_INTERVAL;      // OLED page: NC values
unsigned long scd30DispInterval   = SCD30_INTERVAL;   // OLED page: SCD30 values
unsigned long ipInterval          = IP_INTERVAL;      // OLED page: Wi-Fi/IP

unsigned long sps30Interval       = SPS30_INTERVAL;   // SPS30 sampling
unsigned long scd30Interval       = 20000UL;          // SCD30 sampling (was hard-wired)
unsigned long logInterval         = LOG_INTERVAL;     // SD logging
/* ------------------------------------------------------------------------------------- */
Preferences intervalPrefs;   // NV storage for user-defined intervals

/***************************************************************
 *   Wi-Fi and Web Server Setup
 ***************************************************************/
Preferences preferences;
WebServer server(80);

// Separate Preferences instance for storing the last active RTC epoch time
Preferences rtcPreferences;

// Variables for Wi-Fi credentials
bool connected      = false;
bool askedForChoice = false;
int  numNetworks    = 0;
String ssidList[50];

// Forward declaration for scanning networks
void scanNetworksAndShowMenu();

// ========== Global measurement structure for the SPS30 sensor data ==========
static struct sps30_measurement measurement;

// ========== SCD30 Global Variables ==========
SensirionI2cScd30 scd30;
static float co2Concentration = 0.0f;
static float temperature      = 0.0f;
static float humidity         = 0.0f;
static int16_t scd30Error     = 0;
static char scd30ErrorMsg[128];

/***************************************************************
 *   Common HTML Helpers
 ***************************************************************/

/**
 * @brief Returns a standard HTML header (with embedded CSS) that you can reuse.
 */
String getHtmlHeader(const String &title) {
  // Minimal styling to keep code size smaller.
  // Add or remove as needed.
  String html = F(
    "<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'/>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'/>"
    "<title>"
  );
  html += title;
  html += F(
    "</title>"
    "<style>"
      "body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }"
      "h1, h2, h3 { color: #333; }"
      "p, a, label, input { color: #555; font-size: 14px; }"
      "a { text-decoration: none; }"
      ".card { background: #fff; border-radius: 5px; box-shadow: 0 0 10px rgba(0,0,0,0.1); margin: 20px 0; padding: 20px; }"
      ".card h2 { margin-top: 0; }"
      ".btn { display: inline-block; padding: 8px 16px; margin: 5px 0; background: #333; color: #fff; border-radius: 4px; text-decoration: none; }"
      ".btn:hover { background: #555; }"
      "table { width: 100%; border-collapse: collapse; margin-top: 10px; }"
      "th, td { text-align: left; padding: 8px; border-bottom: 1px solid #ddd; }"
      "th { background-color: #fafafa; }"
      ".form-group { margin-bottom: 10px; }"
      "input[type='number'] { width: 100px; }"
    "</style>"
    "</head><body>"
    "<h1>"
  );
  html += title;
  html += F("</h1>");
  return html;
}

String getHtmlFooter() {
  return F("</body></html>");
}

/***************************************************************
 *   Web Server Handlers
 ***************************************************************/

// -------------------- Handle Root --------------------
void handleRoot() {
  // Build HTML content for the web page
  String html = getHtmlHeader("SPS30 & SCD30 Readings");

  html += F("<div class='card'>");
  html += F("<h2>SPS30 Data</h2>");
  html += F("<table>"
            "<tr><th>Parameter</th><th>Value</th></tr>");
  html += "<tr><td>PM1.0 (ug/m3)</td><td>"  + String(measurement.mc_1p0, 2) + "</td></tr>";
  html += "<tr><td>PM2.5 (ug/m3)</td><td>"  + String(measurement.mc_2p5, 2) + "</td></tr>";
  html += "<tr><td>PM4.0 (ug/m3)</td><td>"  + String(measurement.mc_4p0, 2) + "</td></tr>";
  html += "<tr><td>PM10 (ug/m3)</td><td>"   + String(measurement.mc_10p0, 2) + "</td></tr>";

#ifndef SPS30_LIMITED_I2C_BUFFER_SIZE
  // Number concentration (NC) measurements
  html += "<tr><td>NC0.5 (particles/cm3)</td><td>"  + String(measurement.nc_0p5, 2) + "</td></tr>";
  html += "<tr><td>NC1.0 (particles/cm3)</td><td>"  + String(measurement.nc_1p0, 2) + "</td></tr>";
  html += "<tr><td>NC2.5 (particles/cm3)</td><td>"  + String(measurement.nc_2p5, 2) + "</td></tr>";
  html += "<tr><td>NC4.0 (particles/cm3)</td><td>"  + String(measurement.nc_4p0, 2) + "</td></tr>";
  html += "<tr><td>NC10.0 (particles/cm3)</td><td>" + String(measurement.nc_10p0, 2) + "</td></tr>";
#else
  html += "<tr><td colspan='2'><em>NC data not supported on this board (limited I2C buffer).</em></td></tr>";
#endif

  html += "<tr><td>Typical Particle Size (µm)</td><td>"
       + String(measurement.typical_particle_size, 2) + "</td></tr>";
  html += F("</table></div>");

  // SCD30 Data
  html += F("<div class='card'>");
  html += F("<h2>SCD30 Data</h2>");
  html += F("<table>"
            "<tr><th>Parameter</th><th>Value</th></tr>");
  html += "<tr><td>CO2 (ppm)</td><td>"         + String(co2Concentration, 2) + "</td></tr>";
  html += "<tr><td>Temperature (°C)</td><td>"  + String(temperature, 2)      + "</td></tr>";
  html += "<tr><td>Humidity (%RH)</td><td>"    + String(humidity, 2)         + "</td></tr>";
  html += F("</table></div>");

  // Wi-Fi
  html += F("<div class='card'>");
  html += F("<h2>Wi-Fi Status</h2>");
  if (connected) {
    html += "<p><strong>Connected!</strong></p>";
    html += "<p>IP Address: " + WiFi.localIP().toString() + "</p>";
  } else {
    html += "<p>Not connected</p>";
  }
  html += F("</div>");

  // Links
  html += F("<div class='card'>");
  html += "<h2>Links</h2>";
  html += "<p><a class='btn' href='/rtc'>Set RTC Time</a></p>";
  html += "<p><a class='btn' href='/download'>Download SD Log File</a></p>";
  html += "<p><a class='btn' href='/frc'>Force Recalibration (FRC)</a></p>";
  html += F("<p><a class='btn' href='/tempOffset'>Set SCD30 Temp Offset</a></p>");
  html += "<p><a class='btn' href='/intervals'>Set Time Intervals</a></p>";
  html += F("</div>");

  html += getHtmlFooter();

  server.send(200, "text/html", html);
}

// -------------------- Handle Not Found --------------------
void handleNotFound() {
  server.send(404, "text/plain", "404: Not found");
}

// -------------------- Handle RTC --------------------
void handleRTC() {
  // If user submitted form with RTC data:
  if (server.hasArg("year") && server.hasArg("month") && server.hasArg("day") &&
      server.hasArg("hour") && server.hasArg("minute") && server.hasArg("second")) {

    int year   = server.arg("year").toInt();
    int month  = server.arg("month").toInt();
    int day    = server.arg("day").toInt();
    int hour   = server.arg("hour").toInt();
    int minute = server.arg("minute").toInt();
    int second = server.arg("second").toInt();

    DateTime dt(year, month, day, hour, minute, second);
    rtc.adjust(dt);

    String html = getHtmlHeader("RTC Updated");
    html += "<div class='card'>";
    html += "<h2>RTC Updated</h2>";
    html += "<p>The RTC has been updated to: ";
    html += String(year) + "-" + String(month) + "-" + String(day) + " ";
    html += String(hour) + ":" + String(minute) + ":" + String(second) + "</p>";
    html += "<p><a class='btn' href='/rtc'>Go back</a></p>";
    html += "<p><a class='btn' href='/'>Go to Main</a></p>";
    html += "</div>";
    html += getHtmlFooter();
    server.send(200, "text/html", html);
  } else {
    // Display a form to set RTC
    String html = getHtmlHeader("Set RTC Time");
    html += F("<div class='card'>");
    html += F("<h2>Set RTC Time</h2>");
    html += F("<form action='/rtc' method='get'>");
    
    html += F("<div class='form-group'>");
    html += F("<label>Year: </label>");
    html += F("<input type='number' name='year' required>");
    html += F("</div>");

    html += F("<div class='form-group'>");
    html += F("<label>Month: </label>");
    html += F("<input type='number' name='month' required>");
    html += F("</div>");

    html += F("<div class='form-group'>");
    html += F("<label>Day: </label>");
    html += F("<input type='number' name='day' required>");
    html += F("</div>");

    html += F("<div class='form-group'>");
    html += F("<label>Hour: </label>");
    html += F("<input type='number' name='hour' required>");
    html += F("</div>");

    html += F("<div class='form-group'>");
    html += F("<label>Minute: </label>");
    html += F("<input type='number' name='minute' required>");
    html += F("</div>");

    html += F("<div class='form-group'>");
    html += F("<label>Second: </label>");
    html += F("<input type='number' name='second' required>");
    html += F("</div>");

    html += F("<input class='btn' type='submit' value='Update RTC'>");
    html += F("</form>");
    html += F("</div>");
    html += getHtmlFooter();
    server.send(200, "text/html", html);
  }
}

// -------------------- Handle Download --------------------
void handleDownload() {
  File file = SD.open("/PM&CO2_data.csv", FILE_READ);
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }

  // This sends the file directly as a download.
  server.sendHeader("Content-Disposition", "attachment; filename=PM&CO2_data.csv");
  server.sendHeader("Content-Type", "text/csv");
  server.streamFile(file, "text/csv");
  file.close();
}

// -------------------- Handle Force Recalibration --------------------
void handleFRC() {
  String html = getHtmlHeader("Force Recalibration (FRC)");

  html += F("<div class='card'>");
  html += F("<h2>Force Recalibration (FRC)</h2>");

  // 1) Show the current Force Recalibration status:
  uint16_t frcValue = 0;
  int16_t err = scd30.getForceRecalibrationStatus(frcValue);
  if (err == 0) {
    html += "<p>Current forced calibration offset: " + String(frcValue) + " ppm</p>";
  } else {
    // If there's an error, populate the scd30ErrorMsg
    errorToString(err, scd30ErrorMsg, sizeof(scd30ErrorMsg));
    html += "<p><strong>Could not read FRC status:</strong> ";
    html += scd30ErrorMsg;
    html += "</p>";
  }

  // 2) Check if user submitted a new reference
  if (server.hasArg("ref")) {
    int ref = server.arg("ref").toInt();
    html += "<hr><h3>Applying FRC with reference ppm = " + String(ref) + "</h3>";

    // Attempt forced recalibration
    scd30Error = scd30.forceRecalibration(ref);
    if (scd30Error == 0) {
      html += "<p>Force Recalibration successful.</p>";
      // Re-read the new status
      err = scd30.getForceRecalibrationStatus(frcValue);
      if (err == 0) {
        html += "<p>New forced calibration offset: " + String(frcValue) + " ppm</p>";
      } else {
        errorToString(err, scd30ErrorMsg, sizeof(scd30ErrorMsg));
        html += "<p><strong>Could not read updated FRC status:</strong> ";
        html += scd30ErrorMsg;
        html += "</p>";
      }
    } else {
      errorToString(scd30Error, scd30ErrorMsg, sizeof(scd30ErrorMsg));
      html += "<p><strong>Force Recalibration failed:</strong> ";
      html += scd30ErrorMsg;
      html += "</p>";
    }
  } else {
    // Provide a form to submit a new reference
    html += "<hr>";
    html += F("<form method='get' action='/frc'>");
    html += F("<label for='ref'>Enter new CO2 reference (ppm):</label><br>");
    html += F("<input type='number' id='ref' name='ref' min='300' max='10000' required>");
    html += F("<br><br><input class='btn' type='submit' value='Force Recalibrate'>");
    html += F("</form>");
  }

  html += F("</div>");
  html += F("<p><a class='btn' href='/'>Back to Main</a></p>");
  html += getHtmlFooter();
  server.send(200, "text/html", html);
}

// -------------------- Handle Temperature Offset --------------------
void handleTempOffset() {
  String html = getHtmlHeader("SCD30 Temp Offset");

  // 1) Show current offset
  uint16_t currentOffset = 0;
  int16_t err = scd30.getTemperatureOffset(currentOffset);
  html += F("<div class='card'><h2>Current Temp Offset</h2>");
  if (err == NO_ERROR) {
    html += "<p>" + String(currentOffset) + " &#8451;</p>";
  } else {
    html += "<p><strong>Error reading:</strong> Code " + String(err) + "</p>";
  }
  html += F("</div>");

  // 2) If user submitted a new offset
  if (server.hasArg("offset")) {
    uint16_t newOffset = server.arg("offset").toInt();
    err = scd30.setTemperatureOffset(newOffset);
    html += F("<div class='card'><h2>Apply Offset</h2>");
    if (err == NO_ERROR) {
      html += "<p>Set to " + String(newOffset) + " &#8451; successfully.</p>";
    } else {
      html += "<p><strong>Failed:</strong> Code " + String(err) + "</p>";
    }
    html += F("</div>");
  }

  // 3) Form to enter new offset
  html += F(
    "<div class='card'><h2>Enter New Offset (°C)</h2>"
    "<form method='get' action='/tempOffset'>"
      "<div class='form-group'>"
        "<input type='number' step='0.01' name='offset' required>"
      "</div>"
      "<input class='btn' type='submit' value='Set Offset'>"
    "</form></div>"
  );

  html += F("<p><a class='btn' href='/'>Back to Main</a></p>");
  html += getHtmlFooter();
  server.send(200, "text/html", html);
}

void saveIntervals() {
  intervalPrefs.putULong("pmInt",        pmInterval);
  intervalPrefs.putULong("ncInt",        ncInterval);
  intervalPrefs.putULong("scdDispInt",   scd30DispInterval);
  intervalPrefs.putULong("ipInt",        ipInterval);

  intervalPrefs.putULong("sps30Int",     sps30Interval);
  intervalPrefs.putULong("scd30Int",     scd30Interval);
  intervalPrefs.putULong("logInt",       logInterval);
}

void handleIntervals() {
  /* ----------  if the form sent values, store them  ---------- */
  if (server.hasArg("pm") && server.hasArg("nc")) {
    // Constrain to sane limits (0.5 s … 2 min for OLED, 2 s … 10 min for sensors)
    pmInterval        = constrain(server.arg("pm").toInt(),        500,   120000);
    ncInterval        = constrain(server.arg("nc").toInt(),        500,   120000);
    scd30DispInterval = constrain(server.arg("scdDisp").toInt(),   500,   120000);
    ipInterval        = constrain(server.arg("ip").toInt(),        500,   120000);

    sps30Interval     = constrain(server.arg("sps30").toInt(),     2000,  600000);
    scd30Interval     = constrain(server.arg("scd30").toInt(),     2000,  600000);
    logInterval       = constrain(server.arg("log").toInt(),       2000,  600000);
    saveIntervals();     // commit to flash
  }

  /* ----------------  build HTML page  ---------------- */
  String html = getHtmlHeader("Configure Time Intervals");
  html += F("<div class='card'><h2>OLED Page Durations (ms)</h2>");
  html += "<form action='/intervals' method='get'>";
  html += "<label>PM&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<input type='number' name='pm' value='"        + String(pmInterval)        + "'></label><br>";
  html += "<label>NC&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<input type='number' name='nc' value='"        + String(ncInterval)        + "'></label><br>";
  html += "<label>SCD30 Page <input type='number' name='scdDisp' value='"  + String(scd30DispInterval) + "'></label><br>";
  html += "<label>IP Page&nbsp;<input type='number' name='ip' value='"     + String(ipInterval)        + "'></label></div>";

  html += F("<div class='card'><h2>Sensor & Log Intervals (ms)</h2>");
  html += "<label>SPS30 Read <input type='number' name='sps30' value='" + String(sps30Interval) + "'></label><br>";
  html += "<label>SCD30 Read <input type='number' name='scd30' value='" + String(scd30Interval) + "'></label><br>";
  html += "<label>SD Log&nbsp;&nbsp;&nbsp;&nbsp;<input type='number' name='log' value='"   + String(logInterval)   + "'></label><br></div>";

  html += "<input class='btn' type='submit' value='Save & Apply'>";
  html += "</form><p><a class='btn' href='/'>Back to Main</a></p>";
  html += getHtmlFooter();
  server.send(200, "text/html", html);
}


/***************************************************************
 *   Setup
 ***************************************************************/
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Starting...");

  // ========== OLED Initialization ==========
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED initialization failed!");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED Ready");
  display.display();
  delay(1000);

  // ========== RTC Initialization ==========
  if (!rtc.begin()) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("RTC not found!");
    display.display();
    while (1);
  }
  rtcPreferences.begin("rtcTime", false);
  if (rtc.lostPower()) {
    unsigned long lastActive = rtcPreferences.getULong("lastTime", 0);
    if (lastActive > 0) {
      DateTime dt(lastActive);
      rtc.adjust(dt);
      Serial.println("RTC lost power, restored last active time from preferences.");
    } else {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println("RTC lost power, no last active time stored; using compile time.");
    }
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("RTC Ready");
  display.display();
  delay(1000);

  /* ------------  load stored intervals  ------------ */
intervalPrefs.begin("intervals", false);
pmInterval        = intervalPrefs.getULong("pmInt",        pmInterval);
ncInterval        = intervalPrefs.getULong("ncInt",        ncInterval);
scd30DispInterval = intervalPrefs.getULong("scdDispInt",   scd30DispInterval);
ipInterval        = intervalPrefs.getULong("ipInt",        ipInterval);

sps30Interval     = intervalPrefs.getULong("sps30Int",     sps30Interval);
scd30Interval     = intervalPrefs.getULong("scd30Int",     scd30Interval);
logInterval       = intervalPrefs.getULong("logInt",       logInterval);
/* -------------------------------------------------- */


  // ========== SPS30 Initialization ==========
  sensirion_i2c_init(); 
  while (sps30_probe() != 0) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SPS30 Not Found");
    display.display();
    delay(1000);
  }
  sps30_set_fan_auto_cleaning_interval_days(1);
  if (sps30_start_measurement() < 0) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SPS30 meas err!");
    display.display();
    while (1);
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("SPS30 Ready");
  display.display();
  delay(1500);

  // ========== SCD30 Initialization ==========
  Wire.begin();
  scd30.begin(Wire, SCD30_I2C_ADDR_61);
  scd30.stopPeriodicMeasurement();
  scd30.softReset();
  delay(2000);

  // (Optional) Read firmware version for debug
  uint8_t major = 0, minor = 0;
  scd30Error = scd30.readFirmwareVersion(major, minor);
  if (scd30Error != NO_ERROR) {
    Serial.print("Error in readFirmwareVersion(): ");
    errorToString(scd30Error, scd30ErrorMsg, sizeof(scd30ErrorMsg));
    Serial.println(scd30ErrorMsg);
  } else {
    Serial.print("SCD30 FW ver: ");
    Serial.print(major);
    Serial.print(".");
    Serial.println(minor);
  }

  scd30Error = scd30.startPeriodicMeasurement(0); // ambient pressure = 0 => disabled
  if (scd30Error != NO_ERROR) {
    Serial.print("Error in startPeriodicMeasurement(): ");
    errorToString(scd30Error, scd30ErrorMsg, sizeof(scd30ErrorMsg));
    Serial.println(scd30ErrorMsg);
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("SCD30 Ready");
  display.display();
  delay(1500);

  // ========== SD Card Initialization ==========
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN)) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SD Card Failed");
    display.display();
    while (1);
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("SD Ready");
  display.display();
  delay(1500);

  // Prepare CSV file on SD
  logFile = SD.open("/PM&CO2_data.csv", FILE_APPEND);
  if (logFile) {
    if (logFile.size() == 0) {
      logFile.println("Date,Time,MC1.0,MC2.5,MC4.0,MC10.0,NC0.5,NC1.0,NC2.5,NC4.0,NC10.0,ParticleSize,CO2,Temp,Humidity");
    }
    logFile.close();
  } else {
    Serial.println("Failed to open PM&CO2_data.csv in setup!");
  }

  /***************************************************************
   *   Wi-Fi Setup
   ***************************************************************/
  WiFi.mode(WIFI_STA);
  preferences.begin("wifiCreds", false);
  String storedSSID = preferences.getString("ssid", "");
  String storedPass = preferences.getString("password", "");

  if (storedSSID.length() > 0 && storedPass.length() > 0) {
    Serial.printf("Found stored credentials (SSID: %s). Connecting...\n", storedSSID.c_str());
    WiFi.begin(storedSSID.c_str(), storedPass.c_str());
    unsigned long startAttempt = millis();
    unsigned long timeout      = 10000;  // 10-second timeout
    while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt) < timeout) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      Serial.print("Connected! IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("Failed to connect with stored credentials, will ask user for input.");
      connected      = false;
      askedForChoice = false;
    }
  } else {
    Serial.println("No stored credentials found.");
    connected      = false;
    askedForChoice = false;
  }

  if (connected) {
    server.on("/", handleRoot);
    server.on("/rtc", handleRTC);
    server.on("/download", handleDownload);
    server.on("/frc", handleFRC);
    server.on("/tempOffset", handleTempOffset);
    server.on("/intervals", handleIntervals);


    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("Web server started at port 80.");
  }

  // Final OLED message
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Setup Done");
  if (connected) {
    display.println("WiFi: Connected");
  } else {
    display.println("WiFi: Not connected");
  }
  display.display();
  currentDisplay = SHOW_PM;
  lastToggleTime = millis();
}

/***************************************************************
 *   Loop
 ***************************************************************/
void loop() {
  // If not connected and user hasn't been prompted yet, scan for Wi-Fi networks
  if (!connected && !askedForChoice) {
    scanNetworksAndShowMenu();
    askedForChoice = true;
  }

  // Check Serial input for user response (network selection)
  if (Serial.available()) {
    String userInput = Serial.readStringUntil('\n');
    userInput.trim();
    if (!connected) {
      int choice = userInput.toInt();
      if (choice < 1 || choice > numNetworks) {
        Serial.println("Invalid choice. Try again.");
        askedForChoice = false;
      } else {
        // User selected a network
        String chosenSSID = ssidList[choice - 1];
        Serial.print("You selected: ");
        Serial.println(chosenSSID);
        Serial.print("Enter password for ");
        Serial.print(chosenSSID);
        Serial.println(": ");
        while (!Serial.available()) {
          delay(10);
        }
        String password = Serial.readStringUntil('\n');
        password.trim();
        if (password.length() == 0) {
          Serial.println("Password empty. Aborting.");
          askedForChoice = false;
        } else {
          Serial.print("Connecting to ");
          Serial.print(chosenSSID);
          Serial.println(" ...");
          WiFi.begin(chosenSSID.c_str(), password.c_str());
          unsigned long startAttempt = millis();
          unsigned long timeout      = 10000;
          while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt) < timeout) {
            delay(250);
            Serial.print(".");
          }
          Serial.println();
          if (WiFi.status() == WL_CONNECTED) {
            connected = true;
            Serial.println("WiFi connected!");
            Serial.print("IP Address: ");
            Serial.println(WiFi.localIP());
            // Store credentials
            preferences.putString("ssid", chosenSSID);
            preferences.putString("password", password);
            Serial.println("Credentials stored.");

            server.on("/", handleRoot);
            server.on("/rtc", handleRTC);
            server.on("/download", handleDownload);
            server.on("/frc", handleFRC);

            server.onNotFound(handleNotFound);
            server.begin();
            Serial.println("Web server started on port 80.");
          } else {
            Serial.println("Failed to connect.");
            askedForChoice = false;
          }
        }
      }
    }
  }

  // Handle web server if connected
  if (connected) {
    server.handleClient();
  }

  // ========== Read SPS30 Sensor ==========
  static unsigned long lastSps30Read = 0;
  if (millis() - lastSps30Read >= sps30Interval) {
    lastSps30Read = millis();

    uint16_t data_ready = 0;
    int16_t ret = sps30_read_data_ready(&data_ready);
    if (ret < 0) {
      Serial.println("SPS30 data‑ready err: " + String(ret));
    } else if (data_ready) {
      ret = sps30_read_measurement(&measurement);
      if (ret < 0) {
        Serial.println("SPS30 read_measurement err: " + String(ret));
      }
    }
  }


  // ========== Read SCD30 Sensor ==========
  // For SCD30, you typically read every 20 seconds 
  static unsigned long scd30LastRead = 0;
  if (millis() - scd30LastRead > scd30Interval) {
    scd30LastRead = millis();
    scd30Error = scd30.blockingReadMeasurementData(co2Concentration,
                                                   temperature,
                                                   humidity);
    if (scd30Error != NO_ERROR) {
      Serial.print("Error in blockingReadMeasurementData(): ");
      errorToString(scd30Error, scd30ErrorMsg, sizeof(scd30ErrorMsg));
      Serial.println(scd30ErrorMsg);
    }
  }

  // ========== Handle Display State Machine ==========
  unsigned long ms = millis();
  unsigned long elapsed = ms - lastToggleTime;
  switch (currentDisplay) {
    case SHOW_PM:
      if (elapsed >= pmInterval) {
        currentDisplay = SHOW_NC;
        lastToggleTime = ms;
      }
      break;
    case SHOW_NC:
      if (elapsed >= ncInterval) {
        currentDisplay = SHOW_SCD30;
        lastToggleTime = ms;
      }
      break;
    case SHOW_SCD30:
      if (elapsed >= scd30DispInterval) {
        currentDisplay = SHOW_IP;
        lastToggleTime = ms;
      }
      break;
    case SHOW_IP:
      if (elapsed >= ipInterval) {
        currentDisplay = SHOW_PM;
        lastToggleTime = ms;
      }
      break;
  }

  // Get current date/time from RTC
  DateTime now = rtc.now();
  char dateBuffer[11];
  char timeBuffer[9];
  snprintf(dateBuffer, sizeof(dateBuffer), "%02d-%02d-%04d",
           now.day(), now.month(), now.year());
  snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d",
           now.hour(), now.minute(), now.second());

  // ========== Update OLED ==========
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print(dateBuffer);
  display.print(" ");
  display.println(timeBuffer);

  switch (currentDisplay) {
    case SHOW_PM:
      display.setCursor(0, 16);
      display.println("PM Values [ug/m3]");
      display.print("PM1.0 : ");
      display.println(String(measurement.mc_1p0, 2));
      display.print("PM2.5 : ");
      display.println(String(measurement.mc_2p5, 2));
      display.print("PM4.0 : ");
      display.println(String(measurement.mc_4p0, 2));
      display.print("PM10.0: ");
      display.println(String(measurement.mc_10p0, 2));
      break;

    case SHOW_NC:
#ifndef SPS30_LIMITED_I2C_BUFFER_SIZE
      display.setCursor(0, 16);
      display.println("NC [particles/cm3]");
      display.print("NC0.5 : ");
      display.println(String(measurement.nc_0p5, 2));
      display.print("NC1.0 : ");
      display.println(String(measurement.nc_1p0, 2));
      display.print("NC2.5 : ");
      display.println(String(measurement.nc_2p5, 2));
      display.print("NC4.0 : ");
      display.println(String(measurement.nc_4p0, 2));
      display.print("NC10.0: ");
      display.println(String(measurement.nc_10p0, 2));
#else
      display.setCursor(0, 16);
      display.println("NC not supported");
#endif
      break;

    case SHOW_SCD30:
      display.setCursor(0, 16);
      display.println("SCD30 Values");
      display.print("CO2: ");
      display.println(co2Concentration);
      display.print("Temp: ");
      display.println(temperature);
      display.print("RH: ");
      display.println(humidity);
      break;

    case SHOW_IP:
      display.setCursor(0, 16);
      if (connected) {
        display.println("Wi-Fi: Connected");
        display.print("IP: ");
        display.println(WiFi.localIP().toString());
      } else {
        display.println("Wi-Fi: NOT connected");
      }
      break;
  }
  display.display();

  // ========== Log Data to SD ==========
  if (ms - lastLogTime >= logInterval) {
    lastLogTime = ms;
    File f = SD.open("/PM&CO2_data.csv", FILE_APPEND);
    if (f) {
      f.printf(
        "%s,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
        dateBuffer,
        timeBuffer,
        measurement.mc_1p0,
        measurement.mc_2p5,
        measurement.mc_4p0,
        measurement.mc_10p0,
        measurement.nc_0p5,
        measurement.nc_1p0,
        measurement.nc_2p5,
        measurement.nc_4p0,
        measurement.nc_10p0,
        measurement.typical_particle_size,
        co2Concentration,
        temperature,
        humidity
      );
      f.close();
    } else {
      Serial.println("Failed to open PM&CO2_data.csv in loop for appending!");
    }
    // Store the last RTC time in Preferences
    rtcPreferences.putULong("lastTime", now.unixtime());
  }

  delay(100);
}

/***************************************************************
 *   Wi-Fi Scan Function
 ***************************************************************/
void scanNetworksAndShowMenu() {
  Serial.println("\nScanning for WiFi networks...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  numNetworks = WiFi.scanNetworks();
  if (numNetworks == 0) {
    Serial.println("No networks found. Will retry later.");
    delay(2000);
    return;
  }
  Serial.print(numNetworks);
  Serial.println(" networks found:");
  Serial.println(" Nr | SSID                             | RSSI | CH | Encryption");
  for (int i = 0; i < numNetworks; i++) {
    ssidList[i] = WiFi.SSID(i);
    Serial.printf("%2d", i + 1);
    Serial.print(" | ");
    Serial.printf("%-32.32s", WiFi.SSID(i).c_str());
    Serial.print(" | ");
    Serial.printf("%4d", WiFi.RSSI(i));
    Serial.print(" | ");
    Serial.printf("%2d", WiFi.channel(i));
    Serial.print(" | ");
    switch (WiFi.encryptionType(i)) {
      case WIFI_AUTH_OPEN:            Serial.print("open");       break;
      case WIFI_AUTH_WEP:             Serial.print("WEP");        break;
      case WIFI_AUTH_WPA_PSK:         Serial.print("WPA");        break;
      case WIFI_AUTH_WPA2_PSK:        Serial.print("WPA2");       break;
      case WIFI_AUTH_WPA_WPA2_PSK:    Serial.print("WPA+WPA2");   break;
      case WIFI_AUTH_WPA2_ENTERPRISE: Serial.print("WPA2-EAP");   break;
      case WIFI_AUTH_WPA3_PSK:        Serial.print("WPA3");       break;
      case WIFI_AUTH_WPA2_WPA3_PSK:   Serial.print("WPA2+WPA3");  break;
      case WIFI_AUTH_WAPI_PSK:        Serial.print("WAPI");       break;
      default:                        Serial.print("unknown");    break;
    }
    Serial.println();
    delay(10);
  }
  Serial.println();
  Serial.println("Enter the number (1 - " + String(numNetworks) + ") of the network you want to connect to:");
  WiFi.scanDelete();
}
