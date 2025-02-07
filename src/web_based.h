/*
NOTES:
SESUAIKAN DAHULU DENGAN SPESIFIKASI RATING MAKSIMUM DARI CHARGER/BATTERY UNTUK SAFETY KESALAHAN INPUT VOLTAGE/CURRENT
VARIABLE DARI MAX_ALLOWED_VOLTAGE MAX_ALLOWED_CURRENT
*/

#include <SPI.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h> // Library for internal (non-volatile) storage

#define CAN_CS 5  // Chip Select pin for CAN
#define CAN_INT 4 // Interrupt pin for CAN

MCP_CAN CAN(CAN_CS); // Create CAN instance

// ----- Shared Preferences Setup -----
Preferences preferences;

// ----- WiFi & Web Server Setup -----
const char *ssid = "CAN_Monitor_AP";
const char *password = "password123";
WebServer server(80);

const float CELL_VOLTAGE = 3.64; // Maximum per-cell voltage
const int NUM_CELLS = 32;        // 32S configuration

// HARUS DISESUAIKAN DENGAN SPESIFIKASI MAKSIMUM DARI CHARGER
const float MAX_ALLOWED_VOLTAGE = NUM_CELLS * CELL_VOLTAGE; // Maximum allowed voltage (115.2V)
const float MAX_ALLOWED_CURRENT = 32;                       // Maximum allowed current

// charging parameters (default values)
float targetVoltage = MAX_ALLOWED_VOLTAGE; // Default target voltage
float targetCurrent = 6;                   // Default target current

// ----- Other Global Variables -----
unsigned long pmillis = 0;             // For timing the CAN command transmissions
bool chargerActive = false;            // Charging flag (true = charging, false = stopped)
String chargingStatusText = "Stopped"; // Charging status text for display

// Global variables to store the latest CAN message details for the web page
String webCANId = "";
String webCANData = "";
float batteryVoltage = 0.0;
float batteryCurrent = 0.0;

// ----- Function Prototypes -----
void handleRoot();
void handleSet();
void handleControl();
void handleData();
void sendChargerCommand();
void stopChargerCommand();

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for Serial to be ready

  // Initialize CAN bus
  if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK)
  {
    Serial.println("CAN init failed");
    while (1)
      ;
  }
  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);
  Serial.println("CAN Ready");

  // Initialize shared preferences and load stored settings.
  // If no stored value exists, the default value is used.
  preferences.begin("settings", false);
  targetVoltage = preferences.getFloat("targetVoltage", targetVoltage);
  targetCurrent = preferences.getFloat("targetCurrent", targetCurrent);
  Serial.print("Loaded target voltage: ");
  Serial.println(targetVoltage);
  Serial.print("Loaded target current: ");
  Serial.println(targetCurrent);

  // Initialize WiFi in Access Point mode
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Set up web server routes
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/control", handleControl);
  server.on("/data", handleData);
  server.begin();
}

void loop()
{
  server.handleClient();

  // Periodically send a charger command if charging is active
  if (millis() - pmillis > 1000)
  {
    pmillis = millis();
    if (chargerActive)
    {
      sendChargerCommand();
    }
  }

  if (Serial.available() > 0)
  {
    char data = Serial.read();
    if (data == 'a')
    {
      // Additional serial commands can be implemented here if needed.
    }
  }

  // Check for incoming CAN messages
  if (digitalRead(CAN_INT) == LOW)
  {
    unsigned long id;
    byte len;
    byte msgData[8];
    if (CAN.readMsgBuf(&id, &len, msgData) == CAN_OK)
    {
      // Update global variables for the web display
      webCANId = "0x" + String(id, HEX);
      webCANData = "";
      for (byte i = 0; i < len; i++)
      {
        if (msgData[i] < 0x10)
        {
          webCANData += "0";
        }
        webCANData += String(msgData[i], HEX) + " ";
      }

      // Print improved CAN message info to Serial
      Serial.print("CAN ID: ");
      Serial.print(webCANId);
      Serial.print(" | CAN Data: ");
      Serial.println(webCANData);

      if (len == 8)
      {
        // Parse voltage from bytes 0-1 (0.1 V/bit resolution)
        uint16_t voltageRaw = (msgData[0] << 8) | msgData[1];
        batteryVoltage = voltageRaw * 0.1f;
        // Parse current from bytes 2-3 (0.1 A/bit resolution)
        uint16_t currentRaw = (msgData[2] << 8) | msgData[3];
        batteryCurrent = currentRaw * 0.1f;
        // Parse charging state from byte 4 and update the status text
        uint8_t state = msgData[4];
        chargingStatusText = (state == 0) ? "Charging" : "Stopped";

        Serial.println("-------------------------------------");
        Serial.print("Voltage: ");
        Serial.println(batteryVoltage);
        Serial.print("Current: ");
        Serial.println(batteryCurrent);
        Serial.print("Status: ");
        Serial.println(chargingStatusText);
        Serial.println("-------------------------------------");
      }
    }
  }
  delay(100);
}

// ----- Web Server Handlers -----

// Root page: displays current CAN and battery data, plus a form to update charging parameters.
// The input fields do not have preset default values (so as not to interfere with user input),
// and their current stored value is shown in an adjacent label.
void handleRoot()
{
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>CAN Monitor</title>";
  // JavaScript to poll the /data endpoint every second and update the displayed values
  html += "<script>";
  html += "function updateData() {";
  html += "  var xhttp = new XMLHttpRequest();";
  html += "  xhttp.onreadystatechange = function() {";
  html += "    if (this.readyState == 4 && this.status == 200) {";
  html += "      var data = JSON.parse(this.responseText);";
  html += "      document.getElementById('chargingStatus').innerHTML = data.chargingStatus;";
  html += "      document.getElementById('canId').innerHTML = data.canId;";
  html += "      document.getElementById('canData').innerHTML = data.canData;";
  html += "      document.getElementById('voltage').innerHTML = data.voltage + ' V';";
  html += "      document.getElementById('current').innerHTML = data.current + ' A';";
  // Update the labels next to the input fields instead of the inputs themselves
  html += "      document.getElementById('targetVoltageLabel').innerHTML = data.targetVoltage + ' V';";
  html += "      document.getElementById('targetCurrentLabel').innerHTML = data.targetCurrent + ' A';";
  html += "    }";
  html += "  };";
  html += "  xhttp.open('GET', '/data', true);";
  html += "  xhttp.send();";
  html += "}";
  html += "setInterval(updateData, 1000);";
  html += "</script>";
  html += "</head><body>";
  html += "<h1>CAN Monitor</h1>";
  html += "<p><strong>Charging Status:</strong> <span id='chargingStatus'>" + chargingStatusText + "</span></p>";
  html += "<p><strong>Maximum Voltage Rating:</strong> <span id='maxAlowedVoltage'>" + String(MAX_ALLOWED_VOLTAGE) + " V</span></p>";
  html += "<p><strong>Maximum Current Rating:</strong> <span id='maxAlowedCurrent'>" + String(MAX_ALLOWED_CURRENT) + " A</span></p>";
  html += "<p><strong>Received CAN ID:</strong> <span id='canId'>" + webCANId + "</span></p>";
  html += "<p><strong>Received CAN Data:</strong> <span id='canData'>" + webCANData + "</span></p>";
  html += "<p><strong>Battery Voltage:</strong> <span id='voltage'>" + String(batteryVoltage, 1) + " V</span></p>";
  html += "<p><strong>Battery Current:</strong> <span id='current'>" + String(batteryCurrent, 1) + " A</span></p>";

  // Form to update charger parameters.
  // Input fields include min/max attributes to enforce safety constraints.
  // The current stored values are displayed in adjacent labels.
  html += "<h2>Update Charger Parameters</h2>";
  html += "<form action='/set' method='GET'>";
  html += "Set Charging Voltage (V): <input type='number' step='0.1' name='v' id='targetVoltage' min='0' max='" + String(MAX_ALLOWED_VOLTAGE) + "'> ";
  html += "<span id='targetVoltageLabel'>" + String(targetVoltage, 1) + " V</span><br>";
  html += "Set Charging Current (A): <input type='number' step='0.1' name='c' id='targetCurrent' min='0' max='" + String(MAX_ALLOWED_CURRENT) + "'> ";
  html += "<span id='targetCurrentLabel'>" + String(targetCurrent, 1) + " A</span><br>";
  html += "<input type='submit' value='Update Parameters'>";
  html += "</form>";

  html += "<h2>Control Charger</h2>";
  html += "<button onclick=\"location.href='/control?cmd=start'\">Start Charging</button> ";
  html += "<button onclick=\"location.href='/control?cmd=stop'\">Stop Charging</button>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Handle form submission to update target voltage and current.
// Values are checked for safety and then stored in shared preferences.
void handleSet()
{
  bool valid = true;
  String errorMsg = "";

  if (server.hasArg("v"))
  {
    float newVoltage = server.arg("v").toFloat();
    if (newVoltage >= 0 && newVoltage <= MAX_ALLOWED_VOLTAGE)
    {
      targetVoltage = newVoltage;
      preferences.putFloat("targetVoltage", targetVoltage);
      Serial.print("Updated target voltage: ");
      Serial.println(targetVoltage);
    }
    else
    {
      valid = false;
      errorMsg += "Voltage out of range. ";
    }
  }
  if (server.hasArg("c"))
  {
    float newCurrent = server.arg("c").toFloat();
    if (newCurrent >= 0 && newCurrent <= MAX_ALLOWED_CURRENT)
    {
      targetCurrent = newCurrent;
      preferences.putFloat("targetCurrent", targetCurrent);
      Serial.print("Updated target current: ");
      Serial.println(targetCurrent);
    }
    else
    {
      valid = false;
      errorMsg += "Current out of range. ";
    }
  }
  if (!valid)
  {
    server.send(400, "text/plain", errorMsg);
    return;
  }

  // Redirect back to the root page so the updated values are immediately visible
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

// Handle control commands (start/stop charging) and redirect back to the root page.
void handleControl()
{
  if (server.hasArg("cmd"))
  {
    String cmd = server.arg("cmd");
    if (cmd == "start")
    {
      chargerActive = true;
      Serial.println("Charger started");
    }
    else if (cmd == "stop")
    {
      chargerActive = false;
      stopChargerCommand();
      Serial.println("Charger stopped");
    }
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

// New endpoint that returns updated CAN and parameter data in JSON format.
void handleData()
{
  String json = "{";
  json += "\"voltage\":" + String(batteryVoltage, 1) + ",";
  json += "\"current\":" + String(batteryCurrent, 1) + ",";
  json += "\"canId\":\"" + webCANId + "\",";
  json += "\"canData\":\"" + webCANData + "\",";
  json += "\"chargingStatus\":\"" + chargingStatusText + "\",";
  json += "\"targetVoltage\":" + String(targetVoltage, 1) + ",";
  json += "\"targetCurrent\":" + String(targetCurrent, 1);
  json += "}";
  server.send(200, "application/json", json);
}

// ----- CAN Command Functions -----

// Send the charger command using the current target voltage/current values.
void sendChargerCommand()
{
  uint16_t voltageVal = targetVoltage * 10;
  byte voltage_high = (voltageVal >> 8) & 0xFF;
  byte voltage_low = voltageVal & 0xFF;

  uint16_t currentVal = targetCurrent * 10;
  byte current_high = (currentVal >> 8) & 0xFF;
  byte current_low = currentVal & 0xFF;

  byte data[8] = {
      voltage_high,
      voltage_low,
      current_high,
      current_low,
      0x00, // Control byte: 0x00 to activate charging
      0x00, // Mode selector (if needed)
      0x00, // Reserved
      0x00  // Reserved
  };
  Serial.print("SEND COMMAND: ");
  Serial.print(targetVoltage, 1);
  Serial.print("V, ");
  Serial.print(targetCurrent, 1);
  Serial.println("A");
  if (CAN.sendMsgBuf(0x1806E5F4, 1, 8, data) == CAN_OK)
  {
    Serial.print("Send command: ");
    Serial.print(targetVoltage, 1);
    Serial.print("V, ");
    Serial.print(targetCurrent, 1);
    Serial.println("A");
  }
  else
  {
    Serial.println("Send failed");
  }
}

// Send a command to stop charging.
void stopChargerCommand()
{
  byte data[8] = {
      0x00,
      0x00,
      0x00,
      0x00,
      0x01, // Control byte: 0x01 indicates stop charging
      0x00,
      0x00,
      0x00};

  if (CAN.sendMsgBuf(0x1806E5F4, 1, 8, data) == CAN_OK)
  {
    Serial.println("Stop command sent, charging stopped");
  }
  else
  {
    Serial.println("Stop command failed");
  }
}
