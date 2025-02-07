#include <SPI.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <WebServer.h>

// ----- CAN Setup -----
#define CAN_CS 5  // Chip Select pin for CAN
#define CAN_INT 4 // Interrupt pin for CAN

MCP_CAN CAN(CAN_CS); // Create CAN instance

// ----- WiFi & Web Server Setup -----
const char *ssid = "CAN_Monitor_AP";
const char *password = "password123";
WebServer server(80);

// ----- Battery/Charger Parameters -----
// (For a 32-cell LiFePO4 pack)
const float CELL_VOLTAGE = 3.6;                             // Maximum per-cell voltage
const int NUM_CELLS = 32;                                   // 32S configuration
const float MAX_ALLOWED_VOLTAGE = NUM_CELLS * CELL_VOLTAGE; // Maximum allowed voltage (115.2V)
const float MAX_ALLOWED_CURRENT = 3.2;                      // Maximum allowed current

// User target parameters (can be updated via the web form)
float targetVoltage = MAX_ALLOWED_VOLTAGE; // Default target voltage
float targetCurrent = MAX_ALLOWED_CURRENT; // Default target current

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

  // Initialize WiFi in Access Point mode
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Set up web server routes
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/control", handleControl);
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
        float voltage = voltageRaw * 0.1f;
        batteryVoltage = voltage;

        // Parse current from bytes 2-3 (0.1 A/bit resolution)
        uint16_t currentRaw = (msgData[2] << 8) | msgData[3];
        float current = currentRaw * 0.1f;
        batteryCurrent = current;

        // Parse charging state from byte 4 and update the status text
        uint8_t state = msgData[4];
        chargingStatusText = (state == 0) ? "Charging" : "Stopped";

        Serial.println("-------------------------------------");
        Serial.print("Voltage: ");
        Serial.println(voltage);
        Serial.print("Current: ");
        Serial.println(current);
        Serial.print("Status: ");
        Serial.println(chargingStatusText);
        Serial.println("-------------------------------------");
      }
    }
  }
  delay(100);
}

// ----- Web Server Handlers -----

// Root page: displays status, CAN message details, and a form to update parameters
void handleRoot()
{
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>CAN Monitor</title></head><body>";
  html += "<h1>CAN Monitor</h1>";
  html += "<p><strong>Charging Status:</strong> " + chargingStatusText + "</p>";
  html += "<p><strong>CAN ID:</strong> " + webCANId + "</p>";
  html += "<p><strong>CAN Data:</strong> " + webCANData + "</p>";
  html += "<p><strong>Voltage:</strong> " + String(batteryVoltage, 1) + " V</p>";
  html += "<p><strong>Current:</strong> " + String(batteryCurrent, 1) + " A</p>";

  // Form to update target voltage and current
  html += "<h2>Update Charger Parameters</h2>";
  html += "<form action='/set' method='GET'>";
  html += "Set target Voltage (V): <input type='number' step='0.1' name='v' value='" + String(targetVoltage, 1) + "'><br>";
  html += "Set target Current (A): <input type='number' step='0.1' name='c' value='" + String(targetCurrent, 1) + "'><br>";
  html += "<input type='submit' value='Update Parameters'>";
  html += "</form>";

  // Buttons to control charging (start/stop)
  html += "<h2>Control Charger</h2>";
  html += "<button onclick=\"location.href='/control?cmd=start'\">Start Charging</button> ";
  html += "<button onclick=\"location.href='/control?cmd=stop'\">Stop Charging</button>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Handle the form submission to update target voltage and current
void handleSet()
{
  if (server.hasArg("v"))
  {
    float newVoltage = server.arg("v").toFloat();
    targetVoltage = newVoltage;
    Serial.print("Updated target voltage: ");
    Serial.println(targetVoltage);
  }
  if (server.hasArg("c"))
  {
    float newCurrent = server.arg("c").toFloat();
    targetCurrent = newCurrent;
    Serial.print("Updated target current: ");
    Serial.println(targetCurrent);
  }
  // Redirect back to the root page so the form stays on the current page
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

// Handle control commands (start/stop charging)
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
  // Redirect back to the root page
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

// ----- CAN Command Functions -----

// Send the charger command using the current target voltage/current values
void sendChargerCommand()
{
  // Convert voltage/current to 0.1 units
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
  // Uncomment the following code to enable CAN message sending:
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

// Send a command to stop charging
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
