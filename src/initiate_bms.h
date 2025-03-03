#include <SPI.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <WebServer.h>

// ----- CAN Setup -----
#define CAN_CS 5  // Chip Select pin
#define CAN_INT 4 // Interrupt pin

MCP_CAN CAN(CAN_CS); // CAN controller instance

// ----- WiFi & Web Server Setup -----
const char *ssid = "CAN_Monitor_AP";
const char *password = "password123";
WebServer server(80);

// ----- Battery/Charger Parameters -----
// (For a 32-cell LiFePO4 pack)
const float CELL_VOLTAGE = 3.6;                     // Maximum per-cell voltage
const int NUM_CELLS = 32;                           // 32S configuration
const float MAX_VOLTAGE = NUM_CELLS * CELL_VOLTAGE; // e.g., 115.2V (32 x 3.6)
const float MAX_CURRENT = 3.2;                      // Configurable current limit

unsigned long pmillis = 0;
const int TEMP_COMPENSATION = -40; // From protocol docs

// ----- Global Variables for Status -----
// Use a bool to control sending commands
bool chargerActive = false;
// A string used for displaying charging status on the web
String chargingStatusText = "Stopped";

// Global variables to store the latest CAN message details for web display
String webCANId = "";
String webCANData = "";
float batteryVoltage = 0.0;
float batteryCurrent = 0.0;

// ----- Function Prototypes -----
void handleRoot();
void stopChargerCommand();
void sendChargerCommand();

// ----- (Optional) BMS Message Parsing -----
// This function shows how you might parse a BMS message,
// updating batteryVoltage and batteryCurrent if desired.
void parseBMSMessage(uint8_t data[8])
{
    // Parse voltage from bytes 0-1 (little-endian)
    uint16_t voltageRaw = (data[1] << 8) | data[0];
    float voltage = voltageRaw * 0.1f; // 0.1 V/bit resolution
    // Parse current from bytes 2-3 (little-endian)
    uint16_t currentRaw = (data[3] << 8) | data[2];
    float current = currentRaw * 0.1f; // 0.1 A/bit resolution

    batteryVoltage = voltage;
    batteryCurrent = current;
}

// ----- Stop Charger Command -----
// This command stops charging by sending a CAN message.
void stopChargerCommand()
{
    byte data[8] = {
        0x00, // Voltage high byte (not used)
        0x00, // Voltage low byte (not used)
        0x00, // Current high byte (not used)
        0x00, // Current low byte (not used)
        0x01, // Control byte: 0x01 = stop charging
        0x00, // Mode selector (not used)
        0x00, // Reserved
        0x00  // Reserved
    };

    if (CAN.sendMsgBuf(0x1806E5F4, 1, 8, data) == CAN_OK)
    {
        Serial.println("Charging stopped");
    }
    else
    {
        Serial.println("Stop command failed");
    }
}

// ----- Send Charger Command -----
// This command sends voltage/current settings via CAN.
void sendChargerCommand()
{
    // Convert voltage/current to 0.1 units
    uint16_t voltage = MAX_VOLTAGE * 10;
    byte voltage_high = (voltage >> 8) & 0xFF;
    byte voltage_low = voltage & 0xFF;

    uint16_t current = MAX_CURRENT * 10;
    byte current_high = (current >> 8) & 0xFF;
    byte current_low = current & 0xFF;

    byte data[8] = {
        voltage_high, // Voltage high byte
        voltage_low,  // Voltage low byte
        current_high, // Current high byte
        current_low,  // Current low byte
        0x00,         // Control byte (activate charging)
        0x00,         // Charging mode
        0x00,         // Reserved
        0x00          // Reserved
    };

    if (CAN.sendMsgBuf(0x1806E5F4, 1, 8, data) == CAN_OK)
    {
        Serial.print("Command sent: ");
        Serial.print(MAX_VOLTAGE, 1);
        Serial.print("V, ");
        Serial.print(MAX_CURRENT, 1);
        Serial.println("A");
    }
    else
    {
        Serial.println("Send failed");
    }
}

// ----- Web Server Root Handler -----
// Displays charging status and the most recent CAN message details.
void handleRoot()
{
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>CAN Monitor</title></head><body>";
    html += "<h1>CAN Monitor</h1>";
    html += "<p><strong>Charging Status:</strong> " + chargingStatusText + "</p>";
    html += "<p><strong>CAN ID:</strong> " + webCANId + "</p>";
    html += "<p><strong>CAN Data:</strong> " + webCANData + "</p>";
    html += "<p><strong>Battery Voltage:</strong> " + String(batteryVoltage, 1) + " V</p>";
    html += "<p><strong>Battery Current:</strong> " + String(batteryCurrent, 1) + " A</p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

// ----- Setup -----
void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for Serial port to be ready

    // Initialize CAN bus
    if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK)
    {
        Serial.println("CAN init failed");
        while (1)
            ;
    }
    CAN.setMode(MCP_NORMAL);
    Serial.println("CAN Ready");

    // Initialize WiFi in Access Point mode
    WiFi.softAP(ssid, password);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    // Set up web server routes
    server.on("/", handleRoot);
    server.begin();
}

// ----- Main Loop -----
void loop()
{
    // Service web client requests
    server.handleClient();

    // Periodically send charger command if charging is active
    if (millis() - pmillis > 1000)
    {
        pmillis = millis();
        if (chargerActive)
        {
            sendChargerCommand();
        }
    }

    // (For testing via Serial) Check for commands:
    // Press 's' to start charging and send a charger command,
    // Press 'x' to stop charging.
    if (Serial.available() > 0)
    {
        char inChar = Serial.read();
        if (inChar == 's')
        { // Start charging
            sendChargerCommand();
            chargerActive = true;
        }
        else if (inChar == 'x')
        { // Stop charging
            chargerActive = false;
            stopChargerCommand();
        }
    }

    // ----- Handle Incoming CAN Messages -----
    if (digitalRead(CAN_INT) == LOW)
    {
        unsigned long id;
        byte len;
        byte msgData[8];
        if (CAN.readMsgBuf(&id, &len, msgData) == CAN_OK)
        {
            // Update global variables for web display
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

            // Improved Serial output with better labels
            Serial.print("CAN ID: ");
            Serial.print(webCANId);
            Serial.print(" | CAN Data: ");
            Serial.println(webCANData);

            if (len == 8)
            {
                // Parse voltage from bytes 0-1
                uint16_t voltageRaw = (msgData[0] << 8) | msgData[1];
                float voltage = voltageRaw * 0.1f; // 0.1 V/bit resolution
                batteryVoltage = voltage;

                // Parse current from bytes 2-3
                uint16_t currentRaw = (msgData[2] << 8) | msgData[3];
                float current = currentRaw * 0.1f; // 0.1 A/bit resolution
                batteryCurrent = current;

                // Parse charging state from byte 4 and update the status string
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
