/*
NOTES:
SESUAIKAN DAHULU DENGAN SPESIFIKASI RATING MAKSIMUM DARI CHARGER/BATTERY UNTUK SAFETY! KESALAHAN INPUT VOLTAGE/CURRENT
VARIABLE DARI MAX_ALLOWED_VOLTAGE MAX_ALLOWED_CURRENT BISA MENYEBABKAN KERUSAKAN ATAU BAHAYA!
*/

#include <SPI.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h> // Library for internal (non-volatile) storage

// ----- Function Prototypes -----
// must be defined here to avoid undefined reference
void handleRoot();
void handleSet();
void handleControl();
void handleData();
void sendChargerCommand();
void stopChargerCommand();
void stopCharger();
void startCharger();
void handleReceivingCanbus();
void simulateCanbus(float _batteryVoltage, float _batteryCurrent, bool _state);
void serialLoop();
void decodeChargerBroadcast(byte msgData[], byte len);
bool keyExists(const char *input, const char *key);          // utility function
const char *getValueKey(const char *input, const char *key); // utility function
void printDecodeDataObc();

// ----- Pin Definitions -----
#define CAN_CS 5  // Chip Select pin for CAN
#define CAN_INT 4 // Interrupt pin for CAN

// Function prototypes
void handleSerialCommands();

// ----- CAN Bus Setup -----
MCP_CAN CAN(CAN_CS); // Create CAN instance

// ----- Shared Preferences Setup -----
Preferences preferences;

// ----- WiFi & Web Server Setup -----
const char *ssid = "controlOBC";
const char *password = "password123";
WebServer server(80);

// ----- Constants -----
const float CELL_VOLTAGE = 3.5;                             // Maximum per-cell voltage
const int NUM_CELLS = 120;                                  // Example: 32S configuration.  CHANGE THIS!
const float MAX_ALLOWED_VOLTAGE = NUM_CELLS * CELL_VOLTAGE; // Calculated maximum allowed voltage
const float MAX_ALLOWED_CURRENT = 60;                       // Maximum allowed current.  CHANGE THIS!
#define CHARGER_CONTROL_ID 0x1806E5F4                       // CAN ID for sending commands TO the charger (Report 1)
#define BMS_CONTROL_ID 0x1806E5F4                           // CAN ID for receive command from bms

// ----- Global Variables -----
float targetVoltage = MAX_ALLOWED_VOLTAGE; // Default target voltage
float targetCurrent = 6;                   // Default target current
unsigned long pmillis = 0;                 // For timing the CAN command transmissions
bool chargerActive = false;                // Charging flag (true = charging, false = stopped)
String chargingStatusText = "Stopped";     // Charging status text for display
float cutoffCurrent = 4.0;                 // Default cutoff current
unsigned long cutoffStartTime = 0;         // Timer for cutoff
bool checkingCutoff = false;               // Flag for cutoff checking
bool cutoffTriggered = false;              // Flag to indicate that cutoff has occurred
const unsigned long delayCutoff = 30000;   //  30 seconds
unsigned long uptime;
bool chargingObcState = false;

// CP S2 HANDLING
float cpVoltage = 0.0f; // To store the measured CP voltage

// Variables for web display
String webCANId = "";
String webCANData = "";
float batteryVoltage = 0.0f;
float batteryCurrent = 0.0f;
bool hardwareFailure = false;
bool chargerTemp = false;
bool inputVoltage = false;
bool startState = false;
bool communicationTimeout = false;
bool isActiveOnStartup = false;
#include "handle_canbus.h"
#include "handle_web.h"

// Ambang batas dengan Histeresis (Sesuaikan berdasarkan pengukuran Anda)
const float STATE_A_HIGH = 1.85; // Transisi ke State A di atas tegangan ini
const float STATE_A_LOW = 1.75;
const float STATE_B_HIGH = 1.32; // Transisi ke State B di atas ini (dari A)
const float STATE_B_LOW = 1.22;
const float STATE_C_HIGH = 1.12; // Transisi ke State C di atas ini
const float STATE_C_LOW = 0.7;   // Transisi ke state lain
const float STATE_D_HIGH = 0.6;  // Tidak digunakan dalam DC fast charging
const float STATE_D_LOW = 0.2;
const float STATE_E_HIGH = 0.1; // < 0.1V

// ----- Setup -----
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
  preferences.begin("settings", false);
  targetVoltage = preferences.getFloat("targetVoltage", targetVoltage);
  targetCurrent = preferences.getFloat("targetCurrent", targetCurrent);
  cutoffCurrent = preferences.getFloat("cutoffCurrent", cutoffCurrent);
  isActiveOnStartup = preferences.getBool("onStartup", isActiveOnStartup);

  Serial.print("Loaded target voltage: ");
  Serial.println(targetVoltage);
  Serial.print("Loaded target current: ");
  Serial.println(targetCurrent);
  Serial.print("Loaded target cutoff current: ");
  Serial.println(cutoffCurrent);
  Serial.print("Loaded target isAutoStartUp: ");
  Serial.println(isActiveOnStartup);
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

  // activate the charging whenever isactiveOnstartup is on
  if (isActiveOnStartup)
  {
    startCharger();
  }
}

// ----- Main Loop -----
void loop()
{
  server.handleClient(); // Handle web server requests
  serialLoop();          // Handle Serial Port

  // Periodic CAN command sending (every 1 second)
  if (millis() - pmillis > 1000)
  {
    pmillis = millis();
    uptime = millis() / 1000; // seconds
    if (chargerActive)
    {
      // sendChargerCommand();
    }
  }
  handleReceivingCanbus(); // Check for incoming CAN messages

  // Cutoff logic (Corrected)
  if (chargerActive && !cutoffTriggered)
  {
    // if (batteryCurrent < cutoffCurrent)
    // {
    //   // Start the timer if it hasn't started yet
    //   if (!checkingCutoff)
    //   {
    //     cutoffStartTime = millis();
    //     checkingCutoff = true;
    //     Serial.println("Start monitoring low current...");
    //   }
    //   // Check if 10 seconds have passed continuously
    //   else if ((millis() - cutoffStartTime) >= delayCutoff)
    //   {
    //     // Trigger the cutoff
    //     stopCharger();
    //     cutoffTriggered = true;
    //     chargingStatusText = "cutoff current";
    //     Serial.println("=====================================");
    //     Serial.println("Cutoff Triggered after continuous!");
    //     Serial.print("Battery Voltage: ");
    //     Serial.print(batteryVoltage);
    //     Serial.println(" V");
    //     Serial.print("Battery Current: ");
    //     Serial.print(batteryCurrent);
    //     Serial.println(" A");
    //     Serial.print("Cutoff Current Threshold: ");
    //     Serial.print(cutoffCurrent);
    //     Serial.println(" A");
    //     Serial.println("=====================================");
    //   }
    // }
    // else
    // {
    //   // Reset timer if current rises above cutoff
    //   if (checkingCutoff)
    //   {
    //     checkingCutoff = false;
    //     Serial.println("Current recovered, reset monitoring");
    //   }
    // }
  }
  else
  {
    checkingCutoff = false;
  }

  delay(100); // Short delay
}

// ----- Function Implementations -----

void startCharger()
{
  chargerActive = true;
  checkingCutoff = false;
  cutoffTriggered = false; // Reset the cutoff flag when charging starts
  chargingStatusText = "Charging";
  Serial.println("Charger started");
}

void stopCharger()
{
  chargerActive = false;
  stopChargerCommand();
  checkingCutoff = false;
  cutoffTriggered = false; // Clear any previous cutoff trigger
  chargingStatusText = "Stopped";
  Serial.println("Charger stopped");
}
