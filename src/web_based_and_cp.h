/*
  IMPORTANT:
  Pastikan rating maksimum charger/battery sesuai dengan spesifikasi Anda.
  Kesalahan input pada MAX_ALLOWED_VOLTAGE / MAX_ALLOWED_CURRENT dapat menyebabkan kerusakan atau bahaya!
*/

#include <SPI.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ----- Pin Definitions -----
#define CAN_CS 5  // Chip Select pin for CAN
#define CAN_INT 4 // Interrupt pin for CAN

// ----- CAN Bus Setup -----
MCP_CAN CAN(CAN_CS);

// ----- Shared Preferences Setup -----
Preferences preferences;

// ----- WiFi & Web Server Setup -----
const char *ssid = "controlOBC";
const char *password = "password123";
WebServer server(80);

// ----- Constants -----
const float CELL_VOLTAGE = 3.5;                             // Maximum per-cell voltage
const int NUM_CELLS = 120;                                  // Example: 120 cells (change as needed)
const float MAX_ALLOWED_VOLTAGE = NUM_CELLS * CELL_VOLTAGE; // Calculated maximum allowed voltage
const float MAX_ALLOWED_CURRENT = 16;                       // Maximum allowed current (change as needed)
#define CHARGER_CONTROL_ID 0x1806E5F4                       // CAN ID for sending commands to the charger
#define BMS_CONTROL_ID 0x1806E5F4                           // CAN ID for receiving commands from BMS

// ----- Global Variables -----
float targetVoltage = MAX_ALLOWED_VOLTAGE; // Default target voltage
float targetCurrent = 6;                   // Default target current
unsigned long pmillis = 0;                 // For timing periodic operations
bool chargerActive = false;                // Charging flag (true = charging, false = stopped)
String chargingStatusText = "Stopped";     // Status text for display
float cutoffCurrent = 4.0;                 // Default cutoff current
unsigned long cutoffStartTime = 0;         // Timer for cutoff
bool checkingCutoff = false;               // Flag for cutoff monitoring
bool cutoffTriggered = false;              // Flag to indicate that cutoff has occurred
const unsigned long delayCutoff = 30000;   // 30 seconds delay for cutoff
unsigned long uptime = 0;                  // System uptime (seconds)
bool chargingObcState = false;             // OBC charging state
bool modeCp;

// Pin Definitions
const uint8_t ADC_PIN = 35;       // ADC input pin
const uint8_t FREQUENCY_PIN = 34; // PWM input pin
const uint8_t S2_CTRL_PIN = 12;   // Control pin

// Constants
const float VREF = 3.3;                     // ADC Reference voltage
const float CP_SCALING_FACTOR = 6.16 / 2.0; // CP voltage mapping
const unsigned long UPDATE_INTERVAL = 2000; // Frequency update interval (ms)
const unsigned long PEAK_INTERVAL = 2000;   // Peak voltage update interval (ms)

// Global variables
volatile unsigned long pulseCount = 0;
volatile unsigned long highTime = 0;
volatile unsigned long lowTime = 0;
unsigned long lastEdgeTime = 0;
unsigned long lastFrequencyUpdate = 0;
unsigned long lastPeakReset = 0;

float frequency = 0;
float dutyCycle = 0;
float currentPeak = 0.01; // Avoid sudden zero issues
float peakVoltage = 0.01;

// Function prototypes
void initializeHardware();
float readVoltage();
float convertAdcToCpVoltage(float adcVoltage);
float updatePeakVoltage();
void updateFrequencyAndDutyCycle();
void handleSerialCommands();
float getMaxCurrent(float dutyCycle);
String getCPStatus(float voltage);

// Variables for CAN simulation and web display
float cpVoltage = 0.0f; // CP S2 voltage measurement

// ----- Function Prototypes -----
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
bool cpModeEnabled = false; // Track CP mode state

#include "handle_canbus.h"
#include "handle_web.h"
#include "handle_cp.h"

// ----- RTOS Task Declarations -----
void webServerTask(void *pvParameters);
void serialTask(void *pvParameters);
void canReceiveTask(void *pvParameters);
void periodicTask(void *pvParameters);

void startCharger()
{
    chargerActive = true;
    checkingCutoff = false;
    cutoffTriggered = false; // Reset cutoff flag when charging starts
    chargingStatusText = "Charging";
    Serial.println("Charger started");
}

void stopCharger()
{
    chargerActive = false;
    stopChargerCommand();
    checkingCutoff = false;
    cutoffTriggered = false;
    chargingStatusText = "Stopped";
    Serial.println("Charger stopped");
}

void periodicTask(void *pvParameters)
{
    unsigned long lastTime = millis();
    for (;;)
    {
        unsigned long currentTime = millis();
        // Execute tasks every 1000 ms
        if (currentTime - lastTime >= 1000)
        {
            lastTime = currentTime;
            uptime = millis() / 1000;
            if (chargerActive)
            {
                sendChargerCommand();
            }

            // --- Cutoff Logic ---
            if (chargerActive && !cutoffTriggered)
            {
                // Uncomment and adjust as needed
                /*
                if (batteryCurrent < cutoffCurrent)
                {
                    if (!checkingCutoff)
                    {
                        cutoffStartTime = millis();
                        checkingCutoff = true;
                        Serial.println("Start monitoring low current...");
                    }
                    else if ((millis() - cutoffStartTime) >= delayCutoff)
                    {
                        stopCharger();
                        cutoffTriggered = true;
                        chargingStatusText = "Cutoff current";
                        Serial.println("=====================================");
                        Serial.println("Cutoff Triggered!");
                        Serial.print("Battery Voltage: ");
                        Serial.print(batteryVoltage);
                        Serial.println(" V");
                        Serial.print("Battery Current: ");
                        Serial.print(batteryCurrent);
                        Serial.println(" A");
                        Serial.print("Cutoff Current Threshold: ");
                        Serial.print(cutoffCurrent);
                        Serial.println(" A");
                        Serial.println("=====================================");
                    }
                }
                else
                {
                    if (checkingCutoff)
                    {
                        checkingCutoff = false;
                        Serial.println("Current recovered, reset monitoring");
                    }
                }
                */
            }
            else
            {
                checkingCutoff = false;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// --- Setup and Main Loop ---
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

    // Initialize shared preferences and load stored settings
    preferences.begin("settings", false);
    targetVoltage = preferences.getFloat("targetVoltage", targetVoltage);
    targetCurrent = preferences.getFloat("targetCurrent", targetCurrent);
    cutoffCurrent = preferences.getFloat("cutoffCurrent", cutoffCurrent);
    isActiveOnStartup = preferences.getBool("onStartup", isActiveOnStartup);
    cpModeEnabled = preferences.getBool("cpMode", cpModeEnabled);

    Serial.print("Loaded target voltage: ");
    Serial.println(targetVoltage);
    Serial.print("Loaded target current: ");
    Serial.println(targetCurrent);
    Serial.print("Loaded cutoff current: ");
    Serial.println(cutoffCurrent);
    Serial.print("Loaded isActiveOnStartup: ");
    Serial.println(isActiveOnStartup);
    Serial.print("Loaded cpModeEnabled: ");
    Serial.println(cpModeEnabled);

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

    // Activate charging on startup if enabled
    if (isActiveOnStartup)
    {
        startCharger();
    }

    // Create RTOS tasks
    xTaskCreatePinnedToCore(
        webServerTask,   // Task function
        "WebServerTask", // Task name
        4096,            // Stack size (in bytes)
        NULL,            // Parameter passed to the task
        1,               // Task priority
        NULL,            // Task handle
        1);              // Run on core 1

    xTaskCreatePinnedToCore(
        serialTask,
        "SerialTask",
        2048,
        NULL,
        1,
        NULL,
        1);

    xTaskCreatePinnedToCore(
        canReceiveTask,
        "CANReceiveTask",
        2048,
        NULL,
        1,
        NULL,
        1);

    xTaskCreatePinnedToCore(
        periodicTask,
        "PeriodicTask",
        2048,
        NULL,
        1,
        NULL,
        1);

    initializeCP();
}

void loop()
{
    // The main loop is empty because FreeRTOS tasks handle everything
    vTaskDelay(1 / portTICK_PERIOD_MS);
    handleSerialCommands();
    updatePeakVoltage();
    updateFrequencyAndDutyCycle();
}
void webServerTask(void *pvParameters)
{
    for (;;)
    {
        server.handleClient();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void serialTask(void *pvParameters)
{
    for (;;)
    {
        serialLoop();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void canReceiveTask(void *pvParameters)
{
    for (;;)
    {
        handleReceivingCanbus();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}