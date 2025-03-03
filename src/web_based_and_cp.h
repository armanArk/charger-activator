/*
  IMPORTANT:
  Kesalahan input pada MAX_ALLOWED_VOLTAGE / MAX_ALLOWED_CURRENT dapat menyebabkan kerusakan atau bahaya!
  Pastikan rating maksimum charger/battery sesuai dengan spesifikasi Anda.
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
const char *password = "obc981126";
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
bool vehicleReadyState = false;
bool vehicleReadyStateCleared = false; // Track if it has been cleared
bool hasStartedCharging = false;
String lastCpState = "A";

// Pin Definitions
const uint8_t ADC_PIN = 33;       // ADC input pin
const uint8_t FREQUENCY_PIN = 34; // PWM input pin
const uint8_t S2_CTRL_PIN = 12;   // Control pin

// Constants
const float VREF = 3.3;                     // ADC Reference voltage
const float CP_SCALING_FACTOR = 6 / 1.85;   // CALIBRASI NILAI ADC  (voltage CP / ADC)
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
void vehicleReadyStateClear();
float getMaxCurrentForObc();
float getBatteryVoltage();
float updateAverageVoltage();

// Variables for CAN simulation and web display
float cpVoltage = 0.0f; // CP S2 voltage measurement

// ----- Function Prototypes -----
void handleRoot();
void handleSet();
void handleControl();
void handleData();
void sendChargerCommand(float voltage, float current, bool startCharging);
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

void vehicleReady()
{
    digitalWrite(S2_CTRL_PIN, HIGH);
    Serial.println("vehicleReady");
}

void vehicleReadyStateClear()
{
    if (!vehicleReadyStateCleared)
    {
        if (!getCPStatus(convertAdcToCpVoltage(peakVoltage)) == 'B')
        {
            // Execute the state clear logic only once
            lastCpState = "A";
            vehicleReadyState = false;
            hasStartedCharging = false;
            digitalWrite(S2_CTRL_PIN, LOW);
            Serial.println("vehicleReadyStateClear()");
            vehicleReadyStateCleared = true;
        }
    }
}

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
        if (currentTime - lastTime >= 4000)
        {
            lastTime = currentTime;
            uptime = millis() / 4000;
            Serial.print("STATE:");
            Serial.print(getCPStatus(convertAdcToCpVoltage(peakVoltage)));
            Serial.print(",cpMode:");
            Serial.println(cpModeEnabled);

            // CP Mode Logic
            if (cpModeEnabled)
            {
                float maxAllowedCurrent = getMaxCurrent(dutyCycle);
                float adjustedCurrent = targetCurrent;
                if (adjustedCurrent > getMaxCurrentForObc())
                {
                    adjustedCurrent = getMaxCurrentForObc();
                }
                // Serial.print("Adjusted Current: ");
                sendChargerCommand(targetVoltage, adjustedCurrent, true);

                // Reset to state A if peakVoltage, frequency, and dutyCycle are zero
                // detect reset A only frequency == 0 && dutyCycle == 0!
                if (frequency == 0 && dutyCycle == 0)
                {
                    vehicleReadyStateClear();
                    // sendChargerCommand(targetVoltage, 0, false);
                    // Serial.println("Reset to state A due to zero peakVoltage, frequency, and dutyCycle");
                    continue; // Skip the rest of the loop iteration
                }
                else
                {
                    vehicleReady();
                }
                String currentCpState = getCPStatus(convertAdcToCpVoltage(peakVoltage));
                if (true) // if (!communicationTimeout)
                {
                    // State changes to B (Vehicle Connected)
                    if (currentCpState == "B" && !vehicleReadyState)
                    {
                        vehicleReady();
                        vehicleReadyState = true;
                        vehicleReadyStateCleared = false;
                        Serial.println("Vehicle connected - State B");
                    }
                    // State changes to C (Vehicle Ready for Charging)
                    else if (currentCpState == "C")
                    {
                        float maxAllowedCurrent = getMaxCurrentForObc();
                        float adjustedCurrent = min(targetCurrent, maxAllowedCurrent);
                        // sendChargerCommand(targetVoltage, adjustedCurrent, true);
                        if (!hasStartedCharging)
                        {
                            hasStartedCharging = true;
                            Serial.println("Vehicle ready to charge - State C");
                            Serial.printf("Max allowed current: %.2f A\n", maxAllowedCurrent);
                        }
                    }
                }
                else
                {
                    // Jika komunikasi timeout
                    if (hasStartedCharging)
                    {
                        vehicleReadyStateClear();
                        // sendChargerCommand(targetVoltage, 0, false);
                        Serial.println("Charging stopped due to communication timeout");
                    }
                }

                // State A atau E (Disconnected atau Error)
                if ((currentCpState == "A" || currentCpState == "E") &&
                    lastCpState != "A" && lastCpState != "E")
                {
                    // vehicleReadyStateClear();
                    // sendChargerCommand(targetVoltage, 0, false);
                    Serial.println("Vehicle disconnected or error - State " + currentCpState);
                }

                lastCpState = currentCpState;
            }
            // Non-CP Mode Logic
            else if (chargerActive)
            {
                if (!communicationTimeout)
                {
                    // Kirim perintah charger setiap detik saat mode normal
                    sendChargerCommand(targetVoltage, targetCurrent, true);
                }
                else
                {
                    stopCharger();
                    Serial.println("Charging stopped due to communication timeout");
                }
            }

            // Cutoff Logic (hanya untuk non-CP mode)
            // if (!cpModeEnabled && chargerActive && !cutoffTriggered && !communicationTimeout)
            // {
            //     if (batteryCurrent <= cutoffCurrent)
            //     {
            //         if (!checkingCutoff)
            //         {
            //             checkingCutoff = true;
            //             cutoffStartTime = millis();
            //         }
            //         else if (millis() - cutoffStartTime >= delayCutoff)
            //         {
            //             cutoffTriggered = true;
            //             stopCharger();
            //             Serial.println("Charging stopped due to current cutoff");
            //         }
            //     }
            //     else
            //     {
            //         checkingCutoff = false;
            //     }
            // }
            // else
            // {
            //     checkingCutoff = false;
            // }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void canbusSetup()
{
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
}
// --- Setup and Main Loop ---
void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for Serial to be ready
    // canbusSetup();
    Serial.println("canbus not setupped");
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

    startCharger();

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