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
unsigned long pmillis = 0;                 // For timing periodic operations
unsigned long cutoffStartTime = 0;         // Timer for cutoff
unsigned long uptime = 0;                  // System uptime (seconds)
const unsigned long delayCutoff = 30000;   // 30 seconds delay for cutoff
float targetVoltage = MAX_ALLOWED_VOLTAGE; // Default target voltage
float targetCurrent = 6;                   // Default target current
bool chargerActive = false;                // Charging flag (true = charging, false = stopped)
String chargingStatusText = "Stopped";     // Status text for display
String lastCpState = "A";
float cutoffCurrent = 4.0;     // Default cutoff current
bool checkingCutoff = false;   // Flag for cutoff monitoring
bool cutoffTriggered = false;  // Flag to indicate that cutoff has occurred
bool chargingObcState = false; // OBC charging state
bool vehicleReadyState = false;
bool vehicleReadyStateCleared = false; // Track if it has been cleared
bool hasStartedCharging = false;

// --- Constants ---
const float VREF = 3.3;                       // ADC Reference voltage
const float CP_SCALING_FACTOR_DEFAULT = 3.0;  // Initial scaling factor (raw*3 ≈ 9.9V at full-scale)
const unsigned long UPDATE_INTERVAL = 4000;   // Update interval (ms)
const unsigned long PEAK_INTERVAL = 2000;     // Peak voltage update interval (ms)
const unsigned long AVERAGE_INTERVAL = 2000;  // Average voltage update interval (ms)
const unsigned long S2_DELAY = 3000;          // 5 seconds delay before auto vehicle ready
const unsigned long CALIBRATION_DELAY = 3000; // 1 second delay for calibration

// Pin Definitions
const uint8_t ADC_PIN = 33;       // ADC input pin
const uint8_t FREQUENCY_PIN = 34; // PWM input pin
const uint8_t S2_CTRL_PIN = 12;   // Control pin

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

const float CP_VOLTAGE_TOLERANCE_PLUG = 1.0;
const float CP_VOLTAGE_TOLERANCE_VEHICLE = 1.5;

// --- State Enumeration ---
enum CPState
{
    NO_PLUG,
    PLUG_CONNECTED,
    VEHICLE_READY
};

// Voltage thresholds:
const float NO_PLUG_VOLTAGE_THRESHOLD = 0.2; // Scaled voltage below which no plug is detected
const float PLUG_CONNECTED_VOLTAGE = 9.0;    // Target scaled voltage when plug connected
const float VEHICLE_READY_VOLTAGE = 6.0;     // Target scaled voltage when vehicle ready

// Frequency & ADC tolerances:
const float FREQUENCY_TOLERANCE = 800.0;            // ±800Hz tolerance around 1kHz
const float ADC_READING_VALIDATION_THRESHOLD = 0.1; // Minimum valid raw ADC voltage
const float ADC_READING_MAX_THRESHOLD = 3.3;        // Maximum valid raw ADC voltage
const unsigned int MAX_INVALID_READINGS = 5;        // Number of invalid readings before reset
const unsigned long STATIC_VOLTAGE_TIMEOUT = 10000; // Time to wait for PWM before accepting static voltage

volatile bool compState = false; // Captures comparator (FREQUENCY_PIN) state
unsigned long lastAverageReset = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastCPCheck = 0;
unsigned long s2ActivateTime = 0;          // Time when plug connection was detected for S2 delay
unsigned long staticVoltageDetectTime = 0; // Time when static voltage was first detected
unsigned long plugDetectionTime = 0;       // Time when plug was first detected (for calibration delay)

float rawPeakVoltage = 0.01; // Raw ADC peak (for debugging)
float averageVoltage = 0.0;
float cpScalingFactor = CP_SCALING_FACTOR_DEFAULT; // Auto scaling factor

// New global: store the last finished ADC reading.
float lastAdcReading = 0.0;

// Flags & state
bool s2State = false;                  // True if S2 (vehicle ready) is active
bool plugConnected = false;            // True if plug is connected
bool staticVoltageDetected = false;    // True if static voltage (no PWM) is detected
unsigned long invalidReadingCount = 0; // Count of consecutive invalid readings
CPState currentState = NO_PLUG;        // Current CP state
bool calibrationDone = false;          // Flag indicating if calibration in PLUG_CONNECTED state is complete
float cpVoltage = 0.0f;

// Function prototypes
void initializeHardware();
float readVoltage();
float convertAdcToCpVoltage(float adcVoltage);
void updatePeakVoltage();
void updateFrequencyAndDutyCycle();
void handleSerialCommands();
float getMaxCurrent(float dutyCycle);
String getCPStatus(float voltage);
void vehicleReadyStateClear();
float getMaxCurrentForObc();
float getBatteryVoltage();
void updateAverageVoltage();
void resetLogic();
void adjustScalingFactor(float targetVoltage);
bool validateReadings(float targetVoltage);
bool isPlugConnected();
bool isStaticVoltagePresent();

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
void printSystemStatus();

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
bool cutoffEnabled = false; // Track CP mode state

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
    chargingStatusText = "Charging";
    Serial.println("Charger started");
}

void stopCharger()
{
    chargerActive = false;
    stopChargerCommand();
    checkingCutoff = false;
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
            // CP Mode Logic
            if (cpModeEnabled)
            {
                float cpVoltage = convertAdcToCpVoltage(peakVoltage);
                float currentRawVoltage = readVoltage();

                // *** AUTO CONNECT LOGIC ***
                // Force plug detection if a valid PWM signal is detected and plugConnected is false.
                if (!plugConnected &&
                    frequency >= 900.0 && frequency <= 1100.0 &&
                    peakVoltage > 1.5)
                {
                    if (cutoffTriggered)
                    {
                        printDebugInfo("valid frequency detected - but cutoff triggered");
                    }
                    else
                    {
                        printDebugInfo("Forcing plug detection - valid frequency detected");
                        plugDetectionTime = millis();
                        plugConnected = true;
                        currentState = PLUG_CONNECTED;
                        calibrationDone = false;
                    }
                }

                // --- NO_PLUG: If scaled voltage is very low and frequency is near zero.
                if (cpVoltage < NO_PLUG_VOLTAGE_THRESHOLD && frequency < 10 && !staticVoltageDetected)
                {
                    if (currentState != NO_PLUG)
                    {
                        printDebugInfo("Transitioning to NO_PLUG state");
                        resetLogic();
                    }
                    cutoffTriggered = false;
                }
                // --- Transition from NO_PLUG to PLUG_CONNECTED ---
                else if (currentState == NO_PLUG && isPlugConnected())
                {
                    plugDetectionTime = millis(); // Record time of plug detection for calibration delay
                    currentState = PLUG_CONNECTED;
                    plugConnected = true;
                    calibrationDone = false;
                    printDebugInfo("Plug detected, waiting for calibration delay");
                }
                if (!cutoffTriggered)
                {
                    // --- In PLUG_CONNECTED state: Perform calibration after a delay if not already done ---
                    if (currentState == PLUG_CONNECTED && !calibrationDone)
                    {
                        if (millis() - plugDetectionTime >= CALIBRATION_DELAY)
                        {
                            // Only calibrate if signal is stable (i.e. valid PWM frequency)
                            if (frequency >= 900.0 && frequency <= 1100.0)
                            {
                                adjustScalingFactor(PLUG_CONNECTED_VOLTAGE);
                                calibrationDone = true;
                                s2ActivateTime = millis(); // Start S2 activation delay timer after calibration
                                printDebugInfo("Calibration complete. Waiting for S2 activation delay.");
                            }
                            else
                            {
                                printDebugInfo("Signal not stable for calibration, waiting...");
                            }
                        }
                    }

                    // --- Transition from PLUG_CONNECTED to VEHICLE_READY ---
                    else if (currentState == PLUG_CONNECTED && !s2State && (millis() - s2ActivateTime >= S2_DELAY))
                    {

                        if ((frequency >= 900.0 && frequency <= 1100.0) ||
                            (staticVoltageDetected && currentRawVoltage >= 1.5))
                        {
                            printDebugInfo("Auto transitioning to VEHICLE_READY state after delay");
                            digitalWrite(S2_CTRL_PIN, HIGH);
                            s2State = true;
                            // adjustScalingFactor(VEHICLE_READY_VOLTAGE);
                            currentState = VEHICLE_READY;
                            delay(200);
                        }
                        else
                        {
                            printDebugInfo("Failed to auto-transition: not a valid signal");
                            s2ActivateTime = millis();
                        }
                    }
                    else if (currentState == VEHICLE_READY)
                    {
                        if (validateReadings(VEHICLE_READY_VOLTAGE))
                        {
                            invalidReadingCount = 0;
                            if (millis() - lastCPCheck >= 10000)
                            {
                                printDebugInfo("Vehicle Ready confirmed, S2 is ON");
                                lastCPCheck = millis();
                            }
                            float adjustedCurrent = min(targetCurrent, getMaxCurrentForObc());
                            sendChargerCommand(targetVoltage, adjustedCurrent, true);
                            // --- CUTOFF CURRENT LOGIC (only in CP mode) ---
                            // Check if the charging current (batteryCurrent) is below cutoffCurrent.

                            if (batteryCurrent <= cutoffCurrent)
                            {
                                if (cutoffEnabled)
                                {
                                    if (!checkingCutoff)
                                    {
                                        checkingCutoff = true;
                                        cutoffStartTime = millis();
                                        Serial.println("Cutoff: Low current detected, starting timer...");
                                    }
                                    else if (millis() - cutoffStartTime >= delayCutoff)
                                    {
                                        cutoffTriggered = true;
                                        stopCharger(); // This stops charging immediately.
                                        Serial.println("Charging stopped due to current cutoff");
                                        // Skip further processing in this iteration.
                                        resetLogicCutoff();
                                        vTaskDelay(10 / portTICK_PERIOD_MS);
                                        continue;
                                    }
                                }
                            }
                            else
                            {
                                // If current has recovered above the cutoff threshold, reset the cutoff timer.
                                if (checkingCutoff)
                                {
                                    Serial.println("Cutoff: Current recovered, resetting cutoff timer");
                                }
                                checkingCutoff = false;
                                cutoffStartTime = millis(); // Optional: reset timer
                            }
                        }
                        else if (cpVoltage < NO_PLUG_VOLTAGE_THRESHOLD && frequency < 10 && !staticVoltageDetected)
                        {
                            printDebugInfo("Vehicle disconnected from VEHICLE_READY state");
                            resetLogic();
                        }
                        else
                        {
                            invalidReadingCount++;
                            printDebugInfo("Invalid reading in VEHICLE_READY state (" +
                                           String(invalidReadingCount) + "/" +
                                           String(MAX_INVALID_READINGS) + ")");
                            if (invalidReadingCount >= MAX_INVALID_READINGS)
                            {
                                printDebugInfo("Too many invalid readings, resetting...");
                                resetLogic();
                            }
                        }
                    }

                    // --- If in PLUG_CONNECTED state, periodically confirm valid voltage.
                    else if (currentState == PLUG_CONNECTED)
                    {
                        if (convertAdcToCpVoltage(peakVoltage) < NO_PLUG_VOLTAGE_THRESHOLD && frequency < 10 && !staticVoltageDetected)
                        {
                            printDebugInfo("Lost connection in PLUG_CONNECTED state");
                            resetLogic();
                        }
                        else if (millis() - lastCPCheck >= 5000)
                        {
                            unsigned long remainingTime = (s2ActivateTime + S2_DELAY) - millis();
                            if (remainingTime < S2_DELAY)
                            {
                                printDebugInfo("PLUG_CONNECTED: Waiting " + String(remainingTime / 1000) +
                                               " seconds for auto S2 activation");
                            }
                        }
                    }
                    // --- Check for static voltage in any state ---
                    if (!staticVoltageDetected && isStaticVoltagePresent() &&
                        (millis() - staticVoltageDetectTime > STATIC_VOLTAGE_TIMEOUT) &&
                        currentState != NO_PLUG)
                    {
                        staticVoltageDetected = true;
                        printDebugInfo("Static voltage confirmed (no PWM)");
                    }
                }
                // Non-CP Mode Logic
            }

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

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
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
    cutoffEnabled = preferences.getBool("tgCutoff", cutoffEnabled);

    Serial.print("==================================================");
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
    Serial.print("Loaded tgCuttoff: ");
    Serial.println(cutoffEnabled);
    Serial.print("==================================================");

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
    initializeHardware();
    cpDetectStartup();
}

void loop()
{
    // The main loop is empty because FreeRTOS tasks handle everything
    handleSerialCommands();
    updatePeakVoltage();
    updateFrequencyAndDutyCycle();
    printSystemStatus();
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