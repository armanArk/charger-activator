#include <Arduino.h>

/*
 * ESP32 EV Charging Controller
 *
 * Flow:
 *  - NO_PLUG: ADC reading near zero, no frequency/duty.
 *  - PLUG_CONNECTED: When plug is first connected, frequency ~1kHz (tolerance ±100Hz)
 *    and the raw ADC (≈2.9–3.3V) is auto-scaled to ~9V.
 *  - VEHICLE_READY: After S2_DELAY, the divider is pulldown (via 1.3k resistor)
 *    and the scaled voltage drops to ~6V while frequency remains ~1kHz.
 *  - Unplug: ADC reading goes near zero and state resets.
 */

// --- Pin Definitions ---
const uint8_t ADC_PIN = 33;       // ADC input pin
const uint8_t FREQUENCY_PIN = 34; // PWM/Comparator input pin
const uint8_t S2_CTRL_PIN = 12;   // S2 control pin

// --- Constants ---
const float VREF = 3.3;                      // ADC Reference voltage
const float CP_SCALING_FACTOR_DEFAULT = 3.0; // Initial scaling factor (raw*3 ≈ 9.9V at full-scale)
const unsigned long UPDATE_INTERVAL = 4000;  // Update interval (ms)
const unsigned long PEAK_INTERVAL = 2000;    // Peak voltage update interval (ms)
const unsigned long AVERAGE_INTERVAL = 2000; // Average voltage update interval (ms)
const unsigned long S2_DELAY = 5000;         // 5 seconds delay before auto vehicle ready

// Voltage thresholds:
const float NO_PLUG_VOLTAGE_THRESHOLD = 0.2; // Scaled voltage below which no plug is detected
const float PLUG_CONNECTED_VOLTAGE = 9.0;    // Target scaled voltage when plug connected
const float VEHICLE_READY_VOLTAGE = 6.0;     // Target scaled voltage when vehicle ready

// Frequency & ADC tolerances:
const float FREQUENCY_TOLERANCE = 800.0;            // ±800Hz tolerance around 1kHz;              // ±100Hz tolerance around 1kHz
const float ADC_READING_VALIDATION_THRESHOLD = 0.1; // Minimum valid raw ADC voltage
const float ADC_READING_MAX_THRESHOLD = 3.3;        // Maximum valid raw ADC voltage
const unsigned int MAX_INVALID_READINGS = 5;        // Number of invalid readings before reset
const unsigned long STATIC_VOLTAGE_TIMEOUT = 10000; // Time to wait for PWM before accepting static voltage

// --- Voltage Tolerances (in Volts) ---
// Tolerance used when in PLUG_CONNECTED mode (PWM mode)
const float CP_VOLTAGE_TOLERANCE_PLUG = 1.0;
// Tolerance used when in VEHICLE_READY mode (S2 ON)
const float CP_VOLTAGE_TOLERANCE_VEHICLE = 1.5;

// --- State Enumeration ---
enum CPState
{
    NO_PLUG,
    PLUG_CONNECTED,
    VEHICLE_READY
};

// --- Global Variables ---
volatile unsigned long pulseCount = 0;
volatile unsigned long highTime = 0;
volatile unsigned long lowTime = 0;
volatile unsigned long lastEdgeTime = 0;
volatile bool compState = false; // Captures comparator (FREQUENCY_PIN) state

// Timing variables
unsigned long lastFrequencyUpdate = 0;
unsigned long lastPeakReset = 0;
unsigned long lastAverageReset = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastCPCheck = 0;
unsigned long s2ActivateTime = 0;          // Time when plug connection was detected
unsigned long staticVoltageDetectTime = 0; // Time when static voltage was first detected

// Measurement variables
float frequency = 0;
float dutyCycle = 0;
float peakVoltage = 0.01;
float rawPeakVoltage = 0.01; // Raw ADC peak (for debugging)
float averageVoltage = 0.0;
float cpScalingFactor = CP_SCALING_FACTOR_DEFAULT; // Auto scaling factor

// Flags & state
bool s2State = false;                  // True if S2 (vehicle ready) is active
bool plugConnected = false;            // True if plug is connected
bool staticVoltageDetected = false;    // True if static voltage (no PWM) is detected
unsigned long invalidReadingCount = 0; // Count of consecutive invalid readings
CPState currentState = NO_PLUG;        // Current CP state

// --- Function Prototypes ---
void initializeHardware();
float readVoltage();
float convertAdcToCpVoltage(float adcVoltage);
void updatePeakVoltage();
void updateAverageVoltage();
void updateFrequencyAndDutyCycle();
void handleSerialCommands();
float getMaxCurrent(float dutyCycle);
String getCPStatus(float voltage);
void processCP();
void resetLogic();
void adjustScalingFactor(float targetVoltage);
bool validateReadings(float targetVoltage);
bool isPlugConnected();
bool isStaticVoltagePresent();
void printDebugInfo(String message);
void printSystemStatus();

// --- Supporting Functions ---

float getMaxCurrent(float dutyCycle)
{
    if (dutyCycle >= 50.0)
        return 36.0;
    if (dutyCycle >= 40.0)
        return 30.0;
    if (dutyCycle >= 30.0)
        return 22.0;
    if (dutyCycle >= 25.0)
        return 20.0;
    if (dutyCycle >= 16.0)
        return 9.6;
    if (dutyCycle >= 10.0)
        return 6.0;
    return 0.0;
}

String getCPStatus(float voltage)
{
    // For debugging, return a simple letter code based on scaled voltage.
    if (voltage >= 11.0)
        return "A";
    if (voltage >= 8.0)
        return "B";
    if (voltage >= 5.0)
        return "C";
    if (voltage >= 2.0)
        return "D";
    return "E";
}

float convertAdcToCpVoltage(float adcVoltage)
{
    return adcVoltage * cpScalingFactor;
}

float readVoltage()
{
    const int samples = 3;
    float sum = 0.0;

    for (int i = 0; i < samples; i++)
    {
        sum += analogRead(ADC_PIN) * (VREF / 4095.0);
        delayMicroseconds(100);
    }

    return sum / samples;
}

void IRAM_ATTR handleInterrupt()
{
    unsigned long currentTime = micros();
    bool currentLevel = digitalRead(FREQUENCY_PIN);

    if (currentLevel != compState)
    {
        if (currentLevel)
        {
            // Rising edge
            lowTime += currentTime - lastEdgeTime;
        }
        else
        {
            // Falling edge
            highTime += currentTime - lastEdgeTime;
            pulseCount++;
        }
        lastEdgeTime = currentTime;
        compState = currentLevel;
    }
}

void initializeHardware()
{
    Serial.begin(115200);
    pinMode(ADC_PIN, INPUT);
    analogSetAttenuation(ADC_11db);
    analogReadResolution(12);

    pinMode(FREQUENCY_PIN, INPUT);
    pinMode(S2_CTRL_PIN, OUTPUT);
    digitalWrite(S2_CTRL_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(FREQUENCY_PIN), handleInterrupt, CHANGE);
}

void updatePeakVoltage()
{
    static float currentPeak = 0.01;
    float voltage = readVoltage();

    if (voltage > currentPeak)
    {
        currentPeak = voltage;
    }

    if (millis() - lastPeakReset >= PEAK_INTERVAL)
    {
        peakVoltage = currentPeak;
        rawPeakVoltage = currentPeak;
        currentPeak = 0.01;
        lastPeakReset = millis();
    }
}

void updateAverageVoltage()
{
    static float voltageSum = 0.0;
    static unsigned int sampleCount = 0;

    if (compState)
    {
        voltageSum += readVoltage();
        sampleCount++;
    }

    if (millis() - lastAverageReset >= AVERAGE_INTERVAL)
    {
        averageVoltage = (sampleCount > 0) ? (voltageSum / sampleCount) : 0.0;
        voltageSum = 0.0;
        sampleCount = 0;
        lastAverageReset = millis();
    }
}

void updateFrequencyAndDutyCycle()
{
    if (millis() - lastFrequencyUpdate >= UPDATE_INTERVAL)
    {
        noInterrupts();
        unsigned long tempPulseCount = pulseCount;
        unsigned long tempHighTime = highTime;
        unsigned long tempLowTime = lowTime;
        pulseCount = 0;
        highTime = 0;
        lowTime = 0;
        interrupts();

        frequency = tempPulseCount / (UPDATE_INTERVAL / 1000.0);

        if (tempHighTime + tempLowTime > 0)
        {
            dutyCycle = (tempHighTime * 100.0) / (tempHighTime + tempLowTime);
        }
        else
        {
            dutyCycle = 0;
        }

        lastFrequencyUpdate = millis();
    }
}

void printDebugInfo(String message)
{
    Serial.println("---------- DEBUG INFO ----------");
    Serial.println(message);
    Serial.print("Raw ADC Peak: ");
    Serial.print(rawPeakVoltage, 3);
    Serial.print("V | Scaled Peak: ");
    Serial.print(convertAdcToCpVoltage(rawPeakVoltage), 2);
    Serial.print("V | Current ADC: ");
    Serial.print(readVoltage(), 3);
    Serial.print("V | Frequency: ");
    Serial.print(frequency, 1);
    Serial.print("Hz | Scaling Factor: ");
    Serial.print(cpScalingFactor, 3);
    Serial.println("\n-------------------------------");
}

void printSystemStatus()
{
    if (millis() - lastStatusPrint >= 2000)
    {
        Serial.print("Freq: ");
        Serial.print(frequency, 1);
        Serial.print("Hz | DC: ");
        Serial.print(dutyCycle, 1);
        Serial.print("% | Raw ADC Peak: ");
        Serial.print(String(rawPeakVoltage, 3));
        Serial.print("V | Scaled Peak: ");
        Serial.print(convertAdcToCpVoltage(peakVoltage), 2);
        Serial.print("V | State: ");
        Serial.print(getCPStatus(convertAdcToCpVoltage(peakVoltage)));
        Serial.print(" | S2: ");
        Serial.print(digitalRead(S2_CTRL_PIN) ? "ON" : "OFF");
        Serial.print(" | cp_factor: ");
        Serial.print(String(cpScalingFactor, 3));
        Serial.print(" | CP_State: ");
        switch (currentState)
        {
        case NO_PLUG:
            Serial.print("NO_PLUG");
            break;
        case PLUG_CONNECTED:
            Serial.print("PLUG_CONNECTED");
            break;
        case VEHICLE_READY:
            Serial.print("VEHICLE_READY");
            break;
        }
        Serial.println(staticVoltageDetected ? " [STATIC]" : "");
        lastStatusPrint = millis();
    }
}

void handleSerialCommands()
{
    if (Serial.available())
    {
        char cmd = Serial.read();
        while (Serial.available() && (Serial.peek() == '\r' || Serial.peek() == '\n'))
        {
            Serial.read();
        }

        switch (cmd)
        {
        case 'c':
        case 'C':
            digitalWrite(S2_CTRL_PIN, HIGH);
            s2State = true;
            currentState = VEHICLE_READY;
            printDebugInfo("Manual S2 Activation");
            adjustScalingFactor(VEHICLE_READY_VOLTAGE);
            break;
        case 'b':
        case 'B':
            digitalWrite(S2_CTRL_PIN, LOW);
            s2State = false;
            if (plugConnected)
                currentState = PLUG_CONNECTED;
            printDebugInfo("Manual S2 Deactivation");
            break;
        case 'r':
        case 'R':
            resetLogic();
            printDebugInfo("Manual Reset");
            break;
        case 'd':
        case 'D':
            printDebugInfo("Debug Info Requested");
            break;
        case 's':
        case 'S':
        {
            if (Serial.available() >= 4)
            {
                String value = Serial.readStringUntil('\n');
                float newScale = value.toFloat();
                if (newScale > 0.1 && newScale < 20.0)
                {
                    cpScalingFactor = newScale;
                    Serial.print("Manually set scaling factor to: ");
                    Serial.println(cpScalingFactor, 3);
                }
            }
            break;
        }
        }
    }
}

void resetLogic()
{
    Serial.println("Resetting Logic");
    digitalWrite(S2_CTRL_PIN, LOW);
    s2State = false;
    plugConnected = false;
    staticVoltageDetected = false;
    cpScalingFactor = CP_SCALING_FACTOR_DEFAULT;
    s2ActivateTime = 0;
    staticVoltageDetectTime = 0;
    invalidReadingCount = 0;
    currentState = NO_PLUG;
}

void adjustScalingFactor(float targetVoltage)
{
    float adcValue = readVoltage();
    Serial.print("Raw ADC Value: ");
    Serial.println(adcValue, 3);

    if (adcValue > ADC_READING_VALIDATION_THRESHOLD && adcValue < ADC_READING_MAX_THRESHOLD)
    {
        const int avgSamples = 10;
        float sum = 0;
        int validSamples = 0;

        for (int i = 0; i < avgSamples; i++)
        {
            float sample = readVoltage();
            if (sample > ADC_READING_VALIDATION_THRESHOLD)
            {
                sum += sample;
                validSamples++;
            }
            delay(20);
        }

        if (validSamples > 0)
        {
            float avgValue = sum / validSamples;
            float newScalingFactor = targetVoltage / avgValue;

            if (newScalingFactor > cpScalingFactor * 2)
                newScalingFactor = cpScalingFactor * 2;
            else if (newScalingFactor < cpScalingFactor / 2)
                newScalingFactor = cpScalingFactor / 2;

            cpScalingFactor = newScalingFactor;

            Serial.print("Scaling Factor set to: ");
            Serial.print(cpScalingFactor, 3);
            Serial.print(" (Target: ");
            Serial.print(targetVoltage, 1);
            Serial.print("V, Avg ADC: ");
            Serial.print(avgValue, 3);
            Serial.println("V)");
        }
        else
        {
            Serial.println("No valid samples obtained for scaling factor adjustment");
        }
    }
    else
    {
        Serial.print("Invalid ADC reading (");
        Serial.print(adcValue, 3);
        Serial.println("V), scaling factor NOT updated!");
    }
}

bool validateReadings(float targetVoltage)
{
    float cpVoltage = convertAdcToCpVoltage(peakVoltage);
    // Select tolerance based on the current state.
    float tolerance = (currentState == VEHICLE_READY) ? CP_VOLTAGE_TOLERANCE_VEHICLE : CP_VOLTAGE_TOLERANCE_PLUG;

    // For static voltage mode, only check voltage.
    if (staticVoltageDetected)
    {
        return (cpVoltage >= targetVoltage - tolerance &&
                cpVoltage <= targetVoltage + tolerance * 2);
    }

    // For PWM mode, check both frequency and voltage.
    return ((frequency >= 900.0 && frequency <= 1100.0) &&
            cpVoltage >= targetVoltage - tolerance &&
            cpVoltage <= targetVoltage + tolerance * 2);
}

bool isPlugConnected()
{
    float rawVoltage = readVoltage();
    float scaledVoltage = convertAdcToCpVoltage(rawVoltage);

    // Modified to be more lenient with frequency detection.
    bool validFrequency = (frequency >= 900.0 && frequency <= 1100.0);
    bool validVoltage = (rawVoltage >= 1.5 && rawVoltage <= ADC_READING_MAX_THRESHOLD);

    // Debug information when a plug is potentially detected.
    if (validVoltage)
    {
        if (validFrequency)
        {
            printDebugInfo("Valid PWM signal detected: " + String(rawVoltage) + "V @ " + String(frequency) + "Hz");
            return true;
        }
        else if (frequency < 10)
        {
            if (staticVoltageDetectTime == 0)
            {
                staticVoltageDetectTime = millis();
                printDebugInfo("Potential static voltage detected, waiting to confirm");
            }
            else if (millis() - staticVoltageDetectTime > STATIC_VOLTAGE_TIMEOUT)
            {
                staticVoltageDetected = true;
                printDebugInfo("Static voltage confirmed after timeout");
                return true;
            }
        }
    }
    else if (frequency < 10)
    {
        staticVoltageDetectTime = 0;
    }

    return (validFrequency && validVoltage) || (staticVoltageDetected && validVoltage);
}

bool isStaticVoltagePresent()
{
    float rawVoltage = readVoltage();
    bool validVoltage = (rawVoltage >= 1.5 && rawVoltage <= ADC_READING_MAX_THRESHOLD);
    bool noFrequency = (frequency < 10.0);

    return validVoltage && noFrequency;
}

void processCP()
{
    float cpVoltage = convertAdcToCpVoltage(peakVoltage);
    float currentRawVoltage = readVoltage();

    // --- NO_PLUG: If scaled voltage is very low and frequency is near zero.
    if (cpVoltage < NO_PLUG_VOLTAGE_THRESHOLD && frequency < 10 && !staticVoltageDetected)
    {
        if (currentState != NO_PLUG)
        {
            printDebugInfo("Transitioning to NO_PLUG state");
            resetLogic();
        }
    }
    // --- Transition from NO_PLUG to PLUG_CONNECTED ---
    else if (currentState == NO_PLUG && (isPlugConnected() || (frequency > 900.0 && frequency < 1100.0)))
    {
        printDebugInfo("Plug Connected, transitioning to PLUG_CONNECTED state");
        plugConnected = true;
        currentState = PLUG_CONNECTED;
        adjustScalingFactor(PLUG_CONNECTED_VOLTAGE);
        s2ActivateTime = millis();
    }
    // --- Transition from PLUG_CONNECTED to VEHICLE_READY ---
    else if (currentState == PLUG_CONNECTED && !s2State && (millis() - s2ActivateTime >= S2_DELAY))
    {
        if ((frequency > 900.0 && frequency < 1100.0) ||
            (staticVoltageDetected && currentRawVoltage >= 1.5))
        {
            printDebugInfo("Auto transitioning to VEHICLE_READY state after delay");
            digitalWrite(S2_CTRL_PIN, HIGH);
            s2State = true;
            currentState = VEHICLE_READY;
            delay(200);
            adjustScalingFactor(VEHICLE_READY_VOLTAGE);
        }
        else
        {
            printDebugInfo("Failed to auto-transition: not a valid signal");
            s2ActivateTime = millis();
        }
    }
    // --- In VEHICLE_READY state: Validate readings remain near 6V ---
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
    // If in PLUG_CONNECTED state, periodically confirm valid voltage.
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

    // Check for static voltage in any state.
    if (!staticVoltageDetected && isStaticVoltagePresent() &&
        (millis() - staticVoltageDetectTime > STATIC_VOLTAGE_TIMEOUT) &&
        currentState != NO_PLUG)
    {
        staticVoltageDetected = true;
        printDebugInfo("Static voltage confirmed (no PWM)");
    }
}

void setup()
{
    initializeHardware();

    // Ensure S2 is OFF at startup.
    digitalWrite(S2_CTRL_PIN, LOW);
    s2State = false;

    Serial.println("\n=== ESP32 EV Charging Controller ===");
    Serial.println("Commands: 'c' - Activate S2, 'b' - Deactivate S2, 'r' - Reset, 'd' - Debug info, 's<float>' - Set scaling");

    delay(1000); // Allow time for stabilization

    lastFrequencyUpdate = millis();
    lastPeakReset = millis();
    lastAverageReset = millis();
    lastStatusPrint = millis();
    lastCPCheck = millis();

    for (int i = 0; i < 5; i++)
    {
        updatePeakVoltage();
        updateFrequencyAndDutyCycle();
        delay(100);
    }

    if (frequency > 900.0 && frequency < 1100.0)
    {
        Serial.println("Plug already connected at startup (PWM detected).");
        plugConnected = true;
        currentState = PLUG_CONNECTED;
        adjustScalingFactor(PLUG_CONNECTED_VOLTAGE);
        s2ActivateTime = millis();
    }
    else if (readVoltage() >= 1.5)
    {
        Serial.println("Potential static voltage detected at startup.");
        staticVoltageDetectTime = millis();
    }
    else
    {
        Serial.println("No plug detected at startup.");
        currentState = NO_PLUG;
    }
}

void loop()
{
    handleSerialCommands();
    updatePeakVoltage();
    updateAverageVoltage();
    updateFrequencyAndDutyCycle();
    printSystemStatus();

    if (currentState == NO_PLUG &&
        frequency > 900.0 && frequency < 1100.0 &&
        peakVoltage > 1.5 && !plugConnected)
    {
        printDebugInfo("Forcing plug detection - valid frequency detected");
        plugConnected = true;
        currentState = PLUG_CONNECTED;
        adjustScalingFactor(PLUG_CONNECTED_VOLTAGE);
        s2ActivateTime = millis();
    }

    if (millis() - lastCPCheck >= UPDATE_INTERVAL)
    {
        processCP();
        lastCPCheck = millis();
    }
}
