#include <Arduino.h>
#include "mcp_can.h"

// Pin Definitions
const uint8_t ADC_PIN = 35;       // ADC input pin
const uint8_t FREQUENCY_PIN = 34; // PWM input pin (1000Hz fixed)
const uint8_t S2_CTRL_PIN = 12;   // Control pin

// Constants
const float VREF = 3.3;                     // Reference voltage (3.3V)
const unsigned long UPDATE_INTERVAL = 1000; // Update interval (ms)
const unsigned long PEAK_INTERVAL = 200;    // Peak measurement interval (ms)

// Global variables
volatile unsigned long pulseCount = 0;
volatile unsigned long highTime = 0;
volatile unsigned long lowTime = 0;
unsigned long lastEdgeTime = 0;
unsigned long lastFrequencyUpdate = 0;
unsigned long lastPeakReset = 0;

float frequency = 0;
float dutyCycle = 0;
float currentPeak = 0.0;
float peakVoltage = 0.0;

// Function prototypes
void initializeHardware();
float readVoltage();
float updatePeakVoltage();
void updateFrequencyAndDutyCycle();
void handleSerialCommands();

// Read voltage from ADC
float readVoltage()
{
    int rawadc = analogRead(ADC_PIN);
    return rawadc * (VREF / 4095.0); // 12-bit ADC
}

// Interrupt handler
void IRAM_ATTR handleInterrupt()
{
    unsigned long currentTime = micros();
    bool currentLevel = digitalRead(FREQUENCY_PIN);

    if (currentLevel)
    {
        lowTime += currentTime - lastEdgeTime;
    }
    else
    {
        highTime += currentTime - lastEdgeTime;
        pulseCount++;
    }

    lastEdgeTime = currentTime;
}

void initializeHardware()
{
    Serial.begin(115200);

    // Configure ADC
    pinMode(ADC_PIN, INPUT);
    analogSetAttenuation(ADC_11db);

    // Configure PWM input and control pins
    pinMode(FREQUENCY_PIN, INPUT);
    pinMode(S2_CTRL_PIN, OUTPUT);
    digitalWrite(S2_CTRL_PIN, LOW); // Initialize to LOW

    attachInterrupt(digitalPinToInterrupt(FREQUENCY_PIN), handleInterrupt, CHANGE);

    // Initialize timers
    lastFrequencyUpdate = millis();
    lastPeakReset = millis();
}

// Peak voltage measurement
float updatePeakVoltage()
{
    float voltage = readVoltage();
    if (voltage > currentPeak)
    {
        currentPeak = voltage;
    }

    unsigned long currentTime = millis();
    if (currentTime - lastPeakReset >= PEAK_INTERVAL)
    {
        peakVoltage = currentPeak;
        currentPeak = 0.0;
        lastPeakReset = currentTime;

        Serial.print(F("Peak Voltage: "));
        Serial.print(peakVoltage, 2);
        Serial.println(F("V"));
    }

    return peakVoltage;
}

// Frequency and duty cycle calculation
void updateFrequencyAndDutyCycle()
{
    unsigned long currentTime = millis();
    if (currentTime - lastFrequencyUpdate >= UPDATE_INTERVAL)
    {
        frequency = (pulseCount * (1000.0 / UPDATE_INTERVAL)); // Scale frequency

        if (highTime + lowTime > 0)
        {
            dutyCycle = (static_cast<float>(highTime) / (highTime + lowTime)) * 100.0;
        }
        else
        {
            dutyCycle = 0;
        }

        // Reset counters
        pulseCount = 0;
        highTime = lowTime = 0;
        lastFrequencyUpdate = currentTime;

        Serial.print(F("Frequency: "));
        Serial.print(frequency);
        Serial.print(F(" Hz, Duty Cycle: "));
        Serial.print(dutyCycle, 1);
        Serial.println(F("%"));
    }
}

// Handle serial commands
void handleSerialCommands()
{
    if (Serial.available() > 0)
    {
        char cmd = Serial.read();
        switch (cmd)
        {
        case 'c':
            digitalWrite(S2_CTRL_PIN, HIGH);
            Serial.println(F("Control: HIGH"));
            break;
        case 'b':
            digitalWrite(S2_CTRL_PIN, LOW);
            Serial.println(F("Control: LOW"));
            break;
        default:
            break;
        }
    }
}

void setup()
{
    initializeHardware();
}

void loop()
{
    handleSerialCommands();
    updatePeakVoltage();
    updateFrequencyAndDutyCycle();
}
