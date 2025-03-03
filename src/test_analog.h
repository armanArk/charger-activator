#include <Arduino.h>

// Change this to test a different ADC pin
const uint8_t ADC_PIN = 35; // Example: Change to 32, 33, 36, etc.

const float VREF = 3.3; // ESP32 Reference Voltage (3.3V)

void setup()
{
    Serial.begin(115200);
    pinMode(ADC_PIN, INPUT);

    // Set ADC resolution and range
    analogSetWidth(12);             // 12-bit resolution (0-4095)
    analogSetAttenuation(ADC_11db); // 3.3V range

    Serial.println("ADC Debugging Started...");
}

void loop()
{
    int rawValue = analogRead(ADC_PIN);
    float voltage = rawValue * (VREF / 4095.0); // Convert to voltage

    // Serial.print("ADC Pin: GPIO");
    // Serial.print(ADC_PIN);
    // Serial.print(" | Raw Value: ");
    // Serial.print(rawValue);
    Serial.print(" | Voltage: ");
    Serial.print(voltage, 2);
    Serial.println(" V");

    // Error detection
    // if (voltage < 0.1)
    // {
    //     Serial.println("⚠️ Warning: Low voltage detected!");
    // }
    // else if (voltage > 3.2)
    // {
    //     Serial.println("⚠️ Warning: High voltage detected!");
    // }
}
