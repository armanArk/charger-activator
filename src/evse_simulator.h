#include <Arduino.h>
#include "mcp_can.h" // Include if using MCP_CAN; remove if not needed

// PWM configuration parameters
const int pwmPin = 22;                // PWM output pin
const int desiredPwmFrequency = 1000; // Desired PWM frequency: 1 kHz
const int pwmResolution = 10;         // 10-bit resolution (0-1023)
                                      // You can experiment with 8, 10, or 12 bits

const int ledc_channel = 0; // LEDC channel number
const int ledc_pin = pwmPin;

// Helper function to set up PWM and report actual frequency
void setupPWM(uint8_t channel, uint32_t frequency, uint8_t resolution)
{
    // ledcSetup returns the divider value used by the LEDC timer.
    uint32_t divider = ledcSetup(channel, frequency, resolution);

    // The LEDC base clock is typically 80 MHz on ESP32.
    // The actual frequency is calculated as:
    // actualFrequency = LEDC_BASE_CLK / (divider * 2^resolution)
    double baseClock = 80000000.0;
    double actualFrequency = baseClock / (divider * (1UL << resolution));

    Serial.print("Desired frequency: ");
    Serial.print(frequency);
    Serial.print(" Hz, Resolution: ");
    Serial.print(resolution);
    Serial.print(" bits, Divider: ");
    Serial.print(divider);
    Serial.print(", Actual frequency: ");
    Serial.print(actualFrequency, 2);
    Serial.println(" Hz");
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Wait for serial port to connect. Needed for native USB
    }

    // Set up PWM and print out the actual frequency.
    setupPWM(ledc_channel, desiredPwmFrequency, pwmResolution);
    ledcAttachPin(ledc_pin, ledc_channel);

    // Set an initial duty cycle.
    // For 10-bit resolution, 50% duty cycle is about 512.
    ledcWrite(ledc_channel, 512);
    Serial.println("PWM initialized.");
}

void loop()
{
    // This example lets you change the duty cycle via Serial input.
    // Enter a number between 0 and (2^pwmResolution - 1) (for 10-bit: 0 to 1023).
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        int dutyCycle = input.toInt();

        if (dutyCycle >= 0 && dutyCycle < (1 << pwmResolution))
        {
            ledcWrite(ledc_channel, dutyCycle);
            Serial.print("Duty Cycle Set: ");
            Serial.println(dutyCycle);
        }
        else
        {
            Serial.print("Error: Duty cycle must be between 0 and ");
            Serial.println((1 << pwmResolution) - 1);
        }
    }
}
