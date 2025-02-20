#include <Arduino.h>
#include "mcp_can.h"
const int pwmPin = 22;         // Use GPIO23 for PWM output
const int pwmFrequency = 1000; // 1 kHz PWM frequency
const int pwmResolution = 8;   // 8-bit resolution (0-255)

#define DUTY_A 26  // 10% of 255
#define DUTY_B 51  // 20% of 255
#define DUTY_C 77  // 30% of 255
#define DUTY_D 102 // 40% of 255
#define DUTY_E 128 // 50% of 255
#define DUTY_F 153 // 60% of 255
#define DUTY_G 179 // 70% of 255
#define DUTY_H 204 // 80% of 255
#define DUTY_I 230 // 90% of 255
#define DUTY_J 255 // 100% of 255

const int ledc_channel = 0;
const int ledc_timer = 0; // Timer number (0-3)
const int ledc_pin = pwmPin;

void setup()
{
    Serial.begin(115200); // Initialize serial communication

    // LEDC configuration for PWM

    // Set up the LEDC timer and channel
    ledcSetup(ledc_channel, pwmFrequency, pwmResolution); // Channel 0, 1kHz, 8-bit
    ledcAttachPin(ledc_pin, ledc_channel);                // Attach GPIO23 to channel 0

    // Initialize duty cycle
    ledcWrite(ledc_channel, DUTY_C); // Set initial duty cycle to 50%
}

void loop()
{
    // Check for serial input
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n'); // Read serial input
        int dutyCycle = input.toInt();               // Convert to integer

        // Validate duty cycle range
        if (dutyCycle >= 0 && dutyCycle <= 255)
        {
            ledcWrite(0, dutyCycle); // Set PWM duty cycle
            Serial.print("Duty Cycle Set: ");
            Serial.println(dutyCycle);
        }
        else
        {
            Serial.println("Error: Duty cycle must be 0-255");
        }
    }
    // int duty_levels[] = {DUTY_A, DUTY_B, DUTY_C, DUTY_D, DUTY_E, DUTY_F, DUTY_G, DUTY_H, DUTY_I, DUTY_J};

    // for (int i = 0; i < 10; i++)
    // {
    //     ledcWrite(ledc_channel, duty_levels[i]); // Set PWM duty cycle
    //     delay(3000);                             // Wait 3 seconds before next level
    // }
}
