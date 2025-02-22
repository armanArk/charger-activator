// Pindahkan fungsi ini ke bagian atas file, sebelum fungsi lainnya

float getBatteryVoltage()
{
    float _batteryVoltage = batteryVoltage;
    if (_batteryVoltage <= 10)
    {
        _batteryVoltage = 400;
    }
    return _batteryVoltage;
}
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
    return 22; // Default if below 10% duty cycle
}
float getMaxCurrentForObc()
{
    float maxCurrent = getMaxCurrent(dutyCycle);
    float watt = maxCurrent * 220;
    float _batteryVoltage = getBatteryVoltage();
    return watt / _batteryVoltage;
}
String getCPStatus(float voltage)
{
    if (voltage >= 11.0)
        return "A"; // Standby
    if (voltage >= 8.0 && voltage <= 10.0)
        return "B"; // Vehicle detected
    if (voltage >= 5.0 && voltage <= 7.0)
        return "C"; // Ready (charging)
    if (voltage >= 2.0 && voltage <= 4.0)
        return "D"; // With ventilation
    if (voltage >= -1.0 && voltage <= 1.0)
        return "E"; // No power (shut off)

    return "E";
}
// Convert ADC voltage to real CP voltage
float convertAdcToCpVoltage(float adcVoltage)
{
    return adcVoltage * CP_SCALING_FACTOR;
}

// Read ADC voltage
float readVoltage()
{
    int rawadc = analogRead(ADC_PIN);
    return rawadc * (VREF / 4095.0); // 12-bit ADC conversion
}

// Interrupt handler for frequency measurement
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

// Initialize hardware
void resetCP()
{
    // Reset all CP-related variables
    pulseCount = 0;
    highTime = 0;
    lowTime = 0;
    frequency = 0;
    dutyCycle = 0;
    currentPeak = 0.01;
    peakVoltage = 0.01;

    // Reset physical pin
    digitalWrite(S2_CTRL_PIN, LOW);
}

void initializeCP()
{
    pinMode(ADC_PIN, INPUT);
    analogSetAttenuation(ADC_11db);
    analogReadResolution(12);

    pinMode(FREQUENCY_PIN, INPUT);
    pinMode(S2_CTRL_PIN, OUTPUT);
    digitalWrite(S2_CTRL_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(FREQUENCY_PIN), handleInterrupt, CHANGE);

    lastFrequencyUpdate = millis();
    lastPeakReset = millis();

    resetCP(); // Reset CP state on initialization
}

// Update and reset peak voltage
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
        currentPeak = 0.01; // Avoid zero reset issues
        lastPeakReset = currentTime;

        Serial.print(F("Peak Analog Voltage: "));
        Serial.println(peakVoltage, 2);
        // Serial.print(F("V | Converted CP Voltage: "));
        // Serial.print(convertAdcToCpVoltage(peakVoltage), 2);
        // Serial.println(F("V"));
    }

    return peakVoltage;
}

// Update frequency and duty cycle calculations
void updateFrequencyAndDutyCycle()
{
    unsigned long currentTime = millis();
    if (currentTime - lastFrequencyUpdate >= UPDATE_INTERVAL)
    {
        frequency = pulseCount / (UPDATE_INTERVAL / 1000.0); // Correct frequency scaling

        if (highTime + lowTime > 0)
        {
            dutyCycle = (static_cast<float>(highTime) / (highTime + lowTime)) * 100.0;
        }
        else
        {
            dutyCycle = 0;
        }

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