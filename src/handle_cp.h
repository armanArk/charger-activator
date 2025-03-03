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
    float correction = -0.1; // Adjust this value as needed
    if (dutyCycle >= 10.0 && dutyCycle <= 85.0)
    {
        if (dutyCycle <= 64.0) // First formula (up to 38.4A)
            return (dutyCycle * 0.6) + correction;
        else // Adjusted second formula to ensure continuity
            return (38.4 + (dutyCycle - 64.0) * 2.5) + correction;
    }
    return 22; // Default case
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
    if (voltage >= 11.1)
        return "A"; // Standby
    if (voltage >= 7.6 && voltage <= 11.0)
        return "B"; // Vehicle detected
    if (voltage >= 5.5 && voltage <= 7.5)
        return "C"; // Ready (charging)
    if (voltage >= 2.0 && voltage <= 5.4)
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

        // Serial.print(F("Peak Analog Voltage: "));
        // Serial.println(peakVoltage, 2);
        // Serial.print(F("V | Converted CP Voltage: "));
        // Serial.print(convertAdcToCpVoltage(peakVoltage), 2);
        // Serial.println(F("V"));
    }

    return peakVoltage;
}
// Modified function: Calculate average voltage only when pulse is HIGH
float updateAverageVoltage()
{
    static float voltageSum = 0.0;
    static unsigned long sampleCount = 0;

    // Baca nilai ADC
    float voltage = readVoltage();
    // Akumulasi hanya jika kondisi pulse HIGH
    if (digitalRead(FREQUENCY_PIN) == HIGH)
    {
        voltageSum += voltage;
        sampleCount++;
    }

    unsigned long currentTime = millis();
    if (currentTime - lastPeakReset >= PEAK_INTERVAL)
    {
        // Hitung nilai rata-rata
        float averageVoltage = (sampleCount > 0) ? (voltageSum / sampleCount) : 0.0;
        // Simpan hasil rata-rata ke variabel global (digunakan pada fungsi lain jika diperlukan)
        peakVoltage = averageVoltage;

        // Reset nilai untuk periode selanjutnya
        voltageSum = 0.0;
        lastPeakReset = currentTime;

        Serial.print(F("sampleCount:"));
        Serial.print(String(sampleCount));
        sampleCount = 0;
        Serial.print(F(" | Average "));
        Serial.print(PEAK_INTERVAL);
        Serial.print(F(" ms: "));
        Serial.print(averageVoltage, 2);
        Serial.print(F(" V | Converted CP Voltage: "));
        Serial.print(convertAdcToCpVoltage(averageVoltage), 2);
        Serial.println(F(" V"));
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
        Serial.print(F("freq: "));
        Serial.print(frequency);
        Serial.print(F(" Hz | Duty: "));
        Serial.print(dutyCycle, 1);
        Serial.print(F("%"));
        Serial.print(" | Peak: ");
        Serial.print(String(peakVoltage, 2));
        Serial.print("V | CP_V: ");
        Serial.print(convertAdcToCpVoltage(peakVoltage), 1);
        Serial.print("V | State: ");
        Serial.print(getCPStatus(cpVoltage));
        Serial.print(" | Max CC: ");
        Serial.print(getMaxCurrent(dutyCycle));
        Serial.println("A");
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