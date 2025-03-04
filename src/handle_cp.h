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
        Serial.print(staticVoltageDetected ? " [STATIC]" : "");
        Serial.print(" | cutoffTrg:");
        Serial.println(cutoffTriggered);
        lastStatusPrint = millis();
    }
}
void cpDetectStartup()
{
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
void printDebugInfo(String message)
{
    Serial.println("---------- DEBUG INFO ----------");
    Serial.println(message);
    Serial.print("Raw ADC Peak: ");
    Serial.print(rawPeakVoltage, 3);
    Serial.print("V | Scaled Peak: ");
    Serial.print(convertAdcToCpVoltage(rawPeakVoltage), 2);
    Serial.print("V | Current ADC: ");
    Serial.print(lastAdcReading, 3);
    Serial.print("V | Frequency: ");
    Serial.print(frequency, 1);
    Serial.print("Hz | Scaling Factor: ");
    Serial.print(cpScalingFactor, 3);
    Serial.println("\n-------------------------------");
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
    calibrationDone = false;
}
void resetLogicCutoff()
{
    Serial.println("Resetting Logic resetLogicCutof");
    digitalWrite(S2_CTRL_PIN, LOW);
    s2State = false;
    staticVoltageDetected = false;
    s2ActivateTime = 0;
    staticVoltageDetectTime = 0;
    invalidReadingCount = 0;
    calibrationDone = false;
}
//
// In adjustScalingFactor(), we now use the stored lastAdcReading (the last finished ADC reading)
// for calibration instead of initiating a new ADC conversion immediately.
//
void adjustScalingFactor(float targetVoltage)
{
    // Use the current rawPeakVoltage as the ADC reading for calibration.
    float adcValue = rawPeakVoltage;
    Serial.print("Using Raw ADC Peak: ");
    Serial.println(adcValue, 3);

    if (adcValue > ADC_READING_VALIDATION_THRESHOLD && adcValue < ADC_READING_MAX_THRESHOLD)
    {
        // Calculate new scaling factor based on the target voltage and ADC reading.
        float newScalingFactor = targetVoltage / adcValue;

        // Optionally clamp the new scaling factor to an absolute range.
        const float MIN_SCALING_FACTOR = 1.0;
        const float MAX_SCALING_FACTOR = 20.0;
        if (newScalingFactor < MIN_SCALING_FACTOR)
            newScalingFactor = MIN_SCALING_FACTOR;
        if (newScalingFactor > MAX_SCALING_FACTOR)
            newScalingFactor = MAX_SCALING_FACTOR;

        cpScalingFactor = newScalingFactor;
        Serial.print("Scaling Factor set to: ");
        Serial.print(cpScalingFactor, 3);
        Serial.print(" (Target: ");
        Serial.print(targetVoltage, 1);
        Serial.print("V, ADC: ");
        Serial.print(adcValue, 3);
        Serial.println("V)");
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
    return adcVoltage * cpScalingFactor;
}

float readVoltage()
{
    lastAdcReading = analogRead(ADC_PIN) * (VREF / 4095.0);
    return lastAdcReading;
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