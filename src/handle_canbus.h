void handleReceivingCanbus()
{
    if (digitalRead(CAN_INT) == LOW)
    {
        unsigned long id;
        byte len;
        byte msgData[8];
        if (CAN.readMsgBuf(&id, &len, msgData) == CAN_OK)
        {
            // Update CAN display data
            webCANId = "0x" + String(id, HEX);
            webCANData = "";
            for (byte i = 0; i < len; i++)
            {
                if (msgData[i] < 0x10)
                    webCANData += "0";
                webCANData += String(msgData[i], HEX) + " ";
            }

            // Update battery data if message length is 8
            if (len == 8)
            {

                // Update charger status based on CAN data
                decodeChargerBroadcast(msgData, len);
            }
        }
    }
}

void sendChargerCommand()
{
    uint16_t voltageVal = targetVoltage * 10;
    byte voltage_high = (voltageVal >> 8) & 0xFF;
    byte voltage_low = voltageVal & 0xFF;
    uint16_t currentVal = targetCurrent * 10;
    byte current_high = (currentVal >> 8) & 0xFF;
    byte current_low = currentVal & 0xFF;
    byte data[8] = {
        voltage_high,
        voltage_low,
        current_high,
        current_low,
        0x00, // Control byte: 0x00 to activate charging
        0x00, // Mode selector (if needed)
        0x00, // Reserved
        0x00  // Reserved
    };
    Serial.print("SEND COMMAND: ");
    Serial.print(targetVoltage, 1);
    Serial.print("V, ");
    Serial.print(targetCurrent, 1);
    Serial.println("A");
    if (CAN.sendMsgBuf(CHARGER_CONTROL_ID, 1, 8, data) == CAN_OK)
    {
        Serial.print("Send command: ");
        Serial.print(targetVoltage, 1);
        Serial.print("V, ");
        Serial.print(targetCurrent, 1);
        Serial.println("A");
    }
    else
    {
        Serial.println("Send failed");
    }
}

void stopChargerCommand()
{
    byte data[8] = {
        0x00,
        0x00,
        0x00,
        0x00,
        0x01, // Control byte: 0x01 indicates stop charging
        0x00,
        0x00,
        0x00};

    if (CAN.sendMsgBuf(CHARGER_CONTROL_ID, 1, 8, data) == CAN_OK)
    {
        Serial.println("Stop command sent, charging stopped");
    }
    else
    {
        Serial.println("Stop command failed");
    }
}

void simulateCanbus(float _batteryVoltage, float _batteryCurrent, bool _state)
{
    Serial.print("simulate_canbus:");
    Serial.println(String(_batteryVoltage, 1) + "," + String(_batteryCurrent, 1) + "," + String(_state));
    batteryVoltage = _batteryVoltage;
    batteryCurrent = _batteryCurrent;

    // Parse charging state from byte 4
    uint8_t state = _state;

    // Update charger status based on CAN data

    if (state == 0)
    {
        // Charger is active
        chargerActive = true;
        chargingStatusText = "Charging";
    }
    else
    {
        // Charger is stopped
        chargerActive = false;
        chargingStatusText = "Stopped";
        checkingCutoff = false; // Reset cutoff checking
    }

    Serial.println("-------------------------------------");
    Serial.print("State from CAN: ");
    Serial.println(state);
    Serial.print("Charger Active: ");
    Serial.println(chargerActive ? "Yes" : "No");
    Serial.print("Voltage: ");
    Serial.print(batteryVoltage);
    Serial.println(" V");
    Serial.print("Current: ");
    Serial.print(batteryCurrent);
    Serial.println(" A");
    Serial.println("-------------------------------------");
}

// --- Function to decode the charger broadcast message (Report 2) ---
void decodeChargerBroadcast(byte msgData[], byte len)
{
    if (len != 8)
    {
        Serial.println("Error: Charger broadcast message has incorrect length.");
        return; // Exit if the message length is wrong
    }

    // --- Decode Voltage ---
    uint16_t voltageRaw = (msgData[0] << 8) | msgData[1];
    batteryVoltage = voltageRaw * 0.1f;

    // --- Decode Current ---
    uint16_t currentRaw = (msgData[2] << 8) | msgData[3];
    batteryCurrent = currentRaw * 0.1f;

    // --- Decode Status Flag (Byte 5) ---
    byte statusByte = msgData[4];
    bool hardwareFailure = (statusByte >> 0) & 0x01;
    bool chargerTemp = (statusByte >> 1) & 0x01;
    bool inputVoltage = (statusByte >> 2) & 0x01;
    bool startState = (statusByte >> 3) & 0x01;
    bool communicationTimeout = (statusByte >> 4) & 0x01;

    // --- Update chargerActive and chargingStatusText ---
    if (!startState && !communicationTimeout)
    {
        chargerActive = true;
        chargingStatusText = "Charging";
    }
    else
    {
        chargerActive = false;
        chargingStatusText = "Stopped";
        if (hardwareFailure)
        {
            chargingStatusText += " (Hardware Failure)";
        }
        if (chargerTemp)
        {
            chargingStatusText += " (Overtemperature)";
        }
        if (inputVoltage)
        {
            chargingStatusText += " (Input Voltage Problem)";
        }
        if (communicationTimeout)
        {
            chargingStatusText += " (Comm Timeout)";
        }
        checkingCutoff = false; // Reset if you're using this
    }

    // --- Print Diagnostic Information (Optional, but very helpful) ---
    Serial.println("------ OBC RESPOND ------");
    Serial.print("Voltage: ");
    Serial.print(batteryVoltage);
    Serial.println(" V");
    Serial.print("Current: ");
    Serial.print(batteryCurrent);
    Serial.println(" A");
    Serial.print("Hardware Failure: ");
    Serial.println(hardwareFailure ? "Yes" : "No");
    Serial.print("Charger Temp: ");
    Serial.println(chargerTemp ? "Overtemp" : "Normal");
    Serial.print("Input Voltage: ");
    Serial.println(inputVoltage ? "Problem" : "Normal");
    Serial.print("Start State: ");
    Serial.println(startState ? "Off" : "Starting/On");
    Serial.print("Comm Timeout: ");
    Serial.println(communicationTimeout ? "Yes" : "No");
    Serial.print("Charger Active: ");
    Serial.println(chargerActive ? "Yes" : "No");
    Serial.print("Status: ");
    Serial.println(chargingStatusText);
    Serial.println("--------------------------------------------");
}