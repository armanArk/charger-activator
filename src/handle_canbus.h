// Tambahkan deklarasi di awal file, setelah includes

// --- CAN Bus Functions ---
void sendChargerCommand(float voltage, float current, bool startCharging)
{
    // Use provided parameters or fall back to global variables
    float commandVoltage = voltage > 0 ? voltage : targetVoltage;
    float commandCurrent = current > 0 ? current : targetCurrent;

    // Create CAN message to control the charger (Report 1)
    byte data[8] = {0};

    // Scale and convert voltage (0.1V per bit)
    uint16_t voltageVal = commandVoltage * 10;
    data[0] = highByte(voltageVal); // High byte of voltage
    data[1] = lowByte(voltageVal);  // Low byte of voltage

    // Scale and convert current (0.1A per bit)
    uint16_t currentVal = commandCurrent * 10;
    data[2] = highByte(currentVal); // High byte of current
    data[3] = lowByte(currentVal);  // Low byte of current

    // Control byte: 0x00 = start charging, 0x01 = stop
    data[4] = startCharging ? 0x00 : 0x01;

    // Bytes 5-7: Reserved (data[5] reserved for heating contactor)
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;

    // Send CAN message
    byte sndStat = CAN.sendMsgBuf(CHARGER_CONTROL_ID, 1, 8, data);
    if (sndStat == CAN_OK)
    {
        Serial.println("Sent CAN command. OK, current: " + String(commandCurrent) + ", voltage:" + String(batteryVoltage));
    }
    else
    {
        Serial.println("Sent CAN command. FAILED, current: " + String(commandCurrent) + ", voltage:" + String(batteryVoltage));
    }
}

void stopChargerCommand()
{
    return;
    byte data[8] = {0};
    data[4] = 0x01; // Control byte: 0x01 = stop charging
                    // Send CAN message
    byte sndStat = CAN.sendMsgBuf(CHARGER_CONTROL_ID, 1, 8, data);

    if (sndStat == CAN_OK)
    {
        Serial.println("Stop command sent. Charging stopped");
        cutoffTriggered = true; // Set cutoff flag
        chargingStatusText = "Stopped";
    }
    else
    {
        Serial.println("Stop command failed");
    }
}

// Tambahkan variabel global untuk melacak waktu
unsigned long lastCanReceiveTime = 0;
const unsigned long timeoutReceived = 15000; // Contoh: 5000 ms (5 detik)

// Function to process received CAN messages
void handleReceivingCanbus()
{
    return;
    if (digitalRead(CAN_INT) == LOW)
    { // Check for CAN interrupt
        unsigned long id;
        byte len;
        byte msgData[8];

        if (CAN.readMsgBuf(&id, &len, msgData) == CAN_OK)
        {
            // Update waktu terakhir pesan CAN diterima
            lastCanReceiveTime = millis();

            // Update CAN display data
            webCANId = "0x" + String(id, HEX);
            webCANData = "";
            for (byte i = 0; i < len; i++)
            {
                if (msgData[i] < 0x10)
                    webCANData += "0";
                webCANData += String(msgData[i], HEX) + " ";
            }
            Serial.print("canId:");
            Serial.println(webCANId);
            Serial.print("canData:");
            Serial.println(webCANData);

            if (len == 8)
            {
                // Put here the message processing
                // if id is equal to
                decodeChargerBroadcast(msgData, len);
            }
        }
    }

    // Check for timeout
    if (millis() - lastCanReceiveTime > timeoutReceived)
    {
        communicationTimeout = true;
        // if (cpModeEnabled)
        // {
        //     vehicleReadyStateClear();
        // }
    }
    else
    {
        communicationTimeout = false;
    }
}

void simulateCanbus(float _batteryVoltage, float _batteryCurrent, bool _state)
{
    // Set charging status text
    if (_state)
    {
        chargingStatusText = "Charging";
        chargerActive = true;
        // Start Cutoff verification
        if (_batteryCurrent < cutoffCurrent)
        {
            // Start the timer
            if (!checkingCutoff)
            {
                cutoffStartTime = millis();
                checkingCutoff = true;
            }
            else if ((millis() - cutoffStartTime) >= delayCutoff)
            { // If low current last more than delay, stop charging
                stopCharger();
                cutoffTriggered = true;
                chargingStatusText = "CutOff";
            }
        }
        else
        { // Reset the cutoff
            checkingCutoff = false;
        }
    }
    else
    {
        chargingStatusText = "Stopped";
    }

    // Create CAN message, state: 1 charging, 0: stop
    byte data[8] = {0};
    uint16_t voltageVal = _batteryVoltage * 10; // scale the voltage
    data[0] = highByte(voltageVal);
    data[1] = lowByte(voltageVal);
    uint16_t currentVal = _batteryCurrent * 10; // scale the current
    data[2] = highByte(currentVal);
    data[3] = lowByte(currentVal);
    // byte 4 and 5 as status flag, 0 for charge, 1 for discharge.
    if (_state)
    {
        data[4] = 0x00;
        data[5] = 0x00;
    }
    else
    {
        data[4] = 0x01;
        data[5] = 0x01;
    }
    data[6] = 0x00;
    data[7] = 0x00;

    byte sndStat = CAN.sendMsgBuf(0x98FF50E5, 1, 8, data);
    if (sndStat == CAN_OK)
    {
        Serial.print("Sent Simulate CAN message. Voltage: ");
        Serial.print(_batteryVoltage);
        Serial.print("V, Current: ");
        Serial.print(_batteryCurrent);
        Serial.print("A. State: ");
        Serial.print(_state);
    }
    else
    {
        Serial.println("Error sending Simulate CAN message");
    }
}

void decodeChargerBroadcast(byte msgData[], byte len)
{

    // --- Decode Voltage ---
    uint16_t voltageRaw = (msgData[0] << 8) | msgData[1];
    batteryVoltage = voltageRaw * 0.1f; // Apply scaling

    // --- Decode Current ---
    uint16_t currentRaw = (msgData[2] << 8) | msgData[3];
    batteryCurrent = currentRaw * 0.1f; // Apply scaling

    // --- Parse and Interpret the STATUS Flag (Byte 5) ---
    byte statusByte = msgData[4];
    hardwareFailure = (statusByte >> 0) & 0x01;
    chargerTemp = (statusByte >> 1) & 0x01;
    inputVoltage = (statusByte >> 2) & 0x01;
    startState = (statusByte >> 3) & 0x01;
    communicationTimeout = (statusByte >> 4) & 0x01;

    if (!startState)
    {
        chargingObcState = true;
        chargingStatusText = "Charging";
    }
    else
    {
        chargingObcState = false;
        chargingStatusText = "Stopped";
        checkingCutoff = false;
    }
    printDecodeDataObc();
}

void printDecodeDataObc()
{
    Serial.println();
    Serial.println("=================================================");
    Serial.print("Battery Voltage: ");
    Serial.print(batteryVoltage);
    Serial.println(" V");

    Serial.print("Battery Current: ");
    Serial.print(batteryCurrent);
    Serial.println(" A");

    Serial.print("Hardware Failure: ");
    Serial.println(hardwareFailure ? "Yes" : "No");

    Serial.print("Charger Overtemperature: ");
    Serial.println(chargerTemp ? "Yes" : "No");

    Serial.print("Input Voltage Issue: ");
    Serial.println(inputVoltage ? "Yes" : "No");

    Serial.print("Start State: ");
    Serial.println(startState ? "Stopped" : "Charging");

    Serial.print("Communication Timeout: ");
    Serial.println(communicationTimeout ? "Yes" : "No");

    Serial.print("Charging Status: ");
    Serial.println(chargingStatusText);
    Serial.println("=================================================");
    Serial.println();
}
