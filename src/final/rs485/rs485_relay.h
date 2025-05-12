
void RelaySwitchOnly(int channel, bool state) {
    // choose register value
    uint16_t regValue = state ? 0x0100 : 0x0200;

    // perform the Modbus write right away
    modbus.setSlaveId(1);
    int16_t result = modbus.writeSingleRegister(channel, regValue);

    // print debug info without String allocations
    Serial.print("relaySwitch ch:");    Serial.print(channel);
    Serial.print(", state:");           Serial.print(state);
    Serial.print(", reg:0x");           Serial.print(regValue, HEX);
    Serial.print(" → ");

    if (result == 0) {
        Serial.println("OK");
    } else {
        Serial.print("Error "); Serial.println(result);
    }
}

void relaySwitchAll(bool state) {
    // choose register value: 0x0700 to turn all on, 0x0800 to turn all off
    uint16_t regValue = state ? 0x0700 : 0x0800;

    // perform the Modbus write immediately
    modbus.setSlaveId(1);
    int16_t result = modbus.writeSingleRegister(0x00, regValue);

    // debug output
    if (result == 0) {
        Serial.println("Relay operation successful");
    } else {
        Serial.print("Modbus Error: ");
        Serial.println(result);
    }
}
