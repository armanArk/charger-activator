
#define MODBUS_RX_PIN 27 // RX2
#define MODBUS_TX_PIN 26 // TX2
#define MAX485_RE_NEG 25  // D4
bool readLuxFlag, readTempFlag;
bool stateToogle;

void modbusPreTransmission()
{
  digitalWrite(MAX485_RE_NEG, HIGH); // Switch to transmit data
}

void modbusPostTransmission()
{
  digitalWrite(MAX485_RE_NEG, LOW); // Switch to receive data
}

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


void modbusSetup()
{
  Serial.println("modbusSetup");
  pinMode(MAX485_RE_NEG, OUTPUT);
  digitalWrite(MAX485_RE_NEG, LOW);
  Serial2.begin(9600, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
  modbus.begin(0, Serial2); // Slave ID 0
  modbus.preTransmission(modbusPreTransmission);
  modbus.postTransmission(modbusPostTransmission);
  delay(500);
}

bool getResultMsg(ModbusMaster *node, uint8_t result)
{
  String tmpstr2 = "\r\n";
  switch (result)
  {
  case node->ku8MBSuccess:
    return true;
    break;

  case node->ku8MBIllegalFunction:
    tmpstr2 += "Illegal Function";
    tmpstr2 += " slave " + String(modbus.getSlaveId());
    break;

  case node->ku8MBIllegalDataAddress:
    tmpstr2 += "Illegal Data Address";
    tmpstr2 += " slave " + String(modbus.getSlaveId());
    break;

  case node->ku8MBIllegalDataValue:
    tmpstr2 += "Illegal Data Value";
    tmpstr2 += " slave " + String(modbus.getSlaveId());
    break;

  case node->ku8MBSlaveDeviceFailure:
    tmpstr2 += "Slave Device Failure";
    tmpstr2 += " slave " + String(modbus.getSlaveId());
    break;

  case node->ku8MBInvalidSlaveID:
    tmpstr2 += "Invalid Slave ID";
    tmpstr2 += " slave " + String(modbus.getSlaveId());
    break;

  case node->ku8MBInvalidFunction:
    tmpstr2 += "Invalid Function";
    tmpstr2 += " slave " + String(modbus.getSlaveId());
    break;

  case node->ku8MBResponseTimedOut:
    tmpstr2 += "Response Timed Out";
    tmpstr2 += " slave " + String(modbus.getSlaveId());
    break;

  case node->ku8MBInvalidCRC:
    tmpstr2 += "Invalid CRC";
    tmpstr2 += " slave " + String(modbus.getSlaveId());
    break;

  default:
    tmpstr2 += "Unknown error: " + String(result);
    tmpstr2 += " slave " + String(modbus.getSlaveId());
    break;
  }
  Serial.println(tmpstr2);
  return false;
}
