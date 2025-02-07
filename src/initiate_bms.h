#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS 5     // Chip Select pin
#define CAN_INT 4    // Interrupt pin
MCP_CAN CAN(CAN_CS); // CAN controller instance

// 32-cell LiFePO4 parameters
const float CELL_VOLTAGE = 3.6;                     // Max per cell voltage
const int NUM_CELLS = 32;                           // 32S configuration
const float MAX_VOLTAGE = NUM_CELLS * CELL_VOLTAGE; // 108.8V (32 * 3.4)
const float MAX_CURRENT = 3.2;                      // Configurable current limit
unsigned long pmillis;
const int TEMP_COMPENSATION = -40; // From protocol docs
bool chargingStatus = false;

void parseBMSMessage(uint8_t data[8])
{
    // Parse voltage from bytes 0-1 (little-endian format)
    uint16_t voltageRaw = (data[1] << 8) | data[0];
    voltageRaw = voltageRaw * 0.1f; // 0.1 V/bit resolution [1][14]

    // Parse current from bytes 2-3 (little-endian format)
    uint16_t currentRaw = (data[3] << 8) | data[2];
    currentRaw = (currentRaw * 0.1f);
}

void stopChargerCommand()
{
    byte data[8] = {
        // Array declaration fix
        0x00, // Voltage high byte (not used)
        0x00, // Voltage low byte (not used)
        0x00, // Current high byte (not used)
        0x00, // Current low byte (not used)
        0x01, // Control byte: 0x01 = stop charging
        0x00, // Mode selector (not used)
        0x00, // Reserved
        0x00  // Reserved
    };

    if (CAN.sendMsgBuf(0x1806E5F4, 1, 8, data) == CAN_OK)
    {
        Serial.println("Charging stopped");
    }
    else
    {
        Serial.println("Stop command failed");
    }
}

void sendChargerCommand()
{
    // Convert voltage to 0.1V units (108.8V -> 1088 -> 0x0440)
    uint16_t voltage = MAX_VOLTAGE * 10;
    byte voltage_high = (voltage >> 8) & 0xFF;
    byte voltage_low = voltage & 0xFF;

    // Convert current to 0.1A units (12A -> 120 -> 0x0078)
    uint16_t current = MAX_CURRENT * 10;
    byte current_high = (current >> 8) & 0xFF;
    byte current_low = current & 0xFF;

    byte data[8] = {
        voltage_high, // 0x04
        voltage_low,  // 0x40
        current_high, // Current high byte
        current_low,  // Current low byte
        0x00,         // Control byte (activate charging)
        0x00,         // Charging mode
        0x00,         // Reserved
        0x00          // Reserved
    };

    if (CAN.sendMsgBuf(0x1806E5F4, 1, 8, data) == CAN_OK)
    {
        Serial.print("Command sent: ");
        Serial.print(MAX_VOLTAGE, 1);
        Serial.print("V, ");
        Serial.print(MAX_CURRENT, 1);
        Serial.println("A");
    }
    else
    {
        Serial.println("Send failed");
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    // Initialize CAN bus
    if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK)
    {
        Serial.println("CAN init failed");
        while (1)
            ;
    }
    CAN.setMode(MCP_NORMAL);
    Serial.println("CAN Ready");
}

void loop()
{
    if (millis() - pmillis > 1000)
    {
        pmillis = millis();
        if (chargingStatus)
        {
            sendChargerCommand();
        }
    }
    if (Serial.available() > 0)
    {
        char data = Serial.read();
        // Send command when 's' is pressed
        if (data == 's')
        {
            sendChargerCommand();
            chargingStatus = true;
        }
        // Send command when 'x' is pressed
        if (data == 'x')
        {
            chargingStatus = false;
        }
    }
    // Handle incoming messages
    if (digitalRead(CAN_INT) == LOW)
    {
        unsigned long id;
        byte len, data[8];
        if (CAN.readMsgBuf(&id, &len, data) == CAN_OK)
        {
            Serial.print("Received ID: 0x");
            Serial.print(id, HEX);
            Serial.print(" Data: ");
            for (byte i = 0; i < len; i++)
            {
                if (data[i] < 0x10)
                    Serial.print('0');
                Serial.print(data[i], HEX);
                Serial.print(' ');
            }
            Serial.println();
            if (len == 8)
            {
                // Expected: data[0]=high byte, data[1]=low byte.
                uint16_t voltageRaw = (data[0] << 8) | data[1];
                float voltage = voltageRaw * 0.1f; // 0.1 V/bit resolution

                // Parse current from bytes 2-3 (remains little-endian as specified)
                uint16_t currentRaw = (data[2] << 8) | data[3];
                float current = currentRaw * 0.1f; // 0.1 A/bit resolution

                // Parse charging state from byte 4
                uint8_t state = data[4];
                // Use a string literal for human-readable charging status.
                const char *chargingStatus = (state == 0) ? "Charging" : "Stopped";

                Serial.println("-------------------------------------");
                Serial.print("volt: ");
                Serial.println(voltage);
                Serial.print("current: ");
                Serial.println(current);
                Serial.print("charge: ");
                Serial.println(chargingStatus);
                Serial.println("-------------------------------------");
            }
        }
    }
    delay(100);
}
