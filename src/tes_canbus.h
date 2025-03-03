#include <SPI.h>
#include <mcp_can.h>

#define CAN_INT 2
#define CHARGER_CONTROL_ID 0x1806E5F4 // CAN ID for sending commands to the charger

MCP_CAN CAN(10); // CS pin on 10

// Timer variables for 1-second interval
unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 second

// Forward declarations
void sendChargerCommand(float voltage, float current, bool startCharging);
void handleReceivingCanbus();

void setup()
{
    Serial.begin(115200);
    // Initialize the CAN bus with mode, speed, and clock settings
    while (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK)
    {
        Serial.println("CAN init failed, retrying...");
        delay(100);
    }
    Serial.println("CAN init success");
    pinMode(CAN_INT, INPUT);
    pinMode(12, OUTPUT);
    digitalWrite(12, LOW);
}

void loop()
{
    // Process incoming CAN messages (if any)
    handleReceivingCanbus();

    // Check timer and send command every 1 second
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;
        sendChargerCommand(410, 8, true);
    }
}

// Function to send a charger command over CAN
void sendChargerCommand(float voltage, float current, bool startCharging)
{
    Serial.print("Sending CAN command. Voltage: ");
    Serial.print(voltage);
    Serial.print("V, Current: ");
    Serial.print(current);
    Serial.println("A");

    byte data[8] = {0};

    // Scale voltage (0.1V per bit) and current (0.1A per bit)
    uint16_t voltageVal = voltage * 10;
    data[0] = highByte(voltageVal);
    data[1] = lowByte(voltageVal);

    uint16_t currentVal = current * 10;
    data[2] = highByte(currentVal);
    data[3] = lowByte(currentVal);

    // Control byte: 0x00 for start charging, 0x01 for stop
    data[4] = startCharging ? 0x00 : 0x01;

    // Remaining bytes reserved (set to 0)
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;

    byte sndStat = CAN.sendMsgBuf(CHARGER_CONTROL_ID, 1, 8, data);
    if (sndStat == CAN_OK)
    {
        Serial.println("CAN command sent successfully");
    }
    else
    {
        Serial.println("Error sending CAN command");
    }
}

// Stub for CAN message receiving (add your actual implementation as needed)
void handleReceivingCanbus()
{
    // Check if the CAN_INT pin indicates an available message
    if (digitalRead(CAN_INT) == LOW)
    {
        unsigned long id;
        byte len;
        byte msgData[8];

        if (CAN.readMsgBuf(&id, &len, msgData) == CAN_OK)
        {
            Serial.print("Received CAN message with ID: 0x");
            Serial.println(id, HEX);
            // You can add further processing of msgData here
        }
    }
}
