#include <SPI.h>
#include "mcp_can.h"

// Pin definitions (as shown in your image)
#define CAN_SCK 18  // SCK pin
#define CAN_MISO 19 // MISO pin
#define CAN_MOSI 23 // MOSI pin
#define CAN_CS 5    // CS pin
#define CAN_INT 4   // INT pin

MCP_CAN CAN(CAN_CS); // Create CAN object with CS pin

void setup()
{
    Serial.begin(115200);

    // Configure SPI pins
    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);

    // Initialize MCP2515 running at 8MHz with 250kbps baudrate
    while (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK)
    {
        Serial.println("CAN BUS initialization failed. Retrying...");
        delay(1000);
    }
    Serial.println("CAN BUS initialized successfully!");

    // Set to normal mode
    CAN.setMode(MCP_NORMAL);

    // Configure INT pin as input
    pinMode(CAN_INT, INPUT);

    Serial.println("CAN BUS Shield init ok!");
}

void loop()
{
    unsigned long rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];

    // Check if data is received
    if (!digitalRead(CAN_INT))
    {
        CAN.readMsgBuf(&rxId, &len, rxBuf);

        Serial.print("ID: ");
        Serial.print(rxId, HEX);
        Serial.print("  Length: ");
        Serial.print(len);
        Serial.print("  Data: ");

        for (int i = 0; i < len; i++)
        {
            Serial.print(rxBuf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    delay(100);
}

// Example function to send CAN message
void sendCANMessage(unsigned long id, byte *data, unsigned char len)
{
    CAN.sendMsgBuf(id, 0, len, data); // Standard frame (not extended)
}