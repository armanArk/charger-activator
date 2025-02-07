#include "WebServerHandler.h"
#include "CANHandler.h"
#include "ChargerCommands.h"

// Konfigurasi PIN CAN Bus
#define CAN_CS 5
#define CAN_INT 4

// Deklarasi objek
MCP_CAN CAN(CAN_CS);
WebServer server(80);

// Parameter Charger
float MAX_VOLTAGE = 115.2f;
float MAX_CURRENT = 35.0f;
float setVoltage = 108.8f;
float setCurrent = 5.0f;
bool chargerActive = false;
unsigned long lastUpdate = 0;

// Status Baterai
float batteryVoltage = 0.0f;
float batteryCurrent = 0.0f;
String chargingStatus = "Disconnected";
String receivedCANId = "";
String receivedCANData = "";

void setup()
{
    Serial.begin(115200);

    // Inisialisasi CAN Bus
    if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK)
    {
        Serial.println("Gagal inisialisasi CAN");
        while (1)
            ;
    }
    CAN.setMode(MCP_NORMAL);

    // Setup Web Server
    setupWebServer();
}

void loop()
{
    server.handleClient();

    if (chargerActive && millis() - lastUpdate > 1000)
    {
        sendChargerCommand();
        lastUpdate = millis();
    }

    if (digitalRead(CAN_INT) == LOW)
    {
        handleCANMessages();
    }
}
