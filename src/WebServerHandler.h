#ifndef WEBSERVER_HANDLER_H
#define WEBSERVER_HANDLER_H

#include <WebServer.h>
#include <ArduinoJson.h>
#include "CANHandler.h"      // Include header CAN
#include "ChargerCommands.h" // Include header Charger

// Deklarasi eksternal variabel
extern WebServer server;
extern float MAX_VOLTAGE;
extern float MAX_CURRENT;
extern float setVoltage;
extern float setCurrent;
extern float batteryVoltage;
extern float batteryCurrent;
extern String chargingStatus;

// Variabel CAN
extern String receivedCANId;
extern String receivedCANData;

// Fungsi charger
extern void stopChargerCommand();

void setupWebServer();
void handleRoot();
void handleSet();
void handleControl();
void handleData();

#endif
