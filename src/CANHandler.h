#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <mcp_can.h>

// Deklarasi eksternal variabel
extern MCP_CAN CAN;
extern float batteryVoltage;
extern float batteryCurrent;
extern String chargingStatus;
extern String receivedCANId;
extern String receivedCANData;

#define CAN_ID_RECEIVE 0x98FF50E5
#define CAN_ID_RESPONSE 0x1806E5F4

void handleCANMessages();

#endif
