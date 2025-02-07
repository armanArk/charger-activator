#include "CANHandler.h"

void handleCANMessages()
{
    unsigned long id;
    byte len, data[8];
    int result = CAN.readMsgBuf(&id, &len, data);
    // Convert ID to HEX
    char hexId[9]; // 8 hex digits + null terminator
    sprintf(hexId, "%08lX", receivedCANId);

    // Convert data to HEX string (XX XX XX XX XX XX XX XX)
    char hexData[24]; // 8 * 3 (XX + space) = 24
    for (int i = 0; i < 8; i++)
    {
        sprintf(&hexData[i * 3], "%02X ", receivedCANData[i]);
    }

    if (result == CAN_OK)
    {
        if (len == 8)
        {
            batteryVoltage = ((data[0] << 8) | data[1]) * 0.1f;
            batteryCurrent = ((data[2] << 8) | data[3]) * 0.1f;
            chargingStatus = (data[4] == 0) ? "Charging" : "Stopped";
        }
    }
}
