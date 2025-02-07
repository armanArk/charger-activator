#include "ChargerCommands.h"
#include "CANHandler.h"

void sendChargerCommand()
{
    uint16_t voltage = setVoltage * 10;
    uint16_t current = setCurrent * 10;

    byte data[8] = {
        highByte(voltage), lowByte(voltage),
        highByte(current), lowByte(current),
        0x00, 0x00, 0x00, 0x00};

    CAN.sendMsgBuf(CAN_ID_RESPONSE, 0, 8, data);
}

void stopChargerCommand()
{
    byte data[8] = {0, 0, 0, 0, 0x01, 0, 0, 0};
    CAN.sendMsgBuf(CAN_ID_RESPONSE, 0, 8, data);
    chargerActive = false;
}
