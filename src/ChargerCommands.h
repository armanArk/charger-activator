#ifndef CHARGER_COMMANDS_H
#define CHARGER_COMMANDS_H

#include "CANHandler.h"

// Deklarasi eksternal variabel
extern float setVoltage;
extern float setCurrent;
extern bool chargerActive;

void sendChargerCommand();
void stopChargerCommand();

#endif
