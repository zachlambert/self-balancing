#ifndef INTERFACE_RADIO_H
#define INTERFACE_RADIO_H

#include "zarduino/module/radio.h"

#define CMD_VEL 0
#define CMD_OMEGA 1
#define CMD_COUNT 2

void interface_radio_init(RadioConfig *radio_config);
void interface_radio_read_commands(RadioConfig *radio_config, float *cmd);

#endif
