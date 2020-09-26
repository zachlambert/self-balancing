#ifndef INTERFACE_PARAM_H
#define INTERFACE_PARAM_H

#include "robot.h"
#include "zarduino/module/oled.h"
#include <stdlib.h>

void interface_param_init(OLEDConfig *oled_config, float initial_param);
void interface_param_update(
    OLEDConfig *oled_config,
    float *param_values,
    const char **param_names,
    size_t param_count,
    float adc_scale
);

#endif
