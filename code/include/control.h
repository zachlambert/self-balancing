#ifndef CONTROL_H
#define CONTROL_H

#include "state.h"
#include <stdlib.h>

typedef void *ControllerHandle;
extern const size_t CONTROLLER_PARAM_COUNT;

ControllerHandle controller_init(void);
void controller_update(State *state, ControllerHandle controller_handle);
float controller_get_param(
    ControllerHandle conroller_handle, size_t controller_param_i
);
void controller_set_param(
    ControllerHandle conroller_handle, size_t controller_param_i, float value
);
const char *controller_get_format_string(size_t param_i);

#endif
