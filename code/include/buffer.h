#ifndef BUFFER_H
#define BUFFER_H

#include <stdlib.h>

// Only need to handle insertion and retrieval
typedef struct {
    float *data;
    size_t N;
    size_t tail;
} Buffer;

Buffer buffer_create(size_t N);
void buffer_set(Buffer *buffer, float value);
float buffer_get(Buffer *buffer, size_t steps_back);
float buffer_convolve_window(Buffer *buffer, const float *window);

#endif
