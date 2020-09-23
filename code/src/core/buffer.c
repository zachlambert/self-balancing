#include "buffer.h"
#include "string.h"

Buffer buffer_create(size_t N)
{
    Buffer buffer;
    buffer.data = malloc(N * sizeof(float));
    memset(buffer.data, 0, N * sizeof(float));
    buffer.N = N;
    buffer.tail = 0;
    return buffer;
}

void buffer_set(Buffer *buffer, float value)
{
    buffer->data[buffer->tail] = value;
    buffer->tail = (buffer->tail + 1) % buffer->N;
}

float buffer_get(Buffer *buffer, size_t steps_back) {
    if (steps_back > buffer->tail) {
        return buffer->data[buffer->tail + buffer->N - steps_back];
    } else {
        return buffer->data[buffer->tail - steps_back];
    }
}

float buffer_convolve_window(Buffer *buffer, const float *window)
{
    float sum = 0;
    for (size_t i = 0; i < buffer->N; i++) {
        sum += window[i] * buffer_get(buffer, i);
    }
    return sum;
}
