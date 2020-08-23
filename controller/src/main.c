#include "zarduino/core/pins.h"
#include "zarduino/comms/spi.h"
#include "zarduino/comms/uart.h"
#include "zarduino/module/radio.h"
#include "zarduino/timing/delay.h"

int main(void)
{
    uart_init();

    SPIConfig spi_config = spi_create_config();
    spi_init_master(&spi_config);

    RadioConfig radio_config = radio_create_config();
    radio_config.CE = PIN_ARDUINO_D8;
    radio_config.CSN = PIN_ARDUINO_D6;
    radio_config.IRQ = 0;
    radio_config.tx_address = 0xA000000012;
    radio_init_as_transmitter(&radio_config);
    delay(10);
    radio_start(&radio_config);

    float v_cmd, omega_cmd;
    v_cmd = -0.15;
    omega_cmd = -2;

    uint8_t tx_payload[5];

    const float v_sensitivity = 0.5/32768.0;
    const float omega_sensitivity = 6.0/32768.0;

    while (1) {
        tx_payload[0] = (int16_t)(v_cmd/v_sensitivity)>>8;
        tx_payload[1] = (int16_t)(v_cmd/v_sensitivity);
        tx_payload[2] = (int16_t)(omega_cmd/omega_sensitivity)>>8;
        tx_payload[3] = (int16_t)(omega_cmd/omega_sensitivity);
        tx_payload[4] = 0x55;
        radio_write_tx(&radio_config, tx_payload, 5);

        printf("V = %d, Omega = %d\n", (uint16_t)(1000*v_cmd), (uint16_t)(1000*omega_cmd));

        v_cmd += 0.01;
        if (v_cmd >= 0.15) v_cmd = -0.15;
        omega_cmd += 0.1;
        if (omega_cmd >= 2) omega_cmd = -2;

        delay(250);
    }
}
