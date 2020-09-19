#include "zarduino/core/pins.h"
#include "zarduino/comms/spi.h"
#include "zarduino/comms/uart.h"
#include "zarduino/module/radio.h"
#include "zarduino/timing/delay.h"
#include "zarduino/core/adc.h"

int main(void)
{
    uart_init(0);
    printf("Starting\n");

    SPIConfig spi_config = spi_create_config();
    spi_init_master(&spi_config);

    RadioConfig radio_config = radio_create_config();
    radio_config.CE = PIN_ARDUINO_D10;
    radio_config.CSN = PIN_ARDUINO_D9;
    radio_config.IRQ = 0;
    radio_config.tx_address = 0xA000000012;
    radio_init_as_transmitter(&radio_config);
    delay(10);
    radio_start(&radio_config);

    ADCConfig adc_config = adc_create_config();
    adc_initialise(&adc_config);

    uint8_t tx_payload[5];

    Pin pin_vertical = PIN_ARDUINO_A0;
    Pin pin_horizontal = PIN_ARDUINO_A1;

    int16_t vert_centre = adc_read_wait(pin_vertical);
    int16_t horiz_centre = adc_read_wait(pin_horizontal);

    while (1) {
        int16_t vert = adc_read_wait(pin_vertical) - vert_centre;
        if (abs(vert) < 5) vert = 0;
        int16_t horiz = adc_read_wait(pin_horizontal) - horiz_centre;
        if (abs(horiz) < 5) horiz = 0;

        tx_payload[0] = vert;
        tx_payload[1] = vert>>8;
        tx_payload[2] = horiz;
        tx_payload[3] = horiz>>8;
        radio_write_tx(&radio_config, tx_payload, 5);

        printf("%i %i\n", vert, horiz);
        delay(100);
    }
}
