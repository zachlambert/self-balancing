#include "interface_radio.h"
#include "config.h"
#include "zarduino/comms/spi.h"
#include "zarduino/timing/delay.h"

void interface_radio_init(RadioConfig *radio_config)
{
    SPIConfig spi_config = spi_create_config();
    spi_init_master(&spi_config);

    *radio_config = radio_create_config();
    radio_config->CSN = RADIO_CSN_PIN;
    radio_config->CE = RADIO_CE_PIN;
    radio_config->rx_base_address = 0xA0000000;
    radio_config->rx_pipe_addresses[0] = 0x12;
    radio_config->rx_payload_sizes[0] = 4;

    radio_init_as_receiver(radio_config);
    delay(10);
    radio_start(radio_config);
}

void interface_radio_read_commands(RadioConfig *radio_config, float *cmd)
{
    RadioRxStatus rx_status;
    uint8_t rx_buffer[4];
    rx_status = radio_read_rx(radio_config, rx_buffer, 4);
    if (rx_status != RADIO_RX_STATUS_NOT_USED &&
        rx_status != RADIO_RX_STATUS_EMPTY)
    {
        cmd[CMD_VEL] = ((int16_t)(rx_buffer[0] | rx_buffer[1]<<8))*(1.0/512.0);
        cmd[CMD_OMEGA] = ((int16_t)(rx_buffer[2] | rx_buffer[3]<<8))*(3.0/512.0);
    }
}
