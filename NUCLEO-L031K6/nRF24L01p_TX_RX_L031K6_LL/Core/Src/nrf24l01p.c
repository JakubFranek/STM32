#include "nrf24l01p.h"
#include <stddef.h>

static nrf24l01p_config_t config;

static void cs_high()
{
    LL_GPIO_SetOutputPin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER);
}

static void cs_low()
{
	LL_GPIO_ResetOutputPin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER);
}

static uint8_t transmit_done()
{
	uint8_t done = LL_SPI_IsActiveFlag_TXE(NRF24L01P_SPI) && !LL_SPI_IsActiveFlag_BSY(NRF24L01P_SPI);
	return done;
}

static uint8_t receive_done()
{
	uint8_t done = LL_SPI_IsActiveFlag_RXNE(NRF24L01P_SPI) && !LL_SPI_IsActiveFlag_BSY(NRF24L01P_SPI);
	return done;
}

static uint8_t read_register(uint8_t reg)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
    uint8_t register_value;

    cs_low();

	LL_SPI_TransmitData8(NRF24L01P_SPI, command); 		// Transmit command
	while (!transmit_done()); 	// Wait until transmission is complete

	while (!receive_done());	// Wait until the RXNE flag is set to read the status byte
	(void) LL_SPI_ReceiveData8(NRF24L01P_SPI); 			// Read status

	LL_SPI_TransmitData8(NRF24L01P_SPI, 0xFF); 			// Send dummy byte to receive register value
	while (!transmit_done()); 	// Wait until the transmission is complete
	while (!receive_done());	// Wait until the RXNE flag is set to read the value
	register_value = LL_SPI_ReceiveData8(NRF24L01P_SPI);// Read register value

    cs_high();

    return register_value;
}

static uint64_t read_register_multibyte(uint8_t reg, uint8_t num_bytes)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
    uint64_t register_value = 0;
    uint8_t read_byte;

    cs_low();

	LL_SPI_TransmitData8(NRF24L01P_SPI, command); 		// Transmit command
	while (!transmit_done()); 	// Wait until transmission is complete

	while (!receive_done());	// Wait until the RXNE flag is set to read the status byte
	(void) LL_SPI_ReceiveData8(NRF24L01P_SPI); 			// Read status

	for(int i = 0; i < num_bytes; i++)
	{
		LL_SPI_TransmitData8(NRF24L01P_SPI, 0xFF); 			// Send dummy byte to receive register value
		while (!transmit_done()); 	// Wait until the transmission is complete

		while (!receive_done());	// Wait until the RXNE flag is set to read the value
		read_byte = LL_SPI_ReceiveData8(NRF24L01P_SPI); 	// Read byte
		register_value |= ((uint64_t)read_byte << (i * 8));	// Add read byte into register_value variable
	}

	cs_high();

    return register_value;
}

static uint8_t write_register(uint8_t reg, uint8_t payload)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
    uint8_t status;

    cs_low();

    LL_SPI_TransmitData8(NRF24L01P_SPI, command); 		// Send command
	while (!transmit_done()); 	// Wait until transmission is complete

	while (!receive_done()); 	// Wait for status byte
	status = LL_SPI_ReceiveData8(NRF24L01P_SPI); 		// Read status

	LL_SPI_TransmitData8(NRF24L01P_SPI, payload); 		// Send value
	while (!transmit_done()); 	// Wait until transmission is complete

    cs_high();

    return status;
}

static uint8_t write_register_multibyte(uint8_t reg, uint64_t payload, uint8_t num_bytes)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
    uint8_t payload_byte;
    uint8_t status;

    cs_low();

    LL_SPI_TransmitData8(NRF24L01P_SPI, command); 		// Send command
	while (!transmit_done()); 	// Wait until transmission is complete

	while (!receive_done()); 	// Wait for status byte
	status = LL_SPI_ReceiveData8(NRF24L01P_SPI); 		// Read status

	for(int i = 0; i < num_bytes; i++)
	{
		payload_byte = (uint8_t)((payload >> (i * 8)) & 0xFF);	// Extract byte from payload (LSB first)
		LL_SPI_TransmitData8(NRF24L01P_SPI, payload_byte); 		// Send value
		while (!transmit_done()); 		// Wait until transmission is complete
	}

    cs_high();

    return status;
}

static uint8_t send_command(uint8_t command)
{
	uint8_t status;

	cs_low();

	LL_SPI_TransmitData8(NRF24L01P_SPI, command); 		// Send command
	while (!transmit_done()); 	// Wait until transmission is complete

	while (!receive_done()); 	// Wait for status byte
	status = LL_SPI_ReceiveData8(NRF24L01P_SPI); 		// Read status (optional)

	cs_high();

	return status;
}

void nrf24l01p_set_ce(uint8_t state)
{
	if (state)
		LL_GPIO_SetOutputPin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER);
	else
		LL_GPIO_ResetOutputPin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER);
}

int8_t nrf24l01p_init(nrf24l01p_config_t* _config)
{
	if (_config == NULL)
		return -1;
	config = *_config;

    nrf24l01p_reset();

    nrf24l01p_set_rf_channel(_config->channel_MHz);
    nrf24l01p_set_rf_data_rate(_config->data_rate);
    nrf24l01p_set_crc_length(_config->crc_length);
    nrf24l01p_set_address_widths(_config->address_width);

    nrf24l01p_set_rf_tx_output_power(_config->output_power);
    nrf24l01p_set_auto_retransmit_count(_config->auto_retransmit_count);
    nrf24l01p_set_auto_retransmit_delay(_config->auto_retransmit_delay_250us);
    nrf24l01p_set_auto_ack_pipes(_config->auto_ack_pipes);

    nrf24l01p_rx_set_payload_length(0, _config->data_length);

    return 0;
}

void nrf24l01p_rx_receive(uint8_t* rx_payload)
{
    nrf24l01p_read_rx_fifo(rx_payload);
    nrf24l01p_clear_rx_dr();
}

void nrf24l01p_tx_transmit(uint8_t* tx_payload)
{
    nrf24l01p_write_tx_fifo(tx_payload);
}

void nrf24l01p_tx_irq()
{
	(void) LL_SPI_ReceiveData8(NRF24L01P_SPI); // clear MCU RX buffer

    uint8_t status = nrf24l01p_get_status_and_clear_IRQ_flags();
    uint8_t tx_ds = status & 0x20;

    nrf24l01p_set_ce(0);
    nrf24l01p_power_down();

    if(tx_ds)
    	LL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    else
    	LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
}

/* nRF24L01+ Sub Functions */
void nrf24l01p_reset()
{
    // Reset pins
    cs_high();
    nrf24l01p_set_ce(0);

    // Reset registers
    write_register(NRF24L01P_REG_CONFIG, 		0x08);
    write_register(NRF24L01P_REG_EN_AA, 		0x3F);
    write_register(NRF24L01P_REG_EN_RXADDR, 	0x03);
    write_register(NRF24L01P_REG_SETUP_AW, 		0x03);
    write_register(NRF24L01P_REG_SETUP_RETR, 	0x03);
    write_register(NRF24L01P_REG_RF_CH, 		0x02);
    write_register(NRF24L01P_REG_RF_SETUP, 		0x07);
    write_register(NRF24L01P_REG_STATUS, 		0x70);
    write_register(NRF24L01P_REG_RX_PW_P0, 		0x00);
    write_register(NRF24L01P_REG_RX_PW_P1, 		0x00);
    write_register(NRF24L01P_REG_RX_PW_P2, 		0x00);
    write_register(NRF24L01P_REG_RX_PW_P3, 		0x00);
    write_register(NRF24L01P_REG_RX_PW_P4, 		0x00);
    write_register(NRF24L01P_REG_RX_PW_P5, 		0x00);
    write_register(NRF24L01P_REG_DYNPD, 		0x00);
    write_register(NRF24L01P_REG_FEATURE, 		0x00);

    // Reset FIFOs
    nrf24l01p_flush_rx_fifo();
    nrf24l01p_flush_tx_fifo();
}

void nrf24l01p_set_prx_mode()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_ptx_mode()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFE;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_rx_addr(uint8_t index, uint64_t address)	// index must be integer from 0 to 5
{
	if (index == 0 || index == 1)
		write_register_multibyte(NRF24L01P_REG_RX_ADDR_P0 + index, address, config.address_width);
	else
		write_register(NRF24L01P_REG_RX_ADDR_P0 + index, (uint8_t)address);
}

void nrf24l01p_set_tx_addr(uint64_t address)
{
	write_register_multibyte(NRF24L01P_REG_TX_ADDR, address, config.address_width);
}

uint8_t nrf24l01p_read_rx_fifo(uint8_t* rx_payload)
{
    uint8_t status = send_command(NRF24L01P_CMD_R_RX_PAYLOAD);

    cs_low();

	for (uint8_t i = 0; i < config.data_length; i++)
	{
		LL_SPI_TransmitData8(NRF24L01P_SPI, 0xFF); 			// Send dummy byte to receive payload
		while (!transmit_done()); 	// Wait until transmission is complete
		while (!receive_done()); 	// Wait for payload byte
		rx_payload[i] = LL_SPI_ReceiveData8(NRF24L01P_SPI); // Read payload byte
	}

    cs_high();

    return status;
}

uint8_t nrf24l01p_write_tx_fifo(uint8_t* tx_payload)
{
    uint8_t status;

    cs_low();

    LL_SPI_TransmitData8(NRF24L01P_SPI, NRF24L01P_CMD_W_TX_PAYLOAD); 	// Send command
    while (!transmit_done()); 	// Wait until transmission is complete

    while (!receive_done()); 	// Wait for status byte
    status = LL_SPI_ReceiveData8(NRF24L01P_SPI); 		// Read status (optional)

	for (uint8_t i = 0; i < config.data_length; i++)
	{
		LL_SPI_TransmitData8(NRF24L01P_SPI, tx_payload[i]); // Send payload byte
		while (!transmit_done()); 	// Wait until transmission is complete
	}

    cs_high();

    return status;
}

void nrf24l01p_flush_rx_fifo()
{
    (void) send_command(NRF24L01P_CMD_FLUSH_RX);

}

void nrf24l01p_flush_tx_fifo()
{
    (void) send_command(NRF24L01P_CMD_FLUSH_TX);
}

uint8_t nrf24l01p_get_status()
{
    return send_command(NRF24L01P_CMD_NOP);
}

uint8_t nrf24l01p_get_status_and_clear_IRQ_flags()
{
	uint8_t status;

	cs_low();

	LL_SPI_TransmitData8(NRF24L01P_SPI, NRF24L01P_CMD_W_REGISTER | NRF24L01P_REG_STATUS);
	while (!transmit_done()); 	// Wait until transmission is complete

	while (!receive_done()); 	// Wait for status byte
	status = LL_SPI_ReceiveData8(NRF24L01P_SPI); 		// Read status

	LL_SPI_TransmitData8(NRF24L01P_SPI, status);		// Send value
	while (!transmit_done()); 	// Wait until transmission is complete

	cs_high();

	return status;
}

uint8_t nrf24l01p_get_fifo_status()
{
    return read_register(NRF24L01P_REG_FIFO_STATUS);
}

void nrf24l01p_rx_set_payload_length(uint8_t index, uint8_t bytes)
{
    write_register(NRF24L01P_REG_RX_PW_P0 + index, bytes);
}

void nrf24l01p_clear_rx_dr()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x40;

    write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_tx_ds()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x20;

    write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_max_rt()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status |= 0x10;

    write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_power_up()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 2;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_power_down()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFD;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_crc_length(nrf24l01p_crc_length_t length)
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);

    switch(length)
    {
        case _1byte:
            new_config &= 0xFB;
            break;
        case _2byte:
            new_config |= 1 << 2;
            break;
    }

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_address_widths(uint8_t address_width)
{
    write_register(NRF24L01P_REG_SETUP_AW, address_width - 2);
}

void nrf24l01p_set_auto_retransmit_count(uint8_t count)
{
    uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);

    new_setup_retr &= 0xF0;
    new_setup_retr |= count;
    write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_set_auto_retransmit_delay(uint8_t delay_250us)
{
    uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);

    new_setup_retr &= 0x0F;    // Reset ARD register 0
    new_setup_retr |= (delay_250us - 1) << 4;
    write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_set_auto_ack_pipes(uint8_t pipes)
{
    uint8_t new_en_aa_reg = (pipes & 0x3F);
    write_register(NRF24L01P_REG_EN_AA, new_en_aa_reg);
}

void nrf24l01p_set_rf_channel(uint16_t channel_MHz)
{
	uint16_t new_rf_ch = channel_MHz - 2400;
    write_register(NRF24L01P_REG_RF_CH, new_rf_ch);
}

void nrf24l01p_set_rf_tx_output_power(nrf24l01p_output_power_t output_power)
{
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (output_power << 1);

    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

void nrf24l01p_set_rf_data_rate(nrf24l01p_data_rate_t data_rate)
{
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xD7;

    switch(data_rate)
    {
        case _1Mbps:
            break;
        case _2Mbps:
            new_rf_setup |= 1 << 3;
            break;
        case _250kbps:
            new_rf_setup |= 1 << 5;
    }
    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}
