#include "nrf24l01p.h"
#include <stddef.h>


/* ------------------------------ Macros ------------------------------ */
#ifndef SET_BIT
	#define SET_BIT(reg, bit) 		((reg) |= (1U << (bit)))
#endif
#ifndef CLEAR_BIT
	#define CLEAR_BIT(reg, bit)  	((reg) &= ~(1U << (bit)))
#endif
#ifndef READ_BIT
	#define READ_BIT(REG, BIT)    	((REG) & (BIT))
#endif

#ifndef CLEAR_FIELD
	#define CLEAR_FIELD(reg, mask)    			((reg) &= ~(mask))
#endif

#ifndef SET_FIELD
	#define SET_FIELD(reg, val, mask, pos)  	((reg) |= (((val) << (pos)) & (mask)))
#endif

// Error-checking macro: if "expr" returns anything other than NRF24L01P_SUCCESS,
// this macro returns that return value, exiting the function where this macro was used immediately
#define NRF24L01P_CHECK_ERROR(expr) do { \
  nrf24l01p_error_t retval = expr; \
  if (retval != NRF24L01P_SUCCESS) { \
    return retval; \
  } \
} while (0)

/* ------------------------------ Module variables ------------------------------ */
nrf24l01p_8bit_reg_rst_vals_t reg_rst_vals_8bit[] = {
	{NRF24L01P_REG_CONFIG,			NRF24L01P_REG_CONFIG_RSTVAL},
	{NRF24L01P_REG_EN_AA,			NRF24L01P_REG_EN_AA_RSTVAL},
	{NRF24L01P_REG_EN_RXADDR,		NRF24L01P_REG_EN_RXADDR_RSTVAL},
	{NRF24L01P_REG_SETUP_AW,		NRF24L01P_REG_SETUP_AW_RSTVAL},
	{NRF24L01P_REG_SETUP_RETR,		NRF24L01P_REG_SETUP_RETR_RSTVAL},
	{NRF24L01P_REG_RF_CH,			NRF24L01P_REG_RF_CH_RSTVAL},
	{NRF24L01P_REG_RF_SETUP,		NRF24L01P_REG_RF_SETUP_RSTVAL},
	{NRF24L01P_REG_STATUS,			NRF24L01P_REG_STATUS_RSTVAL},
	{NRF24L01P_REG_RX_ADDR_P2,		NRF24L01P_REG_RX_ADDR_P2_RSTVAL},
	{NRF24L01P_REG_RX_ADDR_P3,		NRF24L01P_REG_RX_ADDR_P3_RSTVAL},
	{NRF24L01P_REG_RX_ADDR_P4,		NRF24L01P_REG_RX_ADDR_P4_RSTVAL},
	{NRF24L01P_REG_RX_ADDR_P5,		NRF24L01P_REG_RX_ADDR_P5_RSTVAL},
	{NRF24L01P_REG_RX_PW_P0,		NRF24L01P_REG_RX_PW_P0_RSTVAL},
	{NRF24L01P_REG_RX_PW_P1,		NRF24L01P_REG_RX_PW_P1_RSTVAL},
	{NRF24L01P_REG_RX_PW_P2,		NRF24L01P_REG_RX_PW_P2_RSTVAL},
	{NRF24L01P_REG_RX_PW_P3,		NRF24L01P_REG_RX_PW_P3_RSTVAL},
	{NRF24L01P_REG_RX_PW_P4,		NRF24L01P_REG_RX_PW_P4_RSTVAL},
	{NRF24L01P_REG_RX_PW_P5,		NRF24L01P_REG_RX_PW_P5_RSTVAL},
	{NRF24L01P_REG_FIFO_STATUS,		NRF24L01P_REG_FIFO_STATUS_RSTVAL},
	{NRF24L01P_REG_DYNPD,			NRF24L01P_REG_DYNPD_RSTVAL},
	{NRF24L01P_REG_FEATURE,			NRF24L01P_REG_FEATURE_RSTVAL}
};

nrf24l01p_40bit_reg_rst_vals_t reg_rst_vals_40bit[] = {
	{NRF24L01P_REG_RX_ADDR_P0,	NRF24L01P_REG_RX_ADDR_P0_RSTVAL},
	{NRF24L01P_REG_RX_ADDR_P1,	NRF24L01P_REG_RX_ADDR_P1_RSTVAL},
	{NRF24L01P_REG_TX_ADDR,		NRF24L01P_REG_TX_ADDR_RSTVAL},
};

/* ------------------------------ Static variables ------------------------------ */
static nrf24l01p_config_t config;


/* ------------------------------ Static functions ------------------------------ */
static nrf24l01p_error_t read_register(uint8_t address, uint8_t* value)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | address;
    uint8_t status;

    config.interface.set_cs(0);

	if(config.interface.spi_tx_rx(command, &status) != 0)
		return NRF24L01P_SPI_ERROR;
	if(config.interface.spi_tx_rx(0xFF, value) != 0)
		return NRF24L01P_SPI_ERROR;

	config.interface.set_cs(1);

    return NRF24L01P_SUCCESS;
}

static nrf24l01p_error_t read_register_multibyte(uint8_t address, uint64_t* value, uint8_t num_bytes)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | address;
    uint8_t read_byte, status;
    *value = 0;

    config.interface.set_cs(0);

    if(config.interface.spi_tx_rx(command, &status) != 0)
    	return NRF24L01P_SPI_ERROR;

    for(int i = 0; i < num_bytes; i++)
	{
		if(config.interface.spi_tx_rx(0xFF, &read_byte) != 0)
			return NRF24L01P_SPI_ERROR;
		*value |= ((uint64_t)read_byte << (i * 8));
	}

    config.interface.set_cs(1);

    return NRF24L01P_SUCCESS;
}

static nrf24l01p_error_t write_register(uint8_t address, uint8_t payload)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | address;
    uint8_t status;

    config.interface.set_cs(0);

    if(config.interface.spi_tx_rx(command, &status) != 0)
    	return NRF24L01P_SPI_ERROR;

    if(config.interface.spi_tx(payload) != 0)
		return NRF24L01P_SPI_ERROR;

    config.interface.set_cs(1);

    return NRF24L01P_SUCCESS;
}

static nrf24l01p_error_t write_register_multibyte(uint8_t reg, uint64_t payload, uint8_t num_bytes)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
    uint8_t payload_byte;
    uint8_t status;

    config.interface.set_cs(0);

    if(config.interface.spi_tx_rx(command, &status) != 0)
        return NRF24L01P_SPI_ERROR;

    for(int i = 0; i < num_bytes; i++)
	{
		payload_byte = (uint8_t)((payload >> (i * 8)) & 0xFF);	// Extract byte from payload (LSB first)
		if(config.interface.spi_tx(payload_byte))
			return NRF24L01P_SPI_ERROR;
	}

    config.interface.set_cs(1);

    return NRF24L01P_SUCCESS;
}

static nrf24l01p_error_t send_command(uint8_t command)
{
	uint8_t status;

	config.interface.set_cs(0);

	if(config.interface.spi_tx_rx(command, &status) != 0)
		return NRF24L01P_SPI_ERROR;

	config.interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

/* ------------------------------ High-level API ------------------------------ */
nrf24l01p_error_t nrf24l01p_init(nrf24l01p_config_t* _config)
{
	if (_config == NULL)
		return NRF24L01P_SPI_ERROR;
	config = *_config;

	NRF24L01P_CHECK_ERROR(nrf24l01p_reset());

    NRF24L01P_CHECK_ERROR(nrf24l01p_set_rf_channel(_config->channel_MHz));
    NRF24L01P_CHECK_ERROR(nrf24l01p_set_rf_data_rate(_config->data_rate));
    NRF24L01P_CHECK_ERROR(nrf24l01p_set_crc_length(_config->crc_length));
    NRF24L01P_CHECK_ERROR(nrf24l01p_set_address_width(_config->address_width));

    NRF24L01P_CHECK_ERROR(nrf24l01p_set_rf_tx_output_power(_config->output_power));
    NRF24L01P_CHECK_ERROR(nrf24l01p_set_auto_retransmit_count(_config->auto_retransmit_count));
    NRF24L01P_CHECK_ERROR(nrf24l01p_set_auto_retransmit_delay(_config->auto_retransmit_delay_250us));
    NRF24L01P_CHECK_ERROR(nrf24l01p_set_auto_ack_pipes(_config->auto_ack_pipes));

    NRF24L01P_CHECK_ERROR(nrf24l01p_rx_set_payload_length(0, _config->data_length));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_rx_receive(uint8_t* rx_payload)
{
	NRF24L01P_CHECK_ERROR(nrf24l01p_read_rx_fifo(rx_payload));
	NRF24L01P_CHECK_ERROR(nrf24l01p_clear_flag(NRF24L01P_REG_STATUS_RX_DR));

	return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_tx_transmit(uint8_t* tx_payload)
{
    NRF24L01P_CHECK_ERROR(nrf24l01p_write_tx_fifo(tx_payload));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_irq(nrf24l01p_irq_t* irq_sources)
{
	uint8_t dummy;
	config.interface.spi_rx(&dummy); 	// Ensure SPI RX buffer is empty

    uint8_t status;
    NRF24L01P_CHECK_ERROR(nrf24l01p_get_status_and_clear_IRQ_flags(&status));

    irq_sources->rx_dr = READ_BIT(status, NRF24L01P_REG_STATUS_RX_DR);
    irq_sources->tx_ds = READ_BIT(status, NRF24L01P_REG_STATUS_TX_DS);
    irq_sources->max_rt = READ_BIT(status, NRF24L01P_REG_STATUS_MAX_RT);

    config.interface.set_ce(0);
    NRF24L01P_CHECK_ERROR(nrf24l01p_power_down());

    return NRF24L01P_SUCCESS;
}

/* nRF24L01+ Sub Functions */
nrf24l01p_error_t nrf24l01p_reset()
{
    // Reset pins
	config.interface.set_cs(1);
    config.interface.set_ce(0);

    // Reset registers
    for (size_t i = 0; i < sizeof(reg_rst_vals_8bit) / sizeof(reg_rst_vals_8bit[0]); i++)
    	NRF24L01P_CHECK_ERROR(write_register(reg_rst_vals_8bit[i].address, reg_rst_vals_8bit[i].value));

    for (size_t i = 0; i < sizeof(reg_rst_vals_40bit) / sizeof(reg_rst_vals_40bit[0]); i++)
    	NRF24L01P_CHECK_ERROR(write_register_multibyte(reg_rst_vals_40bit[i].address, reg_rst_vals_40bit[i].value, 40/8));

    // Reset FIFOs
    NRF24L01P_CHECK_ERROR(nrf24l01p_flush_rx_fifo());
    NRF24L01P_CHECK_ERROR(nrf24l01p_flush_tx_fifo());

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_set_prx_mode()
{
    uint8_t config_reg;
    NRF24L01P_CHECK_ERROR(read_register(NRF24L01P_REG_CONFIG, &config_reg));
    SET_BIT(config_reg, NRF24L01P_REG_CONFIG_PRIM_RX);

    NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_CONFIG, config_reg));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_set_ptx_mode()
{
    uint8_t config_reg;
    NRF24L01P_CHECK_ERROR(read_register(NRF24L01P_REG_CONFIG, &config_reg));
    CLEAR_BIT(config_reg, NRF24L01P_REG_CONFIG_PRIM_RX);

    NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_CONFIG, config_reg));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_set_rx_addr(uint8_t index, uint64_t address)	// index must be integer from 0 to 5
{
	if (index > 5)
		return NRF24L01P_INVALID_VALUE;

	if (index == 0 || index == 1)
		NRF24L01P_CHECK_ERROR(write_register_multibyte(NRF24L01P_REG_RX_ADDR_P0 + index, address, config.address_width));
	else
		NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_RX_ADDR_P0 + index, (uint8_t)address));

	return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_set_tx_addr(uint64_t address)
{
	NRF24L01P_CHECK_ERROR(write_register_multibyte(NRF24L01P_REG_TX_ADDR, address, config.address_width));

	return NRF24L01P_SUCCESS;
}


nrf24l01p_error_t nrf24l01p_power_up()
{
    uint8_t config_reg;
    NRF24L01P_CHECK_ERROR(read_register(NRF24L01P_REG_CONFIG, &config_reg));
    SET_BIT(config_reg, NRF24L01P_REG_CONFIG_PWR_UP);

    NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_CONFIG, config_reg));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_power_down()
{
    uint8_t config_reg;
    NRF24L01P_CHECK_ERROR(read_register(NRF24L01P_REG_CONFIG, &config_reg));
    CLEAR_BIT(config_reg, NRF24L01P_REG_CONFIG_PWR_UP);

    NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_CONFIG, config_reg));

    return NRF24L01P_SUCCESS;
}


/* ------------------------------ Low-level API ------------------------------ */
nrf24l01p_error_t nrf24l01p_get_status(uint8_t* status)
{
	config.interface.set_cs(0);

	NRF24L01P_CHECK_ERROR(config.interface.spi_tx_rx(NRF24L01P_CMD_W_REGISTER | NRF24L01P_REG_STATUS, status));

	config.interface.set_cs(1);

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_get_status_and_clear_IRQ_flags(uint8_t* status)
{
	config.interface.set_cs(0);

	NRF24L01P_CHECK_ERROR(config.interface.spi_tx_rx(NRF24L01P_CMD_W_REGISTER | NRF24L01P_REG_STATUS, status));
	// Following line takes advantage of the fact that active flag is 1, and writing 1 to it clears it
	NRF24L01P_CHECK_ERROR(config.interface.spi_tx(*status));

	config.interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_get_fifo_status(uint8_t* fifo_status)
{
	NRF24L01P_CHECK_ERROR(read_register(NRF24L01P_REG_FIFO_STATUS, fifo_status));

	return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_flush_rx_fifo()
{
	NRF24L01P_CHECK_ERROR(send_command(NRF24L01P_CMD_FLUSH_RX));

	return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_flush_tx_fifo()
{
	NRF24L01P_CHECK_ERROR(send_command(NRF24L01P_CMD_FLUSH_TX));

	return NRF24L01P_SUCCESS;
}


nrf24l01p_error_t nrf24l01p_read_rx_fifo(uint8_t* rx_payload)
{
    NRF24L01P_CHECK_ERROR(send_command(NRF24L01P_CMD_R_RX_PAYLOAD));

    config.interface.set_cs(0);

	for (uint8_t i = 0; i < config.data_length; i++)
		NRF24L01P_CHECK_ERROR(config.interface.spi_tx_rx(0xFF, &rx_payload[i]));

	config.interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_write_tx_fifo(uint8_t* tx_payload)
{
    uint8_t status;

    config.interface.set_cs(0);

    NRF24L01P_CHECK_ERROR(config.interface.spi_tx_rx(NRF24L01P_CMD_W_TX_PAYLOAD, &status));

    for (uint8_t i = 0; i < config.data_length; i++)
    	NRF24L01P_CHECK_ERROR(config.interface.spi_tx(tx_payload[i]));

    config.interface.set_cs(1);

	return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_rx_set_payload_length(uint8_t pipe_index, uint8_t bytes)
{
	if (bytes > 32 || pipe_index > 5)
		return NRF24L01P_INVALID_VALUE;

	NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_RX_PW_P0 + pipe_index, bytes));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_clear_status_flag(uint8_t flags)
{
    uint8_t status;

    NRF24L01P_CHECK_ERROR(nrf24l01p_get_status(&status));
    SET_BIT(status, flags);

    NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_STATUS, status));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_set_crc_length(nrf24l01p_crc_length_t length)
{

    uint8_t config_reg;
    NRF24L01P_CHECK_ERROR(read_register(NRF24L01P_REG_CONFIG, &config_reg));

    CLEAR_BIT(config_reg, NRF24L01P_REG_CONFIG_CRCO);
    config_reg |= length;		// enum values already reflect physical encoding

    NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_CONFIG, config_reg));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_set_address_width(uint8_t bytes)
{
	if (bytes < 3 || bytes > 5)
		return NRF24L01P_INVALID_VALUE;

	NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_SETUP_AW, bytes - 2));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_set_auto_retransmit_count(uint8_t count)
{
	if (count > 15)
		return NRF24L01P_INVALID_VALUE;

    uint8_t setup_retr;

    NRF24L01P_CHECK_ERROR(read_register(NRF24L01P_REG_SETUP_RETR, &setup_retr));

    CLEAR_FIELD(setup_retr, NRF24L01P_REG_SETUP_RETR_ARC_MASK);
    SET_FIELD(setup_retr, count, NRF24L01P_REG_SETUP_RETR_ARC_MASK, NRF24L01P_REG_SETUP_RETR_ARC_POS);

    NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_SETUP_RETR, setup_retr));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_set_auto_retransmit_delay(uint8_t delay_250us)
{
	if (delay_250us < 1 || delay_250us > 16)
		return NRF24L01P_INVALID_VALUE;

    uint8_t setup_retr;

    NRF24L01P_CHECK_ERROR(read_register(NRF24L01P_REG_SETUP_RETR, &setup_retr));

    CLEAR_FIELD(setup_retr, NRF24L01P_REG_SETUP_RETR_ARD_MASK);
    SET_FIELD(setup_retr, (delay_250us - 1), NRF24L01P_REG_SETUP_RETR_ARD_MASK, NRF24L01P_REG_SETUP_RETR_ARD_POS);

    NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_SETUP_RETR, setup_retr));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_set_auto_ack_pipes(uint8_t pipes)
{
    uint8_t en_aa = (pipes & 0b00111111);	// force bits 7 and 6 to zero

    NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_EN_AA, en_aa));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_set_rf_channel(uint16_t channel_MHz)
{
	if (channel_MHz < 2400 || channel_MHz > 2525)
		return NRF24L01P_INVALID_VALUE;

	uint8_t rf_ch = channel_MHz - 2400;

	NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_RF_CH, rf_ch));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_set_rf_tx_output_power(nrf24l01p_output_power_t output_power)
{
    uint8_t rf_setup;

    NRF24L01P_CHECK_ERROR(read_register(NRF24L01P_REG_RF_SETUP, &rf_setup));
    CLEAR_FIELD(rf_setup, NRF24L01P_REG_RF_SETUP_RF_PWR_MASK);
    rf_setup |= output_power; 	// enum values already reflect physical encoding

    NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_RF_SETUP, rf_setup));

    return NRF24L01P_SUCCESS;
}

nrf24l01p_error_t nrf24l01p_set_rf_data_rate(nrf24l01p_data_rate_t data_rate)
{
    uint8_t rf_setup;

    NRF24L01P_CHECK_ERROR(read_register(NRF24L01P_REG_RF_SETUP, &rf_setup));
    rf_setup &= ~(NRF24L01P_REG_RF_SETUP_RF_DR_HIGH | NRF24L01P_REG_RF_SETUP_RF_DR_LOW);
	rf_setup |= data_rate;		// enum values already reflect physical encoding

	NRF24L01P_CHECK_ERROR(write_register(NRF24L01P_REG_RF_SETUP, rf_setup));

    return NRF24L01P_SUCCESS;
}
