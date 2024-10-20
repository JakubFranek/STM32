#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__

#include "spi.h"    // header from STM32CubeMX code generator, expects LL API

/* User Configurations */
#define NRF24L01P_SPI                     SPI1

#define NRF24L01P_SPI_CS_PIN_PORT         nRF24_CSN_GPIO_Port
#define NRF24L01P_SPI_CS_PIN_NUMBER       nRF24_CSN_Pin

#define NRF24L01P_CE_PIN_PORT             nRF24_CE_GPIO_Port
#define NRF24L01P_CE_PIN_NUMBER           nRF24_CE_Pin

typedef enum
{
    _250kbps = 2,
    _1Mbps   = 0,
    _2Mbps   = 1
} nrf24l01p_data_rate_t;

typedef enum
{
    _0dBm  = 3,
    _6dBm  = 2,
    _12dBm = 1,
    _18dBm = 0
} nrf24l01p_output_power_t;

typedef enum
{
	_1byte = 0,
	_2byte = 1
} nrf24l01p_crc_length_t;

typedef void (*nrf24l01p_set_cs_t)(uint8_t state);
typedef void (*nrf24l01p_set_ce_t)(uint8_t state);
typedef uint8_t (*nrf24l01p_spi_tx_t)(uint8_t register_address);
typedef uint8_t (*nrf24l01p_spi_rx_t)(uint8_t* register_value);
typedef uint8_t (*nrf24l01p_spi_tx_rx_t)(uint8_t register_address, uint8_t* register_value);

typedef struct{
	nrf24l01p_set_cs_t set_cs;
	nrf24l01p_set_ce_t set_ce;
	nrf24l01p_spi_tx_t spi_tx;
	nrf24l01p_spi_rx_t spi_rx;
	nrf24l01p_spi_tx_rx_t spi_tx_rx;
} nrf24l01p_interface_t;


typedef struct {
	// Interface
	nrf24l01p_interface_t interface;

	// General settings
	uint16_t channel_MHz; 					// range 2400 to 2525 MHz
	uint8_t address_width;					// range 3 to 5 bytes
	nrf24l01p_data_rate_t data_rate;
	nrf24l01p_crc_length_t crc_length;

	// TX settings
	nrf24l01p_output_power_t output_power;
	uint8_t auto_ack_pipes;					// interpreted as binary, 2 MSBs ignored
	uint8_t auto_retransmit_count;			// range 0 to 15
	uint8_t auto_retransmit_delay_250us;	// range 1 to 16 (in multiples of 250 us)

	// RX settings
	uint8_t data_length;					// range 0 to 32 bytes

} nrf24l01p_config_t;


/* High-level API functions */
int8_t nrf24l01p_init(nrf24l01p_config_t* config);

void nrf24l01p_rx_receive(uint8_t* rx_payload);
void nrf24l01p_tx_transmit(uint8_t* tx_payload);

void nrf24l01p_tx_irq();
void nrf24l01p_reset();

void nrf24l01p_set_prx_mode();
void nrf24l01p_set_ptx_mode();

void nrf24l01p_set_rx_addr(uint8_t index, uint64_t address);
void nrf24l01p_set_tx_addr(uint64_t address);

void nrf24l01p_power_up();
void nrf24l01p_power_down();


/* Low-level API functions */
uint8_t nrf24l01p_get_status();
uint8_t nrf24l01p_get_status_and_clear_IRQ_flags();
uint8_t nrf24l01p_get_fifo_status();

uint8_t nrf24l01p_read_rx_fifo(uint8_t* rx_payload);
uint8_t nrf24l01p_write_tx_fifo(uint8_t* tx_payload);

void nrf24l01p_flush_rx_fifo();
void nrf24l01p_flush_tx_fifo();

void nrf24l01p_clear_rx_dr();
void nrf24l01p_clear_tx_ds();
void nrf24l01p_clear_max_rt();

void nrf24l01p_set_crc_length(nrf24l01p_crc_length_t length);
void nrf24l01p_set_address_widths(uint8_t address_width);
void nrf24l01p_set_auto_retransmit_count(uint8_t count);
void nrf24l01p_set_auto_retransmit_delay(uint8_t delay_250us);
void nrf24l01p_set_auto_ack_pipes(uint8_t pipes);
void nrf24l01p_set_rf_channel(uint16_t channel_MHz);
void nrf24l01p_set_rf_data_rate(nrf24l01p_data_rate_t data_rate);
void nrf24l01p_set_rf_tx_output_power(nrf24l01p_output_power_t output_power);
void nrf24l01p_rx_set_payload_length(uint8_t index, uint8_t bytes);


/* nRF24L01+ Commands */
#define NRF24L01P_CMD_R_REGISTER			0b00000000
#define NRF24L01P_CMD_W_REGISTER			0b00100000
#define NRF24L01P_CMD_R_RX_PAYLOAD			0b01100001
#define NRF24L01P_CMD_W_TX_PAYLOAD			0b10100000
#define NRF24L01P_CMD_FLUSH_TX				0b11100001
#define NRF24L01P_CMD_FLUSH_RX				0b11100010
#define NRF24L01P_CMD_REUSE_TX_PL			0b11100011
#define NRF24L01P_CMD_R_RX_PL_WID			0b01100000
#define NRF24L01P_CMD_W_ACK_PAYLOAD			0b10101000
#define NRF24L01P_CMD_W_TX_PAYLOAD_NOACK	0b10110000
#define NRF24L01P_CMD_NOP					0b11111111

/* nRF24L01+ Registers */
#define NRF24L01P_REG_CONFIG            0x00
#define NRF24L01P_REG_EN_AA             0x01
#define NRF24L01P_REG_EN_RXADDR         0x02
#define NRF24L01P_REG_SETUP_AW          0x03
#define NRF24L01P_REG_SETUP_RETR        0x04
#define NRF24L01P_REG_RF_CH             0x05
#define NRF24L01P_REG_RF_SETUP          0x06
#define NRF24L01P_REG_STATUS            0x07
#define NRF24L01P_REG_OBSERVE_TX        0x08    // Read-Only
#define NRF24L01P_REG_RPD               0x09    // Read-Only
#define NRF24L01P_REG_RX_ADDR_P0        0x0A
#define NRF24L01P_REG_RX_ADDR_P1        0x0B
#define NRF24L01P_REG_RX_ADDR_P2        0x0C
#define NRF24L01P_REG_RX_ADDR_P3        0x0D
#define NRF24L01P_REG_RX_ADDR_P4        0x0E
#define NRF24L01P_REG_RX_ADDR_P5        0x0F
#define NRF24L01P_REG_TX_ADDR           0x10
#define NRF24L01P_REG_RX_PW_P0          0x11
#define NRF24L01P_REG_RX_PW_P1          0x12
#define NRF24L01P_REG_RX_PW_P2          0x13
#define NRF24L01P_REG_RX_PW_P3          0x14
#define NRF24L01P_REG_RX_PW_P4          0x15
#define NRF24L01P_REG_RX_PW_P5          0x16
#define NRF24L01P_REG_FIFO_STATUS       0x17
#define NRF24L01P_REG_DYNPD             0x1C
#define NRF24L01P_REG_FEATURE           0x1D


#endif /* __NRF24L01P_H__ */
