#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__

#include "spi.h"    // header from STM32CubeMX code generator, expects LL API

/*----------- nRF24L01+ Commands -----------*/
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

/*----------- nRF24L01+ Registers -----------*/
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

/*----------- nRF24L01+ Register Reset Values -----------*/
#define NRF24L01P_REG_CONFIG_RSTVAL            0x08
#define NRF24L01P_REG_EN_AA_RSTVAL             0x3F
#define NRF24L01P_REG_EN_RXADDR_RSTVAL         0x03
#define NRF24L01P_REG_SETUP_AW_RSTVAL          0x03
#define NRF24L01P_REG_SETUP_RETR_RSTVAL        0x03
#define NRF24L01P_REG_RF_CH_RSTVAL             0x02
#define NRF24L01P_REG_RF_SETUP_RSTVAL          0x0E
#define NRF24L01P_REG_STATUS_RSTVAL            0x0E
#define NRF24L01P_REG_RX_ADDR_P0_RSTVAL        0xE7E7E7E7E7
#define NRF24L01P_REG_RX_ADDR_P1_RSTVAL        0xC2C2C2C2C2
#define NRF24L01P_REG_RX_ADDR_P2_RSTVAL        0xC3
#define NRF24L01P_REG_RX_ADDR_P3_RSTVAL        0xC4
#define NRF24L01P_REG_RX_ADDR_P4_RSTVAL        0xC5
#define NRF24L01P_REG_RX_ADDR_P5_RSTVAL        0xC6
#define NRF24L01P_REG_TX_ADDR_RSTVAL           0xE7E7E7E7E7
#define NRF24L01P_REG_RX_PW_P0_RSTVAL          0x00
#define NRF24L01P_REG_RX_PW_P1_RSTVAL          0x00
#define NRF24L01P_REG_RX_PW_P2_RSTVAL          0x00
#define NRF24L01P_REG_RX_PW_P3_RSTVAL          0x00
#define NRF24L01P_REG_RX_PW_P4_RSTVAL          0x00
#define NRF24L01P_REG_RX_PW_P5_RSTVAL          0x00
#define NRF24L01P_REG_FIFO_STATUS_RSTVAL       0x11
#define NRF24L01P_REG_DYNPD_RSTVAL             0x00
#define NRF24L01P_REG_FEATURE_RSTVAL           0x00

/*----------- nRF24L01+ Register Bits -----------*/
// CONFIG
#define NRF24L01P_REG_CONFIG_MASK_RX_DR			(1U << 6)
#define NRF24L01P_REG_CONFIG_MASK_TX_DS			(1U << 5)
#define NRF24L01P_REG_CONFIG_MASK_MAX_RT		(1U << 4)
#define NRF24L01P_REG_CONFIG_EN_CRC				(1U << 3)
#define NRF24L01P_REG_CONFIG_CRCO				(1U << 2)
#define NRF24L01P_REG_CONFIG_PWR_UP				(1U << 1)
#define NRF24L01P_REG_CONFIG_PRIM_RX			(1U << 0)
// EN_AA
#define NRF24L01P_REG_EN_AA_ENAA_P5				(1U << 5)
#define NRF24L01P_REG_EN_AA_ENAA_P4				(1U << 4)
#define NRF24L01P_REG_EN_AA_ENAA_P3				(1U << 3)
#define NRF24L01P_REG_EN_AA_ENAA_P2				(1U << 2)
#define NRF24L01P_REG_EN_AA_ENAA_P1				(1U << 1)
#define NRF24L01P_REG_EN_AA_ENAA_P0				(1U << 0)
// EN_RXADDR
#define NRF24L01P_REG_EN_RXADDR_ERX_P5			(1U << 5)
#define NRF24L01P_REG_EN_RXADDR_ERX_P4			(1U << 4)
#define NRF24L01P_REG_EN_RXADDR_ERX_P3			(1U << 3)
#define NRF24L01P_REG_EN_RXADDR_ERX_P2			(1U << 2)
#define NRF24L01P_REG_EN_RXADDR_ERX_P1			(1U << 1)
#define NRF24L01P_REG_EN_RXADDR_ERX_P0			(1U << 0)
// SETUP_AW
#define NRF24L01P_REG_SETUP_AW_AW_POS			(1U << 0)
#define NRF24L01P_REG_SETUP_AW_AW_MASK			(0b11 << 0)
// SETUP_RETR
#define NRF24L01P_REG_SETUP_RETR_ARD_POS		(1U << 4)
#define NRF24L01P_REG_SETUP_RETR_ARD_MASK		(0b1111 << 4)
#define NRF24L01P_REG_SETUP_RETR_ARC_POS		(1U << 0)
#define NRF24L01P_REG_SETUP_RETR_ARC_MASK		(0b1111 << 0)
// RF_CH
#define NRF24L01P_REG_RF_CH_POS					(1U << 0)
#define NRF24L01P_REG_RF_CH_MASK				(0b1111111 << 0)
// RF_SETUP
#define NRF24L01P_REG_RF_SETUP_CONT_WAVE		(1U << 7)
#define NRF24L01P_REG_RF_SETUP_RF_DR_LOW		(1U << 5)
#define NRF24L01P_REG_RF_SETUP_PLL_LOCK			(1U << 4)
#define NRF24L01P_REG_RF_SETUP_RF_DR_HIGH		(1U << 3)
#define NRF24L01P_REG_RF_SETUP_RF_PWR_POS		(1U << 1)
#define NRF24L01P_REG_RF_SETUP_RF_PWR_MASK		(0b11 << 1)
// RF_STATUS
#define NRF24L01P_REG_STATUS_RX_DR				(1U << 6)
#define NRF24L01P_REG_STATUS_TX_DS				(1U << 5)
#define NRF24L01P_REG_STATUS_MAX_RT				(1U << 4)
#define NRF24L01P_REG_STATUS_RX_P_NO_POS		(1U << 1)
#define NRF24L01P_REG_STATUS_RX_P_NO_MASK		(0b111 << 1)
#define NRF24L01P_REG_STATUS_TX_FULL			(1U << 0)
// OBSERVE_TX
#define NRF24L01P_REG_OBSERVE_TX_PLOS_CNT_POS	(1U << 4)
#define NRF24L01P_REG_OBSERVE_TX_PLOS_CNT_MASK	(0b1111 << 4)
#define NRF24L01P_REG_OBSERVE_TX_ARC_CNT_POS	(1U << 0)
#define NRF24L01P_REG_OBSERVE_TX_ARC_CNT_MASK	(0b1111 << 0)
// RPD
#define NRF24L01P_REG_RPD_RPD					(1U << 0)
// RX_PW_P0
#define NRF24L01P_REG_RX_PW_P0_POS				(1U << 0)
#define NRF24L01P_REG_RX_PW_P0_MASK				(0b111111 << 0)
// RX_PW_P1
#define NRF24L01P_REG_RX_PW_P1_POS				(1U << 0)
#define NRF24L01P_REG_RX_PW_P1_MASK				(0b111111 << 0)
// RX_PW_P2
#define NRF24L01P_REG_RX_PW_P2_POS				(1U << 0)
#define NRF24L01P_REG_RX_PW_P2_MASK				(0b111111 << 0)
// RX_PW_P3
#define NRF24L01P_REG_RX_PW_P3_POS				(1U << 0)
#define NRF24L01P_REG_RX_PW_P3_MASK				(0b111111 << 0)
// RX_PW_P4
#define NRF24L01P_REG_RX_PW_P4_POS				(1U << 0)
#define NRF24L01P_REG_RX_PW_P4_MASK				(0b111111 << 0)
// RX_PW_P5
#define NRF24L01P_REG_RX_PW_P5_POS				(1U << 0)
#define NRF24L01P_REG_RX_PW_P5_MASK				(0b111111 << 0)
// FIFO_STATUS
#define NRF24L01P_REG_FIFO_STATUS_TX_REUSE		(1U << 6)
#define NRF24L01P_REG_FIFO_STATUS_TX_FULL		(1U << 5)
#define NRF24L01P_REG_FIFO_STATUS_TX_EMPTY		(1U << 4)
#define NRF24L01P_REG_FIFO_STATUS_RX_FULL		(1U << 1)
#define NRF24L01P_REG_FIFO_STATUS_RX_EMPTY		(1U << 0)
// DYNPD
#define NRF24L01P_REG_DYNPD_DPL_P5				(1U << 5)
#define NRF24L01P_REG_DYNPD_DPL_P4				(1U << 4)
#define NRF24L01P_REG_DYNPD_DPL_P3				(1U << 3)
#define NRF24L01P_REG_DYNPD_DPL_P2				(1U << 2)
#define NRF24L01P_REG_DYNPD_DPL_P1				(1U << 1)
#define NRF24L01P_REG_DYNPD_DPL_P0				(1U << 0)
// FEATURE
#define NRF24L01P_REG_FEATURE_EN_DPL			(1U << 2)
#define NRF24L01P_REG_FEATURE_EN_ACK_PAY		(1U << 1)
#define NRF24L01P_REG_FEATURE_EN_DYN_ACK		(1U << 0)

/*----------- nRF24L01+ Enums, Structs and Function Pointers -----------*/
typedef enum {
	NRF24L01P_SUCCESS = 0,
	NRF24L01P_SPI_ERROR = -1,
	NRF24L01P_INVALID_VALUE = -2
} nrf24l01p_error_t;

typedef struct {
	uint8_t rx_dr;		// data received
	uint8_t tx_ds;		// transmission successful
	uint8_t max_rt;		// maximum retransmit attempts reached
} nrf24l01p_irq_t;

typedef struct {
	uint8_t address;
	uint8_t value;
} nrf24l01p_8bit_reg_rst_vals_t;

extern nrf24l01p_8bit_reg_rst_vals_t reg_rst_vals_8bit[];	// initialized in .c

typedef struct {
	uint8_t address;
	uint64_t value;
} nrf24l01p_40bit_reg_rst_vals_t;

extern nrf24l01p_40bit_reg_rst_vals_t reg_rst_vals_40bit[];	// initialized in .c

typedef enum
{
	// member values reflect physical register map encoding
    NRF24L01P_250KBPS = (1U << 5),	// 250 kbps
    NRF24L01P_1MBPS   = 0U,			// 1 Mbps
    NRF24L01P_2MBPS   = (1U << 3)	// 2 Mbps
} nrf24l01p_data_rate_t;

typedef enum
{
	// member values reflect physical register map encoding
    NRF24L01P_0DBM  	= (3U << 1),	// 0 dBm
    NRF24L01P_NEG6DBM  	= (2U << 1),	// -6 dBm
    NRF24L01P_NEG12DBM 	= (1U << 1),	// -12 dBm
    NRF24L01P_NEG18DBM 	= (0U << 1)		// -18 dBm
} nrf24l01p_output_power_t;

typedef enum
{
	// member values reflect physical register map encoding
	NRF24L01P_CRC_1BYTE = (0U << 2),
	NRF24L01P_CRC_2BYTE = (1U << 2)
} nrf24l01p_crc_length_t;

// Argument state = 0 for low, 1 for high
typedef void (*nrf24l01p_set_cs_t)(uint8_t state);
typedef void (*nrf24l01p_set_ce_t)(uint8_t state);

// Return value of following functions is error code, 0 is only accepted success value
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

// Handler struct for use in application code
typedef struct {
	// Interface
	nrf24l01p_interface_t interface;

	// General settings
	uint16_t channel_MHz; 					// range 2400 to 2525 MHz
	uint8_t address_width;					// range 3 to 5 bytes
	nrf24l01p_data_rate_t data_rate;
	nrf24l01p_crc_length_t crc_length;
	uint8_t data_length;					// range 0 to 32 bytes

	// TX settings
	nrf24l01p_output_power_t output_power;
	uint8_t auto_ack_pipes;					// interpreted as binary, 2 MSBs ignored
	uint8_t auto_retransmit_count;			// range 0 to 15
	uint8_t auto_retransmit_delay_250us;	// range 1 to 16 (in multiples of 250 us)
} nrf24l01p_config_t;


/*----------- High-level API functions -----------*/
nrf24l01p_error_t nrf24l01p_init(nrf24l01p_config_t* config);

nrf24l01p_error_t nrf24l01p_rx_receive(uint8_t* rx_payload);
nrf24l01p_error_t nrf24l01p_tx_transmit(uint8_t* tx_payload);

nrf24l01p_error_t nrf24l01p_irq(nrf24l01p_irq_t* irq_sources);
nrf24l01p_error_t nrf24l01p_reset();

nrf24l01p_error_t nrf24l01p_set_prx_mode();
nrf24l01p_error_t nrf24l01p_set_ptx_mode();

nrf24l01p_error_t nrf24l01p_set_rx_addr(uint8_t index, uint64_t address);
nrf24l01p_error_t nrf24l01p_set_tx_addr(uint64_t address);

nrf24l01p_error_t nrf24l01p_power_up();
nrf24l01p_error_t nrf24l01p_power_down();


/*----------- Low-level API functions -----------*/
nrf24l01p_error_t nrf24l01p_get_status();
nrf24l01p_error_t nrf24l01p_get_status_and_clear_IRQ_flags();
nrf24l01p_error_t nrf24l01p_clear_flag(uint8_t flag);
nrf24l01p_error_t nrf24l01p_get_fifo_status();

nrf24l01p_error_t nrf24l01p_read_rx_fifo(uint8_t* rx_payload);
nrf24l01p_error_t nrf24l01p_write_tx_fifo(uint8_t* tx_payload);
nrf24l01p_error_t nrf24l01p_flush_rx_fifo();
nrf24l01p_error_t nrf24l01p_flush_tx_fifo();

nrf24l01p_error_t nrf24l01p_set_crc_length(nrf24l01p_crc_length_t length);
nrf24l01p_error_t nrf24l01p_set_address_width(uint8_t address_width);
nrf24l01p_error_t nrf24l01p_set_auto_retransmit_count(uint8_t count);
nrf24l01p_error_t nrf24l01p_set_auto_retransmit_delay(uint8_t delay_250us);
nrf24l01p_error_t nrf24l01p_set_auto_ack_pipes(uint8_t pipes);
nrf24l01p_error_t nrf24l01p_set_rf_channel(uint16_t channel_MHz);
nrf24l01p_error_t nrf24l01p_set_rf_data_rate(nrf24l01p_data_rate_t data_rate);
nrf24l01p_error_t nrf24l01p_set_rf_tx_output_power(nrf24l01p_output_power_t output_power);
nrf24l01p_error_t nrf24l01p_rx_set_payload_length(uint8_t index, uint8_t bytes);

#endif /* __NRF24L01P_H__ */
