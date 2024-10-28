#ifndef INC_SHT4X_H_
#define INC_SHT4X_H_

#include <stdint.h>		// definition of uint8_t etc

#define SHT4X_I2C_ADDR_A 	(0x44 << 1)		// if device product name 7th position is 'A'
#define SHT4X_I2C_ADDR_B 	(0x45 << 1)		// if device product name 7th position is 'B'

#define SHT4X_CRC8_POLYNOMIAL	0x31		// x^8 + x^5 + x^4 + 1, initialization to 0xFF

#define SHT4X_I2C_CMD_MEAS_HIGH_PREC 	0xFD	// measurement duration up to 8.2 ms
#define SHT4X_I2C_CMD_MEAS_MED_PREC 	0xF6	// measurement duration up to 4.5 ms
#define SHT4X_I2C_CMD_MEAS_LOW_PREC 	0xE0	// measurement duration up to 1.7 ms
#define SHT4X_I2C_CMD_READ_SERIAL_NUM 	0x89	// two 16 bit words (32 bit total)
#define SHT4X_I2C_CMD_SOFT_RESET 		0x94	// soft reset time 1 ms
#define SHT4X_I2C_CMD_HEAT_200MW_1S 	0x39	// automatically starts high precision measurement
#define SHT4X_I2C_CMD_HEAT_200MW_0P1S 	0x32	// automatically starts high precision measurement
#define SHT4X_I2C_CMD_HEAT_110MW_1S 	0x2F	// automatically starts high precision measurement
#define SHT4X_I2C_CMD_HEAT_110MW_0P1S 	0x24	// automatically starts high precision measurement
#define SHT4X_I2C_CMD_HEAT_20MW_1S 		0x1E	// automatically starts high precision measurement
#define SHT4X_I2C_CMD_HEAT_20MW_0P1S 	0x15	// automatically starts high precision measurement

typedef enum {
	SHT4X_SUCCESS = 0,
	SHT4X_I2C_ERROR = -1,
	SHT4X_INVALID_VALUE = -2,
	SHT4X_INVALID_OPERATION = -3,
	SHT4X_POINTER_NULL = -4,
	SHT4X_CRC_FAILURE = -5
} sht4x_error_t;

typedef struct {
	uint8_t temperature_raw[2];		// 0 is MSB byte
	uint8_t temperature_crc;
	uint8_t humidity_raw[2];		// 0 is MSB byte
	uint8_t humidity_crc;
} sht4x_raw_data_t;

typedef struct {
	uint16_t serial_msb;
	uint16_t serial_lsb;
	uint8_t msb_crc;
	uint8_t lsb_crc;
} sht4x_serial_number_t;

typedef struct {
	uint32_t temperature;	// convert to degrees Celsius via division by 1000
	uint32_t humidity;		// convert to % RH via division by 1000
} sht4x_data_t;

// Return value of following functions is error code, 0 is only accepted success value
typedef uint8_t (*sht4x_i2c_write_t)(uint8_t address, uint8_t payload);
typedef uint8_t (*sht4x_i2c_read_t)(uint8_t address, uint8_t* payload, uint8_t length);
typedef uint8_t (*sht4x_calculate_crc_t)(uint8_t* data, uint8_t length, uint8_t polynomial);

typedef struct {
	uint8_t i2c_address;
	sht4x_i2c_write_t i2c_write;
	sht4x_i2c_read_t i2c_read;
	sht4x_calculate_crc_t calculate_crc;
} sht4x_device_t;

sht4x_error_t sht4x_send_command(sht4x_device_t* device, uint8_t command);
sht4x_error_t sht4x_read_measurement(sht4x_device_t* device, sht4x_data_t* data);
sht4x_error_t sht4x_read_serial_number(sht4x_device_t* device, sht4x_serial_number_t* serial_number);

sht4x_error_t sht4x_read_raw_measurement(sht4x_device_t* device, sht4x_raw_data_t* raw_data);
sht4x_error_t sht4x_convert_raw_data(sht4x_device_t* device, sht4x_raw_data_t* raw_data, sht4x_data_t* data);


#endif /* INC_SHT4X_H_ */
