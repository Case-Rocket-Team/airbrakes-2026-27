#pragma once

#include <stdint.h>

#define BMP388_NO_INT 0xFF

#define BMP388_200HZ_ODR   0
#define BMP388_100HZ_ODR   1
#define BMP388_50HZ_ODR    2
#define BMP388_25HZ_ODR    3

bool bmp388_init(uint8_t cs_pin, uint8_t int_pin = BMP388_NO_INT,
                 uint8_t odr_sel = BMP388_200HZ_ODR);

bool bmp388_data_ready(void);

// STATUS register (0x03) bits
#define BMP388_STATUS_CMD_RDY    (1 << 4)  // command decoder ready
#define BMP388_STATUS_DRDY_PRESS (1 << 5)  // pressure data ready
#define BMP388_STATUS_DRDY_TEMP  (1 << 6)  // temperature data ready

// Read the raw STATUS register.
uint8_t bmp388_get_status(void);

void bmp388_read(float *temp_c, float *press_pa);

// Set output data rate: 0 = 200 Hz, 1 = 100 Hz, ... each step halves (max 0x11).
void bmp388_set_odr(uint8_t odr_sel);
