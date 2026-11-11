#include <Arduino.h>
#include <SPI.h>
#include "bmp388.h"

static uint8_t cs;
static const SPISettings spi_settings(8000000, MSBFIRST, SPI_MODE0);

// Calibration 
static double t1, t2, t3;
static double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;

static void load_calibration(void);

// Set by the data-ready ISR, cleared by bmp388_data_ready().
static volatile bool drdy = false;

static void bmp388_isr(void) {
    drdy = true;
}

static void read_reg(uint8_t reg, uint8_t *buf, int len) {
    SPI.beginTransaction(spi_settings);
    digitalWrite(cs, LOW);

    SPI.transfer(reg | 0x80); // top bit set = read
    SPI.transfer(0x00); // junk byte first

    for (int i = 0; i < len; i++) {
        buf[i] = SPI.transfer(0x00);
    }

    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}

static void write_reg(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(spi_settings);
    digitalWrite(cs, LOW);

    SPI.transfer(reg & 0x7F); // top bit clear = write
    SPI.transfer(val);

    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}

// If int_pin != BMP388_NO_INT, the sensor's data-ready interrupt is enabled.
bool bmp388_init(uint8_t cs_pin, uint8_t int_pin, uint8_t odr_sel) {
    cs = cs_pin;
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    SPI.begin();

    // Dummy read
    uint8_t id;
    read_reg(0x00, &id, 1);

    write_reg(0x7E, 0xB6); // soft reset
    delay(10);

    read_reg(0x00, &id, 1);
    if (id != 0x50) {
        return false;
    }

    load_calibration();

    if (int_pin != BMP388_NO_INT) {
        // INT_CTRL: push-pull, active high, non-latched, data-ready enabled
        write_reg(0x19, 0x42);

        uint8_t status;
        read_reg(0x11, &status, 1); // INT_STATUS: clear any stale flag

        drdy = false;
        pinMode(int_pin, INPUT);
        attachInterrupt(digitalPinToInterrupt(int_pin), bmp388_isr, RISING);
    }

    write_reg(0x1B, 0x33); // press_en | temp_en | normal mode
    return true;
}

bool bmp388_data_ready(void) {
    noInterrupts();
    bool r = drdy;
    drdy = false;
    interrupts();
    return r;
}

// Temperature compensation (9.2).
static double bmp388_compensate_temperature(double ut) {
    double x = ut - t1;
    return x * t2 + x * x * t3;
}

// Pressure compensation (9.3). 
static double bmp388_compensate_pressure(double up, double t) {
    return (p5 + p6 * t + p7 * t * t + p8 * t * t * t)
         + up * (p1 + p2 * t + p3 * t * t + p4 * t * t * t)
         + up * up * (p9 + p10 * t)
         + up * up * up * p11; 
}

void bmp388_read(float *temp_c, float *press_pa) {
    uint8_t d[6];
    read_reg(0x04, d, 6); // pressure (3 bytes), temperature (3 bytes)

    double up = d[0] | d[1] << 8 | (uint32_t)d[2] << 16;
    double ut = d[3] | d[4] << 8 | (uint32_t)d[5] << 16;

    double t = bmp388_compensate_temperature(ut);
    double p = bmp388_compensate_pressure(up, t);

    *temp_c = (float)t;
    *press_pa = (float)p;
}

// Set output data rate (4.3.19)
// odr_sel: 0 = 200 Hz, 1 = 100 Hz, 2 = 50 Hz, 3 = 25 Hz, ... 
void bmp388_set_odr(uint8_t odr_sel) {
    if (odr_sel > 0x11) { 
        odr_sel = 0x11;
    }
    write_reg(0x1D, odr_sel);
}

// Read the STATUS register (0x03).
uint8_t bmp388_get_status(void) {
    uint8_t status;
    read_reg(0x03, &status, 1);
    return status;
}

// Load trim coefficients and apply datasheet scaling. (wtf is all this shit)
static void load_calibration(void) {
    uint8_t c[21];
    read_reg(0x31, c, 21);

    uint16_t T1  = (uint16_t)(c[0] | c[1] << 8);
    uint16_t T2  = (uint16_t)(c[2] | c[3] << 8);
    int8_t   T3  = (int8_t)c[4];
    int16_t  P1  = (int16_t)(c[5] | c[6] << 8);
    int16_t  P2  = (int16_t)(c[7] | c[8] << 8);
    int8_t   P3  = (int8_t)c[9];
    int8_t   P4  = (int8_t)c[10];
    uint16_t P5  = (uint16_t)(c[11] | c[12] << 8);
    uint16_t P6  = (uint16_t)(c[13] | c[14] << 8);
    int8_t   P7  = (int8_t)c[15];
    int8_t   P8  = (int8_t)c[16];
    int16_t  P9  = (int16_t)(c[17] | c[18] << 8);
    int8_t   P10 = (int8_t)c[19];
    int8_t   P11 = (int8_t)c[20];

    t1  = T1 * 256.0;
    t2  = T2 / 1073741824.0;                
    t3  = T3 / 281474976710656.0;            
    p1  = (P1 - 16384.0) / 1048576.0;        
    p2  = (P2 - 16384.0) / 536870912.0;      
    p3  = P3 / 4294967296.0;                 
    p4  = P4 / 137438953472.0;               
    p5  = P5 * 8.0;
    p6  = P6 / 64.0;
    p7  = P7 / 256.0;
    p8  = P8 / 32768.0;                      
    p9  = P9 / 281474976710656.0;            
    p10 = P10 / 281474976710656.0;           
    p11 = P11 / 36893488147419103232.0;      
}
