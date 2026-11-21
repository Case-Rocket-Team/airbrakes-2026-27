#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "bmp388.h"

#define ISA_LAPSE_RATE          0.0065f   // K/m
#define ISA_BAROMETRIC_EXPONENT 0.190263f // R*L / (g0*M)

// Register map (datasheet section 5)
#define BMP388_REG_CHIP_ID    0x00
#define BMP388_REG_ERR        0x02
#define BMP388_REG_STATUS     0x03
#define BMP388_REG_DATA       0x04 // pressure (3 bytes), temperature (3 bytes)
#define BMP388_REG_INT_STATUS 0x11
#define BMP388_REG_INT_CTRL   0x19
#define BMP388_REG_PWR_CTRL   0x1B
#define BMP388_REG_OSR        0x1C
#define BMP388_REG_ODR        0x1D
#define BMP388_REG_CONFIG     0x1F
#define BMP388_REG_CALIB      0x31 // 21 bytes of trim coefficients
#define BMP388_REG_CMD        0x7E

#define BMP388_CHIP_ID        0x50
#define BMP388_CMD_SOFTRESET  0xB6

#define BMP388_SPI_READ       0x80 // top bit set == read

// ERR_REG bits
#define BMP388_ERR_CONF       (1 << 2)

// PWR_CTRL bits
#define BMP388_PWR_PRESS_EN   (1 << 0)
#define BMP388_PWR_TEMP_EN    (1 << 1)
#define BMP388_PWR_MODE_SLEEP  (0x0 << 4)
#define BMP388_PWR_MODE_NORMAL (0x3 << 4)

// INT_CTRL bits
#define BMP388_INT_LEVEL_HIGH (1 << 1)
#define BMP388_INT_LATCH      (1 << 2)
#define BMP388_INT_DRDY_EN    (1 << 6)

// CONFIG: IIR filter coefficient select, bits 3:1
#define BMP388_IIR_COEFF_1    (0x1 << 1)

static uint8_t           cs;
static uint8_t           irq_pin = BMP388_NO_INT;
static const SPISettings spi_settings(8000000, MSBFIRST, SPI_MODE0);

// Calibration Values
static double t1, t2, t3;
static double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;

static void load_calibration(void);

static void read_reg(uint8_t reg, uint8_t *buf, int len);

static uint8_t        isr_sample[6];
static volatile bool  drdy = false;

static void bmp388_isr(void) {
    uint8_t status;
    read_reg(BMP388_REG_INT_STATUS, &status, 1); // releases the latched INT pin

    read_reg(BMP388_REG_DATA, isr_sample, sizeof(isr_sample));
    drdy = true;
}

static void read_reg(uint8_t reg, uint8_t *buf, int len) {
    SPI.beginTransaction(spi_settings);
    digitalWrite(cs, LOW);

    SPI.transfer(reg | BMP388_SPI_READ);
    SPI.transfer(0x00);       // first byte is a junk byte

    for (int i = 0; i < len; i++) {
        buf[i] = SPI.transfer(0x00);
    }

    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}

static void write_reg(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(spi_settings);
    digitalWrite(cs, LOW);

    SPI.transfer(reg & ~BMP388_SPI_READ); // top bit clear == write
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
    read_reg(BMP388_REG_CHIP_ID, &id, 1);

    write_reg(BMP388_REG_CMD, BMP388_CMD_SOFTRESET);
    delay(10);

    read_reg(BMP388_REG_CHIP_ID, &id, 1);
    if (id != BMP388_CHIP_ID) {
        return false;
    }

    load_calibration();

    if (int_pin != BMP388_NO_INT) {
        // INT_CTRL: push-pull, active high, LATCHED, data-ready enabled.
        write_reg(BMP388_REG_INT_CTRL,
                  BMP388_INT_LEVEL_HIGH | BMP388_INT_LATCH | BMP388_INT_DRDY_EN);

        uint8_t status;
        read_reg(BMP388_REG_INT_STATUS, &status, 1); // clear any stale flag

        irq_pin = int_pin;
        drdy    = false;
        pinMode(int_pin, INPUT_PULLDOWN);

        SPI.usingInterrupt(digitalPinToInterrupt(int_pin));
        attachInterrupt(digitalPinToInterrupt(int_pin), bmp388_isr, RISING);
    }

    write_reg(BMP388_REG_CONFIG, BMP388_IIR_COEFF_1);

    // Sets ODR + matching OSR and enters normal mode.
    bmp388_set_odr(odr_sel);
    return true;
}

// Highest oversampling (OSR register 0x1C value) whose conversion time fits
// in 1/ODR for each odr_sel. See table in bmp388.h.
static uint8_t osr_for_odr(uint8_t odr_sel) {
    static const uint8_t osr[] = {
        0x00, // 200 Hz:  P x1,  T x1
        0x09, // 100 Hz:  P x2,  T x2
        0x03, // 50 Hz:   P x8,  T x1
        0x0C, // 25 Hz:   P x16, T x2
        0x15, // 12.5 Hz: P x32, T x4
    };
    const uint8_t n = sizeof(osr) / sizeof(osr[0]);
    // Slower rates than 12.5 Hz get the same (already maximum) pressure OSR.
    return osr[odr_sel < n ? odr_sel : n - 1];
}

// True when a new sample is waiting for bmp388_read().
bool bmp388_data_ready(void) {
    if (irq_pin == BMP388_NO_INT) {
        return bmp388_get_status() & BMP388_STATUS_DRDY_PRESS;
    }
    return drdy;
}

// Temperature compensation (9.2returns).
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

bmp388_data bmp388_read(void) {
    uint8_t d[6];

    if (irq_pin != BMP388_NO_INT) {
        // Copy the ISR's latest sample
        noInterrupts();
        memcpy(d, isr_sample, sizeof(d));
        drdy = false;
        interrupts();
    } else {
        read_reg(BMP388_REG_DATA, d, 6);
    }

    double up = d[0] | d[1] << 8 | (uint32_t)d[2] << 16;
    double ut = d[3] | d[4] << 8 | (uint32_t)d[5] << 16;

    double t = bmp388_compensate_temperature(ut);
    double p = bmp388_compensate_pressure(up, t);

    bmp388_data data;
    data.temp_c   = (float)t;
    data.press_pa = (float)p;
    return data;
}

// Set output data rate (4.3.19) and the matching oversampling.
// odr_sel: 0 = 200 Hz, 1 = 100 Hz, 2 = 50 Hz, 3 = 25 Hz, ...
void bmp388_set_odr(uint8_t odr_sel) {
    if (odr_sel > BMP388_ODR_SLOWEST) {
        odr_sel = BMP388_ODR_SLOWEST;
    }

    write_reg(BMP388_REG_PWR_CTRL,
              BMP388_PWR_PRESS_EN | BMP388_PWR_TEMP_EN | BMP388_PWR_MODE_SLEEP);
    write_reg(BMP388_REG_OSR, osr_for_odr(odr_sel));
    write_reg(BMP388_REG_ODR, odr_sel);
    write_reg(BMP388_REG_PWR_CTRL,
              BMP388_PWR_PRESS_EN | BMP388_PWR_TEMP_EN | BMP388_PWR_MODE_NORMAL);

    // ERR_REG (0x02): bit 2 = conf_err, set if ODR/OSR settings are invalid
    // (e.g. Tconv doesn't fit in 1/ODR).
    uint8_t err;
    read_reg(BMP388_REG_ERR, &err, 1);

    // Fall back to the slowest rate, where every OSR fits in 1/ODR.
    // The odr_sel check stops it recursing more than once.
    if ((err & BMP388_ERR_CONF) && odr_sel != BMP388_ODR_SLOWEST) {
        bmp388_set_odr(BMP388_ODR_SLOWEST);
    }
}

// Read the STATUS register (0x03).
uint8_t bmp388_get_status(void) {
    uint8_t status;
    read_reg(BMP388_REG_STATUS, &status, 1);
    return status;
}

// Load trim coefficients and apply datasheet scaling.
static void load_calibration(void) {
    uint8_t c[21];
    read_reg(BMP388_REG_CALIB, c, 21);

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

// ISA barometric formula, troposphere (0-11 km):
//   h = (T_ref / L) * (1 - (p / p_ref)^(R*L / (g0*M)))
float bmp388_altitude_m(float press_pa, float ref_press_pa, float ref_temp_c) {
    return ((ref_temp_c + 273.15f) / ISA_LAPSE_RATE)
           * (1.0f - powf(press_pa / ref_press_pa, ISA_BAROMETRIC_EXPONENT));
}
