# BMP 388 SPI Driver

This folder contains a simple SPI driver for the bmp388 barometer with interrupt support.



## Usage

The sensor can be initalized using `bmp388_init(cs_pin, int_pin, ord_sel)`. If no interrupt pin or ODR (Output Data Rate) are selected, they will default to `BMP388_NO_INT` and `MP388_200HZ_ODR` respectively. See `bmp388.h` for more predefined ODR options.

For any given ODR, the "best" available oversampling rate will be automatically applied, see the table in `bmp388.h` for more details.

Once initalized, temperature (C) and pressure (Pa) can be retrieved using `bmp_388_read()`. Temperature and pressure are automatically compensated.

Altitude can be calculated using the helper `bmp388_altitude(press_pa, ref_pa, ref_c)` which uses the ISA troposphere equation to approximate altitude.

The status register of the BMP388 can be queried using `bmp388_get_status()`, refer to the [datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf) for more information.