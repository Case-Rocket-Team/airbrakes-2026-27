/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>

extern "C" int main(void)
{
#ifdef USING_MAKEFILE

	// To use Teensy 4.0 without Arduino, simply put your code here.
	// For example:

	// BMP388 on SPI0: SCK=13, MOSI=11, MISO=12 (fixed by the SPI object)
	const uint8_t BMP_CS_PIN  = 10;
	const uint8_t BMP_INT_PIN = 9;

	Serial.begin(115200);
	while (!Serial && millis() < 3000) {}

	if (!bmp388_init(BMP_CS_PIN, BMP_INT_PIN, BMP388_50HZ_ODR)) {
		while (1) {
			Serial.println("BMP388 init failed (check wiring / chip ID)");
			delay(1000);
		}
	}
	Serial.println("BMP388 ok");

	while (1) {
		if (bmp388_data_ready()) {
			float t, p;
			bmp388_read(&t, &p);
			Serial.print(t, 2);
			Serial.print(" C  ");
			Serial.print(p, 1);
			Serial.println(" Pa");
		}
	}


#else
	// Arduino's main() function just calls setup() and loop()....
	setup();
	while (1) {
		loop();
		yield();
	}
#endif
}

// https://forum.pjrc.com/index.php?threads/75084
// https://forum.pjrc.com/index.php?threads/77937/page-3#post-368518
#if 1
namespace __gnu_cxx {
void __verbose_terminate_handler()
{
	while (1) asm ("WFI");
}
}
#endif
