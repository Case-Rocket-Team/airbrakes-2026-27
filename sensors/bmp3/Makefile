TARGET   := test_bmp388
CORE     := ../../teensy4
BUILD    := build
PREFIX   := /usr/bin/arm-none-eabi-

CC       := $(PREFIX)gcc
CXX      := $(PREFIX)g++
OBJCOPY  := $(PREFIX)objcopy
SIZE     := $(PREFIX)size

MCU_FLAGS := -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb
DEFINES   := -DF_CPU=600000000 -DUSB_SERIAL -DLAYOUT_US_ENGLISH \
             -D__IMXRT1062__ -DARDUINO=10813 -DTEENSYDUINO=159 -DARDUINO_TEENSY41

CPPFLAGS  := -Wall -g -O2 -MMD -ffunction-sections -fdata-sections -I$(CORE) -I. $(MCU_FLAGS) $(DEFINES)
CXXFLAGS  := -std=gnu++17 -felide-constructors -fno-exceptions -fpermissive -fno-rtti -Wno-error=narrowing
LDFLAGS   := -Os -Wl,--gc-sections,--relax $(MCU_FLAGS) -T$(CORE)/imxrt1062_t41.ld
LIBS      := -larm_cortexM7lfsp_math -lm -lstdc++

SRCS := $(wildcard $(CORE)/*.c) \
        $(wildcard $(CORE)/*.cpp) \
        bmp388.cpp test_bmp388.cpp

OBJS := $(patsubst %,$(BUILD)/%.o,$(basename $(notdir $(SRCS))))
VPATH := $(CORE) .

all: $(BUILD)/$(TARGET).hex

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(BUILD)/%.o: %.c
	@mkdir -p $(BUILD)
	$(CC) $(CPPFLAGS) -c -o $@ $<

$(BUILD)/%.o: %.cpp
	@mkdir -p $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

upload: $(BUILD)/$(TARGET).hex
	teensy_loader_cli --mcu=TEENSY41 -w -v $<

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)
.PHONY: all upload clean
