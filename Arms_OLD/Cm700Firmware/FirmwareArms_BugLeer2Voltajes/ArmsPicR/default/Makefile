###############################################################################
# Makefile for the project ArmsPic
###############################################################################

## General Flags
PROJECT = ArmsPic
MCU = atmega2561
TARGET = ArmsPic.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99 -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -DF_CPU=16000000UL
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -wl  -u  vfprintf -Wl,-Map=ArmsPic.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"C:\Users\biorrobotica\Desktop\laura\Brazos Justina\FirmwareArms\ArmsPicR newServos\include" -I"C:\Users\biorrobotica\Desktop\laura\Brazos Justina\FirmwareArms\ArmsPicR newServos\lib" -I"C:\Users\biorrobotica\Desktop\laura\Brazos Justina\FirmwareArms\ArmsPicR newServos\src" -I"C:\Users\biorrobotica\Desktop\laura\Brazos Justina\FirmwareArms\ArmsPicR newServos\src\Serial" -I"C:\Users\biorrobotica\Desktop\laura\Brazos Justina\FirmwareArms\ArmsPicR newServos\src\Dynamixel" -I"C:\Users\biorrobotica\Desktop\laura\Brazos Justina\FirmwareArms\ArmsPicR newServos\src\ZigBee" 

## Library Directories
LIBDIRS = -L"C:\Users\biorrobotica\Desktop\laura\Brazos Justina\FirmwareArms\ArmsPicR newServos\lib" 

## Libraries
LIBS = -lprintf_flt -lm -lprintf_min -lscanf_flt -lscanf_min -lc -lserial -ldynamixel 

## Objects that must be built in order to link
OBJECTS = ArmsPic.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) ArmsPic.hex ArmsPic.eep ArmsPic.lss size

## Compile
ArmsPic.o: ../ArmsPic.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) ArmsPic.elf dep/* ArmsPic.hex ArmsPic.eep ArmsPic.lss ArmsPic.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

