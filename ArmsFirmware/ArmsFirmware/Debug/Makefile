################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../ArmsFirmware.c \
../cmdMan.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
ArmsFirmware.o \
cmdMan.o

OBJS_AS_ARGS +=  \
ArmsFirmware.o \
cmdMan.o

C_DEPS +=  \
ArmsFirmware.d \
cmdMan.d

C_DEPS_AS_ARGS +=  \
ArmsFirmware.d \
cmdMan.d

OUTPUT_FILE_PATH +=ArmsFirmware.elf

OUTPUT_FILE_PATH_AS_ARGS +=ArmsFirmware.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+=  \
D:/Justina\ Apps/Sources/Arms/CM700\ Firmware/embeded_c(cm700_v1.02)/lib/libdynamixel.a \
D:/Justina\ Apps/Sources/Arms/CM700\ Firmware/embeded_c(cm700_v1.02)/lib/libzigbee.a

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler





./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.3.3
	$(QUOTE)C:\WinAVR\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DF_CPU=16000000UL  -I"../../../embeded_c(cm700_v1.02)/include"  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -g2 -Wall -mmcu=atmega2561 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 4.3.3
	$(QUOTE)C:\WinAVR\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="ArmsFirmware.map" -Wl,--start-group -Wl,-ldynamixel -Wl,-lzigbee -Wl,-lc  -Wl,--end-group -Wl,-L"D:\Justina Apps\Sources\Arms\CM700 Firmware\embeded_c(cm700_v1.02)\lib"  -Wl,--gc-sections -mrelax -mmcu=atmega2561  
	@echo Finished building target: $@
	"C:\WinAVR\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "ArmsFirmware.elf" "ArmsFirmware.hex"
	"C:\WinAVR\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "ArmsFirmware.elf" "ArmsFirmware.eep" || exit 0
	"C:\WinAVR\bin\avr-objdump.exe" -h -S "ArmsFirmware.elf" > "ArmsFirmware.lss"
	"C:\WinAVR\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "ArmsFirmware.elf" "ArmsFirmware.srec"
	"C:\WinAVR\bin\avr-size.exe" "ArmsFirmware.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "ArmsFirmware.elf" "ArmsFirmware.a" "ArmsFirmware.hex" "ArmsFirmware.lss" "ArmsFirmware.eep" "ArmsFirmware.map" "ArmsFirmware.srec" "ArmsFirmware.usersignatures"
	