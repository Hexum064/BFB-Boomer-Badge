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
../init.c \
../main.c


PREPROCESSING_SRCS +=  \
../qt_asm_xmega.S


ASM_SRCS += 


OBJS +=  \
init.o \
main.o \
qt_asm_xmega.o

OBJS_AS_ARGS +=  \
init.o \
main.o \
qt_asm_xmega.o

C_DEPS +=  \
init.d \
main.d \
qt_asm_xmega.d

C_DEPS_AS_ARGS +=  \
init.d \
main.d \
qt_asm_xmega.d

OUTPUT_FILE_PATH +=Controller-mxega32e5.elf

OUTPUT_FILE_PATH_AS_ARGS +=Controller-mxega32e5.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+=  \
M:/qTouch_5.00_lib/Atmel_QTouch_Libraries_5.0/Generic_QTouch_Libraries/AVR_Tiny_Mega_XMega/QTouch/library_files/libavrxmega2g1-4qt-k-0rs.a

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler
./init.o: .././init.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 5.4.0
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -DDEBUG -DBOARD=USER_BOARD  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\XMEGAE_DFP\1.2.109\include" -I"../src/ASF/common/boards/user_board" -I"../src/ASF/common/boards" -I"../src/ASF/xmega/utils/preprocessor" -I"../src/ASF/xmega/utils" -I"../src/ASF/common/utils" -I"../src" -I"../src/config"  -O1 -fdata-sections -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g3 -Wall -mmcu=atxmega32e5 -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\XMEGAE_DFP\1.2.109\gcc\dev\atxmega32e5" -c -std=gnu99 -fno-strict-aliasing -Wstrict-prototypes -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith -mrelax -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

./main.o: .././main.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 5.4.0
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -DDEBUG -DBOARD=USER_BOARD  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\XMEGAE_DFP\1.2.109\include" -I"../src/ASF/common/boards/user_board" -I"../src/ASF/common/boards" -I"../src/ASF/xmega/utils/preprocessor" -I"../src/ASF/xmega/utils" -I"../src/ASF/common/utils" -I"../src" -I"../src/config"  -O1 -fdata-sections -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g3 -Wall -mmcu=atxmega32e5 -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\XMEGAE_DFP\1.2.109\gcc\dev\atxmega32e5" -c -std=gnu99 -fno-strict-aliasing -Wstrict-prototypes -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith -mrelax -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	




# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler
./qt_asm_xmega.o: .././qt_asm_xmega.S
	@echo Building file: $<
	@echo Invoking: AVR/GNU Assembler : 5.4.0
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -Wa,-gdwarf2 -x assembler-with-cpp -c -mmcu=atxmega32e5 -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\XMEGAE_DFP\1.2.109\gcc\dev\atxmega32e5" -mrelax -DBOARD=USER_BOARD -I "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\XMEGAE_DFP\1.2.109\include" -I "../src/ASF/common/boards/user_board" -I "../src/ASF/common/boards" -I "../src/ASF/xmega/utils/preprocessor" -I "../src/ASF/xmega/utils" -I "../src/ASF/common/utils" -I "../src" -I "../src/config"  -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -Wa,-g   -o "$@" "$<" 
	@echo Finished building: $<
	





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
	@echo Invoking: AVR/GNU Linker : 5.4.0
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="Controller-mxega32e5.map" -Wl,--start-group -Wl,-lm -Wl,-lavrxmega2g1-4qt-k-0rs  -Wl,--end-group -Wl,-L"M:\qTouch_5.00_lib\Atmel_QTouch_Libraries_5.0\Generic_QTouch_Libraries\AVR_Tiny_Mega_XMega\QTouch\library_files"  -Wl,--gc-sections -mmcu=atxmega32e5 -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\Atmel\XMEGAE_DFP\1.2.109\gcc\dev\atxmega32e5" -Wl,--relax  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "Controller-mxega32e5.elf" "Controller-mxega32e5.hex"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "Controller-mxega32e5.elf" "Controller-mxega32e5.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "Controller-mxega32e5.elf" > "Controller-mxega32e5.lss"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "Controller-mxega32e5.elf" "Controller-mxega32e5.srec"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe" "Controller-mxega32e5.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "Controller-mxega32e5.elf" "Controller-mxega32e5.a" "Controller-mxega32e5.hex" "Controller-mxega32e5.lss" "Controller-mxega32e5.eep" "Controller-mxega32e5.map" "Controller-mxega32e5.srec" "Controller-mxega32e5.usersignatures"
	