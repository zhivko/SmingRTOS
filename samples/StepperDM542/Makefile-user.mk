## Local build configuration
## Parameters configured here will override default and ENV values.
## Uncomment and change examples:

EXTRA_INCDIR = include
EXTRA_INCDIR += /home/kz/git/SmingRTOS/samples/StepperDM542/include/machinetalk/generated
EXTRA_INCDIR += /home/kz/git/SmingRTOS/samples/StepperDM542/include/machinetalk/nanopb
EXTRA_INCDIR += /home/kz/git/SmingRTOS/samples/StepperDM542/include/machinetalk/google

## Add your source directories here separated by space
#MODULES = app

## ESP_HOME sets the path where ESP tools and SDK are located.
## Windows:
# ESP_HOME = c:/Espressif

## MacOS / Linux:
#ESP_HOME = /home/kz/git/ESP8266_RTOS_SDK

## SMING_HOME sets the path where Sming framework is located.
## Windows:
# SMING_HOME = c:/tools/sming/Sming 

## MacOS / Linux
#SMING_HOME = /home/kz/git/Sming_RTOS_POC/sming/sming

## COM port parameter is reqruied to flash firmware correctly.
## Windows: 
# COM_PORT = COM3

## MacOS / Linux:
# COM_PORT = /dev/tty.usbserial

## Com port speed
# COM_SPEED	= 115200

## Configure flash parameters (for ESP12-E and other new boards):
# SPI_MODE = dio

## SPIFFS options
# DISABLE_SPIFFS = 1
SPIFF_FILES = files
RBOOT_SPIFFS_0   = 0x100000
RBOOT_SPIFFS_1   = 0x300000

#### overridable rBoot options ####
## use rboot build mode
RBOOT_ENABLED ?= 1
## enable big flash support (for multiple roms, each in separate 1mb block of flash)
RBOOT_BIG_FLASH ?= 1
## two rom mode (where two roms sit in the same 1mb block of flash)
#RBOOT_TWO_ROMS  ?= 1
## size of the flash chip
SPI_SIZE        ?= 4M
## output file for first rom (.bin will be appended)
#RBOOT_ROM_0     ?= rom0
## input linker file for first rom
#RBOOT_LD_0      ?= rom0.ld
## these next options only needed when using two rom mode
#RBOOT_ROM_1     ?= rom1
#RBOOT_LD_1      ?= rom1.ld
## size of the spiffs to create
SPIFF_SIZE      ?= 65536
## option to completely disable spiffs
#DISABLE_SPIFFS  = 1
## flash offsets for spiffs, set if using two rom mode or not on a 4mb flash
## (spiffs location defaults to the mb after the rom slot on 4mb flash)
RBOOT_SPIFFS_0  ?= 0x100000
RBOOT_SPIFFS_1  ?= 0x300000
## esptool2 path
#ESPTOOL2        ?= esptool2

USER_CFLAGS = -DSPIFF_SIZE=$(SPIFF_SIZE)
USER_CFLAGS += -DRBOOT_SPIFFS_0=$(RBOOT_SPIFFS_0)
USER_CFLAGS += -DRBOOT_SPIFFS_1=$(RBOOT_SPIFFS_1)
