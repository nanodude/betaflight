H750xB_TARGETS += $(TARGET)

HSE_VALUE    = 16000000
START_ADDRESS = 0x000000

FEATURES       += VCP ONBOARDFLASH CHIBIOS BRAINFPV SPECTROGRAPH

TARGET_SRC += \
              drivers/accgyro/accgyro_mpu.c \
              drivers/accgyro/accgyro_spi_bmi270.c \
              drivers/barometer/barometer_bmp388.c \
              
