F446_TARGETS    += $(TARGET)
FEATURES        += VCP CHIBIOS BRAINFPV SPECTROGRAPH ONBOARDFLASH

HSE_VALUE       = 16000000

TARGET_SRC =  \
              drivers/accgyro/accgyro_spi_bmi160.c \
              osd/osd.c \
              osd/osd_elements.c \
              drivers/light_ws2811strip.c \
              drivers/barometer/barometer_bmp280.c \
