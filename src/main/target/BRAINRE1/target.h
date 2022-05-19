/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#define TARGET_BOARD_IDENTIFIER "BRAINRE1"

#define USBD_PRODUCT_STRING     "BrainRE1"

// For ChibiOS
// Priority needs to be lower than any of the BF interrupts that don't use CH_IRQ_EPILOGUE
#define STM32_ST_IRQ_PRIORITY               8
#define STM32_ST_USE_TIMER                  3

#define USE_TARGET_CONFIG
#undef USE_CUSTOM_DEFAULTS_ADDRESS

#define USE_HUFFMAN
#undef USE_PINIO
#undef USE_PINIOBOX
#undef USE_USB_MSC

#define LED0_PIN                    PB13
#define LED1_PIN                    PC8
#define LED0_INVERTED
#define LED1_INVERTED

#define USE_BEEPER
#define BEEPER_PIN              NONE

#define USE_LED_STRIP

#undef USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define USE_BRAINFPV_IR_TRANSPONDER

#define USE_FLASHFS
#define USE_FLASH_TOOLS
#define USE_FLASH_M25P16
#define M25P16_FIRST_SECTOR     4
#define FLASH_SPI_SHARED
#define FLASH_CS_PIN           PB15
#define FLASH_SPI_INSTANCE     SPI3

#define USE_BRAINFPV_FPGA
#define BRAINFPVFPGA_SPI_INSTANCE SPI3
#define BRAINFPVFPGA_SPI_DIVISOR  16
#define BRAINFPVFPGA_CS_PIN       PC15
#define BRAINFPVFPGA_CDONE_PIN    PB0
#define BRAINFPVFPGA_CRESET_PIN   PB1
#define BRAINFPVFPGA_CLOCK_PIN    PA8
#define BRAINFPVFPGA_RESET_PIN    PB10
#define USE_BRAINFPV_IR_TRANSPONDER
#define USE_BRAINFPV_FPGA_BUZZER

#define BRAINFPV
#define USE_MAX7456
#define USE_OSD
#define USE_CMS
#define OSD_CALLS_CMS
#define USE_BRAINFPV_OSD
#define VIDEO_BITS_PER_PIXEL 2
#define INCLUDE_VIDEO_QUADSPI
#define VIDEO_QSPI_CLOCK_PIN PB2
#define VIDEO_QSPI_IO0_PIN   PC9
#define VIDEO_QSPI_IO1_PIN   PC10
#define VIDEO_VSYNC          PB5
#define VIDEO_HSYNC          PC2
#define BRAINFPV_OSD_SYNC_TH_DEFAULT 120
#define BRAINFPV_OSD_SYNC_TH_MIN 110
#define BRAINFPV_OSD_SYNC_TH_MAX 130

#define BRAINFPV_OSD_BLACK_LEVEL_DEFAULT 20
#define BRAINFPV_OSD_BLACK_LEVEL_MIN 15
#define BRAINFPV_OSD_BLACK_LEVEL_MAX 40

#define BRAINFPV_OSD_WHITE_LEVEL_DEFAULT 110
#define BRAINFPV_OSD_WHITE_LEVEL_MIN 100
#define BRAINFPV_OSD_WHITE_LEVEL_MAX 200

#define USE_VTX_CONTROL
#define VTX_SMARTAUDIO

#undef USE_BRAINFPV_SPECTROGRAPH

#define USE_EXTI
#define USE_GYRO
#define USE_ACC
#undef USE_MULTI_GYRO

#define USE_SPI_GYRO
#define USE_MPU_DATA_READY_SIGNAL
#define USE_GYRO_EXTI
#define USE_ACCGYRO_BMI160
#define USE_GYRO_SPI_BMI160
#define USE_ACC_SPI_BMI160
#define GYRO_BMI160_ALIGN    CW0_DEG
#define ACC_BMI160_ALIGN     CW0_DEG
#define BMI160_SPI_DIVISOR   16

#define GYRO_1_CS_PIN             PC14
#define GYRO_1_EXTI_PIN           PC13
#define GYRO_1_SPI_INSTANCE       SPI1
#define GYRO_1_ALIGN              CW0_DEG

#define USE_BARO
#define DEFAULT_BARO_BMP280
#define BARO_ZERO_ON_ARM
#define USE_BARO_BMP280
#define BARO_I2C_INSTANCE         I2CDEV_1

#define USABLE_TIMER_CHANNEL_COUNT 7

#define USE_VCP
#define VBUS_SENSING_PIN        PA9
#define VBUS_SENSING_ENABLED

#define USE_UART
#define USE_UART1
#undef USE_UART1_TX_DMA
#define USE_UART1_TX_NODMA
#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define USE_UART3
#define UART3_RX_PIN            PC5
#define UART3_TX_PIN            NONE

#define USE_UART6
#undef USE_UART6_TX_DMA
#define UART6_RX_PIN            PC7
#define UART6_TX_PIN            PC6

#define SERIAL_PORT_COUNT       4 //VCP, USART1, USART3, USART6

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12
// Disable DMA for SPI3
#define SPI3_TX_DMA_OPT         -2
#define SPI3_RX_DMA_OPT         -2

#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_PULLUP
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define BOARD_HAS_VOLTAGE_DIVIDER
#define USE_ADC
#define VBAT_ADC_PIN            PC0
#define RSSI_ADC_PIN            PC3
#define CURRENT_METER_ADC_PIN   PC1
#define VBAT_SCALE_DEFAULT             66
#define CURRENT_METER_SCALE_DEFAULT   250
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define DEFAULT_FEATURES        (FEATURE_OSD)
#define SERIALRX_UART           SERIAL_PORT_USART3
#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_CRSF

#define SPEKTRUM_BIND
// PPM input
#define BIND_PIN                PB14

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define USE_DSHOT
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(5) | TIM_N(12) )

#undef USE_DSHOT_BITBANG

extern bool brainfpv_settings_updated_from_cms;
void brainFPVUpdateSettings(void);
