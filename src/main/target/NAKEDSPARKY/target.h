/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "NSKY" // NakedSparKY

/* Start of common_pre.h overrides */

#undef MINIMAL_CLI
#undef USE_BRUSHED_ESC_AUTODETECT
#undef USE_PPM
#undef USE_SERIALRX_CRSF       // Team Black Sheep Crossfire protocol
#undef USE_SERIALRX_GHST       // ImmersionRC Ghost Protocol
#undef USE_SERIALRX_IBUS       // FlySky and Turnigy receivers
#undef USE_SERIALRX_SPEKTRUM   // SRXL, DSM2 and DSMX protocol
#undef USE_SERIALRX_SUMD       // Graupner Hott protocol
#undef USE_ACRO_TRAINER
#undef USE_BLACKBOX
#undef USE_SERVOS
#undef USE_TELEMETRY_FRSKY_HUB
#undef USE_TELEMETRY_CRSF
#undef USE_TELEMETRY_GHST
#undef USE_TELEMETRY_SRXL
#define USE_BOARD_INFO
#define USE_SENSOR_NAMES
#define USE_TELEMETRY_SENSORS_DISABLED_DETAILS
#define USE_BATTERY_VOLTAGE_SAG_COMPENSATION
#define USE_SENSOR_NAMES
#define USE_TIMER_MGMT
#define USE_QUAD_MIXER_ONLY
#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER

/* End of common_pre.h overrides */

#define USE_DSHOT
#define USE_DSHOT_TELEMETRY
#define ENABLE_DSHOT_DMAR       DSHOT_DMAR_ON
#define USE_DMA_SPEC
//#undef USE_DSHOT_DMAR

#define LED0_PIN                PB4  // Blue (Rev 1 & 2) - PB4
#define LED1_PIN                PB5  // Green (Rev 1) / Red (Rev 2) - PB5

#define USE_BEEPER
#define BEEPER_PIN              PA1
#define BEEPER_INVERTED

// MPU6050 interrupts
#define USE_EXTI
#define USE_GYRO_EXTI
#define GYRO_1_EXTI_PIN         PA15
#define USE_MPU_DATA_READY_SIGNAL

// MPU 9150 INT connected to PA15, pulled up to VCC by 10K Resistor, contains MPU6050 and AK8975 in single component.
#define USE_GYRO
#define USE_GYRO_MPU6050
#define GYRO_1_ALIGN            CW270_DEG

#define USE_ACC
#define USE_ACC_MPU6050

#define USE_BARO
#define USE_BARO_MS5611

#define USE_MAG
#define USE_MAG_AK8975

#define MAG_AK8975_ALIGN        CW180_DEG_FLIP

#define USE_VCP
#define USE_USB_DETECT
#define USB_DETECT_PIN          PB3
#define USE_UART1 // Conn 1 - TX (PB6) RX PB7 (AF7)
#define USE_UART2 // Input - RX (PA3)
#define USE_UART3 // Servo out - 10/RX (PB11) 11/TX (PB10)

#define SERIAL_PORT_COUNT       4

// TODO
#define AVOID_UART2_FOR_PWM_PPM

#define UART1_TX_PIN            PB6
#define UART1_RX_PIN            PB7

#define UART2_TX_PIN            PA2 // PA2 - Clashes with PWM6 input.
#define UART2_RX_PIN            PA3

#define UART3_TX_PIN            PB10 // PB10 (AF7)
#define UART3_RX_PIN            PB11 // PB11 (AF7)

// Note: PA5 and PA0 are N/C on the sparky - potentially use for ADC or LED STRIP?

#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C_DEVICE              (I2CDEV_2)
#define I2C2_SCL                PA9
#define I2C2_SDA                PA10

#define USE_ADC
#define ADC_INSTANCE            ADC1
#define VBAT_ADC_PIN            PA0
#define CURRENT_METER_ADC_PIN   PA7

#define DEFAULT_RX_FEATURE      FEATURE_RX_PPM

//#define USE_RANGEFINDER
//#define USE_RANGEFINDER_HCSR04
//#define RANGEFINDER_HCSR04_ECHO_PIN          PB1
//#define RANGEFINDER_HCSR04_TRIGGER_PIN       PA2

// available IO pins (from schematics)
#define TARGET_IO_PORTA         (0xffff & ~(BIT(5)))
#define TARGET_IO_PORTB         (0xffff & ~(BIT(2)|BIT(12)|BIT(13)))
#define TARGET_IO_PORTC         (BIT(13))

//#define USABLE_TIMER_CHANNEL_COUNT //4
//#define USED_TIMERS             (TIM_N(1) | TIM_N(3) | TIM_N(15))
