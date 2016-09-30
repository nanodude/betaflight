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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"

#include "drivers/nvic.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_rx.h"
#include "drivers/pwm_output.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/inverter.h"
#include "drivers/flash_m25p16.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sdcard.h"
#include "drivers/usb_io.h"
#include "drivers/transponder_ir.h"
#include "drivers/io.h"
#include "drivers/exti.h"
#include "drivers/vtx_soft_spi_rtc6705.h"

#ifdef USE_BST
#include "bus_bst.h"
#endif

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/transponder_ir.h"
#include "io/osd.h"
#include "io/vtx.h"

#include "scheduler/scheduler.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/initialisation.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "target/BRAINRE1/fpga_drv.h"


const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
//    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    // prevent crashing, but do nothing
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
//    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
//    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    // prevent crashing, but do nothing
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
//    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM12, IO_TAG(PB8), TIM_Channel_1, TIM8_BRK_TIM12_IRQn,       0, IOCFG_AF_PP, GPIO_AF_TIM12 },       // PPM_IN

    { TIM5,  IO_TAG(PA2),  TIM_Channel_3, TIM5_IRQn,          1, IOCFG_AF_PP, GPIO_AF_TIM5 },          // dRonin: 3
    { TIM5,  IO_TAG(PA1),  TIM_Channel_2, TIM5_IRQn,          1, IOCFG_AF_PP, GPIO_AF_TIM5 },          // dRonin: 2
    { TIM5,  IO_TAG(PA3),  TIM_Channel_4, TIM5_IRQn,          1, IOCFG_AF_PP, GPIO_AF_TIM5 },          // dRonin: 4
    { TIM5,  IO_TAG(PA0),  TIM_Channel_1, TIM5_IRQn,          1, IOCFG_AF_PP, GPIO_AF_TIM5 },          // dRonin: 1
    { TIM1,  IO_TAG(PA10), TIM_Channel_3, TIM1_CC_IRQn,       1, IOCFG_AF_PP, GPIO_AF_TIM1 },          // dRonin: 5
    { TIM2,  IO_TAG(PA15), TIM_Channel_1, TIM2_IRQn,          1, IOCFG_AF_PP, GPIO_AF_TIM2 },          // dRonin: 6
};

bool brainre1_settings_updated = true;
extern master_t masterConfig;

void brainRE1UpdateSettings(void) {
    RE1FPGA_SetBwLevels(masterConfig.bfOsdConfig.black_level, masterConfig.bfOsdConfig.white_level);
    RE1FPGA_SetSyncThreshold(masterConfig.bfOsdConfig.sync_threshold);
    RE1FPGA_SetXOffset(masterConfig.bfOsdConfig.x_offset);
    RE1FPGA_SetXScale(masterConfig.bfOsdConfig.x_scale);
    RE1FPGA_Set3DConfig(masterConfig.bfOsdConfig.sbs_3d_enabled, masterConfig.bfOsdConfig.sbs_3d_right_eye_offset);
}
