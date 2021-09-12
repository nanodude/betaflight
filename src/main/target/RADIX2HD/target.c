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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

#include "brainfpv/brainfpv_osd.h"
#include "brainfpv/brainfpv_system.h"

#if defined(USE_BRAINFPV_RGB_LED_TIMER)
#include "brainfpv_rgb_led_timer.h"
#endif

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM17, CH1, PB9,   TIM_USE_LED,                 TIMER_OUTPUT_INVERTED,  10,  0 ), // LED Strip
    DEF_TIM(TIM12, CH2, PB15,  TIM_USE_PPM,                 0,  0,  0 ), // PPM input

    DEF_TIM(TIM2,  CH1, PA15,  TIM_USE_MOTOR,               0,  0,  0 ), // M1
	DEF_TIM(TIM3,  CH2, PB5,   TIM_USE_MOTOR,               0,  1,  0 ), // M2
	DEF_TIM(TIM4,  CH1, PD12,  TIM_USE_MOTOR,               0,  2,  0 ), // M3
    DEF_TIM(TIM4,  CH2, PD13,  TIM_USE_MOTOR,               0,  3,  0 ), // M4

    DEF_TIM(TIM8,  CH4, PC9,   TIM_USE_MOTOR,               0,  4,  2 ), // M5
    DEF_TIM(TIM8,  CH3, PC8,   TIM_USE_MOTOR,               0,  5,  2 ), // M6
    DEF_TIM(TIM1,  CH3, PE13,  TIM_USE_MOTOR,               0,  6,  1 ), // M7
    DEF_TIM(TIM1,  CH2, PE11,  TIM_USE_MOTOR,               0,  7,  1 ), // M8
};

#if defined(USE_BRAINFPV_RGB_LED_TIMER)
const timerHardware_t timerHardwareRgbLed[3] = {
    DEF_TIM(TIM5, CH1, PA0, TIM_USE_NONE, TIMER_OUTPUT_INVERTED,  0,  0 ), // R
    DEF_TIM(TIM5, CH3, PA2, TIM_USE_NONE, TIMER_OUTPUT_INVERTED,  0,  0 ), // G
    DEF_TIM(TIM5, CH4, PA3, TIM_USE_NONE, TIMER_OUTPUT_INVERTED,  0,  0 ), // B
};
#endif

bool brainfpv_settings_updated_from_cms = false;

extern bfOsdConfig_t bfOsdConfigCms;
extern brainFpvSystemConfig_t brainFpvSystemConfigCms;

void brainFPVUpdateSettings(void)
{
    const brainFpvSystemConfig_t * brainFpvSystemConfigUse;

    if (brainfpv_settings_updated_from_cms) {
        brainFpvSystemConfigUse = &brainFpvSystemConfigCms;
    }
    else {
        brainFpvSystemConfigUse = brainFpvSystemConfig();
    }

#if defined(USE_BRAINFPV_RGB_LED_TIMER)
    brainFPVRgbLedSetLedColor(0, brainFpvSystemConfigUse->status_led_color, brainFpvSystemConfigUse->status_led_brightness);
#endif
}
