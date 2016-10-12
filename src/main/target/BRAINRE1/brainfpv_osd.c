/**
 ******************************************************************************
 * @addtogroup OnScreenDisplay OSD Module
 * @brief Process OSD information
 *
 *
 * @file       brainfpv_osd.c
 * @author     dRonin, http://dronin.org Copyright (C) 2015-2016
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2014
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2014.
 * @brief      OSD gen module, handles OSD draw. Parts from CL-OSD and SUPEROSD projects
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */


#include <string.h>
#include <math.h>

#include "brainfpv_osd.h"
#include "ch.h"
#include "video.h"
#include "images.h"
#include "osd_utils.h"

#include "debug.h"
#include "version.h"
#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/printf.h"
#include "common/typeconversion.h"

#include "drivers/gpio.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/compass.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"
#include "drivers/light_ws2811strip.h"
#include "drivers/sound_beeper.h"
#include "drivers/max7456.h"
#include "drivers/max7456_symbols.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/flashfs.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/beeper.h"
#include "io/osd.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "scheduler/scheduler_tasks.h"

#if defined(USE_BRAINFPV_OSD) | 1

extern binary_semaphore_t onScreenDisplaySemaphore;

extern uint8_t *draw_buffer;
extern uint8_t *disp_buffer;


static void simple_artificial_horizon(int16_t roll, int16_t pitch, int16_t x, int16_t y,
        int16_t width, int16_t height, int8_t max_pitch,
        uint8_t n_pitch_steps);

/*******************************************************************************/
// MAX7456 Emulation
#define MAX_X(x) (x * 12)
#define MAX_Y(y) (y * 18 + 5)

uint16_t maxScreenSize = VIDEO_BUFFER_CHARS_PAL;

void max7456Init(uint8_t system)
{
    (void)system;
}

void    max7456DrawScreen(void)
{}

void  max7456WriteNvm(uint8_t char_address, uint8_t *font_data)
{
    (void)char_address; (void)font_data;
}

uint8_t max7456GetRowsCount(void)
{
    if (Video_GetType() == VIDEO_TYPE_NTSC)
        return VIDEO_LINES_NTSC;
    else
        return VIDEO_LINES_PAL;
}

void max7456Write(uint8_t x, uint8_t y, char *buff)
{
    write_string(buff, MAX_X(x), MAX_Y(y), 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, masterConfig.bfOsdConfig.font);
}

void max7456WriteChar(uint8_t x, uint8_t y, uint8_t c)
{
    char buff[2] = {c, 0};
    write_string(buff, MAX_X(x), MAX_Y(y), 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, masterConfig.bfOsdConfig.font);
}

void max7456ClearScreen(void)
{
    clearGraphics();
}

void  max7456RefreshAll(void)
{
}

static uint8_t dummyBuffer[VIDEO_BUFFER_CHARS_PAL+40];
uint8_t* max7456GetScreenBuffer(void)
{
    return dummyBuffer;
}
/*******************************************************************************/

extern uint8_t armState;
void brainFpvOsdInit(void)
{
    char string_buffer[40];

    armState = ARMING_FLAG(ARMED);

#define GY (GRAPHICS_BOTTOM / 2 - 30)

    draw_image(GRAPHICS_X_MIDDLE - image_betaflight.width - 5, GY - image_brainfpv.height / 2, &image_brainfpv);
    draw_image(GRAPHICS_X_MIDDLE + 5, GY - image_betaflight.height / 2, &image_betaflight);

    sprintf(string_buffer, "BF VERSION: %s", FC_VERSION_STRING);
    write_string(string_buffer, GRAPHICS_X_MIDDLE, GRAPHICS_BOTTOM - 60, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, BETAFLIGHT_DEFAULT);
    write_string("MENU: THRT MID YAW RIGHT PITCH UP", GRAPHICS_X_MIDDLE, GRAPHICS_BOTTOM - 35, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, FONT8X10);
}


void osdMain(void) {
    LED1_TOGGLE;
    clearGraphics();

    if (millis() < 5000) {
        brainFpvOsdInit();
    }
    else {
        updateOsd();
    }
}


void resetBfOsdConfig(bfOsdConfig_t *bfOsdConfig)
{
    bfOsdConfig->sync_threshold = 120;
    bfOsdConfig->white_level    = 110;
    bfOsdConfig->black_level    = 20;
    bfOsdConfig->x_offset       = 0;
    bfOsdConfig->x_scale        = 8;
    bfOsdConfig->sbs_3d_enabled = 0;
    bfOsdConfig->sbs_3d_right_eye_offset = 30;
    bfOsdConfig->font = 0;
    bfOsdConfig->ir_system = 0;
    bfOsdConfig->ir_trackmate_id = 0;
    bfOsdConfig->ir_ilap_id = 0;
    bfOsdConfig->ahi_steps = 2;
}

void brainFpvOsdArtificialHorizon(void)
{
    simple_artificial_horizon(-1 * attitude.values.roll, -1 * attitude.values.pitch,
                              GRAPHICS_X_MIDDLE, GRAPHICS_Y_MIDDLE,
                              GRAPHICS_BOTTOM * 0.8f, GRAPHICS_RIGHT * 0.8f, 30,
                              masterConfig.bfOsdConfig.ahi_steps);
}

#define CENTER_BODY       3
#define CENTER_WING       7
#define CENTER_RUDDER     5
#define PITCH_STEP       10
void brainFpvOsdCenterMark(void)
{
    write_line_outlined(GRAPHICS_X_MIDDLE - CENTER_WING - CENTER_BODY, GRAPHICS_Y_MIDDLE ,
            GRAPHICS_X_MIDDLE - CENTER_BODY, GRAPHICS_Y_MIDDLE, 2, 0, 0, 1);
    write_line_outlined(GRAPHICS_X_MIDDLE + 1 + CENTER_BODY, GRAPHICS_Y_MIDDLE,
            GRAPHICS_X_MIDDLE + 1 + CENTER_BODY + CENTER_WING, GRAPHICS_Y_MIDDLE, 0, 2, 0, 1);
    write_line_outlined(GRAPHICS_X_MIDDLE, GRAPHICS_Y_MIDDLE - CENTER_RUDDER - CENTER_BODY, GRAPHICS_X_MIDDLE,
            GRAPHICS_Y_MIDDLE - CENTER_BODY, 2, 0, 0, 1);
}


static void simple_artificial_horizon(int16_t roll, int16_t pitch, int16_t x, int16_t y,
        int16_t width, int16_t height, int8_t max_pitch, uint8_t n_pitch_steps)
{
    width /= 2;
    height /= 2;

    float sin_roll = sinf(DECIDEGREES_TO_RADIANS(roll));
    float cos_roll = cosf(DECIDEGREES_TO_RADIANS(roll));

    int pitch_step_offset = pitch / (PITCH_STEP * 10);

    /* how many degrees the "lines" are offset from their ideal pos
         * since we need both, don't do fmodf.. */
    float modulo_pitch =DECIDEGREES_TO_DEGREES(pitch) - pitch_step_offset * 10.0f;

    // roll to pitch transformation
    int16_t pp_x = x + width * ((sin_roll * modulo_pitch) / (float)max_pitch);
    int16_t pp_y = y + height * ((cos_roll * modulo_pitch) / (float)max_pitch);

    int16_t d_x, d_x2; // delta x
    int16_t d_y, d_y2; // delta y

    d_x = cos_roll * width / 2;
    d_y = sin_roll * height / 2;

    d_x = 3 * d_x / 4;
    d_y = 3 * d_y / 4;
    d_x2 = 3 * d_x / 4;
    d_y2 = 3 * d_y / 4;

    int16_t d_x_10 = width * sin_roll * PITCH_STEP / (float)max_pitch;
    int16_t d_y_10 = height * cos_roll * PITCH_STEP / (float)max_pitch;

    int16_t d_x_2 = d_x_10 / 6;
    int16_t d_y_2 = d_y_10 / 6;

    for (int i = (-max_pitch / 10)-1; i<(max_pitch/10)+1; i++) {
        int angle = (pitch_step_offset + i);

        if (angle < -n_pitch_steps) continue;
        if (angle > n_pitch_steps) continue;

        angle *= PITCH_STEP;

        /* Wraparound */
        if (angle > 90) {
            angle = 180 - angle;
        } else if (angle < -90) {
            angle = -180 - angle;
        }

        int16_t pp_x2 = pp_x - i * d_x_10;
        int16_t pp_y2 = pp_y - i * d_y_10;

        char tmp_str[5];

        sprintf(tmp_str, "%d", angle);

        if (angle < 0) {
            write_line_outlined_dashed(pp_x2 - d_x2, pp_y2 + d_y2, pp_x2 + d_x2, pp_y2 - d_y2, 2, 2, 0, 1, 5);
            write_line_outlined(pp_x2 - d_x2, pp_y2 + d_y2, pp_x2 - d_x2 - d_x_2, pp_y2 + d_y2 - d_y_2, 2, 2, 0, 1);
            write_line_outlined(pp_x2 + d_x2, pp_y2 - d_y2, pp_x2 + d_x2 - d_x_2, pp_y2 - d_y2 - d_y_2, 2, 2, 0, 1);

            write_string(tmp_str, pp_x2 - d_x - 4, pp_y2 + d_y, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, FONT_OUTLINED8X8);
            write_string(tmp_str, pp_x2 + d_x + 4, pp_y2 - d_y, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, FONT_OUTLINED8X8);
        } else if (angle > 0) {
            write_line_outlined(pp_x2 - d_x2, pp_y2 + d_y2, pp_x2 + d_x2, pp_y2 - d_y2, 2, 2, 0, 1);
            write_line_outlined(pp_x2 - d_x2, pp_y2 + d_y2, pp_x2 - d_x2 + d_x_2, pp_y2 + d_y2 + d_y_2, 2, 2, 0, 1);
            write_line_outlined(pp_x2 + d_x2, pp_y2 - d_y2, pp_x2 + d_x2 + d_x_2, pp_y2 - d_y2 + d_y_2, 2, 2, 0, 1);

            write_string(tmp_str, pp_x2 - d_x - 4, pp_y2 + d_y, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, FONT_OUTLINED8X8);
            write_string(tmp_str, pp_x2 + d_x + 4, pp_y2 - d_y, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, FONT_OUTLINED8X8);
        } else {
            write_line_outlined(pp_x2 - d_x, pp_y2 + d_y, pp_x2 - d_x / 3, pp_y2 + d_y / 3, 2, 2, 0, 1);
            write_line_outlined(pp_x2 + d_x / 3, pp_y2 - d_y / 3, pp_x2 + d_x, pp_y2 - d_y, 2, 2, 0, 1);
        }
    }
}

#endif /* USE_BRAINFPV_OSD */
