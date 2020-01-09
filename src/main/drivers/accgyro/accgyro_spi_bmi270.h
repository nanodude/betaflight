/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_BMI270 BMI270 Functions
 * @brief Hardware functions to deal with the 6DOF gyro / accel sensor
 * @{
 *
 * @file       pios_bmi270.c
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2016
 * @brief      BMI270 Gyro / Accel Sensor Routines
 * @see        The GNU Public License (GPL) Version 3
 ******************************************************************************/

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

#pragma once

#include "drivers/bus.h"

enum bmi270_orientation { // clockwise rotation from board forward
    BMI270_TOP_0DEG,
    BMI270_TOP_90DEG,
    BMI270_TOP_180DEG,
    BMI270_TOP_270DEG,
    BMI270_BOTTOM_0DEG,
    BMI270_BOTTOM_90DEG,
    BMI270_BOTTOM_180DEG,
    BMI270_BOTTOM_270DEG,
};

enum bmi270_odr {
    BMI270_ODR_800_Hz = 0x0B,
    BMI270_ODR_2700_Hz = 0x0C,
    BMI270_ODR_3200_Hz = 0x0D
};

enum bmi270_acc_range {
    BMI270_RANGE_2G = 0x00,
    BMI270_RANGE_4G = 0x01,
    BMI270_RANGE_8G = 0x02,
    BMI270_RANGE_16G = 0x03,
};

enum bmi270_gyro_range {
    BMI270_RANGE_125DPS = 0x04,
    BMI270_RANGE_250DPS = 0x03,
    BMI270_RANGE_500DPS = 0x02,
    BMI270_RANGE_1000DPS = 0x01,
    BMI270_RANGE_2000DPS = 0x00,
};

uint8_t bmi270Detect(const busDevice_t *bus);
bool bmi270SpiAccDetect(accDev_t *acc);
bool bmi270SpiGyroDetect(gyroDev_t *gyro);
