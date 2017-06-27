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

#include "platform.h"

#include "system.h"
#include "io.h"

#include "sound_beeper.h"

#ifdef USE_BRAINFPV_FPGA
#include "fpga_drv.h"
#endif

#ifdef BEEPER

#ifndef USE_BRAINFPV_FPGA
static IO_t beeperIO = DEFIO_IO(NONE);
static bool beeperInverted = false;
#endif

#endif

void systemBeep(bool onoff)
{
#ifndef BEEPER
    UNUSED(onoff);
#else
#ifndef USE_BRAINFPV_FPGA
    IOWrite(beeperIO, beeperInverted ? onoff : !onoff);
#else
    BRAINFPVFPGA_Buzzer(onoff);
#endif
#endif
}

void systemBeepToggle(void)
{
#ifdef BEEPER
#ifndef USE_BRAINFPV_FPGA
    IOToggle(beeperIO);
#else
     BRAINFPVFPGA_BuzzerToggle();
#endif
#endif
}

void beeperInit(const beeperConfig_t *config)
{
#ifndef BEEPER
    UNUSED(config);
#else
#ifndef USE_BRAINFPV_FPGA
    beeperIO = IOGetByTag(config->ioTag);
    beeperInverted = config->isInverted;

    if (beeperIO) {
        IOInit(beeperIO, OWNER_BEEPER, 0);
        IOConfigGPIO(beeperIO, config->isOpenDrain ? IOCFG_OUT_OD : IOCFG_OUT_PP);
    }
#else
    UNUSED(config);
#endif
    systemBeep(false);
#endif
}
