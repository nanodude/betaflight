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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"
#include "brainfpv/brainfpv_system.h"

#include "config/config.h"
#include "flight/mixer.h"
#include "drivers/system.h"
#include "drivers/time.h"

#if defined(BRAINFPV)

static BrainFPVSystemReq_t brainfpv_req = BRAINFPV_REQ_NONE;

// Set the request
void brainFPVSystemSetReq(BrainFPVSystemReq_t req)
{
    brainfpv_req = req;
}

// Execute request (called from betaflight system task)
void brainFPVSystemCheck(void)
{

    switch(brainfpv_req) {
        case BRAINFPV_REQ_NONE:
            // Nothing to do
            break;
        case BRAINFPV_REQ_UPDATE_HW_SETTINGS:
            brainFPVUpdateSettings();
            break;
        case BRAINFPV_REQ_SAVE_SETTINGS:
            saveConfigAndNotify();
            break;
        case BRAINFPV_REQ_SAVE_SETTINGS_REBOOT:
            saveConfigAndNotify();

            stopMotors();
            motorShutdown();
            delay(200);

            systemReset();
            break;
    }
    brainfpv_req = BRAINFPV_REQ_NONE;
}
#endif
