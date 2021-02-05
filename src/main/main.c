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

#include "platform.h"
#include "brainfpv/brainfpv_system.h"

#include "fc/init.h"

#include "scheduler/scheduler.h"

void main_step(void)
{
    scheduler();
    processLoopback();
}

#ifndef NOMAIN
#if !defined(USE_CHIBIOS)
void run(void);

int main(void)
{
    init();

    run();

    return 0;
}

void FAST_CODE FAST_CODE_NOINLINE run(void)
{
    while (true) {
        scheduler();
        processLoopback();
#ifdef SIMULATOR_BUILD
        delayMicroseconds_real(50); // max rate 20kHz
#endif
    }
}
#endif

#if defined(USE_CHIBIOS)
#include "ch.h"
#include "hal.h"
#include "nvic.h"

volatile bool idleCounterClear = 0;
volatile uint32_t idleCounter = 0;

void appIdleHook(void)
{
    // Called when the scheduler has no tasks to run
    if (idleCounterClear) {
        idleCounter = 0;
        idleCounterClear = 0;
    } else {
        ++idleCounter;
    }
}

static THD_WORKING_AREA(waBetaFlightThread, 2048);
static THD_FUNCTION(BetaFlightThread, arg)
{
    (void)arg;
    chRegSetThreadName("Betaflight");
    while(1) {
        main_step();
    }
}

#if defined(USE_BRAINFPV_OSD)
#include "brainfpv/brainfpv_osd.h"
#include "drivers/osd.h"
#include "drivers/display.h"
#include "io/displayport_max7456.h"
#include "pg/vcd.h"
#include "config/config_eeprom.h"
#include "config/feature.h"
#include "cms/cms.h"


displayPort_t *osdDisplayPort;

static THD_WORKING_AREA(waOSDThread, 1024);
static THD_FUNCTION(OSDThread, arg)
{
    (void)arg;
    chRegSetThreadName("OSD");

    brainFpvOsdInit();
    osdMain();
}
#endif /* defined(USE_BRAINFPV_OSD) */

#if defined(USE_BRAINFPV_SPECTROGRAPH)
#include "brainfpv/spectrograph.h"
extern binary_semaphore_t spectrographDataReadySemaphore;
binary_semaphore_t gyroSem;

static THD_WORKING_AREA(waSpecThread, 512);
static THD_FUNCTION(SpecThread, arg)
{
    (void)arg;
    chRegSetThreadName("Spectrograph");
    while (1) {
        // wait for data ready
        chBSemWait(&spectrographDataReadySemaphore);
        spectrographMain();
    }
}
#endif /* defined(USE_BRAINFPV_SPECTROGRAPH) */


//#define USE_DUMMY_TASK
#if defined(USE_DUMMY_TASK)
static THD_WORKING_AREA(waDummyThread, 512);
static THD_FUNCTION(DummyThread, arg)
{
    (void)arg;
    chRegSetThreadName("Dummy");
    while (1) {
        chThdSleepMilliseconds(1);
    }
}
#endif

uint8_t safe_boot = 0;

int main()
{
#if (STM32_NO_INIT == TRUE)
  init();
#endif

  halInit();
  chSysInit();

#if (STM32_NO_INIT == FALSE)
  init();
#endif

  st_lld_init();

  brainFPVSystemInit();

  chBSemObjectInit(&gyroSem, FALSE);

#if defined(USE_BRAINFPV_SPECTROGRAPH)
  chBSemObjectInit(&spectrographDataReadySemaphore, FALSE);
#endif

  chThdCreateStatic(waBetaFlightThread, sizeof(waBetaFlightThread), HIGHPRIO, BetaFlightThread, NULL);

#if defined(USE_BRAINFPV_OSD)
  if (VideoIsInitialized()) {
      chThdCreateStatic(waOSDThread, sizeof(waOSDThread), NORMALPRIO, OSDThread, NULL);
  }
#endif /* USE_BRAINFPV_OSD */

#if defined(USE_BRAINFPV_SPECTROGRAPH)
  if (bfOsdConfig()->spec_enabled) {
    spectrographInit();
    chThdCreateStatic(waSpecThread, sizeof(waSpecThread), LOWPRIO, SpecThread, NULL);
  }
#endif /* USE_BRAINFPV_SPECTROGRAPH */

#if defined(USE_DUMMY_TASK)
  chThdCreateStatic(waDummyThread, sizeof(waDummyThread), NORMALPRIO, DummyThread, NULL);
#endif

  // sleep forever
  chThdSleep(TIME_INFINITE);
}

#endif /* defined(USE_CHIBIOS) */
#endif /* NOMAIN */
