
#include <string.h>
#include "ch.h"
#include "video.h"
#include "osd_utils.h"
#include "drivers/light_led.h"
#include "drivers/max7456.h"

#include "scheduler/scheduler_tasks.h"

#if defined(USE_BRAINFPV_OSD) | 1

extern binary_semaphore_t onScreenDisplaySemaphore;

extern uint8_t *draw_buffer;
extern uint8_t *disp_buffer;


/*******************************************************************************/
// MAX7456 Emulation
#define MAX_X(x) (x * 12)
#define MAX_Y(y) (y * 18)

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
    // XXX
    return VIDEO_LINES_NTSC;
}

void max7456Write(uint8_t x, uint8_t y, char *buff)
{
    write_string(buff, MAX_X(x), MAX_Y(y), 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, BETAFLIGHT_DEFAULT);
}

void max7456WriteChar(uint8_t x, uint8_t y, uint8_t c)
{
    char buff[2] = {c, 0};
    write_string(buff, MAX_X(x), MAX_Y(y), 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, BETAFLIGHT_DEFAULT);
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


void osdMain(void) {
    LED1_TOGGLE;
    clearGraphics();
    updateOsd();
}
#endif /* USE_BRAINFPV_OSD */
