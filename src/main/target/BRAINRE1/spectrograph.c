#include <stdint.h>
#include "common/maths.h"

#include "ch.h"
#include "osd_utils.h"
#include "arm_math.h"
#include "platform.h"
#include "drivers/light_led.h"

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


#define SAMPLING_FREQ 3200

#include "target/BRAINRE1/spectrograph.h"

#if defined(USE_BRAINRE1_SPECTROGRAPH)

float spec_gyro_data_roll[SPEC_FFT_LENGTH];
float spec_gyro_data_pitch[SPEC_FFT_LENGTH];
float spec_gyro_data_yaw[SPEC_FFT_LENGTH];

float fft_out[SPEC_FFT_LENGTH];

static uint8_t spec_disp_buffer[SPEC_FFT_LENGTH / 2 -1];


volatile bool spec_data_processed = false;

static mutex_t fftOutputMtx;
binary_semaphore_t spectrographDataReadySemaphore;



static arm_rfft_fast_instance_f32 fft_inst;

void spectrographInit()
{
    arm_rfft_fast_init_f32(&fft_inst, SPEC_FFT_LENGTH);
    chMtxObjectInit(&fftOutputMtx);
}


#define GRAPH_X0 20
#define GRAPH_XM (GRAPHICS_RIGHT - 20)

#define GRAPH_Y0 (GRAPHICS_BOTTOM - 20)
#define GRAPH_YM (GRAPHICS_TOP + 20)

void spectrographMain()
{
    LED1_TOGGLE;

    static float max_rpy[3] = {0, 0, 0};

    chMtxLock(&fftOutputMtx);
    arm_rfft_fast_f32(&fft_inst, spec_gyro_data_roll, fft_out, 0);
    arm_cmplx_mag_f32(fft_out, fft_out, SPEC_FFT_LENGTH / 2);

    float max_val;
    uint32_t max_idx;
    arm_max_f32(&fft_out[1], SPEC_FFT_LENGTH / 2 - 1, &max_val, &max_idx);

    max_rpy[0] = MAX(max_rpy[0], max_val);

    for (int i=1; i<SPEC_FFT_LENGTH / 2; i++){
        fft_out[i] = fft_out[i] / max_rpy[0];
    }
    chMtxUnlock(&fftOutputMtx);

    spec_data_processed = true;
}



#define FFT_BIN(freq) (1 + ((SPEC_FFT_LENGTH / 2 - 1) * freq) / (SAMPLING_FREQ / 2))


void spectrographDraw(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint16_t max_freq)
{
    char tmp_str[10];

    uint16_t freq;
    write_hline_lm(x0 ,x0 + width , y0, 1, 1);
    write_hline_lm(x0 ,x0 + width , y0 + 1, 0, 1);
    write_vline_lm(x0, y0, y0 - height, 1, 1);
    write_vline_lm(x0 - 1, y0 + 1, y0 - height, 0, 1);

    for (int i=0; i<5; i++) {
        sprintf(tmp_str, "%d", i * max_freq / 4);
        write_string(tmp_str, x0 + i * width / 4, y0 + 2, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, FONT_OUTLINED8X8);
    }

    chMtxLock(&fftOutputMtx);
    for (int i=0; i < width; i++) {
        freq = (i * max_freq) / width;
        write_vline_lm(x0 + i, y0, y0 -fft_out[FFT_BIN(freq)] * height , 1, 1);
        write_pixel_lm(x0 + i, y0 -fft_out[FFT_BIN(freq)] * height , 1, 0);
    }
    chMtxUnlock(&fftOutputMtx);

    if (masterConfig.gyro_soft_lpf_hz) {
        uint16_t pos = x0 + (masterConfig.gyro_soft_lpf_hz * width) / max_freq;
        write_vline_lm(pos, y0, y0 - height + 10, 0, 1);
        write_string("LP", pos, y0 - height, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, FONT_OUTLINED8X8);
    }

    if (masterConfig.gyro_soft_notch_hz_1) {
        uint16_t pos = x0 + (masterConfig.gyro_soft_notch_hz_1 * width) / max_freq;
        write_vline_lm(pos, y0, y0 - height + 10, 0, 1);
        write_string("N", pos, y0 - height, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, FONT_OUTLINED8X8);
        pos = x0 + (masterConfig.gyro_soft_notch_cutoff_1 * width) / max_freq;
        write_vline_lm(pos, y0, y0 - height + 10, 0, 1);
        write_string("N", pos, y0 - height, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, FONT_OUTLINED8X8);
    }
}









#endif /* defined(USE_BRAINRE1_SPECTROGRAPH) */































