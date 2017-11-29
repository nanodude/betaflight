COMMON_SRC = \
            build/build_config.c \
            build/debug.c \
            build/version.c \
            $(TARGET_DIR_SRC) \
            main.c \
            common/bitarray.c \
            common/crc.c \
            common/encoding.c \
            common/filter.c \
            common/huffman.c \
            common/huffman_table.c \
            common/maths.c \
            common/printf.c \
            common/streambuf.c \
            common/typeconversion.c \
            config/config_eeprom.c \
            config/feature.c \
            config/parameter_group.c \
            config/config_streamer.c \
            drivers/adc.c \
            drivers/buf_writer.c \
            drivers/bus.c \
            drivers/bus_i2c_config.c \
            drivers/bus_i2c_busdev.c \
            drivers/bus_i2c_soft.c \
            drivers/bus_spi.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/bus_spi_soft.c \
            drivers/buttons.c \
            drivers/display.c \
            drivers/exti.c \
            drivers/io.c \
            drivers/light_led.c \
            drivers/resource.c \
            drivers/rcc.c \
            drivers/serial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart.c \
            drivers/serial_uart_pinconfig.c \
            drivers/sound_beeper.c \
            drivers/stack_check.c \
            drivers/system.c \
            drivers/timer.c \
            drivers/transponder_ir.c \
            drivers/transponder_ir_arcitimer.c \
            drivers/transponder_ir_ilap.c \
            drivers/transponder_ir_erlt.c \
            fc/config.c \
            fc/fc_dispatch.c \
            fc/fc_hardfaults.c \
            fc/fc_msp.c \
            fc/fc_msp_box.c \
            fc/fc_tasks.c \
            fc/runtime_config.c \
            io/beeper.c \
            io/serial.c \
            io/statusindicator.c \
            io/transponder_ir.c \
            io/rcsplit.c \
            msp/msp_serial.c \
            scheduler/scheduler.c \
            sensors/battery.c \
            sensors/current.c \
            sensors/voltage.c \

OSD_SLAVE_SRC = \
            io/displayport_max7456.c \
            osd_slave/osd_slave_init.c \
            io/osd_slave.c

FC_SRC = \
            fc/fc_init.c \
            fc/controlrate_profile.c \
            drivers/camera_control.c \
            drivers/gyro_sync.c \
            drivers/rx_nrf24l01.c \
            drivers/rx_spi.c \
            drivers/rx_xn297.c \
            drivers/pwm_esc_detect.c \
            drivers/pwm_output.c \
            drivers/rx_pwm.c \
            drivers/serial_softserial.c \
            fc/fc_core.c \
            fc/fc_rc.c \
            fc/rc_adjustments.c \
            fc/rc_controls.c \
            fc/rc_modes.c \
            fc/cli.c \
            fc/settings.c \
            flight/altitude.c \
            flight/failsafe.c \
            flight/imu.c \
            flight/mixer.c \
            flight/pid.c \
            flight/servos.c \
            io/serial_4way.c \
            io/serial_4way_avrootloader.c \
            io/serial_4way_stk500v2.c \
            rx/ibus.c \
            rx/jetiexbus.c \
            rx/msp.c \
            rx/nrf24_cx10.c \
            rx/nrf24_inav.c \
            rx/nrf24_h8_3d.c \
            rx/nrf24_syma.c \
            rx/nrf24_v202.c \
            rx/pwm.c \
            rx/rx.c \
            rx/rx_spi.c \
            rx/crsf.c \
            rx/sbus.c \
            rx/spektrum.c \
            rx/sumd.c \
            rx/sumh.c \
            rx/xbus.c \
            sensors/acceleration.c \
            sensors/boardalignment.c \
            sensors/compass.c \
            sensors/gyro.c \
            sensors/gyroanalyse.c \
            sensors/initialisation.c \
            blackbox/blackbox.c \
            blackbox/blackbox_encoding.c \
            blackbox/blackbox_io.c \
            cms/cms.c \
            cms/cms_menu_blackbox.c \
            cms/cms_menu_builtin.c \
            cms/cms_menu_imu.c \
            cms/cms_menu_ledstrip.c \
            cms/cms_menu_misc.c \
            cms/cms_menu_osd.c \
            cms/cms_menu_vtx_rtc6705.c \
            cms/cms_menu_vtx_smartaudio.c \
            cms/cms_menu_vtx_tramp.c \
            common/colorconversion.c \
            common/gps_conversion.c \
            drivers/display_ug2864hsweg01.c \
            drivers/light_ws2811strip.c \
            drivers/serial_escserial.c \
            drivers/sonar_hcsr04.c \
            drivers/vtx_common.c \
            flight/navigation.c \
            io/dashboard.c \
            io/displayport_max7456.c \
            io/displayport_msp.c \
            io/displayport_oled.c \
            io/gps.c \
            io/ledstrip.c \
            io/osd.c \
            sensors/sonar.c \
            sensors/barometer.c \
            telemetry/telemetry.c \
            telemetry/crsf.c \
            telemetry/srxl.c \
            telemetry/frsky.c \
            telemetry/hott.c \
            telemetry/smartport.c \
            telemetry/ltm.c \
            telemetry/mavlink.c \
            telemetry/msp_shared.c \
            telemetry/ibus.c \
            telemetry/ibus_shared.c \
            sensors/esc_sensor.c \
            io/vtx_string.c \
            io/vtx_rtc6705.c \
            io/vtx_smartaudio.c \
            io/vtx_tramp.c \
            io/vtx_control.c

COMMON_DEVICE_SRC = \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC)

ifeq ($(OSD_SLAVE),yes)
TARGET_FLAGS := -DUSE_OSD_SLAVE $(TARGET_FLAGS)
COMMON_SRC := $(COMMON_SRC) $(OSD_SLAVE_SRC) $(COMMON_DEVICE_SRC)
else
COMMON_SRC := $(COMMON_SRC) $(FC_SRC) $(COMMON_DEVICE_SRC)
endif


SPEED_OPTIMISED_SRC := ""
SIZE_OPTIMISED_SRC  := ""

ifneq ($(TARGET),$(filter $(TARGET),$(F1_TARGETS)))
SPEED_OPTIMISED_SRC := $(SPEED_OPTIMISED_SRC) \
            common/encoding.c \
            common/filter.c \
            common/maths.c \
            common/typeconversion.c \
            drivers/adc.c \
            drivers/buf_writer.c \
            drivers/bus.c \
            drivers/bus_spi.c \
            drivers/exti.c \
            drivers/io.c \
            drivers/pwm_output.c \
            drivers/rcc.c \
            drivers/serial.c \
            drivers/serial_uart.c \
            drivers/system.c \
            drivers/timer.c \
            fc/fc_core.c \
            fc/fc_tasks.c \
            fc/fc_rc.c \
            fc/rc_controls.c \
            fc/runtime_config.c \
            flight/imu.c \
            flight/mixer.c \
            flight/pid.c \
            io/serial.c \
            rx/ibus.c \
            rx/rx.c \
            rx/rx_spi.c \
            rx/crsf.c \
            rx/sbus.c \
            rx/spektrum.c \
            rx/sumd.c \
            rx/xbus.c \
            scheduler/scheduler.c \
            sensors/acceleration.c \
            sensors/boardalignment.c \
            sensors/gyro.c \
            sensors/gyroanalyse.c \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC) \
            drivers/light_ws2811strip.c \
            io/displayport_max7456.c \
            io/osd.c \
            io/osd_slave.c

SIZE_OPTIMISED_SRC := $(SIZE_OPTIMISED_SRC) \
            drivers/bus_i2c_config.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart_init.c \
            drivers/serial_uart_pinconfig.c \
            drivers/vtx_rtc6705_soft_spi.c \
            drivers/vtx_rtc6705.c \
            drivers/vtx_common.c \
            fc/fc_init.c \
            fc/cli.c \
            fc/settings.c \
            config/config_eeprom.c \
            config/feature.c \
            config/parameter_group.c \
            config/config_streamer.c \
            io/serial_4way.c \
            io/serial_4way_avrootloader.c \
            io/serial_4way_stk500v2.c \
            io/dashboard.c \
            msp/msp_serial.c \
            cms/cms.c \
            cms/cms_menu_blackbox.c \
            cms/cms_menu_builtin.c \
            cms/cms_menu_imu.c \
            cms/cms_menu_ledstrip.c \
            cms/cms_menu_misc.c \
            cms/cms_menu_osd.c \
            io/vtx_string.c \
            io/vtx_rtc6705.c \
            io/vtx_smartaudio.c \
            io/vtx_tramp.c \
            io/vtx_control.c
endif #!F1

# check if target.mk supplied
SRC := $(STARTUP_SRC) $(MCU_COMMON_SRC) $(SRC) $(VARIANT_SRC)

ifneq ($(DSP_LIB),)

INCLUDE_DIRS += $(DSP_LIB)/Include

SRC += $(DSP_LIB)/Source/BasicMathFunctions/arm_mult_f32.c
SRC += $(DSP_LIB)/Source/TransformFunctions/arm_rfft_fast_f32.c
SRC += $(DSP_LIB)/Source/TransformFunctions/arm_cfft_f32.c
SRC += $(DSP_LIB)/Source/TransformFunctions/arm_rfft_fast_init_f32.c
SRC += $(DSP_LIB)/Source/TransformFunctions/arm_cfft_radix8_f32.c
SRC += $(DSP_LIB)/Source/CommonTables/arm_common_tables.c

SRC += $(DSP_LIB)/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c
SRC += $(DSP_LIB)/Source/StatisticsFunctions/arm_max_f32.c

SRC += $(wildcard $(DSP_LIB)/Source/*/*.S)
endif

ifneq ($(filter ONBOARDFLASH,$(FEATURES)),)
SRC += \
            drivers/flash_m25p16.c \
            io/flashfs.c
endif

SRC += $(COMMON_SRC)

#excludes
SRC   := $(filter-out ${MCU_EXCLUDES}, $(SRC))

ifneq ($(filter SDCARD,$(FEATURES)),)
SRC += \
            drivers/sdcard.c \
            drivers/sdcard_standard.c \
            io/asyncfatfs/asyncfatfs.c \
            io/asyncfatfs/fat_standard.c
endif

ifneq ($(filter VCP,$(FEATURES)),)
SRC += $(VCP_SRC)
endif


ifneq ($(filter CHIBIOS,$(FEATURES)),)
CHIBIOS := $(ROOT)/lib/main/ChibiOS

#include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk

DEVICE_FLAGS += -DUSE_CHIBIOS -DCORTEX_USE_FPU=TRUE -DCORTEX_SIMPLIFIED_PRIORITY=TRUE $(DDEFS)


SRC += $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/crt1.c
SRC += $(STARTUPSRC)
SRC += $(PLATFORMSRC)
SRC += $(HALSRC)
SRC += $(PORTSRC)
SRC += $(KERNSRC)
SRC += $(STARTUPASM)
SRC += $(PORTASM)
SRC += $(OSALASM)


INCLUDE_DIRS += $(CHIBIOS)/os/ext/CMSIS/ST/STM32F4xx/
INCLUDE_DIRS += $(CHIBIOS)/os/common/ports/ARMCMx/devices/STM32F4xx
INCLUDE_DIRS += $(STARTUPINC)
INCLUDE_DIRS += $(KERNINC)
INCLUDE_DIRS += $(PORTINC)
INCLUDE_DIRS += $(OSALINC)
INCLUDE_DIRS += $(HALINC)
INCLUDE_DIRS += $(PLATFORMINC)
endif

ifneq ($(filter BRAINFPV,$(FEATURES)),)
SRC += brainfpv/spectrograph.c \
       brainfpv/brainfpv_osd.c \
       brainfpv/video_quadspi.c \
       brainfpv/osd_utils.c \
       brainfpv/fonts.c \
       brainfpv/images.c \
       brainfpv/video_quadspi.c \
       brainfpv/ir_transponder.c \
       io/displayport_max7456.c \
       cms/cms_menu_brainfpv.c

SRC += $(TARGET_SRC)
endif

ifneq ($(filter SPECTROGRAPH,$(FEATURES)),)
DSPLIB := $(ROOT)/lib/main/DSP_Lib
DEVICE_FLAGS += -DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1 -DUNALIGNED_SUPPORT_DISABLE

INCLUDE_DIRS += $(DSPLIB)/Include

SRC += brainfpv/spectrograph.c

SRC += $(DSPLIB)/Source/TransformFunctions/arm_rfft_fast_f32.c
SRC += $(DSPLIB)/Source/TransformFunctions/arm_cfft_f32.c
SRC += $(DSPLIB)/Source/TransformFunctions/arm_rfft_fast_init_f32.c
SRC += $(DSPLIB)/Source/TransformFunctions/arm_cfft_radix8_f32.c
SRC += $(DSPLIB)/Source/CommonTables/arm_common_tables.c

SRC += $(DSPLIB)/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c
SRC += $(DSPLIB)/Source/StatisticsFunctions/arm_max_f32.c

SRC += $(wildcard $(DSPLIB)/Source/*/*.S)
endif
# end target specific make file checks

# Search path and source files for the ST stdperiph library
VPATH        := $(VPATH):$(STDPERIPH_DIR)/src
