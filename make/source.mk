COMMON_SRC = \
            build/build_config.c \
            build/debug.c \
            build/debug_pin.c \
            build/version.c \
            $(TARGET_DIR_SRC) \
            main.c \
            $(addprefix pg/, $(notdir $(wildcard $(SRC_DIR)/pg/*.c))) \
            $(addprefix common/,$(notdir $(wildcard $(SRC_DIR)/common/*.c))) \
            $(addprefix config/,$(notdir $(wildcard $(SRC_DIR)/config/*.c))) \
            cli/cli.c \
            cli/settings.c \
            config/config.c \
            drivers/adc.c \
            drivers/dshot.c \
            drivers/dshot_dpwm.c \
            drivers/dshot_command.c \
            drivers/buf_writer.c \
            drivers/bus.c \
            drivers/bus_i2c_config.c \
            drivers/bus_i2c_busdev.c \
            drivers/bus_i2c_soft.c \
            drivers/bus_quadspi.c \
            drivers/bus_spi.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/buttons.c \
            drivers/display.c \
            drivers/display_canvas.c \
            drivers/dma_reqmap.c \
            drivers/exti.c \
            drivers/io.c \
            drivers/light_led.c \
            drivers/mco.c \
            drivers/motor.c \
            drivers/pinio.c \
            drivers/pin_pull_up_down.c \
            drivers/resource.c \
            drivers/rcc.c \
            drivers/serial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart.c \
            drivers/serial_uart_pinconfig.c \
            drivers/sound_beeper.c \
            drivers/stack_check.c \
            drivers/system.c \
            drivers/timer_common.c \
            drivers/timer.c \
            drivers/transponder_ir_arcitimer.c \
            drivers/transponder_ir_ilap.c \
            drivers/transponder_ir_erlt.c \
            fc/board_info.c \
            fc/dispatch.c \
            fc/hardfaults.c \
            fc/tasks.c \
            fc/runtime_config.c \
            fc/stats.c \
            io/beeper.c \
            io/piniobox.c \
            io/serial.c \
            io/smartaudio_protocol.c \
            io/statusindicator.c \
            io/tramp_protocol.c \
            io/transponder_ir.c \
            io/usb_cdc_hid.c \
            io/usb_msc.c \
            msp/msp.c \
            msp/msp_box.c \
            msp/msp_serial.c \
            scheduler/scheduler.c \
            sensors/adcinternal.c \
            sensors/battery.c \
            sensors/current.c \
            sensors/voltage.c \
            target/config_helper.c \
            fc/init.c \
            fc/controlrate_profile.c \
            drivers/camera_control.c \
            drivers/accgyro/gyro_sync.c \
            drivers/pwm_esc_detect.c \
            drivers/pwm_output.c \
            drivers/rx/rx_spi.c \
            drivers/rx/rx_xn297.c \
            drivers/rx/rx_pwm.c \
            drivers/serial_softserial.c \
            fc/core.c \
            fc/rc.c \
            fc/rc_adjustments.c \
            fc/rc_controls.c \
            fc/rc_modes.c \
            flight/position.c \
            flight/failsafe.c \
            flight/gps_rescue.c \
            flight/gyroanalyse.c \
            flight/imu.c \
            flight/interpolated_setpoint.c \
            flight/mixer.c \
            flight/mixer_tricopter.c \
            flight/pid.c \
            flight/rpm_filter.c \
            flight/servos.c \
            flight/servos_tricopter.c \
            io/serial_4way.c \
            io/serial_4way_avrootloader.c \
            io/serial_4way_stk500v2.c \
            rx/ibus.c \
            rx/jetiexbus.c \
            rx/msp.c \
            rx/pwm.c \
            rx/frsky_crc.c \
            rx/rx.c \
            rx/rx_bind.c \
            rx/rx_spi.c \
            rx/rx_spi_common.c \
            rx/crsf.c \
            rx/sbus.c \
            rx/sbus_channels.c \
            rx/spektrum.c \
            rx/srxl2.c \
            io/spektrum_vtx_control.c \
            io/spektrum_rssi.c \
            rx/sumd.c \
            rx/sumh.c \
            rx/xbus.c \
            rx/fport.c \
            sensors/acceleration.c \
            sensors/boardalignment.c \
            sensors/compass.c \
            sensors/gyro.c \
            sensors/gyro_init.c \
            sensors/initialisation.c \
            blackbox/blackbox.c \
            blackbox/blackbox_encoding.c \
            blackbox/blackbox_io.c \
            cms/cms.c \
            cms/cms_menu_blackbox.c \
            cms/cms_menu_failsafe.c \
            cms/cms_menu_firmware.c \
            cms/cms_menu_gps_rescue.c\
            cms/cms_menu_imu.c \
            cms/cms_menu_ledstrip.c \
            cms/cms_menu_main.c \
            cms/cms_menu_misc.c \
            cms/cms_menu_osd.c \
            cms/cms_menu_power.c \
            cms/cms_menu_saveexit.c \
            cms/cms_menu_vtx_common.c \
            cms/cms_menu_vtx_rtc6705.c \
            cms/cms_menu_vtx_smartaudio.c \
            cms/cms_menu_vtx_tramp.c \
            drivers/display_ug2864hsweg01.c \
            drivers/light_ws2811strip.c \
            drivers/rangefinder/rangefinder_hcsr04.c \
            drivers/rangefinder/rangefinder_lidartf.c \
            drivers/serial_escserial.c \
            drivers/vtx_common.c \
            drivers/vtx_table.c \
            io/dashboard.c \
            io/displayport_frsky_osd.c \
            io/displayport_max7456.c \
            io/displayport_msp.c \
            io/displayport_oled.c \
            io/displayport_srxl.c \
            io/displayport_crsf.c \
            io/displayport_hott.c \
            io/frsky_osd.c \
            io/rcdevice_cam.c \
            io/rcdevice.c \
            io/gps.c \
            io/ledstrip.c \
            io/pidaudio.c \
            osd/osd.c \
            osd/osd_elements.c \
            sensors/barometer.c \
            sensors/rangefinder.c \
            telemetry/telemetry.c \
            telemetry/crsf.c \
            telemetry/srxl.c \
            telemetry/frsky_hub.c \
            telemetry/hott.c \
            telemetry/jetiexbus.c \
            telemetry/smartport.c \
            telemetry/ltm.c \
            telemetry/mavlink.c \
            telemetry/msp_shared.c \
            telemetry/ibus.c \
            telemetry/ibus_shared.c \
            sensors/esc_sensor.c \
            io/vtx.c \
            io/vtx_rtc6705.c \
            io/vtx_smartaudio.c \
            io/vtx_tramp.c \
            io/vtx_control.c \

COMMON_DEVICE_SRC = \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC)

COMMON_SRC := $(COMMON_SRC) $(COMMON_DEVICE_SRC)

ifeq ($(EXST),yes)
TARGET_FLAGS := -DUSE_EXST $(TARGET_FLAGS)
endif

ifeq ($(RAM_BASED),yes)
TARGET_FLAGS := -DUSE_EXST -DCONFIG_IN_RAM -DRAMBASED $(TARGET_FLAGS)
endif

ifeq ($(SIMULATOR_BUILD),yes)
TARGET_FLAGS := -DSIMULATOR_BUILD $(TARGET_FLAGS)
endif

SPEED_OPTIMISED_SRC := ""
SIZE_OPTIMISED_SRC  := ""

ifneq ($(TARGET),$(filter $(TARGET),$(F1_TARGETS)))
SPEED_OPTIMISED_SRC := $(SPEED_OPTIMISED_SRC) \
            common/encoding.c \
            common/filter.c \
            common/maths.c \
            common/typeconversion.c \
            drivers/accgyro/accgyro_mpu.c \
            drivers/accgyro/accgyro_mpu3050.c \
            drivers/accgyro/accgyro_spi_bmi160.c \
            drivers/accgyro/accgyro_spi_bmi270.c \
            drivers/accgyro_legacy/accgyro_adxl345.c \
            drivers/accgyro_legacy/accgyro_bma280.c \
            drivers/accgyro_legacy/accgyro_l3g4200d.c \
            drivers/accgyro_legacy/accgyro_l3gd20.c \
            drivers/accgyro_legacy/accgyro_lsm303dlhc.c \
            drivers/accgyro_legacy/accgyro_mma845x.c \
            drivers/adc.c \
            drivers/buf_writer.c \
            drivers/bus.c \
            drivers/bus_quadspi.c \
            drivers/bus_spi.c \
            drivers/exti.c \
            drivers/io.c \
            drivers/pwm_output.c \
            drivers/rcc.c \
            drivers/serial.c \
            drivers/serial_uart.c \
            drivers/system.c \
            drivers/timer.c \
            fc/core.c \
            fc/tasks.c \
            fc/rc.c \
            fc/rc_controls.c \
            fc/runtime_config.c \
            flight/gyroanalyse.c \
            flight/imu.c \
            flight/mixer.c \
            flight/pid.c \
            flight/rpm_filter.c \
            rx/ibus.c \
            rx/rx.c \
            rx/rx_spi.c \
            rx/crsf.c \
            rx/frsky_crc.c \
            rx/sbus.c \
            rx/sbus_channels.c \
            rx/spektrum.c \
            rx/srxl2.c \
            rx/sumd.c \
            rx/xbus.c \
            rx/fport.c \
            scheduler/scheduler.c \
            sensors/acceleration.c \
            sensors/boardalignment.c \
            sensors/gyro.c \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC) \

SIZE_OPTIMISED_SRC := $(SIZE_OPTIMISED_SRC) \
            bus_bst_stm32f30x.c \
            cli/cli.c \
            cli/settings.c \
            drivers/accgyro/accgyro_fake.c \
            drivers/barometer/barometer_bmp085.c \
            drivers/barometer/barometer_bmp280.c \
            drivers/barometer/barometer_fake.c \
            drivers/barometer/barometer_ms5611.c \
            drivers/barometer/barometer_lps.c \
            drivers/barometer/barometer_qmp6988.c \
            drivers/bus_i2c_config.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/compass/compass_ak8963.c \
            drivers/compass/compass_ak8975.c \
            drivers/compass/compass_fake.c \
            drivers/compass/compass_hmc5883l.c \
            drivers/compass/compass_qmc5883l.c \
            drivers/compass/compass_lis3mdl.c \
            drivers/display_ug2864hsweg01.c \
            drivers/inverter.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_hal.c \
            drivers/light_ws2811strip_stdperiph.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_tcp.c \
            drivers/serial_uart_init.c \
            drivers/serial_uart_pinconfig.c \
            drivers/serial_usb_vcp.c \
            drivers/transponder_ir_io_hal.c \
            drivers/transponder_ir_io_stdperiph.c \
            drivers/vtx_rtc6705_soft_spi.c \
            drivers/vtx_rtc6705.c \
            drivers/vtx_common.c \
            fc/init.c \
            fc/board_info.c \
            config/config_eeprom.c \
            config/feature.c \
            config/config_streamer.c \
            i2c_bst.c \
            io/dashboard.c \
            io/serial.c \
            io/serial_4way.c \
            io/serial_4way_avrootloader.c \
            io/serial_4way_stk500v2.c \
            io/transponder_ir.c \
            io/usb_cdc_hid.c \
            msp/msp_serial.c \
            cms/cms.c \
            cms/cms_menu_blackbox.c \
            cms/cms_menu_failsafe.c \
            cms/cms_menu_firmware.c \
            cms/cms_menu_gps_rescue.c\
            cms/cms_menu_imu.c \
            cms/cms_menu_ledstrip.c \
            cms/cms_menu_main.c \
            cms/cms_menu_misc.c \
            cms/cms_menu_osd.c \
            cms/cms_menu_power.c \
            cms/cms_menu_saveexit.c \
            cms/cms_menu_vtx_common.c \
            cms/cms_menu_vtx_rtc6705.c \
            cms/cms_menu_vtx_smartaudio.c \
            cms/cms_menu_vtx_tramp.c \
            io/vtx.c \
            io/vtx_rtc6705.c \
            io/vtx_smartaudio.c \
            io/vtx_tramp.c \
            io/vtx_control.c \
            io/spektrum_vtx_control.c \
            osd/osd.c \
            osd/osd_elements.c \
            rx/rx_bind.c \
            sensors/gyro_init.c

# Gyro driver files that only contain initialization and configuration code - not runtime code
SIZE_OPTIMISED_SRC := $(SIZE_OPTIMISED_SRC) \
            drivers/accgyro/accgyro_mpu6050.c \
            drivers/accgyro/accgyro_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu6000.c \
            drivers/accgyro/accgyro_spi_mpu6500.c \
            drivers/accgyro/accgyro_spi_mpu9250.c \
            drivers/accgyro/accgyro_spi_icm20689.c


# F4 and F7 optimizations
ifneq ($(TARGET),$(filter $(TARGET),$(F3_TARGETS)))
SPEED_OPTIMISED_SRC := $(SPEED_OPTIMISED_SRC) \
            drivers/bus_i2c_hal.c \
            drivers/bus_spi_ll.c \
            rx/frsky_crc.c \
            drivers/max7456.c \
            drivers/pwm_output_dshot.c \
            drivers/pwm_output_dshot_shared.c \
            drivers/pwm_output_dshot_hal.c

SIZE_OPTIMISED_SRC := $(SIZE_OPTIMISED_SRC) \
            drivers/bus_i2c_hal_init.c
endif #!F3
endif #!F1

# check if target.mk supplied
SRC := $(STARTUP_SRC) $(MCU_COMMON_SRC) $(SRC) $(VARIANT_SRC)

# Files that should not be optimized, useful for debugging IMPRECISE cpu faults.
# Specify FULL PATH, e.g. "./lib/main/STM32F7/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_sdmmc.c"
NOT_OPTIMISED_SRC := $(NOT_OPTIMISED_SRC) \

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

DEVICE_FLAGS += -DFFT_SIZE_32
endif

ifneq ($(filter ONBOARDFLASH,$(FEATURES)),)
SRC += \
            drivers/flash.c \
            drivers/flash_m25p16.c \
            drivers/flash_w25n01g.c \
            drivers/flash_w25m.c \
            io/flashfs.c \
            $(MSC_SRC)
endif

SRC += $(COMMON_SRC)

#excludes
SRC   := $(filter-out $(MCU_EXCLUDES), $(SRC))

ifneq ($(filter SDCARD_SPI,$(FEATURES)),)
SRC += \
            drivers/sdcard.c \
            drivers/sdcard_spi.c \
            drivers/sdcard_standard.c \
            io/asyncfatfs/asyncfatfs.c \
            io/asyncfatfs/fat_standard.c \
            $(MSC_SRC)
endif

ifneq ($(filter SDCARD_SDIO,$(FEATURES)),)
SRC += \
            drivers/sdcard.c \
            drivers/sdcard_sdio_baremetal.c \
            drivers/sdcard_standard.c \
            io/asyncfatfs/asyncfatfs.c \
            io/asyncfatfs/fat_standard.c \
            $(MSC_SRC)
endif

ifneq ($(filter VCP,$(FEATURES)),)
SRC += $(VCP_SRC)
endif


ifneq ($(filter BRAINFPV,$(FEATURES)),)
SRC += brainfpv/brainfpv_system.c \
	   brainfpv/spectrograph.c \
       brainfpv/brainfpv_osd.c \
       brainfpv/video_quadspi.c \
       brainfpv/osd_utils.c \
       brainfpv/fonts.c \
       brainfpv/images.c \
       brainfpv/video_quadspi.c \
       brainfpv/ir_transponder.c \
       brainfpv/brainfpv_rgb_led_timer.c \
       io/displayport_max7456.c \
       cms/cms_menu_brainfpv.c
ifeq ($(TARGET),$(filter $(TARGET),$(H750xB_TARGETS)))
SRC +=  $(ROOT)/lib/main/STM32H7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c \
        $(ROOT)/lib/main/STM32H7/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_comp.c
endif

INCLUDE_DIRS += $(ROOT)/src/main/brainfpv

endif

ifneq ($(filter CHIBIOS,$(FEATURES)),)
CHIBIOS_DIR := $(ROOT)/lib/main/ChibiOS

CHIBIOS_RT_DIR := $(CHIBIOS_DIR)/os/rt
CHIBIOS_HAL_INC_DIR := $(CHIBIOS_DIR)/os/hal/include
CHIBIOS_OSAL_DIR := $(CHIBIOS_DIR)/os/hal/osal/rt-nil
CHIBIOS_PORT_DIR := $(CHIBIOS_DIR)/os/common/ports/ARMCMx
CHIBIOS_HAL_LLD_DIR := $(CHIBIOS_DIR)/os/hal/ports/STM32/LLD
CHIBIOS_HAL_PORT_DIR := $(CHIBIOS_DIR)/os/hal/ports/common/ARMCMx

INCLUDE_DIRS += $(CHIBIOS_OSAL_DIR)
INCLUDE_DIRS += $(CHIBIOS_RT_DIR)/include
INCLUDE_DIRS += $(CHIBIOS_PORT_DIR)
INCLUDE_DIRS += $(CHIBIOS_HAL_INC_DIR)
INCLUDE_DIRS += $(CHIBIOS_HAL_PORT_DIR)
INCLUDE_DIRS += $(CHIBIOS_PORT_DIR)/compilers/GCC
INCLUDE_DIRS += $(CHIBIOS_DIR)/os/license
INCLUDE_DIRS += $(CHIBIOS_DIR)/os/oslib/include
INCLUDE_DIRS += $(CHIBIOS_HAL_LLD_DIR)/TIMv1

SRC += $(wildcard $(CHIBIOS_RT_DIR)/src/*.c)
SRC += $(wildcard $(CHIBIOS_OSAL_DIR)/*.c)
SRC += $(CHIBIOS_PORT_DIR)/chcore.c
SRC += $(CHIBIOS_PORT_DIR)/chcore_v7m.c
SRC += $(CHIBIOS_PORT_DIR)/compilers/GCC/chcoreasm_v7m.S
SRC += $(CHIBIOS_HAL_LLD_DIR)/TIMv1/hal_st_lld.c
SRC += $(CHIBIOS_DIR)/os/hal/src/hal_st.c
SRC += $(CHIBIOS_HAL_PORT_DIR)/nvic.c

DEVICE_FLAGS += -DUSE_CHIBIOS -DCORTEX_USE_FPU=TRUE -DCORTEX_SIMPLIFIED_PRIORITY=TRUE $(DDEFS)

ifeq ($(TARGET),$(filter $(TARGET), $(F446_TARGETS)))
DEVICE_FLAGS += -DSTM32F4XX
INCLUDE_DIRS += $(CHIBIOS_DIR)/os/common/startup/ARMCMx/devices/STM32F4xx
INCLUDE_DIRS += $(CHIBIOS_DIR)/os/hal/ports/STM32/STM32F4xx
endif

ifeq ($(TARGET),$(filter $(TARGET),$(H750xB_TARGETS)))
DEVICE_FLAGS += -DSTM32H7XX
INCLUDE_DIRS += $(CHIBIOS_DIR)/os/common/startup/ARMCMx/devices/STM32H7xx
INCLUDE_DIRS += $(CHIBIOS_DIR)/os/hal/ports/STM32/STM32H7xx
endif

endif

SRC += $(TARGET_SRC)

ifneq ($(filter SPECTROGRAPH,$(FEATURES)),)
SRC += brainfpv/spectrograph.c
DEVICE_FLAGS += -DFFT_SIZE_512
endif

ifneq ($(filter MSC,$(FEATURES)),)
SRC += $(MSC_SRC)
endif
# end target specific make file checks

# Search path and source files for the ST stdperiph library
VPATH        := $(VPATH):$(STDPERIPH_DIR)/src
