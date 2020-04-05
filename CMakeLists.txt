cmake_minimum_required(VERSION 3.15)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/arm-gcc-toolchain.cmake)
project(playful_turtle_xiao_proj)
set(CMAKE_VERBOSE_MAKEFILE ON)

enable_language(ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

include(cmake/utils.cmake)

set(CPU_FLAGS "-flto -mthumb -mabi=aapcs-linux -mcpu=cortex-m0plus -fdata-sections -ffunction-sections -ffreestanding -nostartfiles -lc -lnosys --specs=nano.specs")

set(GCC_FLAGS "-fsingle-precision-constant -fno-strict-aliasing -Wdouble-promotion -Wno-endif-labels
                -Wall -Wextra -Werror -Werror-implicit-function-declaration -Wfatal-errors -Wfloat-equal -Wundef
                -Wshadow -Wwrite-strings -Wsign-compare -Wmissing-format-attribute -Wno-deprecated-declarations
                -Wnested-externs -Wunreachable-code -Wno-error=lto-type-mismatch -Wno-unused-parameter
                -ffunction-sections -fdata-sections")

add_definitions(${CPU_FLAGS} ${GCC_FLAGS} -O0 -g
        -D__SAMD21G18A__ -DCONF_DFLL_OVERWRITE_CALIBRATION=0 -DCFG_TUSB_MCU=OPT_MCU_SAMD21)

set(LINKER_FILE ${CMAKE_CURRENT_LIST_DIR}/libs/tinyusb/hw/bsp/seeeduino_xiao/samd21g18a_flash.ld)
string(APPEND CMAKE_EXE_LINKER_FLAGS " ${CPU_FLAGS} -Wl,-T ${LINKER_FILE}")
string(APPEND CMAKE_EXE_LINKER_FLAGS " -Wl,--gc-sections -Wl,-Map=output.map -fshort-enums")

set(RTT_FILES
        libs/rtt/SEGGER_RTT.c
        libs/rtt/SEGGER_RTT_printf.c)

set(TINYUSB_FILES
        libs/tinyusb/src/class/cdc/cdc_device.c
        libs/tinyusb/src/class/hid/hid_device.c
        libs/tinyusb/src/common/tusb_fifo.c
        libs/tinyusb/src/device/usbd.c
        libs/tinyusb/src/device/usbd_control.c
        libs/tinyusb/src/portable/microchip/samd/dcd_samd.c
        libs/tinyusb/src/tusb.c)

set(HAL_SRC_DIR libs/tinyusb/hw/mcu/microchip/samd/asf4/samd21)
set(HAL_FILES
        ${HAL_SRC_DIR}/gcc/gcc/startup_samd21.c
        ${HAL_SRC_DIR}/gcc/system_samd21.c

        ${HAL_SRC_DIR}/hpl/gclk/hpl_gclk.c
        ${HAL_SRC_DIR}/hpl/pm/hpl_pm.c
        ${HAL_SRC_DIR}/hpl/sysctrl/hpl_sysctrl.c

        ${HAL_SRC_DIR}/hal/src/hal_atomic.c
        )

set(MCU_FILES)

set(SRC_FILES
        libs/ringbuf/ringbuf.c

        src/main.c
        src/descriptors.c)

include_directories(
        src

        libs/rtt
        libs/ringbuf

        libs/tinyusb/hw
        libs/tinyusb/src

        ${HAL_SRC_DIR}/
        ${HAL_SRC_DIR}/config
        ${HAL_SRC_DIR}/include
        ${HAL_SRC_DIR}/hal/include
        ${HAL_SRC_DIR}/hal/utils/include
        ${HAL_SRC_DIR}/hpl/pm/
        ${HAL_SRC_DIR}/hpl/port
        ${HAL_SRC_DIR}/hri
        ${HAL_SRC_DIR}/CMSIS/Include
)

add_executable(playful_turtle_xiao ${SRC_FILES} ${MCU_FILES} ${RTT_FILES} ${TINYUSB_FILES} ${HAL_FILES})

firmware_size(playful_turtle_xiao)
generate_object(playful_turtle_xiao .bin binary)