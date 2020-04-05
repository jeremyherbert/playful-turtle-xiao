/* SPDX-License-Identifier: MIT */

#include "sam.h"
#include "compiler.h"
#include "utils.h"

#include "hal/include/hal_gpio.h"
#include "hal/include/hal_init.h"
#include "hri/hri_nvmctrl_d21.h"

#include "hpl/gclk/hpl_gclk_base.h"
#include "hpl_pm_config.h"
#include "hpl/pm/hpl_pm_base.h"

#include "tusb.h"
#include "SEGGER_RTT.h"
#include "ringbuf.h"

#define LED_BUSY_PIN PIN_PA17
#define LED_TX_PIN PIN_PA19
#define LED_RX_PIN PIN_PA18

#define UART_TX_PIN PIN_PB08 // SERCOM4_PAD0
#define UART_RX_PIN PIN_PB09 // SERCOM4_PAD1

#define SPI_MISO_PIN PIN_PA08 // SERCOM0_PAD0
#define SPI_MOSI_PIN PIN_PA10 // SERCOM0_PAD2
#define SPI_SCK_PIN PIN_PA11 // SERCOM0_PAD3
#define SPI_CS_PIN PIN_PA09 // SERCOM0_PAD1

#define GPO_0_PIN PIN_PA02
#define GPO_1_PIN PIN_PA04
#define GPO_2_PIN PIN_PA07
#define GPO_3_PIN PIN_PA05
#define GPO_4_PIN PIN_PA06

volatile uint32_t busy_led_blink_ms = 500;
volatile uint32_t tx_led_blink_ms = 0;
volatile uint32_t rx_led_blink_ms = 0;

/*****************************************************
 * Hardware setup
 ****************************************************/

/* Referenced GCLKs, should be initialized firstly */
#define _GCLK_INIT_1ST (1 << 0 | 1 << 1)

/* Not referenced GCLKs, initialized last */
#define _GCLK_INIT_LAST (~_GCLK_INIT_1ST)

void clock_init() {
    // Clock init ( follow hpl_init.c )
    hri_nvmctrl_set_CTRLB_RWS_bf(NVMCTRL, 2);

    _pm_init();
    _sysctrl_init_sources();
    _gclk_init_generators_by_fref(_GCLK_INIT_1ST);
    _sysctrl_init_referenced_generators();
    _gclk_init_generators_by_fref(_GCLK_INIT_LAST);

    // Update SystemCoreClock since it is hard coded with asf4 and not correct
    // Init 1ms tick timer (samd SystemCoreClock may not correct)
    SystemCoreClock = CONF_CPU_FREQUENCY;
    SysTick_Config(CONF_CPU_FREQUENCY / 1000);

    // enable USB clock
    _pm_enable_bus_clock(PM_BUS_APBB, USB);
    _pm_enable_bus_clock(PM_BUS_AHB, USB);
    _gclk_enable_channel(USB_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable UART clock
    _pm_enable_bus_clock(PM_BUS_APBC, SERCOM4);
    _gclk_enable_channel(SERCOM4_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable SPI clock
    _pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
    _gclk_enable_channel(SERCOM0_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);

    // enable DMA clock
    _pm_enable_bus_clock(PM_BUS_APBB, DMAC);
    _pm_enable_bus_clock(PM_BUS_AHB, DMAC);
    // DMAC doesn't have a GCLK channel
}

void gpio_init() {
    // USB
    gpio_set_pin_direction(PIN_PA24, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(PIN_PA24, false);
    gpio_set_pin_pull_mode(PIN_PA24, GPIO_PULL_OFF);

    gpio_set_pin_direction(PIN_PA25, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(PIN_PA25, false);
    gpio_set_pin_pull_mode(PIN_PA25, GPIO_PULL_OFF);

    gpio_set_pin_function(PIN_PA24, PINMUX_PA24G_USB_DM);
    gpio_set_pin_function(PIN_PA25, PINMUX_PA25G_USB_DP);

    // LEDs (active low)
    gpio_set_pin_direction(LED_BUSY_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(LED_BUSY_PIN, true);
    gpio_set_pin_pull_mode(LED_BUSY_PIN, GPIO_PULL_OFF);

    gpio_set_pin_direction(LED_TX_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(LED_TX_PIN, true);
    gpio_set_pin_pull_mode(LED_TX_PIN, GPIO_PULL_OFF);

    gpio_set_pin_direction(LED_RX_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(LED_RX_PIN, true);
    gpio_set_pin_pull_mode(LED_RX_PIN, GPIO_PULL_OFF);

    // UART
    gpio_set_pin_function(UART_TX_PIN, PINMUX_PB08D_SERCOM4_PAD0);
    gpio_set_pin_function(UART_RX_PIN, PINMUX_PB09D_SERCOM4_PAD1);
    gpio_set_pin_pull_mode(UART_RX_PIN, GPIO_PULL_UP);

    // SPI
    gpio_set_pin_level(SPI_MISO_PIN, false);
    gpio_set_pin_direction(SPI_MISO_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_function(SPI_MISO_PIN, PINMUX_PA08C_SERCOM0_PAD0);

    gpio_set_pin_direction(SPI_CS_PIN, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(SPI_CS_PIN, GPIO_PULL_OFF);
    gpio_set_pin_function(SPI_CS_PIN, PINMUX_PA09C_SERCOM0_PAD1);

    gpio_set_pin_direction(SPI_MOSI_PIN, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(SPI_MOSI_PIN, GPIO_PULL_OFF);
    gpio_set_pin_function(SPI_MOSI_PIN, PINMUX_PA10C_SERCOM0_PAD2);

    gpio_set_pin_level(SPI_SCK_PIN,false);
    gpio_set_pin_direction(SPI_SCK_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_function(SPI_SCK_PIN, PINMUX_PA11C_SERCOM0_PAD3);

    // general purpose output pins
    gpio_set_pin_direction(GPO_0_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(GPO_0_PIN, false);
    gpio_set_pin_pull_mode(GPO_0_PIN, GPIO_PULL_OFF);

    gpio_set_pin_direction(GPO_1_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(GPO_1_PIN, false);
    gpio_set_pin_pull_mode(GPO_1_PIN, GPIO_PULL_OFF);

    gpio_set_pin_direction(GPO_2_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(GPO_2_PIN, false);
    gpio_set_pin_pull_mode(GPO_2_PIN, GPIO_PULL_OFF);

    gpio_set_pin_direction(GPO_3_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(GPO_3_PIN, false);
    gpio_set_pin_pull_mode(GPO_3_PIN, GPIO_PULL_OFF);

    gpio_set_pin_direction(GPO_4_PIN, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(GPO_4_PIN, false);
    gpio_set_pin_pull_mode(GPO_4_PIN, GPIO_PULL_OFF);
}

/*****************************************************
 * HID/SPI interface
 ****************************************************/

volatile uint8_t gpio_reg = 0;
volatile uint8_t gamepad_registers[8];
volatile bool hid_pending = false;

volatile uint8_t pending_data[2];
volatile uint8_t pending_index = 0;

void SERCOM0_Handler() {
    if (hri_sercomspi_get_interrupt_SSL_bit(SERCOM0)) {
        hri_sercomspi_clear_interrupt_SSL_bit(SERCOM0);
        pending_index = 0;
    }
    if (hri_sercomspi_get_interrupt_RXC_bit(SERCOM0)) {
        uint8_t tmp = hri_sercomusart_read_DATA_reg(SERCOM0);
        if (pending_index < 2) {
            pending_data[pending_index++] = tmp;
        }
        busy_led_blink_ms = 100;
    }
    if (hri_sercomspi_get_interrupt_TXC_bit(SERCOM0)) {
        hri_sercomspi_clear_interrupt_TXC_bit(SERCOM0);

        if (pending_index == 2) {
            if (pending_data[0] < 8) {
                gamepad_registers[pending_data[0]] = pending_data[1];
            } else if (pending_data[0] == 0x08) {
                gpio_reg = pending_data[1];
            } else if (pending_data[0] == 0xFF) {
                hid_pending = true;
            }
        }
    }
}

void spi_init() {
    // SPI is via SERCOM0
    hri_sercomspi_wait_for_sync(SERCOM0, SERCOM_SPI_SYNCBUSY_SWRST);
    hri_sercomspi_set_CTRLA_SWRST_bit(SERCOM0);
    hri_sercomspi_wait_for_sync(SERCOM0, SERCOM_SPI_SYNCBUSY_SWRST);

    hri_sercomspi_write_CTRLA_reg(SERCOM0,
                                    SERCOM_SPI_CTRLA_DIPO(2) | // MOSI is PAD2
                                    SERCOM_SPI_CTRLA_DOPO(3) | // MISO is PAD0, SCK is PAD3, CS is PAD1
                                    SERCOM_SPI_CTRLA_MODE_SPI_SLAVE // SPI slave mode
                                    );

    // enable RX and CS low detection
    hri_sercomspi_write_CTRLB_reg(SERCOM0, SERCOM_SPI_CTRLB_RXEN | SERCOM_SPI_CTRLB_SSDE);

    hri_sercomspi_set_INTEN_TXC_bit(SERCOM0); // enable TX complete interrupt
    hri_sercomspi_set_INTEN_RXC_bit(SERCOM0); // enable RX interrupt
    hri_sercomspi_set_INTEN_SSL_bit(SERCOM0); // enable slave select low interrupt
    NVIC_EnableIRQ(SERCOM0_IRQn);

    // enable SERCOM peripheral
    hri_sercomspi_wait_for_sync(SERCOM0, SERCOM_SPI_SYNCBUSY_SWRST);
    hri_sercomspi_set_CTRLA_ENABLE_bit(SERCOM0);
    hri_sercomspi_wait_for_sync(SERCOM0, SERCOM_SPI_SYNCBUSY_SWRST);
}

// HID get report callback
uint16_t tud_hid_get_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    // dummy implementation, should never be used anyway
    SEGGER_RTT_printf(0, "tud_hid_get_report_cb: report_id: %d, report_type: %d, reqlen: %d\n", report_id, report_type,
                      reqlen);

    for (int i = 0; i < reqlen; i++) {
        buffer[i] = i;
    }

    return reqlen;
}

// HID set report callback
void tud_hid_set_report_cb(uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    // unimplemented as this is not needed

    (void) buffer;

    SEGGER_RTT_printf(0, "tud_hid_set_report_cb: report_id: %d, report_type: %d, bufsize: %d\n", report_id, report_type,
                      bufsize);
}

// HID process task; sends a HID report if there is a new one pending
void hid_process(void) {
    uint8_t buf[8];

    if (hid_pending) {
        __disable_irq();
        memcpy(buf, (uint8_t *) &gamepad_registers[0], 8);
        __enable_irq();

        tud_hid_report(0, buf, 8);
        SEGGER_RTT_printf(0, "hid report sent\n");
        hid_pending = false;
    }
}

// update general purpose outputs
void gpio_process() {
    __disable_irq();
    uint8_t tmp = gpio_reg;
    __enable_irq();
    gpio_set_pin_level(GPO_0_PIN, tmp & (1 << 0));
    gpio_set_pin_level(GPO_1_PIN, tmp & (1 << 1));
    gpio_set_pin_level(GPO_2_PIN, tmp & (1 << 2));
    gpio_set_pin_level(GPO_3_PIN, tmp & (1 << 3));
    gpio_set_pin_level(GPO_4_PIN, tmp & (1 << 4));
}

/*****************************************************
 * CDC/UART interface
 ****************************************************/
uint8_t usart_to_pc_ringbuf_buffer[512];
ringbuf_t usart_to_pc_ringbuf;

COMPILER_ALIGNED(16)
static uint8_t dma_buffer[64];

COMPILER_ALIGNED(16)
static DmacDescriptor _descriptor_section[DMAC_CH_NUM] SECTION_DMAC_DESCRIPTOR;

COMPILER_ALIGNED(16)
static DmacDescriptor _descriptor_writeback_section[DMAC_CH_NUM] SECTION_DMAC_DESCRIPTOR;

// USART interrupt
void SERCOM4_Handler() {
    if (hri_sercomusart_get_interrupt_RXC_bit(SERCOM4)) {
        if (hri_sercomusart_read_STATUS_reg(SERCOM4)
            & (SERCOM_USART_STATUS_PERR | SERCOM_USART_STATUS_FERR | SERCOM_USART_STATUS_BUFOVF
               | SERCOM_USART_STATUS_ISF
               | SERCOM_USART_STATUS_COLL)) {
            hri_sercomusart_clear_STATUS_reg(SERCOM4, SERCOM_USART_STATUS_MASK);
            return;
        }
        uint8_t tmp = hri_sercomusart_read_DATA_reg(SERCOM4);
        ringbuf_push(&usart_to_pc_ringbuf, &tmp, 1);
        rx_led_blink_ms = 100;
    }
}

void usart_init() {
    ringbuf_init(&usart_to_pc_ringbuf, usart_to_pc_ringbuf_buffer, sizeof(usart_to_pc_ringbuf_buffer));

    // USART is via SERCOM4
    hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_SWRST);
    hri_sercomusart_set_CTRLA_SWRST_bit(SERCOM4);
    hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_SWRST);

    hri_sercomusart_write_CTRLA_reg(SERCOM4,
            SERCOM_USART_CTRLA_DORD | // send LSB first (this is the normal way for UART)
            SERCOM_USART_CTRLA_SAMPR(1) | // 16x oversampling with fractional baud
            SERCOM_USART_CTRLA_RXPO(1) | // RX uses PAD1
            SERCOM_USART_CTRLA_TXPO(0) | // TX uses PAD0
            SERCOM_USART_CTRLA_MODE(1) // use internal clock
            );

    // enable TX and RX
    hri_sercomusart_write_CTRLB_reg(SERCOM4, SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);

    hri_sercomusart_write_BAUD_FRAC_BAUD_bf(SERCOM4, 312);
    hri_sercomusart_write_BAUD_FRAC_FP_bf(SERCOM4, 3); // 9600 baud

    hri_sercomusart_write_INTEN_RXC_bit(SERCOM4, true); // enable RX interrupt
    NVIC_EnableIRQ(SERCOM4_IRQn);

    // enable SERCOM peripheral
    hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_SWRST);
    hri_sercomusart_set_CTRLA_ENABLE_bit(SERCOM4);
    hri_sercomusart_wait_for_sync(SERCOM4, SERCOM_USART_SYNCBUSY_SWRST);

    // clear all DMA descriptors
    memset(_descriptor_section, 0, sizeof(_descriptor_section));
    memset(_descriptor_writeback_section, 0, sizeof(_descriptor_writeback_section));

    // reset DMA
    hri_dmac_clear_CTRL_DMAENABLE_bit(DMAC);
    hri_dmac_clear_CTRL_CRCENABLE_bit(DMAC);
    hri_dmac_set_CHCTRLA_SWRST_bit(DMAC);

    // enable only priority 0 as we are only doing a single request
    hri_dmac_write_CTRL_LVLEN0_bit(DMAC, true);

    // set up DMA descriptor addresses
    hri_dmac_write_BASEADDR_reg(DMAC, (uint32_t)_descriptor_section);
    hri_dmac_write_WRBADDR_reg(DMAC, (uint32_t)_descriptor_writeback_section);

    // set DMA target as the USART data register
    hri_dmacdescriptor_write_DSTADDR_reg(&_descriptor_section[0], (uint32_t) &(SERCOM4->USART.DATA));

    // trigger a beat if USART TX is complete
    hri_dmac_write_CHID_reg(DMAC, 0);
    hri_dmac_write_CHCTRLB_TRIGACT_bf(DMAC, DMAC_CHCTRLB_TRIGACT_BEAT_Val);
    hri_dmac_write_CHCTRLB_TRIGSRC_bf(DMAC, SERCOM4_DMAC_ID_TX);

    hri_dmacdescriptor_set_BTCTRL_SRCINC_bit(&_descriptor_section[0]); // increment source address
    hri_dmacdescriptor_set_BTCTRL_STEPSEL_bit(&_descriptor_section[0]); // stepsize applies to SRC
    hri_dmacdescriptor_set_BTCTRL_VALID_bit(&_descriptor_section[0]); // mark descriptor as valid

    // start DMA
    hri_dmac_set_CTRL_DMAENABLE_bit(DMAC);
}

void cdc_process() {
    uint8_t buf[64];

    __disable_irq();
    size_t len = ringbuf_pop(&usart_to_pc_ringbuf, buf, 64);
    __enable_irq();

    if (tud_cdc_connected()) {
        if (len) { // if data needs to be sent to CDC
            tud_cdc_write(buf, len);
            tud_cdc_write_flush();
        }

        if (tud_cdc_available()) { // if data has been received from CDC
            hri_dmac_write_CHID_reg(DMAC, 0);
            if (!hri_dmac_get_CHCTRLA_ENABLE_bit(DMAC)) { // is DMA transaction complete?
                uint32_t tx_len = tud_cdc_read(dma_buffer, 64);

                if (tx_len) {
                    // set pointer to the final beat of the transaction
                    hri_dmacdescriptor_write_SRCADDR_reg(&_descriptor_section[0], (uint32_t)(dma_buffer+tx_len));

                    // set beat count
                    hri_dmacdescriptor_write_BTCNT_reg(&_descriptor_section[0], (uint16_t)tx_len);

                    // enable DMA channel
                    hri_dmac_set_CHCTRLA_ENABLE_bit(DMAC);
                    tx_led_blink_ms = 100;
                }
            }
        }
    }

}

/*****************************************************
 * Main system code
 ****************************************************/
volatile uint32_t system_ticks = 0;

int main() {
    clock_init();
    usart_init();
    spi_init();
    gpio_init();
    tusb_init();

    SEGGER_RTT_printf(0, "Started\n");

    while(1) {
        tud_task(); // handle USB packets
        cdc_process();
        hid_process();
        gpio_process();
    }
}

volatile bool led_state = false;
void SysTick_Handler (void)
{
    if (busy_led_blink_ms) {
        busy_led_blink_ms--;
        gpio_set_pin_level(LED_BUSY_PIN, led_state);
    } else {
        gpio_set_pin_level(LED_BUSY_PIN, true);
    }

    if (tx_led_blink_ms) {
        tx_led_blink_ms--;
        gpio_set_pin_level(LED_TX_PIN, led_state);
    } else {
        gpio_set_pin_level(LED_TX_PIN, true);
    }

    if (rx_led_blink_ms) {
        rx_led_blink_ms--;
        gpio_set_pin_level(LED_RX_PIN, led_state);
    } else {
        gpio_set_pin_level(LED_RX_PIN, true);
    }

    if ((system_ticks % 25) == 0) led_state = !led_state;
    system_ticks++;
}

void HardFault_Handler(void) {
    while(1);
}

// Required by __libc_init_array in startup code if we are compiling using -nostdlib/-nostartfiles.
void _init(void)
{

}