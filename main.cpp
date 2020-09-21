/*-
 * $Copyright$
-*/
#include <common/Infrastructure.hpp>
#include <phisch/log.h>

/* for vTaskStartScheduler */
#include <FreeRTOS.h> 
#include <FreeRTOS/include/task.h>

#include <stm32/Cpu.hpp>

#include <stm32/Pll.hpp>
#include <stm32/Pwr.hpp>
#include <stm32/Flash.hpp>
#include <stm32/Gpio.hpp>
#include <stm32/Rcc.hpp>
#include <stm32/Scb.hpp>
#include <stm32/Nvic.hpp>

#include <gpio/GpioAccess.hpp>
#include <gpio/GpioEngine.hpp>
#include <gpio/GpioPin.hpp>

#include <stm32/Uart.hpp>
#include <uart/UartAccess.hpp>
#include <uart/UartDevice.hpp>

#include <tasks/Heartbeat.hpp>

#include <stm32/dma/Engine.hpp>
#include <stm32/dma/Channel.hpp>
#include <stm32/dma/Stream.hpp>

#include <stm32/Spi.hpp>
#include <spi/SpiAccess.hpp>
#include <spi/SpiDevice.hpp>
#include <devices/Ws2812bStrip.hpp>

/*******************************************************************************
 * System Devices
 ******************************************************************************/
static const constexpr stm32::PllCfg pllCfg = {
    .m_pllSource        = stm32::PllCfg::PllSource_t::e_PllSourceHSI,
    .m_hseSpeedInHz     = 0,
    .m_pllM             = 8,
    .m_pllN             = 192,
    .m_pllP             = stm32::PllCfg::PllP_t::e_PllP_Div4,
    .m_pllQ             = stm32::PllCfg::PllQ_t::e_PllQ_Div8,
    .m_sysclkSource     = stm32::PllCfg::SysclkSource_t::e_SysclkPLL,
    .m_ahbPrescaler     = stm32::PllCfg::AHBPrescaler_t::e_AHBPrescaler_None,
    .m_apb1Prescaler    = stm32::PllCfg::APBPrescaler_t::e_APBPrescaler_Div2,
    .m_apb2Prescaler    = stm32::PllCfg::APBPrescaler_t::e_APBPrescaler_None
};

static stm32::Scb                       scb(SCB);
static stm32::Nvic                      nvic(NVIC, scb);

static stm32::Pwr                       pwr(PWR);
static stm32::Flash                     flash(FLASH);
static stm32::Rcc                       rcc(RCC, pllCfg, flash, pwr);

/*******************************************************************************
 * GPIO Engine Handlers 
 ******************************************************************************/
static stm32::Gpio::A                   gpio_A(rcc);
static gpio::GpioEngine                 gpio_engine_A(&gpio_A);

static stm32::Gpio::B                   gpio_B(rcc);
static gpio::GpioEngine                 gpio_engine_B(&gpio_B);

/*******************************************************************************
 * LEDs
 ******************************************************************************/
static gpio::AlternateFnPin             g_mco1(gpio_engine_A, 8);
static gpio::DigitalOutPin              g_led_green(gpio_engine_A, 5);

/*******************************************************************************
 * UART
 ******************************************************************************/
static gpio::AlternateFnPin             uart_tx(gpio_engine_A, 2);
static gpio::AlternateFnPin             uart_rx(gpio_engine_A, 3);
static stm32::Uart::Usart2<gpio::AlternateFnPin>    uart_access(rcc, uart_rx, uart_tx);
uart::UartDevice                        g_uart(&uart_access);

/*******************************************************************************
 * DMA Engines
 ******************************************************************************/
static stm32::Dma::Engine1  dma_1(rcc);
static stm32::Dma::Engine2  dma_2(rcc);

/*******************************************************************************
 * DMA Streams and Channels
 ******************************************************************************/
static stm32::dma::StreamT<decltype(nvic), decltype(dma_2), 3>                  ws2812b_spiTxDmaStream(nvic, dma_2);
static stm32::dma::ChannelT<decltype(ws2812b_spiTxDmaStream)>   ws2812b_spiTxDmaChannel(ws2812b_spiTxDmaStream, 3);

static stm32::dma::StreamT<decltype(nvic), decltype(dma_2), 2>                  ws2812b_spiRxDmaStream(nvic, dma_2);
static stm32::dma::ChannelT<decltype(ws2812b_spiRxDmaStream)>   ws2812b_spiRxDmaChannel(ws2812b_spiRxDmaStream, 3);

/*******************************************************************************
 * WS2812b Strip
 ******************************************************************************/
static gpio::AlternateFnPin spi_sclk(gpio_engine_B, 3);
static gpio::AlternateFnPin spi_nsel(gpio_engine_A, 15);
static gpio::AlternateFnPin spi_mosi(gpio_engine_B, 5);
static gpio::AlternateFnPin spi_miso(gpio_engine_B, 4);

static stm32::Spi::Spi1<
    decltype(ws2812b_spiTxDmaChannel),
    decltype(ws2812b_spiRxDmaChannel),
    gpio::AlternateFnPin
>
ws2812b_spibus( /* p_rcc = */ rcc,
                                        /* p_txDmaChannel = */ ws2812b_spiTxDmaChannel,
                                        /* p_rxDmaChannel = */ ws2812b_spiRxDmaChannel,
                                        /* p_sclk = */ spi_sclk,
                                        /* p_nsel = */ spi_nsel,
                                        /* p_mosi = */ spi_mosi,
                                        /* p_miso = */ spi_miso,
                                        /* p_prescaler = */ decltype(ws2812b_spibus)::e_SpiPrescaler2048);

static spi::DeviceT<decltype(ws2812b_spibus)>               ws2812b_spidev(&ws2812b_spibus);
static devices::Ws2812bStripT<1, decltype(ws2812b_spidev)>  ws2812bStrip(ws2812b_spidev);

/*******************************************************************************
 * Tasks
 ******************************************************************************/
static tasks::HeartbeatT<decltype(g_led_green)> heartbeat_gn("hrtbt_g", g_led_green, 3, 500);

#if 0
/* FIXME Only for testing the WS2812B Driver via SPI and DMA. */
void
toggleLed(bool p_status) {
  if (p_status) {
    ws2812bStrip.setPixel(0, FastLED::CRGB(0x0F0000));
  } else {
    ws2812bStrip.setPixel(0, FastLED::CRGB(0x000F00));
  }
  ws2812bStrip.show();
}
#endif

/*******************************************************************************
 *
 ******************************************************************************/
const uint32_t SystemCoreClock = pllCfg.getSysclkSpeedInHz();

static_assert(pllCfg.isValid() == true,                            "PLL Configuration is not valid!");
static_assert(SystemCoreClock               == 96 * 1000 * 1000,   "Expected System Clock to be at 96 MHz!");
static_assert(pllCfg.getAhbSpeedInHz()      == 96 * 1000 * 1000,   "Expected AHB to be running at 96 MHz!");
static_assert(pllCfg.getApb1SpeedInHz()     == 48 * 1000 * 1000,   "Expected APB1 to be running at 48 MHz!");
static_assert(pllCfg.getApb2SpeedInHz()     == 96 * 1000 * 1000,   "Expected APB2 to be running at 96 MHz!");

/*******************************************************************************
 *
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

int
main(void) {
    // rcc.setMCO(g_mco1, decltype(rcc)::MCO1Output_e::e_PLL, decltype(rcc)::MCOPrescaler_t::e_MCOPre_5);

    uart_access.setBaudRate(decltype(uart_access)::BaudRate_e::e_115200);

    const unsigned sysclk = pllCfg.getSysclkSpeedInHz() / 1000;
    const unsigned ahb    = pllCfg.getAhbSpeedInHz() / 1000;
    const unsigned apb1   = pllCfg.getApb1SpeedInHz() / 1000;
    const unsigned apb2   = pllCfg.getApb2SpeedInHz() / 1000;

    PrintStartupMessage(sysclk, ahb, apb1, apb2);

    if (SysTick_Config(SystemCoreClock / 1000)) {
        PHISCH_LOG("FATAL: Capture Error!\r\n");
        goto bad;
    }

    PHISCH_LOG("Starting FreeRTOS Scheduler...\r\n");
    vTaskStartScheduler();

bad:
    PHISCH_LOG("FATAL ERROR!\r\n");
    while (1) ;

    return (0);
}

/*******************************************************************************
 * Interrupt Handlers 
 ******************************************************************************/
void
SPI1_IRQHandler(void) {
    ws2812b_spibus.handleIrq();
}

void
DMA2_Stream2_IRQHandler(void) {
    ws2812b_spiRxDmaStream.handleIrq();
}

void
DMA2_Stream3_IRQHandler(void) {
    ws2812b_spiTxDmaStream.handleIrq();
}

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */
