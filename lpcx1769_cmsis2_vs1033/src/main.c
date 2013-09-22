// CMSIS headers required for setting up SysTick Timer
#include "LPC17xx.h"

#include <cr_section_macros.h>
#include <NXP/crp.h>

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

#include "leds.h"
#include "uart.h"
#include "logger.h"
#include "s0_input.h"
#include "lpc17xx_spi.h"


// VS1002 commands
#define VS1002_READ     0x03
#define VS1002_WRITE    0x02
#define DREQ_PIN 		(1 << 22)
#define RESET_PIN  8
#define XCS_PIN   10
#define XDCS_PIN  11
#define HARD_RESET 2
#define SOFT_RESET 1

#define WAIT_BUSY while(!(LPC_GPIO0->FIOPIN & DREQ_PIN))
#define XCS_HIGH  LPC_GPIO2->FIOSET = (1 << XCS_PIN)
#define XCS_LOW   LPC_GPIO2->FIOCLR = (1 << XCS_PIN)
#define XDCS_HIGH  LPC_GPIO2->FIOSET = (1 << XDCS_PIN)
#define XDCS_LOW   LPC_GPIO2->FIOCLR = (1 << XDCS_PIN)
#define RESET_HIGH  LPC_GPIO2->FIOSET = (1 << RESET_PIN)
#define RESET_LOW   LPC_GPIO2->FIOCLR = (1 << RESET_PIN)




volatile uint32_t msTicks; // counter for 1ms SysTicks
extern volatile unsigned int eint3_count;
extern volatile uint32_t UART0Count, UART1Count, UART2Count, UART3Count, UART0TxEmpty, UART1TxEmpty, UART2TxEmpty, UART3TxEmpty;
extern volatile uint8_t UART0Buffer[BUFSIZE], UART1Buffer[BUFSIZE], UART2Buffer[BUFSIZE], UART3Buffer[BUFSIZE];



// ****************
//  SysTick_Handler - just increment SysTick counter
void SysTick_Handler(void) {
  msTicks++;
}

// ****************
// systick_delay - creates a delay of the appropriate number of Systicks (happens every 10 ms)
__INLINE static void systick_delay (uint32_t delayTicks) {
  uint32_t currentTicks;

  currentTicks = msTicks;	// read current tick counter
  // Now loop until required number of ticks passes.
  while ((msTicks - currentTicks) < delayTicks);
}

// ****************


void vs_SCI_write(uint8_t address, uint16_t data) {

    WAIT_BUSY;

    XCS_LOW;

    SPI_SendByte(VS1002_WRITE);
	SPI_SendByte(address);

	SPI_SendByte(data >> 8);
	SPI_SendByte(data);

    XCS_HIGH;

    systick_delay(2);

    WAIT_BUSY;
}


uint16_t vs_SCI_read(uint8_t address)
{
    WAIT_BUSY;

    XCS_LOW;

    uint16_t retData = 0;

    SPI_SendByte(VS1002_READ);
    SPI_SendByte(address);

    retData = SPI_SendByte(0) << 8;
    retData |= SPI_SendByte(0);

    XCS_HIGH;

    systick_delay(2);

    WAIT_BUSY;

    return retData;
}


void vs_reset(uint8_t reset_type) {
    if (reset_type == HARD_RESET) {
    	RESET_LOW;

        systick_delay(2);
        RESET_HIGH;
        systick_delay(10);

        WAIT_BUSY;
    }
    else  {
    	vs_SCI_write(0x00, 0x04);
    	systick_delay(50);
    	WAIT_BUSY;
    }
}

void vs_start_sinetest(uint8_t pitch)
{
    XDCS_LOW;
    // 0x53, 0xEF, 0x6E, 126, 0, 0, 0, 0
    SPI_SendByte(0x53);
    SPI_SendByte(0xEF);
    SPI_SendByte(0x6E);
    SPI_SendByte(pitch);
    SPI_SendByte(0);
    SPI_SendByte(0);
    SPI_SendByte(0);
    SPI_SendByte(0);
    XDCS_HIGH;
}

void vs_stop_sinetest(void)
{
    XDCS_LOW;
    SPI_SendByte(0x45);
    SPI_SendByte(0x78);
    SPI_SendByte(0x69);
    SPI_SendByte(0x74);
    SPI_SendByte(0);
    SPI_SendByte(0);
    SPI_SendByte(0);
    SPI_SendByte(0);
    XDCS_HIGH;
}

void vs_set_volume(uint16_t volume)
{
    vs_SCI_write(0x0b, volume);
}







int main(void) {
	
	// Setup SysTick Timer to interrupt at 10 msec intervals
	if (SysTick_Config(SystemCoreClock / 100)) {
	    while (1);  // Capture error
	}

	led_init();	// Setup GPIO for LED2
	led2_on();	// Turn LED2 on
	//led_on(0);
	//led_on(1);

	systick_delay(100);
	led2_off();
	systick_delay(100);
	led2_off();


	UARTInit(0, 115200); // baud rate setting
	UARTSendCRLF(0);
	UARTSendCRLF(0);
	UARTSendStringln(0, "UART online ...");

	logger_logStringln("logger online ...");
	led_off(7);

	/*
	 * VS1033
	 */


	SPI_Init();
    SPI_ConfigClockRate (SPI_CLOCKRATE_LOW);
    SPI_CS_Low();
    XCS_HIGH;
    XDCS_HIGH;

    // set output
    LPC_GPIO2->FIODIR |= (1 << RESET_PIN) | (1 << XCS_PIN) | (1 << XDCS_PIN);

    // SEND VS RESET
    UARTSendStringln(0, "hard reset\n");
    vs_reset(HARD_RESET);
    UARTSendStringln(0, "done\n");

    UARTSendStringln(0, "read 0x00: ");
    UARTSendNumberln(0, vs_SCI_read(0x00));
    systick_delay(10);

    UARTSendStringln(0, "write 0x00: ");
    //vs_SCI_write(0x00, 0x0800); // SM_SDINEW=1
    vs_SCI_write(0x00, 0x0820); // SM_SDINEW=1, SM_TEST=1
    //vs_SCI_write(0x00, 0x0c00); // SM_SDINEW=1, SM_SDISHARE=1
    //vs_SCI_write(0x00, 0x0c20); // SM_SDINEW=1, SM_SDISHARE=1, SM_TEST=1
    systick_delay(10);

    UARTSendStringln(0, "read 0x00: ");
    UARTSendNumberln(0, vs_SCI_read(0x00));
    systick_delay(10);


    UARTSendStringln(0, "write 0x03: ");
    vs_SCI_write(0x03, 0x9000);
    //vs_SCI_write(0x03, 0x9800);
    systick_delay(10);

    UARTSendStringln(0, "read 0x03: ");
    UARTSendNumberln(0, vs_SCI_read(0x03));
    systick_delay(10);

    vs_start_sinetest(100);

    systick_delay(100);
    vs_set_volume(0x5050);
    systick_delay(100);
    vs_set_volume(0x1010);
    systick_delay(100);
    vs_set_volume(0x0808);
    systick_delay(100);
    vs_set_volume(0x0707);
    systick_delay(100);
    vs_set_volume(0x0606);
    systick_delay(100);
    vs_set_volume(0x0505);
    systick_delay(100);
    vs_set_volume(0x0404);
    systick_delay(100);
    vs_set_volume(0x0303);
    systick_delay(100);
    vs_set_volume(0x0202);
    systick_delay(100);
    vs_set_volume(0x0101);
    systick_delay(100);
    vs_set_volume(0x0000);

	while(1) {

		/* process logger */
		if (logger_dataAvailable() && UARTTXReady(0)) {
			uint8_t data = logger_read();
			UARTSendByte(0,data);
		}

		process_leds(msTicks);

		process_s0(msTicks);

		uint32_t triggerValue = s0_triggered(0);
		if (triggerValue) {
			logger_logString("s0_0:");
			logger_logNumberln(triggerValue);
			led_signal(1, 30, msTicks);
		}

		triggerValue = s0_triggered(1);
		if (triggerValue) {
			logger_logString("s0_1:");
			logger_logNumberln(triggerValue);
			led_signal(2, 30, msTicks);
		}
	}
	return 0 ;
}
