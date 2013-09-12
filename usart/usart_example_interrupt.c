/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA USART interrupt driven driver example source.
 *
 *      This file contains an example application that demonstrates the
 *      interrupt driven USART driver. The code example sends three bytes, waits
 *      for three bytes to be received and tests if the received data equals the
 *      sent data.
 *
 * \par Application note:
 *      AVR1307: Using the XMEGA USART
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 1694 $
 * $Date: 2008-07-29 14:21:58 +0200 (ti, 29 jul 2008) $  \n
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include "usart_driver.h"
#include "avr_compiler.h"
#include "clksys_driver.h"

/*! Number of bytes to send in test example. */
#define NUM_BYTES  3
/*! Define that selects the Usart used in example. */
#define USART USARTE0

/*! USART data struct used in example. */
USART_data_t USART_data;
/*! Test data to send. */
uint8_t sendArray[NUM_BYTES] = {'H', 'A', 'L'};
/*! Array to put received data in. */
uint8_t receiveArray[NUM_BYTES];
/*! Success variable, used to test driver. */
bool success;

/*! \brief This function enables the internal 32MHz oscillator and the
 *         prescalers needed by the HiRes extension.
 *
 *  \note  The optimization of the compiler must be set above low to ensure
 *         that the setting of the CLK register is set within 4 clock cylcles
 *         after the CCP register is set.
 */
void ConfigClockSystem( void )
{
	/*  Enable internal 32 MHz ring oscillator and wait until it's
	 *  stable.
	 */
	CLKSYS_Enable( OSC_RC32MEN_bm );
	do {} while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );
	
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );
	
	/*  Configure PLL with the 32 MHz ring oscillator/4 as source and
	 *  multiply by 16 to get 128 MHz PLL clock and enable it. Wait
	 *  for it to be stable and set prescalers B and C to divide by four
	 *  to set the CPU clock to 32 MHz.
	 */
	CLKSYS_PLL_Config( OSC_PLLSRC_RC32M_gc, 16 );
	CLKSYS_Enable( OSC_PLLEN_bm );
	CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_2_2_gc );
	do {} while ( CLKSYS_IsReady( OSC_PLLRDY_bm ) == 0 );
		
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_PLL_gc );
}

void ConfigUart(void){
	/* This PORT setting is only valid to USARTE0 if other USARTs is used a
	 * different PORT and/or pins are used. */
  	/* PC3 (TXD0) as output. */
	PORTE.DIRSET   = PIN3_bm;
	/* PC2 (RXD0) as input. */
	PORTE.DIRCLR   = PIN2_bm;

	/* Use USARTE0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

	/* USARTE0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 115200 bps:
	 * Use the default I/O clock frequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART, 2094 , -7);

	/* Enable both RX and TX. */
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
}

/*! \brief Example application.
 *
 *  Example application. This example configures USARTE0 for with the parameters:
 *      - 8 bit character size
 *      - No parity
 *      - 1 stop bit
 *      - 9600 Baud
 *
 *  This function then sends three bytes and tests if the received data is
 *  equal to the sent data. The code can be tested by connecting PC3 to PC2. If
 *  the variable 'success' is true at the end of the function, the three bytes
 *  have been successfully sent and received.
*/
int main(void)
{
	/* counter variable. */
	uint8_t i;
	
	ConfigClockSystem();

	ConfigUart();

	/* Enable global interrupts. */
	sei();

	/* Fetch received data as it is received. */
	i = 0;
	while (i != '@') {
		if (USART_RXBufferData_Available(&USART_data)) {
			i = USART_RXBuffer_GetByte(&USART_data);
			USART_TXBuffer_PutByte(&USART_data, i);
		}
	}
	
	/* Disable both RX and TX. */
	USART_Rx_Disable(&USART);
	USART_Tx_Disable(&USART);
	
	return 0;
}

/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTE0_RXC_vect)
{
	USART_RXComplete(&USART_data);
}

/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTE0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data);
}