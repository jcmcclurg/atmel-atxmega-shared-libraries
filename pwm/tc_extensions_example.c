/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA Timer/Counter extension driver example source.
 *
 *      This file contains an example application that demonstrates the driver
 *      for the Timer/Counter extension modules.
 *
 * \par Application note:
 *      AVR1311: Using the XMEGA Timer/Counter Extensions.
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 1569 $
 * $Date: 2008-04-22 13:03:43 +0200 (ti, 22 apr 2008) $  \n
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

#include "avr_compiler.h"
#include "hires_driver.h"
#include "awex_driver.h"
#include "clksys_driver.h"
#define set_pwm(x)	TCC0.CCA = (x); \
					TCC0.CCB = (x)+DEAD_TIME_CYCLES; \
					TCC0.CCC = TIMER_TOP_VALUE-(x); \
					TCC0.CCD = TIMER_TOP_VALUE-(x)-DEAD_TIME_CYCLES; \
					TCD0.CCA = (x); \
					TCD0.CCB = (x)+DEAD_TIME_CYCLES; \
					TCD0.CCC = TIMER_TOP_VALUE-(x); \
					TCD0.CCD = TIMER_TOP_VALUE-(x)-DEAD_TIME_CYCLES;
					
#define start_pwm()	TCC0.CNT   = 0; \
					TCD0.CNT   = TIMER_TOP_VALUE/2; \
					TCC0.CTRLA = TC_CLKSEL_DIV1_gc; \
					TCD0.CTRLA = TC_CLKSEL_DIV1_gc; \
/* Prototyping of functions. */
void ConfigClockSystem( void );
void ConfigTimer( TC0_t* timer, PORT_t* port, uint16_t period );

/*! Set for 250 kilohertz, assuming 4xCPU = 128 MHz
 */
#define TIMER_TOP_VALUE   512
#define TIMER_START_VALUE (TIMER_TOP_VALUE/8)

/*! Dead time length */
#define DEAD_TIME_CYCLES    16



/*! /brief Example using the Timer/Counter extension modules.
 *
 *  The example code is configured by commenting/uncommenting the following
 *  functions in main:
 *              - ConfigDTI()
 *              - HIRES_Enable()
 *              - ConfigFaultProtection()
 *
 *  Timer/Counter C0 is set up for PWM genereation in dual slope mode. The
 *  compare value is updated once every PWM cycle to produce a sine wave
 *  output. The sine wave compare values are updated in the Timer/Counter C0
 *  overflow interrupt service routine.
 *
 *  If ConfigDTI() is uncommented, the hardware dead-time insertion is enabled,
 *  with a dead-time of "DEAD_TIME_CYCLES" main system clock cycles.
 *
 *  If HIRES_Enable() is uncommented, the High-Resolution extension is enabled,
 *  increasing the resolution of the output by a factor of 4. The frequency of
 *  the output sine wave is also increased by a factor of 4.
 *
 *  If ConfigFaultProtection() is uncommented, the fault protection in the AWeX
 *  is enabled. The fault protection is triggered by a falling edge on PD0. In
 *  the case of a fault input, the fault protection module will disable output
 *  on the pins currently used by the AWeX module. In this case, the direction
 *  of the port pins are set to input. The fault protection works only for
 *  outputs that are overridden by the AWeX module. If CongigDTI() is commented,
 *  the fault protection will have no effect.
 */
int main( void )
{

	ConfigClockSystem();
	
	HIRES_Enable( &HIRESC, HIRES_HREN_BOTH_gc );
	HIRES_Enable( &HIRESD, HIRES_HREN_BOTH_gc );

	ConfigTimer(&TCC0,&PORTC,TIMER_TOP_VALUE);
	ConfigTimer(&TCD0,&PORTD,TIMER_TOP_VALUE);
	
	set_pwm(TIMER_START_VALUE);
	start_pwm();
	
	PMIC.CTRL = PMIC_HILVLEN_bm;
	sei( );

	do {
		/* Wait while interrupt executes. */
		nop();
	} while (1);
}


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

void ConfigTimer( TC0_t* timer, PORT_t* port, uint16_t period )
{
	// Enable outputs
	port->DIR |= 0x0F;
	
	// Invert the output on the LS gate signals
	port->PIN1CTRL = PORT_INVEN_bm;
	port->PIN2CTRL = PORT_INVEN_bm;
	
	// Set period, etc.
	timer->PER   = TIMER_TOP_VALUE;
	timer->CTRLB = TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm | TC_WGMODE_DSBOTH_gc;
}