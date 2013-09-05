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


/* Prototyping of functions. */
void ConfigClockSystem( void );
void ConfigDTI( uint8_t deadTime );
void ConfigFaultProtection( void );


/*! Top value for the Timer/Counter. Determines frequency and resolution of
 *  sine wave output.
 */
#define TIMER_TOP_VALUE   0xFFFF

/*! Dead time length, given in main system clock cycles. */
#define DEAD_TIME_CYCLES    4


/*! 16-bit sine table with 256 elements. */
uint16_t sineTable[256] = {
	32768, 33572, 34376, 35179, 35980, 36779, 37576, 38370,
	39161, 39948, 40730, 41508, 42280, 43047, 43807, 44561,
	45308, 46047, 46778, 47501, 48215, 48919, 49614, 50299,
	50973, 51636, 52288, 52928, 53556, 54171, 54774, 55363,
	55938, 56500, 57047, 57580, 58098, 58601, 59088, 59559,
	60014, 60452, 60874, 61279, 61667, 62037, 62390, 62725,
	63042, 63340, 63621, 63882, 64125, 64349, 64554, 64740,
	64906, 65054, 65181, 65290, 65378, 65447, 65497, 65526,
	65530, 65526, 65497, 65447, 65378, 65290, 65181, 65054,
	64906, 64740, 64554, 64349, 64125, 63882, 63621, 63340,
	63042, 62725, 62390, 62037, 61667, 61279, 60874, 60452,
	60014, 59559, 59088, 58601, 58098, 57580, 57047, 56500,
	55938, 55363, 54774, 54171, 53556, 52928, 52288, 51636,
	50973, 50299, 49614, 48919, 48215, 47501, 46778, 46047,
	45308, 44561, 43807, 43047, 42280, 41508, 40730, 39948,
	39161, 38370, 37576, 36779, 35980, 35179, 34376, 33572,
	32768, 31964, 31160, 30357, 29556, 28757, 27960, 27166,
	26375, 25588, 24806, 24028, 23256, 22489, 21729, 20975,
	20228, 19489, 18758, 18035, 17321, 16617, 15922, 15237,
	14563, 13900, 13248, 12608, 11980, 11365, 10762, 10173,
	 9598,  9036,  8489,  7956,  7438,  6935,  6448,  5977,
	 5522,  5084,  4662,  4257,  3869,  3499,  3146,  2811,
	 2494,  2196,  1915,  1654,  1411,  1187,   982,   796,
	  630,   482,   355,   246,   158,    89,    39,    10,
	    6,    10,    39,    89,   158,   246,   355,   482,
	  630,   796,   982,  1187,  1411,  1654,  1915,  2196,
	 2494,  2811,  3146,  3499,  3869,  4257,  4662,  5084,
	 5522,  5977,  6448,  6935,  7438,  7956,  8489,  9036,
	 9598, 10173, 10762, 11365, 11980, 12608, 13248, 13900,
	14563, 15237, 15922, 16617, 17321, 18035, 18758, 19489,
	20228, 20975, 21729, 22489, 23256, 24028, 24806, 25588,
	26375, 27166, 27960, 28757, 29556, 30357, 31160, 31964
};


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

	/* Comment out the following line to disable dead-time insertion. */
	ConfigDTI( DEAD_TIME_CYCLES );

	/* Comment out the following line to disable the HiRes. */
	HIRES_Enable( &HIRESC, HIRES_HREN_TC0_gc );

	/* Comment out the following line to disable fault protection. */
	ConfigFaultProtection();

	/* Enable output on PORTC. */
	PORTC.DIR = 0xFF;

	/* Configure timer. */
	TCC0.PER = TIMER_TOP_VALUE;
	TCC0.CTRLB = TC0_CCAEN_bm | TC_WGMODE_DS_T_gc;
	TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc;
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;

	/* Enable low level interrupts. */
	PMIC.CTRL = PMIC_LOLVLEN_bm;
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
	/* Start internal 32MHz RC oscillator. */
	OSC.CTRL = OSC_RC32MEN_bm;

	do {
		/* Wait while oscillator stabilizes. */
	} while ( ( OSC.STATUS & OSC_RC32MRDY_bm ) == 0 );

	/* Enable prescaler B and C. */
	CCP = CCP_IOREG_gc;
	CLK.PSCTRL = CLK_PSBCDIV_2_2_gc;

	/* Select 32 MHz as master clock. */
	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;

}


/*! \brief This function configures the Dead-time insertion extension.
 *
 *  \param deadTime  The dead time to insert between the pulses.
 */
void ConfigDTI( uint8_t deadTime )
{
	/* Configure dead time insertion. */
	AWEX_EnableDeadTimeInsertion( &AWEXC, AWEX_DTICCAEN_bm );
	AWEX_SetOutputOverrideValue( AWEXC, 0x03 );
	AWEX_SetDeadTimesSymmetricalUnbuffered( AWEXC, deadTime );
}

/*! \brief This function configures fault protection, using the falling edge of
*          PD0 as fault input through event channel 0.
*/
void ConfigFaultProtection( void )
{
	/* Configure PD0 as input, trigger on falling edge. */
	PORTD.DIRCLR = 0x01;
	PORTD.PIN0CTRL = PORT_ISC_FALLING_gc;

	/* Select PD0 as input for event channel 0 multiplexer. */
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTD_PIN0_gc;

	/* Enable fault detection for AWEX C, using event channel 0. */
	AWEX_ConfigureFaultDetection( &AWEXC, AWEX_FDACT_CLEARDIR_gc, EVSYS_CHMUX0_bm );
}


/*! \brief Timer/Counter interrupt service routine
*
*  This interrupt service routine is responsible for updating the Timer/Counter
*  compare value once every PWM cycle to produce a sine wave. The sine values
*  are scaled to the current resolution of the Timer/Counter. A sync-signal
*  is generated on PC2 once every sine wave period.
*/
ISR(TCC0_OVF_vect)
{
	static uint8_t index = 0;

	/* Write the next ouput compare A value. */
	TCC0.CCABUF = sineTable[index];

	/* Increment table index. */
	index++;

}


