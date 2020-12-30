/*
 * Init.c
 *
 * Created: 2020-12-28 12:59:41
 *  Author: Branden
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "touch_api.h"
#include "init.h"


void init_start_mode_pins()
{
	//this pin will be used to detect the startup mode
	PORTD.DIRSET = PIN7_bm;
	PORTD.PIN6CTRL = PORT_OPC_PULLDOWN_gc;
	
	//This pin is for a mode indicator light
	PORTD.DIRCLR = PIN6_bm;

}

void enable_interrupts()
{
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm 	;	  //  enable interrupts
}

void init_clock()
{
	
	OSC_CTRL |= OSC_RC32MEN_bm; //Setup 32Mhz internal
	
	while(!(OSC_STATUS & OSC_RC32MRDY_bm));
	
	CCP = CCP_IOREG_gc; //Trigger protection mechanism
	CLK_CTRL = CLK_SCLKSEL_RC32M_gc; //Enable internal  32Mhz internal
	CLK.RTCCTRL = CLK_RTCSRC_ULP_gc | CLK_RTCEN_bm; //Set the RTC clock source to the 1kHz ULP and enable it.
}




/*Section for normal run mode init*/

void init_pins_normal()
{
	PORTA.DIRSET = PIN0_bm | PIN2_bm;

	PORTD.DIRSET = PIN1_bm | PIN3_bm | PIN4_bm | PIN7_bm;
	PORTD.DIRCLR = PIN2_bm | PIN6_bm;
	
	
	// Setup port pins for TxD, XCK and LUT0OUT
	PORTC.PIN0CTRL = PORT_OPC_TOTEM_gc;                         // LUT0OUT (data to WS2812)
	PORTC.PIN1CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_RISING_gc;    // XCK
	PORTC.PIN3CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_LEVEL_gc;     // TxD
	PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN3_bm;
	


}


void init_usart_normal()
{

	
	//Mem USART in SPI
	USARTD0.CTRLA = USART_TXCINTLVL_OFF_gc; //No interrupts
	USARTD0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;// | USART_CLK2X_bm;
	USARTD0.CTRLC = USART_CMODE_MSPI_gc;
	USARTD0.CTRLD = 0; //No decoding or encoding
	USARTD0_BAUDCTRLA = 1;
	USARTD0.BAUDCTRLB = 0;
	

	
	//Lights USART in SPI
	// Setup USART in master SPI mode 1, MSB first
	USARTC0.BAUDCTRLA = 19;                                     // 800.000 baud (1250 ns @ 32 MHz)
	USARTC0.BAUDCTRLB = 0;
	USARTC0.CTRLA = USART_RXCINTLVL_OFF_gc | USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc;
	USARTC0.CTRLC = USART_CMODE_MSPI_gc | (1 << 1);             // UDORD=0 UCPHA=1
	USARTC0.CTRLD = USART_DECTYPE_DATA_gc | USART_LUTACT_OFF_gc | USART_PECACT_OFF_gc;
	USARTC0.CTRLB = USART_TXEN_bm;
}



void init_edma_normal()
{
	//RX
	EDMA.CH0.CTRLA = EDMA_CH_SINGLE_bm;//no repeat, single shot, burst len = 1
	EDMA.CH0.CTRLB = EDMA_CH_TRNINTLVL_HI_gc; //ERR level = 0, TRN int level hi
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; //
	EDMA.CH0.TRIGSRC = EDMA_CH_TRIGSRC_USARTD0_RXC_gc;

	//TX
	EDMA.CH1.CTRLA = EDMA_CH_SINGLE_bm;//no repeat, single shot, burst len = 1
	EDMA.CH1.CTRLB = 0; //ERR and TRN int level = 0
	EDMA.CH1.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_FIXED_gc; //
	EDMA.CH1.TRIGSRC = EDMA_CH_TRIGSRC_USARTD0_DRE_gc;
	
	//Audio
	EDMA.CH2.ADDRCTRL =  EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc;	 // Reload after transaction increment source address
	EDMA.CH2.TRIGSRC = EDMA_CH_TRIGSRC_DACA_CH1_gc;	  // DACA Ch1 is trigger source
	EDMA.CH2.CTRLA = EDMA_CH_SINGLE_bm | EDMA_CH_BURSTLEN_bm;  //one burst per trigger 2 bytes per burst
	EDMA.CH2.CTRLB = EDMA_CH_TRNIF_bm |   EDMA_CH_TRNINTLVL_MED_gc; //  Clear flag
	// For XMegaE5, this ISR is necessary to re-enable channel after transaction
	
	
	//Lights
	EDMA.CH3.CTRLB = 0; //ERR and TRN int level = 0
	EDMA.CH3.ADDRCTRL = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DIR_INC_gc;
	EDMA.CH3.DESTADDRCTRL = EDMA_CH_RELOAD_NONE_gc | EDMA_CH_DESTDIR_FIXED_gc;
	EDMA.CH3.DESTADDR = (uint16_t)&USARTC0.DATA;
	EDMA.CH3.TRIGSRC = EDMA_CH_TRIGSRC_USARTC0_DRE_gc;
	
	EDMA.CTRL = EDMA_ENABLE_bm | EDMA_PRIMODE_RR123_gc; //perif mode, no double buff,  Ch0 > round robin


}



void init_xcl(void)
{

	// Setup XCL BTC0 timer to 1shot pwm generation
	XCL.CTRLE = XCL_CMDSEL_NONE_gc | XCL_TCSEL_BTC0_gc | XCL_CLKSEL_DIV1_gc;
	XCL.CTRLF = XCL_CMDEN_DISABLE_gc | XCL_MODE_1SHOT_gc;                    // 0x03 : One-shot PWM
	XCL.CTRLG = XCL_EVACTEN_bm | XCL_EVACT0_RESTART_gc | XCL_EVSRC_EVCH6_gc; // 0x03<<3 : EVACT0=RESTART
	XCL.PERCAPTL = 22;                                          // Output high time if data is 1 (from RESTART to falling edge of one-shot)
	XCL.CMPL = 13;                                              // Output high time if data is 0 (from RESTART to rising edge of one-shot)

	// Setup XCL LUT
	XCL.CTRLA = XCL_LUT0OUTEN0_bm | XCL_PORTSEL_PC_gc | XCL_LUTCONF_MUX_gc;  // Setup glue logic for MUX
	XCL.CTRLB = XCL_IN3SEL_XCL_gc | XCL_IN2SEL_XCL_gc | XCL_IN1SEL_EVSYS_gc | XCL_IN0SEL_EVSYS_gc; //0x50;// IN3SEL=XCL, IN2SEL=XCL, IN1SEL=EVSYS, IN0SEL=EVSYS
	XCL.CTRLC = XCL_EVASYSEL0_bm;      // Async inputs, no delay
	XCL.CTRLD = 0xA0;                                           // LUT truthtables (only LUT1 is used)

}

void init_audio_timer()
{
	TCC4.CTRLA =
	TC_CLKSEL_DIV1_gc;		//  prescaler clk/1
}

void init_light_map_timer()
{
	TCC5_CTRLA = 0; //Prescaler off
	TCC5_INTCTRLA = TC_OVFINTLVL_LO_gc;

	TCC5.PER = TIMER_PER_MS;
}

void init_qTouch_timer()
{
	RTC.PER = TICKS_PER_MS * qt_measurement_period_msec;
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;
	RTC.INTCTRL = RTC_OVFINTLVL_LO_gc;
	
}

void init_event_channels()
{
	
	//Audio
	EVSYS.CH1MUX = EVSYS_CHMUX_TCC4_OVF_gc;        // Event ch0 = tcc4 overflow
	
	//Lights
	// Setup Event channel 0 to TxD (async)
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN3_gc;
	EVSYS.CH0CTRL = EVSYS_DIGFILT_1SAMPLE_gc;
	// Setup Event channel 6 to XCK rising edge
	EVSYS.CH6MUX = EVSYS_CHMUX_PORTC_PIN1_gc;
	EVSYS.CH6CTRL = EVSYS_DIGFILT_1SAMPLE_gc;
}

void init_DAC()
{
	DACA.CTRLB =
	DAC_CHSEL_SINGLE1_gc |          // DAC ch1 is active
	DAC_CH1TRIG_bm;			// DAC ch1 auto triggered by an event (CH1)
	DACA.CTRLC =
	DAC_REFSEL_AVCC_gc;// | DAC_LEFTADJ_bm ;             // Use AVCC (3.3v), non-left adj
	DACA.EVCTRL =
	DAC_EVSEL_1_gc;                 // Event Ch1 triggers the DAC conversion
	DACA.CTRLA = DAC_CH1EN_bm | DAC_ENABLE_bm;  // enable DACA channel 1
}

void qt_set_parameters( void )
{
	/*  This will be modified by the user to different values   */
	qt_config_data.qt_di              = DEF_QT_DI;
	qt_config_data.qt_neg_drift_rate  = DEF_QT_NEG_DRIFT_RATE;
	qt_config_data.qt_pos_drift_rate  = DEF_QT_POS_DRIFT_RATE;
	qt_config_data.qt_max_on_duration = DEF_QT_MAX_ON_DURATION;
	qt_config_data.qt_drift_hold_time = DEF_QT_DRIFT_HOLD_TIME;
	qt_config_data.qt_recal_threshold = DEF_QT_RECAL_THRESHOLD;
	qt_config_data.qt_pos_recal_delay = DEF_QT_POS_RECAL_DELAY;
}

void config_keys(void)
{
	/*  enable sensors 0..3: keys on channels 0..3  */
	qt_enable_key( CHANNEL_0, NO_AKS_GROUP, 10u, HYST_6_25 );
	qt_enable_key( CHANNEL_1, NO_AKS_GROUP, 10u, HYST_6_25 );
	qt_enable_key( CHANNEL_2, NO_AKS_GROUP, 10u, HYST_6_25 );
	//qt_enable_key( CHANNEL_3, AKS_GROUP_1, 10u, HYST_6_25 );
}



/*Start of mem access init*/

void init_input_timeout_timer()
{
	RTC.PER = INPUT_TIMEOUT_PER_MS;
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;

}

void init_pins_mem_access()
{
	PORTA.DIRSET = PIN0_bm | PIN2_bm;

	PORTD.DIRSET = PIN1_bm | PIN3_bm | PIN4_bm | PIN7_bm;
	PORTD.DIRCLR = PIN2_bm | PIN6_bm;
	
	PORTC.DIRSET = PIN3_bm;
	PORTC.DIRCLR = PIN2_bm;
	
}

void init_edma_mem_access()
{
	//RX
	EDMA.CH0.CTRLA = EDMA_CH_SINGLE_bm;//no repeat, single shot, burst len = 1
	EDMA.CH0.CTRLB = EDMA_CH_TRNINTLVL_HI_gc; //ERR level = 0, TRN int level hi
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; //
	EDMA.CH0.TRIGSRC = EDMA_CH_TRIGSRC_USARTD0_RXC_gc;

	//TX
	EDMA.CH1.CTRLA = EDMA_CH_SINGLE_bm;//no repeat, single shot, burst len = 1
	EDMA.CH1.CTRLB =  0; //EDMA_CH_TRNINTLVL_HI_gc; //ERR level = 0, TRN int level hi
	EDMA.CH1.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; //
	EDMA.CH1.TRIGSRC = EDMA_CH_TRIGSRC_USARTD0_DRE_gc;
	
	EDMA.CTRL = EDMA_ENABLE_bm | EDMA_PRIMODE_RR123_gc; //perif mode, no double buff,  Ch0 > round robin
}

void init_usart_mem_access()
{

	
	//Mem USART in SPI
	USARTD0.CTRLA = USART_TXCINTLVL_OFF_gc; //No interrupts
	USARTD0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;// | USART_CLK2X_bm;
	USARTD0.CTRLC = USART_CMODE_MSPI_gc;
	USARTD0.CTRLD = 0; //No decoding or encoding
	USARTD0_BAUDCTRLA = 1;
	USARTD0.BAUDCTRLB = 0;
	

	
	//interface. BAUD=921600
	USARTC0.BAUDCTRLA = 75;// 131;
	USARTC0.BAUDCTRLB = 0xA0;//-6 0xD0; //-3
	USARTC0.CTRLA = USART_TXCINTLVL_OFF_gc; //No interrupts
	USARTC0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	USARTC0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc; //Mode:Async,8,None,1
	USARTC0.CTRLD = 0; //No decoding or encoding

}



void init_main_config()
{
	init_pins_normal();
	enable_interrupts();
	
	#ifdef QTOUCH_STUDIO_MASKS
	SNS_array[0][0]= 0x51;
	SNS_array[0][1]= 0x0;
	SNS_array[1][0]= 0x0;
	SNS_array[1][1]= 0x0;

	SNSK_array[0][0]= 0xa2;
	SNSK_array[0][1]= 0x0;
	SNSK_array[1][0]= 0x0;
	SNSK_array[1][1]= 0x0;
	#endif	
	
	cli();
	
	init_clock();
	init_usart_normal();
	init_edma_normal();
	
	init_event_channels();
	init_audio_timer();
	init_qTouch_timer();
	init_DAC();
	
	init_light_map_timer();
	init_xcl();
	
	//Configure the Sensors as keys
	config_keys();
		
	//initialize touch sensing
	qt_init_sensing();
		
	//Set the parameters like recalibration threshold, Max_On_Duration etc in this function by the user */
	qt_set_parameters( );
	
	sei();
	
	//Set CS high
	PORTD.OUTSET = PIN4_bm;
}

void init_mem_access_config()
{
	enable_interrupts();
	init_pins_mem_access();
	
	cli();
	
	init_clock();
	init_usart_mem_access();
	init_edma_mem_access();
	init_input_timeout_timer();
	
	sei();
	
	//Set CS high
	PORTD.OUTSET = PIN4_bm;
}