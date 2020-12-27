/*
 * xmega35e_spi_wav_edma.c
 *
 * Created: 2020-11-11 16:01:40
 * Author : Branden
 */ 
#define  F_CPU 32000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "touch_api.h"

#define MEM_READ_DATA 0x03
#define FILE_COUNT_BUFF_LEN 2 
#define FILE_ENTRY_BUFF_LEN 8 
#define WAV_FILE_BUFF_LEN 128  
#define LIGHT_FILE_BUFF_LEN 100
#define WAV_METADATA_LEN 9
#define LIGHT_METADATA_LEN 4
#define MAX_LIGHT_COUNT 84
#define TIMER_PER_MS 125



/*----------------------------------------------------------------------------
                                static variables
----------------------------------------------------------------------------*/

/* flag set by timer ISR when it's time to measure touch */
volatile uint8_t time_to_measure_touch = 0;

/* current time, set by timer ISR */
volatile uint16_t current_time_ms_touch = 0;

/* Timer period in msec. */
uint16_t qt_measurement_period_msec = 25u;

/*----------------------------------------------------------------------------
                                extern variables
----------------------------------------------------------------------------*/
/* This configuration data structure parameters if needs to be changed will be
   changed in the qt_set_parameters function */
extern qt_touch_lib_config_data_t qt_config_data;
/* touch output - measurement data */
extern qt_touch_lib_measure_data_t qt_measure_data;
/* Get sensor delta values */
extern int16_t qt_get_sensor_delta( uint8_t sensor);
#ifdef QTOUCH_STUDIO_MASKS
extern TOUCH_DATA_T SNS_array[2][2];
extern TOUCH_DATA_T SNSK_array[2][2];
#endif
/* Output can be observed in the watch window using this pointer */
qt_touch_lib_measure_data_t *pqt_measure_data = &qt_measure_data;

volatile uint8_t TX_DATA = 0x55;
volatile uint8_t _wavBuff0[WAV_FILE_BUFF_LEN + 1]; //first byte is length, so +1 for the size
volatile uint8_t _wavBuff1[WAV_FILE_BUFF_LEN + 1];
volatile uint8_t _lightBuff[LIGHT_FILE_BUFF_LEN];
volatile uint16_t _entryCount = 0;
volatile uint16_t _holdTimerCounter = 0;
volatile uint16_t _entryIndex = 0;
volatile uint32_t _totalSampleBytes = 0;
volatile uint8_t _readLightMem = 0;
volatile uint16_t _clkPer = 0;
volatile uint8_t _usartCnt = 0;
volatile uint8_t _setCS = 0;
uint8_t * _inputBuff; //Pointer to the buffer we are currently filling
uint8_t * _outputBuff; //Pointer to the buffer we are currently outputting as audio

volatile uint8_t _readingMem = 0;

struct WAV_Metadata
{
	uint32_t sampleRate;
	uint8_t numChannels;
	uint32_t sampleCount;
	
	
} _wavMetadata;

struct Light_Metadata
{
	uint16_t mapCount;
	uint16_t lightCount;	
} _lightMetadata;

struct Data_Addresses
{
	uint32_t wavFileAddress;
	uint32_t lightFileAddress;	
	
} _addresses;

struct Light_Map
{
	uint16_t holdTime;
	uint8_t lights[MAX_LIGHT_COUNT * 3]; 	
	
} _lightMap;



void init_pins()
{
	PORTA.DIRSET = PIN0_bm | PIN2_bm;

	PORTD.DIRSET = PIN1_bm | PIN3_bm | PIN4_bm;
	PORTD.DIRCLR = PIN2_bm;
	
	
	// Setup port pins for TxD, XCK and LUT0OUT
	PORTC.PIN0CTRL = PORT_OPC_TOTEM_gc;                         // LUT0OUT (data to WS2812)
	PORTC.PIN1CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_RISING_gc;    // XCK
	PORTC.PIN3CTRL = PORT_OPC_TOTEM_gc | PORT_ISC_LEVEL_gc;     // TxD
	PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN3_bm;

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

void init_usart()
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

void init_edma()
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



void read_mem(uint32_t starting_address, uint8_t *buffer, uint8_t len)
{
	volatile uint8_t read[4];
	int trash;
	_readingMem = 1;
	_setCS = 0;
	PORTD.OUTCLR = PIN4_bm; //Set Pin 4 (CS) to low
		
	read[0] = MEM_READ_DATA;
	read[1] = (starting_address >> 16) & 0xFF;
	read[2] = (starting_address >> 8) & 0xFF;
	read[3] = (starting_address >> 0) & 0xFF;

	EDMA.CH0.TRFCNT = 4;
	EDMA.CH0.ADDR = (uint16_t)buffer;
	EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm;
	
	
	EDMA.CH1.TRFCNT = 4;
	EDMA.CH1.ADDR = read;
	EDMA.CH1.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; //
	EDMA.CH1.CTRLA |= EDMA_CH_ENABLE_bm | EDMA_CH_TRFREQ_bm;
	
	while(_readingMem);
	_readingMem = 1;	
	_setCS = 1;
	
	EDMA.CH0.TRFCNT = len;
	EDMA.CH0.ADDR = (uint16_t)buffer;
	EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm;
	
	EDMA.CH1.TRFCNT = len;
	EDMA.CH1.ADDR = (uint16_t)&TX_DATA;
	EDMA.CH1.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_FIXED_gc; //
	EDMA.CH1.CTRLA |= EDMA_CH_ENABLE_bm | EDMA_CH_TRFREQ_bm;
	
	//Read is finished by interrupt handler.
	
}



void write_lights()
{
	EDMA.CH3.ADDR = (uint16_t)_lightMap.lights;
	EDMA.CH3.TRFCNT = _lightMetadata.lightCount * 3;
	EDMA.CH3.CTRLA = EDMA_CH_ENABLE_bm | EDMA_CH_SINGLE_bm; // Start DMA transfer to LEDs	
}

void load_entry_count()
{
	read_mem(0, &_entryCount, 2);
	while(_readingMem);
}

//entryNum is 0-based index
void load_starting_addresses(uint16_t entryNum)
{

	uint32_t filesAddressesAndSizesAddress = (entryNum * 8) + 2; //(4 bytes for each address) * 2 addresses + 2 to skip the count
	read_mem(filesAddressesAndSizesAddress, &_addresses, FILE_ENTRY_BUFF_LEN);
	while(_readingMem);
	

}

void load_wav_metadata()
{
	read_mem(_addresses.wavFileAddress, &_wavMetadata, WAV_METADATA_LEN );
	while(_readingMem);
	
	_addresses.wavFileAddress += WAV_METADATA_LEN; //Increment the current read address for the wav data
	_totalSampleBytes = _wavMetadata.sampleCount * 2; // * 2 because each sample is 2 bytes
}

void load_light_sequenc_metadata()
{

	read_mem(_addresses.lightFileAddress, &_lightMetadata, LIGHT_METADATA_LEN );
	
	while(_readingMem);
	
	_addresses.lightFileAddress += LIGHT_METADATA_LEN; //Increment the current read address for the light sequence data
}

void load_next_light_map()
{
	//2 bytes for the hold time + 3 * the light count (3 bytes per light)
	uint8_t len = 2 + (_lightMetadata.lightCount * 3);
	//Clear the flag and stop the counter
	_holdTimerCounter = 0;


	TCC5.CTRLA = 0; //prescaler off
	//perform the mem read for the light map data
	
	//Make sure nothing else is reading the mem.
	
	read_mem(_addresses.lightFileAddress, &_lightMap, len);
	while(_readingMem);	
	
	//update the _addresses.lightFileAddress and decrement the _lightMetadata.mapCount
	_addresses.lightFileAddress += len;
	_lightMetadata.mapCount--;
		
	//start the light hold timer
	//TIMER_PER_MS is based on 256 prediv of 32MHz clock on the timer
	//Should result in a 1ms timer interval
	TCC5_CTRLA = TC_CLKSEL_DIV256_gc;

	//Start the light output process. 
	write_lights();
}

void calc_clock_per()
{
	//We are going to assume that, if this is a 2-channel wav, we can play one sample of one channel and then
	//one sample of the other channel, instead of mixing by sum.
	//rate indicates how many times per second we need to output a sample
	//We will double the rate for 2 channel audio
	uint32_t rate = _wavMetadata.sampleRate * _wavMetadata.numChannels; //shoud be 1 or 2;
	
	//delay = cpu_f / rate. Assuming timer clk div is 1
	_clkPer = F_CPU/rate; //using a global var because the clock per will have to be reset again, so avoiding recalculating it

}

void load_input_buffer()
{
	//The first byte of the input buffer holds the length
	
	_inputBuff[0] = WAV_FILE_BUFF_LEN;
	
	if (_totalSampleBytes < _inputBuff[0])
	{
		_inputBuff[0] = _wavMetadata.sampleCount;
	}
	
	//Update the number of sample bytes left
	if (_inputBuff[0] > _totalSampleBytes)
	{
		_totalSampleBytes = 0;
	}
	else
	{
		_totalSampleBytes -= _inputBuff[0];
	}
	//When number of sample bytes <= 0, we reached the end.
	
	//Make sure nothing else is reading the mem.
	//Not ideal making the audio wait but might be the only way.
	
	read_mem(_addresses.wavFileAddress, _inputBuff + 1, _inputBuff[0]);
	
	_addresses.wavFileAddress += _inputBuff[0];
	

	
}

void swap_buffers()
{
	if (_inputBuff == _wavBuff0)
	{
		_inputBuff = _wavBuff1;
		_outputBuff = _wavBuff0;		
	}
	else
	{
		_inputBuff = _wavBuff0;
		_outputBuff = _wavBuff1;		
	}
	
}

void start_audio_output()
{
	//Since we are starting the song, swap the input and output buffers and load the next buffer
	swap_buffers();
	load_input_buffer();
	
	//Set up EDMA and trigger the output DMA transfer by starting the clock
	EDMA.CH2.TRFCNT = _outputBuff[0] ;                  // buff len is stored in first byte of buff
	EDMA.CH2.ADDR = (uint16_t)_outputBuff + 1;           // this is the source SRAM address, skip first 1 bytes
	EDMA.CH2.CTRLA |= EDMA_CH_ENABLE_bm | EDMA_CH_TRFREQ_bm;
	TCC4.PER = _clkPer;


	
}

void play_entry(uint16_t entryNum)
{	
	//Ensure that the audio edma and timer is disabled in case a new song is started before the current one is finished

	while(_readingMem);
	TCC4.PER = 0;
	//Disable light timer
	TCC5.CTRLA = 0;
	
	//Get audio metadata
	load_starting_addresses(entryNum);
	load_wav_metadata();	
	calc_clock_per();
	
	//if light file address == 0, the lights are skipped
	if (_addresses.lightFileAddress != 0)
	{
		load_light_sequenc_metadata();
	}
	
	//Load the first buffer.
	_inputBuff = _wavBuff0;
	load_input_buffer();
	while(_readingMem);
	
	//if light file address == 0, the lights are skipped
	if (_addresses.lightFileAddress != 0)
	{
		load_next_light_map();
			
	}
	
	//Start the audio
	start_audio_output();
	
}

void blank_lights()
{
	_lightMetadata.lightCount = MAX_LIGHT_COUNT;
	for (uint8_t i = 0; i < MAX_LIGHT_COUNT * 3; i++)
	{
		_lightMap.lights[i] = 0;
	}
	
	write_lights();
}

/*============================================================================
Name    :   qt_set_parameters
------------------------------------------------------------------------------
Purpose :   This will fill the default threshold values in the configuration 
            data structure.But User can change the values of these parameters .
Input   :   n/a
Output  :   n/a
Notes   :   initialize configuration data for processing
============================================================================*/

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

int main(void)
{
	//status flags to indicate the re-burst for library
	uint16_t status_flag = 0;
	uint16_t burst_flag = 0;
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
		
	init_pins();
	enable_interrupts();
		
	cli();
	
	init_clock();
	init_usart();
	init_edma();
	
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
	
	//TODO: Remove this line
	DACA.CH0DATA = 2048;
	
	//Set CS high
	PORTD.OUTSET = PIN4_bm;
	
	load_entry_count();
	
    while (1) 
    {
        if(time_to_measure_touch )
        {

	        /*  clear flag: it's time to measure touch  */
	        time_to_measure_touch = 0u;

	        do {
		        /*  one time measure touch sensors    */
		        status_flag = qt_measure_sensors( current_time_ms_touch );
		        if (status_flag & QTLIB_STATUS_CHANGE)
		        {
					if (qt_measure_data.qt_touch_status.sensor_states[0] == 1)
					{
						play_entry(_entryIndex);
					}
					else if (qt_measure_data.qt_touch_status.sensor_states[0] == 2)
					{
						
						_entryIndex++;
						
						if (_entryIndex == _entryCount)
						{
							_entryIndex = 0;
						}
						
						play_entry(_entryIndex);
					}
					else if (qt_measure_data.qt_touch_status.sensor_states[0] == 4)
					{
						if (_entryIndex > 0)
						{
							_entryIndex--;
						}
						else
						{
							_entryIndex = _entryCount - 1;
						}
						play_entry(_entryIndex);
					}
					

		        }
		        
		        burst_flag = status_flag & QTLIB_BURST_AGAIN;

		        /* Time-critical host application code goes here */

	        }while (burst_flag) ;
        }
    }
}

//IRQ handled when data is done being send to the DAC
ISR(EDMA_CH2_vect)
{	
	while( (EDMA.CH2.CTRLB & EDMA_CH_TRNIF_bm) == 0);
	//if number of sample bytes is <= 0 then we reached the end of the audio file
	if (_totalSampleBytes > 0)
	{
		EDMA.CH2.CTRLB &= EDMA_CH_TRNIF_bm | 0x0F; //Only clear the TRNIF flag and not the INT LVLs
		

		//light file address of 0 means skip lights
		//_loadNextLightMap will be non-zero when we are ready to load the next light map
		//Also make sure we have not reached 0 in the map count
		if (_addresses.lightFileAddress != 0 && _holdTimerCounter > _lightMap.holdTime && _lightMetadata.mapCount)
		{
			load_next_light_map();		//This calls read_mem which can't be done within the IRQ handler
			//_readLightMem = 1;
		}
	
		start_audio_output();
	}
	else
	{
		//Disable audio edma and timer
		EDMA.CH2.CTRLB |= EDMA_CH_TRNIF_bm;
		TCC4.PER = 0;
		//Disable light timer
		TCC5.CTRLA = 0;
		blank_lights();
	}

}

//IRQ handled when memory read is complete.
ISR(EDMA_CH0_vect)
{

	//Light file read is started here so we can be sure we are not trying to do two memory reads at the same time
	//which could corrupt the data stream
	
	//End the read
	while(!(EDMA.CH0.CTRLB & EDMA_CH_TRNIF_bm) && !(EDMA.CH1.CTRLB & EDMA_CH_TRNIF_bm));

	//Clear interrupt flags

	EDMA.CH0.CTRLB &= EDMA_CH_TRNIF_bm | 0x0F; //Only clear the TRNIF flag and not the INT LVLs
	EDMA.CH1.CTRLB &= EDMA_CH_TRNIF_bm | 0x0F; //Only clear the TRNIF flag and not the INT LVLs

	
	if (_setCS)
	{
		//Set CS high
		PORTD.OUTSET = PIN4_bm;	
	}
	
	_readingMem = 0;
}

ISR(TCC5_OVF_vect)
{
	TCC5.INTFLAGS = TC5_OVFIF_bm;  //clear the overflow flag
	_holdTimerCounter++;
}

ISR(RTC_OVF_vect)
{
	/*  set flag: it's time to measure touch    */
	time_to_measure_touch = 1;

	/*  update the current time  */
	current_time_ms_touch += qt_measurement_period_msec;

}