/*
 * xmega35e_spi_wav_edma.c
 *
 * Created: 2020-11-11 16:01:40
 * Author : Branden
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "touch_api.h"
#include "init.h"

#define MEM_READ_DATA 0x03
#define FILE_COUNT_BUFF_LEN 2 
#define FILE_ENTRY_BUFF_LEN 8 
#define WAV_FILE_BUFF_LEN 128  
#define LIGHT_FILE_BUFF_LEN 100
#define WAV_METADATA_LEN 9
#define LIGHT_METADATA_LEN 4
#define MAX_LIGHT_COUNT 84




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


int main(void)
{
	//status flags to indicate the re-burst for library
	uint16_t status_flag = 0;
	uint16_t burst_flag = 0;

		
	init_main_config();
	

	
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