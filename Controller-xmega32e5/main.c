/*
 * xmega35e_spi_wav_edma.c
 *
 * Created: 2020-11-11 16:01:40
 * Author : Branden
 */ 



#include "init.h"

#define FILE_COUNT_BUFF_LEN 2 
#define FILE_ENTRY_BUFF_LEN 8 
#define WAV_FILE_BUFF_LEN 128  
#define LIGHT_FILE_BUFF_LEN 100
#define WAV_METADATA_LEN 9
#define LIGHT_METADATA_LEN 4
#define MAX_LIGHT_COUNT 84

#define MEM_READ 0x03
#define MEM_WRITE 0x02
#define MEM_ERASE 0xC7
#define MEM_ENABLE_WRITE 0x06
#define MEM_READ_STATUS 0x05
#define MEM_READ_MAN_ID 0x90
#define MEM_READ_JEDEC 0x9F
#define MEM_READ_ID 0x4B
//Operations: Erase All, Write n bytes to address, Read n bytes from address, Read mem id, Get status

#define USART_ERASE_ALL 'e'
#define USART_WRITE 'w'
#define USART_READ 'r'
#define USART_READ_ID 'i'
#define USART_READ_JEDEC 'j'
#define USART_GET_STATUS 's'

#define OP_STATUS_FAILD 0x00
#define OP_STATUS_SUCCESS 0x01
#define OP_STATUS_BUSY 0x02
#define OP_STATUS_WRITING 0x03
#define OP_STATUS_READING 0x04

#define OP_FAILED_BAD_INPUT 0x01
#define OP_FAILED_NO_RESPONSE 0x02

#define MEM_WRITE_ENABLED_STATUS_BIT 0x02
#define MEM_BUSY_STATUS_BIT 0x01

#define WRITE_CHUNK_SIZE 128 //Sum of this number needs to fall on a 256 page boundary
#define DUMMY_BYTE 0x55


#define RETRY_MS 250
#define RETRY_ATTEMPTS 10


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



//For these arrays, could reuse the _waveBuff arrays but this lowers confusion.
volatile uint8_t _buff0[255];
volatile uint8_t _buff1[255];
volatile uint8_t _propBuff[8];
volatile uint8_t _currentOp;

volatile uint8_t _inputTimeout = 0;
volatile uint8_t *_memInBuff;
volatile uint8_t *_memOutBuff;
volatile uint8_t _dummyByte = DUMMY_BYTE;

volatile uint8_t _readingMem = 0;
volatile uint8_t _setCS = 0;

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
		
	read[0] = MEM_READ;
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
	EDMA.CH1.ADDR = (uint16_t)&_dummyByte;
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
	
	_memInBuff[0] = WAV_FILE_BUFF_LEN;
	
	if (_totalSampleBytes < _memInBuff[0])
	{
		_memInBuff[0] = _wavMetadata.sampleCount;
	}
	
	//Update the number of sample bytes left
	if (_memInBuff[0] > _totalSampleBytes)
	{
		_totalSampleBytes = 0;
	}
	else
	{
		_totalSampleBytes -= _memInBuff[0];
	}
	//When number of sample bytes <= 0, we reached the end.
	
	//Make sure nothing else is reading the mem.
	//Not ideal making the audio wait but might be the only way.
	
	read_mem(_addresses.wavFileAddress, _memInBuff + 1, _memInBuff[0]);
	
	_addresses.wavFileAddress += _memInBuff[0];
	

	
}

void swap_buffers()
{
	if (_memInBuff == _wavBuff0)
	{
		_memInBuff = _wavBuff1;
		_memOutBuff = _wavBuff0;		
	}
	else
	{
		_memInBuff = _wavBuff0;
		_memOutBuff = _wavBuff1;		
	}
	
}

void start_audio_output()
{
	//Since we are starting the song, swap the input and output buffers and load the next buffer
	swap_buffers();
	load_input_buffer();
	
	//Set up EDMA and trigger the output DMA transfer by starting the clock
	EDMA.CH2.TRFCNT = _memOutBuff[0] ;                  // buff len is stored in first byte of buff
	EDMA.CH2.ADDR = (uint16_t)_memOutBuff + 1;           // this is the source SRAM address, skip first 1 bytes
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
	_memInBuff = _wavBuff0;
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




/*Start Mem Access Code*/




void enable_rtc()
{
	CLK.RTCCTRL = CLK_RTCSRC_ULP_gc | CLK_RTCEN_bm; //Set the RTC clock source to the 1kHz ULP and enable it.
	RTC.INTFLAGS = RTC_OVFIF_bm;
}

void disable_rtc()
{
	RTC.INTFLAGS = RTC_OVFIF_bm;
	CLK.RTCCTRL = 0;
}




void mem_read_prop(uint8_t prop, uint8_t *buffer, uint8_t len)
{
	//6 bytes: 1 for command, 3 for data retrieval
	buffer[0] = prop;

	for (uint8_t i = 1; i < len; i++)
	{
		buffer[i] = DUMMY_BYTE;
	}


	PORTD.OUTCLR = PIN4_bm; //Set Pin 4 (CS) to low
	
	_readingMem = 1;
	_setCS = 1;
	
	EDMA.CH0.TRFCNT = len;
	EDMA.CH0.ADDR = (uint16_t)buffer;
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; //
	EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm;
	
	
	EDMA.CH1.TRFCNT = len;
	EDMA.CH1.ADDR = (uint16_t)buffer;;
	EDMA.CH1.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; //
	EDMA.CH1.CTRLA |= EDMA_CH_ENABLE_bm | EDMA_CH_TRFREQ_bm;


	while(_readingMem);
	

	
}

void mem_write_prop(uint8_t prop, uint8_t *buffer, uint8_t len)
{
	//6 bytes: 1 for command, 3 for data retrieval
	buffer[0] = prop;

	for (uint8_t i = 1; i < len; i++)
	{
		buffer[i] = DUMMY_BYTE;
	}


	PORTD.OUTCLR = PIN4_bm; //Set Pin 4 (CS) to low
	
	_readingMem = 1;
	_setCS = 1;
	
	EDMA.CH0.TRFCNT = len;
	EDMA.CH0.ADDR = (uint16_t)buffer;
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; //
	EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm;
	
	
	EDMA.CH1.TRFCNT = len;
	EDMA.CH1.ADDR = (uint16_t)buffer;;
	EDMA.CH1.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; //
	EDMA.CH1.CTRLA |= EDMA_CH_ENABLE_bm | EDMA_CH_TRFREQ_bm;


	while(_readingMem);
	

	
}

void mem_write_data(uint32_t starting_address, uint8_t *buffer, uint8_t len)
{
	
	volatile uint8_t write[4];
	volatile uint8_t trash;
	_setCS = 0;
	_readingMem = 1;
	PORTD.OUTCLR = PIN4_bm; //Set Pin 4 (CS) to low
	
	write[0] = MEM_WRITE;
	write[1] = (starting_address >> 16) & 0xFF;
	write[2] = (starting_address >> 8) & 0xFF;
	write[3] = (starting_address >> 0) & 0xFF;

	EDMA.CH0.TRFCNT = 4;
	EDMA.CH0.ADDR = (uint16_t)&trash;
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_FIXED_gc; //
	EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm;
	
	
	EDMA.CH1.TRFCNT = 4;
	EDMA.CH1.ADDR = write;
	EDMA.CH1.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; //
	EDMA.CH1.CTRLA |= EDMA_CH_ENABLE_bm | EDMA_CH_TRFREQ_bm;
	
	while(_readingMem);
	_readingMem = 1;
	_setCS = 1;
	EDMA.CH0.TRFCNT = len;
	EDMA.CH0.ADDR = (uint16_t)&trash;
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_FIXED_gc; //
	EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm;
	
	EDMA.CH1.TRFCNT = len;
	EDMA.CH1.ADDR = (uint16_t)buffer;
	EDMA.CH1.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; //
	EDMA.CH1.CTRLA |= EDMA_CH_ENABLE_bm | EDMA_CH_TRFREQ_bm;
	
	//while(_readingMem);

	//PORTD.OUTSET = PIN4_bm;
	
}

void mem_read_data(uint32_t starting_address, uint8_t *buffer, uint8_t len)
{
	volatile uint8_t read[4];
	volatile uint8_t trash;
	_readingMem = 1;
	_setCS = 0;
	PORTD.OUTCLR = PIN4_bm; //Set Pin 4 (CS) to low
	
	read[0] = MEM_READ;
	read[1] = (starting_address >> 16) & 0xFF;
	read[2] = (starting_address >> 8) & 0xFF;
	read[3] = (starting_address >> 0) & 0xFF;

	EDMA.CH0.TRFCNT = 4;
	EDMA.CH0.ADDR = (uint16_t)&trash;
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_FIXED_gc; //
	EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm;
	
	
	EDMA.CH1.TRFCNT = 4;
	EDMA.CH1.ADDR = read;
	EDMA.CH1.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; //
	EDMA.CH1.CTRLA |= EDMA_CH_ENABLE_bm | EDMA_CH_TRFREQ_bm;
	
	while(_readingMem);
	_setCS = 1;
	_readingMem = 1;
	
	EDMA.CH0.TRFCNT = len;
	EDMA.CH0.ADDR = (uint16_t)buffer;
	EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; //
	EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm;
	
	EDMA.CH1.TRFCNT = len;
	EDMA.CH1.ADDR = (uint16_t)&_dummyByte;
	EDMA.CH1.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_FIXED_gc; //
	EDMA.CH1.CTRLA |= EDMA_CH_ENABLE_bm | EDMA_CH_TRFREQ_bm;
	
	while(_readingMem);

	
	
}

uint8_t get_data_from_uart(uint8_t *buffer, uint8_t len, uint8_t offset)
{

	uint16_t i = 0;

	//RTC.CNT = 0;
	//enable_rtc();
	

	while(i < len)
	{
		while(!(USARTC0.STATUS & USART_RXCIF_bm) /*&& !(RTC.INTFLAGS & RTC_OVFIF_bm)*/);
		//RTC.CNT = 0;
		//if ((RTC.INTFLAGS & RTC_OVFIF_bm))
		//{
			//disable_rtc();
			//
//
			//return 0;
		//}
		
		buffer[i + offset] = USARTC0.DATA;
		i++;
		//RTC.INTFLAGS = RTC_OVFIF_bm;
	}
	
	
	//disable_rtc();
	return 1;
}

uint8_t get_uint_from_uart(uint32_t *intVal)
{
	static volatile uint8_t arr[4];
	uint8_t i = 0;
	volatile uint32_t val;
	uint32_t *valPtr;

	if (!get_data_from_uart(arr, 4, 0))
	{
		return 0;
	}
	
	
	valPtr = arr;
	val = *valPtr;
	*intVal = val;
	return 1;
}



void transmit_data(uint8_t *data, uint8_t len, uint8_t offset)
{
	uint16_t i = 0;
	uint8_t trash = USARTC0.DATA; //make sure data is clear
	USARTC0.STATUS = USART_TXCIF_bm | USART_DREIF_bm ;
	while(i < len)
	{

		USARTC0.DATA = data[offset + i];
		while(!(USARTC0.STATUS & USART_TXCIF_bm));
		USARTC0.STATUS = USART_TXCIF_bm | USART_DREIF_bm ;
		i++;
	}
	
}

void transmit_status(uint8_t status)
{
	_propBuff[0] = status;
	//Transmit status byte
	transmit_data(_propBuff, 1, 0);
	return;
}

void transmit_status_and_data(uint8_t status, uint32_t data)
{
	_propBuff[0] = status;
	_propBuff[1] = (data >> 0) & 0xFF;
	_propBuff[2] = (data >> 8) & 0xFF;
	_propBuff[3] = (data >> 16) & 0xFF;
	_propBuff[4] = (data >> 24) & 0xFF;

	//Transmit status byte
	transmit_data(_propBuff, 5, 0);
	return;
}

void get_mem()
{

	volatile uint32_t address;
	volatile uint32_t length;

	if (!get_uint_from_uart(&address) || !get_uint_from_uart(&length))
	{
		transmit_status_and_data(OP_STATUS_FAILD, OP_FAILED_BAD_INPUT);
		return;
	}

	transmit_status(OP_STATUS_SUCCESS);
	while (length > 255)
	{
		mem_read_data(address, _buff0, 255);
		address += 255;
		length -= 255;
		transmit_data(_buff0, 255, 0);
	}
	
	if (length > 0)
	{
		mem_read_data(address, _buff0, length);
		transmit_data(_buff0, length, 0);
	}
	
}

void get_mem_id()
{
	mem_read_prop(MEM_READ_ID, _propBuff, 6);
	transmit_status(OP_STATUS_SUCCESS);
	transmit_data(_propBuff, 2, 4);
}

void get_mem_jedec()
{
	mem_read_prop(MEM_READ_JEDEC, _propBuff, 4);
	transmit_status(OP_STATUS_SUCCESS);
	transmit_data(_propBuff, 3, 1);
}

void get_mem_status()
{
	mem_read_prop(MEM_READ_STATUS, _propBuff, 2);
	transmit_status(OP_STATUS_SUCCESS);
	transmit_data(_propBuff, 1, 1);
}

uint8_t mem_enable_write()
{
	uint8_t retryCount = 0;
	mem_write_prop(MEM_ENABLE_WRITE, _propBuff, 1);
	
	
	while(retryCount++ < RETRY_ATTEMPTS)
	{
		mem_read_prop(MEM_READ_STATUS, _propBuff, 2);
		if ((_propBuff[1] & MEM_WRITE_ENABLED_STATUS_BIT))
		{
			return 1;
		}
		_delay_ms(RETRY_MS);
	}
	
	return 0;
}


void erase_mem()
{
	if (!mem_enable_write())
	{
		transmit_status_and_data(OP_STATUS_FAILD, OP_FAILED_NO_RESPONSE);
		return;
	}

	mem_write_prop(MEM_ERASE, _buff1, 1);
	RTC.PER = INPUT_TIMEOUT_PER_MS;
	
	while(1)
	{
		mem_read_prop(MEM_READ_STATUS, _buff1, 2);
		if (!(_buff1[1] & MEM_BUSY_STATUS_BIT))
		{
			transmit_status(OP_STATUS_SUCCESS);
			return;
		}
		_delay_ms(RETRY_MS);
		
		transmit_status(OP_STATUS_BUSY);
	}
	
	
	
}

void swap_write_buffs()
{
	
	if (_memInBuff == _buff0)
	{
		_memInBuff = _buff1;
		_memOutBuff = _buff0;
	}
	else
	{
		_memInBuff = _buff0;
		_memOutBuff = _buff1;
	}
}

uint8_t small_write_mem(uint8_t *buffer, uint32_t address, uint8_t length)
{
	
	if (!mem_enable_write())
	{
		transmit_status_and_data(OP_STATUS_FAILD, OP_FAILED_NO_RESPONSE);
		return 0;
	}
	
	mem_write_data(address, buffer, length);

	return 1;
}

void write_mem()
{
	uint32_t address;
	uint32_t length;
	uint32_t total = 0;
	uint32_t leftToRead = length;
	uint32_t leftToWrite = length;
	uint8_t transLen;
	uint8_t lengthOffset;
	
	if (!get_uint_from_uart(&address) || !get_uint_from_uart(&length))
	{
		transmit_status_and_data(OP_STATUS_FAILD, OP_FAILED_BAD_INPUT);
		return;
	}


	
	lengthOffset = (address % WRITE_CHUNK_SIZE);
	
	//If there is an offset, calculate the difference 
	if (lengthOffset > 0)
	{
		lengthOffset = WRITE_CHUNK_SIZE - lengthOffset;
	}
	
	//because of timing (needing to read everything from the UART), if the length is small, handle the write in one or two ops first
	if (length < WRITE_CHUNK_SIZE)
	{
		if (!get_data_from_uart(_buff0, length, 0))
		{
			transmit_status_and_data(OP_STATUS_FAILD, OP_FAILED_BAD_INPUT);
			return;
		}
		
		//if the data will still fit within a page boundary, even with the address offset, set the lengthOffset to 0 so we just write the data normally
		if (lengthOffset > length)
		{
			lengthOffset = 0;
		}
		
		//If the data is going to go over a page boundary
		if (lengthOffset > 0)
		{
			
			
			if (!small_write_mem(_buff0, address, lengthOffset))
			{
				return;
			}
			while(_readingMem);
			//We need to wait until the memory is ready for the last write
			_delay_ms(1);
		}
		
		//Send the rest. if lengthOffset was 0, then everything gets sent here
		if (!small_write_mem(_buff0 + lengthOffset, address + lengthOffset, length - lengthOffset))
		{
			return;
		}
		
		while(_readingMem);
		
		transmit_status_and_data(OP_STATUS_SUCCESS, length);
		return;
	}


	leftToRead = length;
	leftToWrite = length;
	
	_memInBuff = _buff0;
	_memOutBuff = _buff1;
	
	//Here we know that the size of the data is larger so get a chunck to handle the address offset and page-align the writes.
	if (lengthOffset > 0)
	{
		if (!get_data_from_uart(_memInBuff, lengthOffset, 0))
		{
			transmit_status_and_data(OP_STATUS_FAILD, OP_FAILED_BAD_INPUT);
			return;
		}
		swap_write_buffs();
		
		if (!small_write_mem(_memOutBuff, address, lengthOffset))
		{
			return;
		}
		
		leftToRead -= lengthOffset;
		leftToWrite -= lengthOffset;
		address += lengthOffset;
	}
	
	while (leftToRead > 0 || leftToWrite > 0)
	{
		

		if (leftToRead > WRITE_CHUNK_SIZE)
		{
			transLen = WRITE_CHUNK_SIZE;
		}
		else
		{
			transLen = leftToRead;
		}
		
		if (transLen > 0)
		{
			get_data_from_uart(_memInBuff, transLen, 0);
			//if (!get_data_from_uart(_memInBuff, transLen, 0))
			//{
				//transmit_status_and_data(OP_STATUS_FAILD, OP_FAILED_BAD_INPUT);
				//return;
			//}
			swap_write_buffs();
		}
		
		leftToRead -= transLen;
		
		if (leftToWrite > WRITE_CHUNK_SIZE)
		{
			transLen = WRITE_CHUNK_SIZE;
		}
		else
		{
			transLen = leftToWrite;
			//We need to wait until the memory is ready for the last write
			_delay_ms(1);
			
		}
		

		if (!small_write_mem(_memOutBuff, address, transLen))
		{
			return;
		}

		leftToWrite -= transLen;
		address += transLen;

	}
	
	
	while(_readingMem);
	
	transmit_status_and_data(OP_STATUS_SUCCESS, length);
	
}

void get_uart_command()
{

	while(!(USARTC0.STATUS & USART_RXCIF_bm))
	USARTC0.STATUS = USART_RXCIF_bm;
	
	_currentOp = USARTC0.DATA;
	switch(_currentOp)
	{
		
		case USART_ERASE_ALL:
		erase_mem();
		break;
		case USART_WRITE:
		write_mem();
		break;
		case USART_READ:
		get_mem();
		break;
		case USART_READ_ID:
		get_mem_id();
		break;
		case USART_READ_JEDEC:
		get_mem_jedec();
		break;
		case USART_GET_STATUS:
		get_mem_status();
		break;

	}
}



/*Start global code*/





uint8_t get_start_mode()
{
	if (PORTD.IN & PIN6_bm)
	{
		PORTD.OUTSET = PIN7_bm;
		return 1;
	}
	else
	{
		PORTD.OUTCLR = PIN7_bm;
		return 0;
	}
	
}


int main(void)
{
	init_start_mode_pins();
	
	if (get_start_mode())
	{
		//Start mem access mode
		init_mem_access_config();

		/* Replace with your application code */
		while (1)
		{
			get_uart_command();
		}
	}
	else
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