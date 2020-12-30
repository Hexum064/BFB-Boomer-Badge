/*
 * usart_to_spi_flash_poc.c
 *
 * Created: 2020-12-28 10:35:22
 * Author : Branden
 */ 
#define  F_CPU 32000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

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
//1 second timeout will reset operation
#define INPUT_TIMEOUT_PER_MS 1000
#define RETRY_MS 250
#define RETRY_ATTEMPTS 10

volatile uint8_t _readingMem = 0;

volatile uint8_t _buff0[255];
volatile uint8_t _buff1[255];
volatile uint8_t _propBuff[8];
volatile uint8_t _currentOp;

volatile uint8_t _setCS = 0;
volatile uint8_t _inputTimeout = 0;
volatile uint8_t *_memInBuff;
volatile uint8_t *_memOutBuff;
volatile uint8_t _dummyByte = DUMMY_BYTE;



void enable_interrupts()
{
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm 	;	  //  enable interrupts
}

void init_pins_mem_access()
{
	PORTA.DIRSET = PIN0_bm | PIN2_bm;

	PORTD.DIRSET = PIN1_bm | PIN3_bm | PIN4_bm;
	PORTD.DIRCLR = PIN2_bm;
	
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
	

	
	//interface
	USARTC0.BAUDCTRLA = 75;// 131;                                     
	USARTC0.BAUDCTRLB = 0xA0;//-6 0xD0; //-3
	USARTC0.CTRLA = USART_TXCINTLVL_OFF_gc; //No interrupts
	USARTC0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	USARTC0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc; //Mode:Async,8,None,1
	USARTC0.CTRLD = 0; //No decoding or encoding

}



void init_input_timeout_timer()
{
	RTC.PER = INPUT_TIMEOUT_PER_MS;
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;

}

void enable_rtc()
{
	CLK.RTCCTRL = CLK_RTCSRC_ULP_gc | CLK_RTCEN_bm; //Set the RTC clock source to the 1kHz ULP and enable it.
}

void disable_rtc()
{
	CLK.RTCCTRL = 0;	
}

void init_clock_mem_access()
{
	
	OSC_CTRL |= OSC_RC32MEN_bm; //Setup 32Mhz internal
	
	while(!(OSC_STATUS & OSC_RC32MRDY_bm));
	
	CCP = CCP_IOREG_gc; //Trigger protection mechanism
	CLK_CTRL = CLK_SCLKSEL_RC32M_gc; //Enable internal  32Mhz internal
	
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

	RTC.CNT = 0;
	enable_rtc();
	

	while(i < len)
	{
		while(!(USARTC0.STATUS & USART_RXCIF_bm) && !(RTC.INTFLAGS & RTC_OVFIF_bm));
		RTC.CNT = 0;
		if ((RTC.INTFLAGS & RTC_OVFIF_bm))
		{
			disable_rtc();
			RTC.INTFLAGS = RTC_OVFIF_bm;

			return 0;
		}
		
		buffer[i + offset] = USARTC0.DATA;
		i++;
	}
	
	
	disable_rtc();	
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


	
	lengthOffset = WRITE_CHUNK_SIZE - (address % WRITE_CHUNK_SIZE);
	
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
			if (!get_data_from_uart(_memInBuff, transLen, 0))
			{
				transmit_status_and_data(OP_STATUS_FAILD, OP_FAILED_BAD_INPUT);
				return;
			}
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

int main(void)
{
	enable_interrupts();
	init_pins_mem_access();
	
	cli();
	
	init_clock_mem_access();
	init_usart_mem_access();
	init_edma_mem_access();
	init_input_timeout_timer();
	
	sei();
	PORTD.OUTSET = PIN4_bm;	


    /* Replace with your application code */
    while (1) 
    {
		get_uart_command();
    }
}

//IRQ handled when memory read is complete.
ISR(EDMA_CH0_vect)
{
	
	//End the read
	while(!(EDMA.CH0.CTRLB & EDMA_CH_TRNIF_bm) && !(EDMA.CH1.CTRLB & EDMA_CH_TRNIF_bm));

	//Clear interrupt flags

	EDMA.CH0.CTRLB &= EDMA_CH_TRNIF_bm | 0x0F; //Only clear the TRNIF flag and not the INT LVLs
	EDMA.CH1.CTRLB &= EDMA_CH_TRNIF_bm | 0x0F; //Only clear the TRNIF flag and not the INT LVLs

	
	_readingMem = 0;
	
	if (_setCS)
	{
		PORTD.OUTSET = PIN4_bm; //Set Pin 4 (CS) to high
	}

}

