/******************************************************************************* 
*   $FILE:  touch_config.h
*   Atmel Corporation:  http://www.atmel.com \n
*   Support email:  touch@atmel.com
******************************************************************************/
/*  License
*   Copyright (c) 2010, Atmel Corporation All rights reserved.
*   
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions are met:
*   
*   1. Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
*   
*   2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*   
*   3. The name of ATMEL may not be used to endorse or promote products derived
*   from this software without specific prior written permission.
*   
*   THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
*   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
*   SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
*   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
*   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
*   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TOUCH_CONFIG_H
#define TOUCH_CONFIG_H
/*----------------------------------------------------------------------------
                       Acquisition method definition
----------------------------------------------------------------------------*/
#define _QTOUCH_

/*----------------------------------------------------------------------------
               Library configuration notes for QTouch acquisition.
----------------------------------------------------------------------------*/

/**
  * The following rules need to be followed while configuring 
  * the QTouch library for QTouch acquisition method.
  * 1. Always setup the following to have valid values
  *			#define QT_DELAY_CYCLES	<value>
  *			#define SNS1			<value>
  *			#define SNSK1			<value>
  * 2. If SNS1 and SNSK1 pins are using the same port in intraport configuration, define
  * 		#define _SNS1_SNSK1_SAME_PORT_
  * 3. If using two port pairs, define
  *			#define SNS2			<value>
  *			#define SNSK2			<value>
  * 4. If SNS2 and SNSK2 pins are using the same port in intraport configuration, define
  *			#define _SNS2_SNSK2_SAME_PORT_
  * 5. If using a debug interface, define
  * 		#define _DEBUG_INTERFACE_
  * 6. If using power optimized code on ATTiny or ATMega devices, define
  *			#define _POWER_OPTIMIZATION 1
  *    Otherwise, for ATTiny or ATMega devices, add
  *			#define _POWER_OPTIMIZATION 0
  * 7. If using custom pin configuration, and would like to define pins used, 
  * 		#define QTOUCH_STUDIO_MASKS
  *    Otherwise, the library will automatically calculate pins used.
  */

/*----------------------------------------------------------------------------
                QTouch Library Configuration Options.
----------------------------------------------------------------------------*/
/**
  * Enable/Disable manual pin configuration for QTouch channels.
  * Note: If the below macro is not defined pin configuration is automatically 
  *       assigned by QTouch library.
  */
#define QTOUCH_STUDIO_MASKS	
#define _CONF_4QT_0RS_
/**
  * Example configuration based on option selected in project options.
  * For custom configuration make changes to the "#else" section.
  */
#if defined(_CONF_16QT_RS_) /* Port and pin mask selection for 16 channel and 4 router/slider option*/
#define QT_NUM_CHANNELS 16
/* Port selection */
#define SNS1	C
#define SNSK1	A
#define SNS2	E
#define SNSK2	B

#define _ROTOR_SLIDER_

#elif defined(_CONF_12QT_RS_) /* Port and pin mask selection for 12 channel and 3 router/slider option*/
#define QT_NUM_CHANNELS 12
/* Port selection */
#define SNS1	C
#define SNSK1	A
#define _SNS2_SNSK2_SAME_PORT_
#define SNS2	B
#define SNSK2	B

#define _ROTOR_SLIDER_

#elif defined(_CONF_8QT_) /* Port and pin mask selection for 8 channel and key only option*/
#define QT_NUM_CHANNELS 8
/* Port selection */
#define SNS1	C
#define SNSK1	A

#elif defined(_CONF_4QT_RS_) /* Port and pin mask selection for 4 channel and 1 router/slider option*/
#define QT_NUM_CHANNELS 4
/* Port selection */
#define SNS1	C
#define SNSK1	B

#define _ROTOR_SLIDER_

#elif defined(_CONF_4QT_0RS_) /* Port and pin mask selection for 4 channel and 0 router/slider option*/
#define QT_NUM_CHANNELS 4
/* Port selection */
#define _SNS2_SNSK2_SAME_PORT_
#define SNS1	A
#define SNSK1	A


#else				/* For custom user configuration use the below section. */
/**
  * Number of Channels(dependent on the library used). Please refer to the user guide
  * more information on selecting the number of channels.
  *
  * Possible values: 4, 8, 12, 16.
  */
#define QT_NUM_CHANNELS 16
/**
  * Define the ports to be used for SNS1,SNS2 and SNSK1,SNSK2 pins. SNS1,SNS2 and SNSK1,SNSK2 port pins
  * can be available on the same port or on different ports.
  * Define the appropriate port pins enabled in SNS_ARRAY_xx and SNSK_ARRAY_xx.
  *
  * Possible values: refer to the device data sheet and QTouch libraries user guide.
  */
#define SNS1	C
#define SNSK1	A
#define SNS2	E
#define SNSK2	B

/**
  * Enable/Disable rotor/slider.
  */									 
#define _ROTOR_SLIDER_

#endif /* Project configuration */

/*----------------------------------------------------------------------------
                     QTouch sensor configuration options.
----------------------------------------------------------------------------*/

/**
  * Delay cycles that determine the capacitance charge pulse width.
  *
  * Possible values: 1 to 255
  */
#define QT_DELAY_CYCLES 1

/**
  * Enabling _POWER_OPTIMIZATION_ will lead to a 40% reduction in power consumed
  * by the library, but by disabling spread spectrum feature. When power optimization
  * is enabled the unused pins, within a port used for QTouch, may not be usable for
  * interrupt driven applications. This option is available only for ATtiny and ATmega
  * devices.
  *
  * Possible values: 0 or 1 (For ATtiny and ATmega devices)
  *                  0 (For ATxmega devices)
  */
#define _POWER_OPTIMIZATION_ 0

/*----------------------------------------------------------------------------
             Post Processing Configuration for Touch Library.
----------------------------------------------------------------------------*/
/**
  * Total ticks per msec.
  * 1 in this case for RTC clock source of 1kHz ULP
  */
#define TICKS_PER_MS                1

/**
  * Sensor detect integration (DI) limit.
  * Default value: 4.
  */
#define DEF_QT_DI                      4u

/**
  * Sensor negative drift rate.
  *
  * Units: 200ms
  * Default value: 20 (4 seconds per LSB).
  */
#define DEF_QT_NEG_DRIFT_RATE          20      /* 4s per LSB */

/**
  * Sensor positive drift rate.
  *
  * Units: 200ms
  * Default value: 5 (1 second per LSB).
  */
#define DEF_QT_POS_DRIFT_RATE          5       /* 1s per LSB */

/**
  * Sensor drift hold time.
  *
  * Units: 200ms
  * Default value: 20 (hold off drifting for 4 seconds after leaving detect).
  */
#define DEF_QT_DRIFT_HOLD_TIME         20      /* 4s */

/**
  * Sensor maximum on duration. 
  *
  * Units: 200ms (e.g., 150 = recalibrate after 30s). 0 = no recalibration.
  * Default value: 0 (recalibration disabled).
  */
#define DEF_QT_MAX_ON_DURATION         0       /* disabled */

/**
  * Sensor recalibration threshold.
  *
  * Default: RECAL_50 (recalibration threshold = 50% of detection threshold).
  */
#define DEF_QT_RECAL_THRESHOLD         RECAL_50 /* recal threshold = 50% of detect */

/**
  * Positive recalibration delay.
  *
  * Default: 3
  */
#define DEF_QT_POS_RECAL_DELAY         3u

/*----------------------------------------------------------------------------
                          Debug Interface settings
----------------------------------------------------------------------------*/
/**
  * Enable/Disable QDebug for touch debug information communication with
  * QTouch Studio PC Software.
  * QDebug is enabled by defining the below macro.
  */
//#define _DEBUG_INTERFACE_
 
#ifdef _DEBUG_INTERFACE_
/**
  * Select the type of interface to use for the debug protocol.
  */

//! When 0, QDebug interface type is on-chip SPI peripheral
//! When 1, QDebug interface type is on-chip TWI peripheral
//! When 2, QDebug interface type is Bit bang SPI 
#define DEF_QTOUCH_INTERFACE_TYPE     (2)

/**
  * Bit bang SPI port and pin configuration.
  * Note: Ensure that the port pins mentioned below are not used for SNS/SNSK
  */
#define SS_BB       		4
#define SCK_BB      		7
#define MOSI_BB   			5
#define MISO_BB   			6

#define SPI_BB_SS      		D
#define SPI_BB_SCK      	D
#define SPI_BB_MOSI      	D
#define SPI_BB_MISO      	D

#endif /* _DEBUG_INTERFACE */

/**
  * Set up project info
  */
#define		PROJECT_ID			DUMMY

#endif /* TOUCH_CONFIG_H */
