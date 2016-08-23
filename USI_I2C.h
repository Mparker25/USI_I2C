/*-----------------------------------------------*\
|	USI I2C Library 							  |				
|												  |	
|	The following library aims at providing I2C	  |												
|	communications protocol to the ATtiny 		  |											
|	microcontrollers using the provided onboard   |													
|	Universal Serial Interface (USI)			  |										
|												  |
|												  |
|	*Currently only works with ATtiny44A*		  |
|												  |
|												  |	
|	Author: Malik B. Parker						  |							
|	Email: Malik.bernard.parker@gmail.com		  |											
|												  |		
|	Made for 4moms								  |					
|												  |		
\*-----------------------------------------------*/

#ifndef USI_I2C_H
#define USI_I2C_H

#define F_CPU 8000000UL		// User defined to match the clock frequency of MCU

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>


// I2C Bus Specs for version 6.0 FAST mode 
#ifdef FAST_MODE
	#define TSTART	0.6		// Time needed to hold SDA low to be recognized as a START condition
	#define TLOW	1.3		// How long the clock needs to be held low 
	#define THIGH	0.6		// How long the clock needs to be held high
	#define TSETUP	0.6		// Time needed to hold lines HIGH before commencing a START condition
	#define TSTOP	0.6		// Time needed to hold SDA low to be recognized as a STOP condition
	

// I2C Bus Specs for version 6.0 STANDARD mode
#else
	#define TSTART	4.0		// Time needed to hold SDA low to be recognized as a START condition
	#define TLOW	4.7		// How long the clock needs to be held low 
	#define THIGH	4.0		// How long the clock needs to be held high
	#define TSETUP	4.7		// Time needed to hold lines HIGH before commencing a START condition
	#define TSTOP	4.0		// Time needed to hold SDA low to be recognized as a STOP condition
#endif

// PORT A
#define DIR			DDRA				// Direction 
#define SCL			0b10000				// Pin 4  [Clock]
#define SDA 		0b1000000			// Pin 6  [Data]
#define OVF			0b1000000			// Pin 6  [Overflow Flag]


// USER DEFINED MACROS

// USER-DEFINED SLAVE ADDRESS
#define SLAVE_ADDR 0x36 

#ifndef REG_COUNT
	#define REG_COUNT 15				// Number of Registers for slave device
#endif

#define OFF_PERCENTAGE 90				// Percentage of time sensor needs to be off before it registers as off
#define ON_PERCENTAGE  15				// Percentage of time sensor needs to be on before it registers as on


// Type Definitions 
typedef struct packpage{
	uint8_t* buffer;					// Packaged data to be sent to Master
	uint8_t  numbytes;					// Number of bytes possible for the master to request (Needed for protection)
} packet;

extern uint8_t Registers[REG_COUNT];	// Configurable I2C System Registers
										// Register Number is the Register Address

typedef packet (*request)(void);		// Function to be called when the Master requests, responsible for gather data and packaging
										// it into a typedef packet to be sent to the Master

// Public Function Prototypes
uint8_t write(uint8_t address, uint8_t reg, uint8_t* data, uint8_t datasize);		// Puts device in Master mode and writes data to a slave
uint8_t read(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t numbytes);		// Puts device in Master Mode and requests data from slave
void slave_init(void);//packet onRequest);		// Puts device in slave mode and calls user-defined function when Master requests info

#endif