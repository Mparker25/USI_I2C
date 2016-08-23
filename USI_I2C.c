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


#include "USI_I2C.h"
#include <assert.h>



// Private Function Protoypes 
static uint8_t transfer(uint8_t byte);
static uint8_t receive(bool endTransmission);
static void clock_edges(uint8_t edges);
static void start(void);
static void stop(void);
static void resetLine(void);
static void resetState(void);

// Register Flags & Pins
#define OVERFLOW		(USISR & OVF)		// Counter Overlfow Flag
#define SCL_HIGH		(PINA  & SCL)		// SCL PIN
#define SDA_HIGH 		(PINA & SDA)		// SDA PIN

// I2C Bus Conditions
#define	HOLD_START_COND() 	{_delay_us(TSTART);}			// Hold SDA Low & SCL High for TSTART
#define SETUP_START_COND() 	{_delay_us(TSETUP);}			// Hold SDA & SCL High for TSTART
#define SETUP_STOP_COND()	{_delay_us(TSTOP);}				// Hold SDA Low for TSTOP
#define ACK()				{USIDR = 0x00;clock_edges(2);}	// Send an acknowledgment on the bus
#define NACK()				{USIDR = 0xFF;clock_edges(2);}	// Send a Non-Acknowledgment on the bus

// Clock Generation
#define HOLD_CLK_LOW()			{_delay_us(TLOW);}			// Hold the clock low for TLOW
#define HOLD_CLK_HIGH() 		{_delay_us(THIGH);}			// Hold the clock high for THIGH
#define CLOCK() 				{USICR = 0b00101011;}		// Toggle SCL for Master
#define ENABLE_START_FLAG()		{USICR = 0b10111000;}		// Set the Start Interrupt and set clock as external
#define ENABLE_OVF_FLAG()		{USICR = 0b11111000;}		// Set the Overflow Interrupt 
#define CLR_STATUS()			{USISR = 0xD0;}				// Clear Start | Overflow | Arbitration Flags (Releases clock from low)
#define SET_COUNT(X)			{USISR = 0x50 | X;}			// Clear the overlfow flag and reset the count


// Data Line Configuration
#define SET_SDA_OUTPUT()	{DDRA |= SDA;}			//	Set the direction of SDA port as input
#define SET_SDA_INPUT()		{DDRA &= ~SDA;}			//	Set the direction of SDA port as output
#define SET_SDA_HIGH()		{PORTA |= SDA;}			//	Set the SDA port HIGH
#define SET_SDA_LOW()		{PORTA &= ~SDA;}		//	Set the SDA port LOW	

// Clock Line Configuration				
#define SET_SCL_OUTPUT()	{DDRA |= SCL;}			//	Set the direction of SCL port as input
#define SET_SCL_INPUT()		{DDRA &= ~SCL;}			//	Set the direction of SCL port as output
#define SET_SCL_HIGH()		{PORTA |= SCL;}			//	Set the SCL port HIGH
#define SET_SCL_LOW()		{PORTA &= ~SCL;}		//	Set the SCL port LOW		



// Global Variables

enum {
	ADDRESS,
	REGADDR,
	ADDR_ACK,
	READ_ACK,
	WRITE_ACK,
	READ_MULT,
	READ,
	WRITE
} current_state, next_state;

// Is device a Slave or Master
bool Slave = false;
bool StopCond = true;

// Global Variables for Slave Mode
uint8_t data;				// Universal data to be read or written to I2C bus from slave
bool direction;			// LSB in Address Byte (0 indicates write | 1 indicates read)


// On-Board I2C Registers
uint8_t Registers[15];		// System Registers
uint8_t reg_pointer = 0;	// Register Pointer indicating which register to read from or write to



/**********************************************************************************************************
 **********************************************************************************************************
 ******************************************  Master  Functions  *******************************************
 **********************************************************************************************************
 *********************************************************************************************************/

// Write to an I2C Slave Device
// 		
//		Address: 	Provide the address of the slave without any appended bits
//		Reg: 		Initial Register you want to write to.	
//		Data:		Data to send to specific register. (Supports Multi-Byte Writing) 
//		Datasize:	The number of data bytes you're sending to the slave
//
uint8_t write(uint8_t address, uint8_t reg, uint8_t* data, uint8_t datasize){
	// Safety Check!
	if (datasize < 1)
		assert(data == NULL);
	uint8_t addr = (address << 1);

	// Begin a transaction on the I2C Bus
	start();	

	// Transmit slave address to I2C bus
	if(transfer(addr)){		
		return 1;				// Didn't receive an acknowledgment from the Slave
	}

	// Transmit slave register address to slave 
	if(transfer(reg)){
		return 1;				// Didn't receive an acknowledgment from the Slave
	}

	// Transmit data to slave
	while(datasize--){
		if(transfer(*data)){
			return 1;			// Didn't receive an acknowledgment from the Slave
		}
		data++;
	}

	stop();
	return 0;	
}


// Read from a I2C slave device
// Can read from a specific register or read from the device in general
// Can read from multiple registers by using incremental reading
// Data returned will be an array provided by the user
//
// Read from an I2C Slave Device
// 		
//		Address: 	Provide the address of the slave without any appended bits
//		Reg: 		Initial Register you want to read from.	
//		Data:		Data to send to specific register. (Supports Multi-Byte Writing) 
//		Datasize:	The number of data bytes you're sending to the slave
//
uint8_t read(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t numbytes){
	// Safety Check!
	if (numbytes < 1)
		assert(buffer == NULL);
	uint8_t addr = (address << 1);
	
	// Begin a transaction on the I2C Bus
	start();
	
	// Transmit slave address to I2C bus
	if(transfer(addr)){		
		return 1;				// Didn't receive an acknowledgement from the Slave
	}
	
	// Transmit slave register address to slave 
	if(transfer(reg)){
		return 1;				// Didn't receive an acknowledgment from the Slave
	}
	
	
	// Issue a Restart to start reading from the slave
	SET_SDA_INPUT();
	SET_SDA_HIGH();
	start();
	
	addr = (address << 1) | 1;
	
	// Transmit slave address to I2C bus
	if(transfer(addr)){		
		return 1;				// Didn't receive an acknowledgment from the Slave
	}
	
	// Device is receicing data as a Master 
	Slave = false;
	
	// Read multiple bytes until it's one left to read	
	while(numbytes-- > 1){
		*buffer = receive(false);
		buffer++;
	}
	
	// Read the last byte and end the transmission
	*buffer = receive(true);
	stop();
	
	return 0;
}


/**********************************************************************************************************
 **********************************************************************************************************
 ****************************************   Slave Functions    ********************************************
 **********************************************************************************************************
 *********************************************************************************************************/

//	Slave Mode	WARNING: THIS IS AN ATOMIC FUNCTION
//
//		Puts the device in slave mode and waits until a master requests data. 
//		This function should be called once before scheduling loop.
void slave_init(void){ 
	
	// Clear Interrupts 
	cli();
	
	// Configure System clock to work with I2C Fast Mode and below
	CLKPR = (1<<CLKPCE);
	CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);	//	8MHz
	
	// Release the Clock and Data lines
	SET_SCL_INPUT();
	SET_SDA_INPUT();
	SET_SCL_HIGH();
	SET_SDA_HIGH();	
	
	// Enable the Start Condition Interrupt
	ENABLE_START_FLAG();
	
	// Enable Global Interrupts
	sei();
}

static void resetState(void){
	CLR_STATUS();
	ENABLE_START_FLAG();
}

static void resetLine(void){
	SET_SCL_INPUT();
	SET_SDA_INPUT();
	SET_SCL_HIGH();
	SET_SDA_HIGH();
	SET_SCL_OUTPUT();
	
	resetState();
}



// Start Interrupt
//
//		Prepares the device for data acquisition from Master Device 
//		when a start condition is placed on the bus.
//
ISR(USI_STR_vect){
	
	// Prepare device to receive data from Master
	SET_SDA_INPUT();
	
	
	// Wait for start condition on the bus to clear before continuing
	while(!(PINA & _BV(PINA6)) && (PINA & _BV(PINA4))){
		if((PINA & _BV(PINA6))){	// stop condition
			SET_SCL_INPUT();
			SET_SDA_INPUT();
			SET_SCL_HIGH();
			SET_SDA_HIGH();
			SET_SCL_OUTPUT();
			resetLine();
		}
	}
	
	// AUTHOR COMMENT:: Not mandatory but consider removing for completeness!
	// reg_pointer = 0;
	
	// Enable Overflow interrupt to start clocking the addr
	ENABLE_OVF_FLAG();			
	
	// Setup the counter to overflow for addressing (8 bits or 16 clock ticks)
	CLR_STATUS();				// Clear:	Start | Overflow | Arbitration Flags (Releases clock from low)
	
	// Setup the Slave State Machine
	current_state = ADDRESS;
	SET_SCL_OUTPUT();
}

uint8_t data2;
uint8_t temp;


// Overflow already happened bub!
 ISR(USI_OVF_vect){

	SET_SCL_OUTPUT();
	SET_SDA_INPUT();
	
	data  = USIDR;		// Needs to be read immediately to avoid inaccurate data
	uint8_t count = 0;
	
	
	
	
	
	if (current_state == ADDRESS){
		if ((data>>1) == SLAVE_ADDR){
			SET_SDA_OUTPUT();
			USIDR = 0x00;
			count = 0xE;
			current_state = ADDR_ACK;
			direction = data & 0x1;
		}
		else{
			SET_SCL_INPUT();
			SET_SCL_HIGH();
			resetState();
		}
	}
	else{
		switch(current_state){
		

			// STATE 2:: ACK was sent to Master, Prepare for Master to Read or Write 
			case(ADDR_ACK):
		
				// If Master is reading, send data out Immediately!
				if (direction) {
					SET_SDA_OUTPUT();
					goto send;
				}
			
				// If Master is writing to device, prepare first write to set Register Pointer
				current_state = REGADDR;
				SET_SDA_INPUT();	
				SET_SDA_HIGH();
				count = 0;			
				break;
		
			// STATE 3:: 
			case(WRITE_ACK):
				SET_SDA_INPUT();
				SET_SDA_HIGH();
				count = 0;
				current_state = WRITE;
				break;
			
	
			// * STATE 4:: NACK/ACK from Master to determine if reading continues
			case(READ_ACK):
				SET_SDA_INPUT();
				SET_SDA_HIGH();
				count = 0xE;
				current_state = READ_MULT;
				break;
			
			//  STATE 5:: Master is configuring which register to write to 
			case(REGADDR):
				reg_pointer = data;
			
				// Prepare the ACK to be sent
				USIDR = 0x00;
				count = 0x0E;					// Set the counter for the ACK (Don't clear Start or Stop flags)
				SET_SDA_OUTPUT();				
				current_state = WRITE_ACK;
				break;
			
			// STATE 6:: 
			case(READ_MULT):
				if (!(data & 0x01)){ // Acknowledgment 
					reg_pointer++;
					goto send;
				}		
				current_state = ADDRESS;
				break;
			
			// STATE 7:: Master is reading from device
	send:	case(READ):
		
				// Check that the register pointer is within bounds before sending out data
				if (reg_pointer < 10){
					temp = Registers[reg_pointer];
					USIDR = temp;
				}
				else USIDR = 0xF2;		// Dummy variable to send in case of corruption
			
				SET_SDA_INPUT();
				SET_SDA_HIGH();
				SET_SDA_OUTPUT();

				count = 0;
				current_state = READ_ACK;
				break;
			
			
			// STATE 8:: Master is writing to this device 
			case(WRITE):
		
				// Check that the register pointer is within bounds before master writes data
				if (reg_pointer < 10) {
					Registers[reg_pointer] = data;
					reg_pointer++;
				}
			
				// Send ACK to confirm that Data has been written
				USIDR = 0x00;
				count = 0xE;					
				SET_SDA_OUTPUT();				
				current_state = WRITE_ACK;
				break;	
		
		}	
	}
	
	


	/*switch(current_state){
		
		/ * STATE 1::	Address read from bus * /
		case(ADDRESS):
			direction = data & 0x01;
			
			// ADDRESS MATCHES: Send out acknowledgment
			if ((data >> 1) == SLAVE_ADDR){			
				count = 0xE;					
				SET_SDA_OUTPUT();				
				SET_SDA_LOW();
				while(PINA & _BV(PINA6));		
				next_state = ADDR_ACK;			
			}
			
			// ADDRESS DOESN'T MATCH: Clear the line and reset the state
			else{
				SET_SCL_INPUT();				
				SET_SCL_HIGH();
				USIDR = 0;						
				count = 0;
				resetState();					
				next_state = ADDRESS;
			}
			break;
		*/
/*
		/ * STATE 2:: ACK was sent to Master, Prepare for Master to Read or Write * /
		case(ADDR_ACK):
		
			// If Master is reading, send data out Immediately!
			if (direction) {
				SET_SDA_OUTPUT();
				goto send;
			}
			
			// If Master is writing to device, prepare first write to set Register Pointer
			next_state = REGADDR;
			SET_SDA_INPUT();	
			SET_SDA_HIGH();
			count = 0;			
			break;
		
		/ * STATE 3:: * /
		case(WRITE_ACK):
			SET_SDA_INPUT();
			SET_SDA_HIGH();
			count = 0;
			next_state = WRITE;
			break;
			
			
		/ * STATE 4:: NACK/ACK from Master to determine if reading continues* /
		case(READ_ACK):
			SET_SDA_INPUT();
			SET_SDA_HIGH();
			count = 0xE;
			next_state = READ_MULT;
			break;
			
		/ * STATE 5:: Master is configuring which register to write to * /
		case(REGADDR):
			reg_pointer = data;
			
			// Prepare the ACK to be sent
			USIDR = 0x00;
			count = 0x0E;					// Set the counter for the ACK (Don't clear Start or Stop flags)
			SET_SDA_OUTPUT();				
			next_state = WRITE_ACK;
			break;
			
		/ * STATE 6:: * /
		case(READ_MULT):
			if (!(data & 0x01)){ // Acknowledgment 
				reg_pointer++;
				goto send;
			}		
			next_state = ADDRESS;
			break;
			
		/ * STATE 7:: Master is reading from device * /
send:	case(READ):
		
			// Check that the register pointer is within bounds before sending out data
			if (reg_pointer < 10){
				temp = Registers[reg_pointer];
				USIDR = temp;
			}
			else USIDR = 0xF2;		// Dummy variable to send in case of corruption
			
			SET_SDA_INPUT();
			SET_SDA_HIGH();
			SET_SDA_OUTPUT();

			count = 0;
			next_state = READ_ACK;
			break;
			
			
		/ * STATE 8:: Master is writing to this device * /
		case(WRITE):
		
			// Check that the register pointer is within bounds before master writes data
			if (reg_pointer < 10) {
				Registers[reg_pointer] = data;
				reg_pointer++;
			}
			
			// Send ACK to confirm that Data has been written
			USIDR = 0x00;
			count = 0xE;					
			SET_SDA_OUTPUT();				
			next_state = WRITE_ACK;
			break;	
		
	}
	
	
	current_state = next_state;*/
	SET_COUNT(count); 
 }




/**********************************************************************************************************
 **********************************************************************************************************
 ***********************************  Private Helper Functions  *******************************************
 **********************************************************************************************************
 *********************************************************************************************************/


// Start
//
//		Place a Start Condition on the I2C bus to begin a transaction
//
static void start(void){
	
		

	// Start Condition [Refer to I2C rev 6.0 for timing specifications] 
	SET_SCL_INPUT();	
	SET_SCL_HIGH();			// Release the clock and allow line to pull up to VCC
		
	while(!SCL_HIGH);
	
	SET_SDA_OUTPUT();
	SET_SCL_OUTPUT();
	SETUP_START_COND();
	SET_SDA_LOW();
	HOLD_START_COND();
	SET_SCL_LOW();

	// Needed to infer data available on SDA 
	// [Refer to ATtiny44A Datasheet p.127 2nd Paragraph]
	SET_SDA_HIGH();	
}


// Transfer
// 		
//		After loading the Data Register (USIDR) with a byte
//		transfer it to the I2C bus
//		Acknowledgement from slave is taken care of 
static uint8_t transfer(uint8_t byte){
	
	USIDR = byte;

	// Transfer Byte
	SET_SDA_OUTPUT();	// Set the Data Line to output
	clock_edges(16);	// Run through 8 clock cycles
	HOLD_CLK_LOW();		

	// Check for Acknowledgement from Slave
	SET_SDA_INPUT();
	clock_edges(2);

	// Reset the bus if no Acknowledgement is seen
	if (USIDR & 0x01){
		SET_SCL_HIGH();
		SET_SDA_HIGH();
		return 1;
	}

	return 0;
}


// Receive
// 		
//		After loading the Data Register (USIDR) with a byte
//		transfer it to the I2C bus
//		Acknowledgment from slave is taken care of 
static uint8_t receive(bool endTransmission){
	
	uint8_t received_packet;

	// Read Byte from slave
	//SET_SDA_INPUT();			// Set the Data Line to output
	clock_edges(16);			// Run through 8 clock cycles
	received_packet = USIDR;	// Transfer Data to a temporary variable
	//HOLD_CLK_LOW();		
	SET_SDA_OUTPUT();


	// If Master/Slave is requesting another byte, send ACK to the bus
	if(!endTransmission){ACK();}
	else {NACK();}
		
	return received_packet;
}


// Clock Edges
//
//		Control the Clock line from the master
//		Responsible for generating the clock edges on the SCL line
//
static void clock_edges(uint8_t edges){
	USISR = 0xF0 | (16 - edges);  // Clear the start flag & Configure the counter to overflow correctly
	//SET_SCL_INPUT();
	
	// Clocking for a Slave Device
	if (Slave) {
		while(!OVERFLOW);
	}
	// Clocking for a Master Device 
	else{
		while(!OVERFLOW){
			HOLD_CLK_LOW();		// Hold the Clock Low 
			CLOCK();			// Positive Clock Edge + Add to counter + Output Data
			while(!SCL_HIGH);	// Wait for the clock to go high 
			HOLD_CLK_HIGH();	// Hold the Clock High 
			CLOCK();			// Negative Clock Edge + Add to counter + Shift Data
		}
	}
}


// Stop
//
//		Place a Stop Condition on the I2C bus to end a transaction
//
static void stop(void){
	SET_SDA_OUTPUT();
	SET_SDA_LOW();
	HOLD_CLK_LOW();

	SET_SCL_INPUT();		// Release control of the clock
	while(!SCL_HIGH);		
	SETUP_STOP_COND();
	SET_SDA_INPUT();		// Release control of the data line
	while(!SDA_HIGH);
}
