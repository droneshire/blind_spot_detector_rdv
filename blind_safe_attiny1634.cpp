//#
//# blind_safe_attiny1634.cpp
//#
//# Created: 12/3/2014 8:40:30 PM
//# Author: Ross
//#
//#
//# Filename:		blind_safe_attiny1634.cpp
//#
//# Description:	This program is the main program for the blind spot detector
//#
//#								I/O's
//#				        	  __________
//#				INT_ACC1-----|			|-----SDA
//#				INT_ACC2-----|	 MCU	|-----SCL
//#				 TRIGGER-----|			|-----LED_CTRL1
//#					ECHO-----|			|-----LED_CTRL2
//#					 CE_REG--|			|-----MISO
//#					SPARE----|			|-----MOSI
//#					RESET----|			|-----SCK
//#					SONIC----|			|-----VBATT_OK
//#					         |__________|
//#
//#
//# Authors:     	Ross Yeager
//#
//#END_MODULE_HEADER//////////////////////////////////////////////////////////
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include "Arduino.h"
#include "MMA8452REGS.h"
#include "Ultrasonic.h"
#include "MMA8452.h"

#include "i2c.h"
/************************************************************************/
//PIN/PORT DEFINITIONS:
/************************************************************************/
#define TRIGGER			PORTA0	//OUTPUT
#define CE_REG			PORTA3	//OUTPUT 5V FROM STEP UP CONVERTER 
#define INT2_ACC		PORTA4	//INPUT INTERRUPT DATA READY (PCIE1)
#define LED_CNTL1		PORTA5	//OUTPUT PWM AMBER LED
#define SPARE			PORTA6	//OUTPUT SPARE PIN
#define SONIC_PWR		PORTA7	//OUTPUT TO POWER USS
#define ECHO_3V3		PORTB0	//INPUT FROM USS
#define LED_CNTL2		PORTB3	//OUTPUT PWM RED LED
#define INT1_ACC		PORTC0	//INPUT INTERRUPT MOTION DETECTION (PCIE0)
#define VBATT_OK		PORTC2	//INPUT BATTERY GOOD INDICATOR

#define LED1_PORT			PORTA
#define LED2_PORT			PORTB
#define CE_REG_PORT			PORTA
#define VBATT_OK_PORT		PORTC
#define SONIC_PWR_PORT		PORTA

#define LED1_DDR		DDRA
#define LED2_DDR		DDRB
#define CE_REG_DDR		DDRA

#define MAX_SONAR_DISTANCE		500
#define MIN_SONAR_DISTANCE		200
#define LED_ON_TIME				500
#define SONIC_BRINGUP_TIME_uS	50
#define PWM_PULSE_WIDTH			0xFF	//0-FF defines PWM duty cycle

#define FAST_TOGGLE				1
#define SLOW_TOGGLE				2

#define INT1_PCINT		PCINT12
#define INT2_PCINT		PCINT4
#define INT1_PCIE		PCIE2
#define INT2_PCIE		PCIE0
#define INT1_PCMSK		PCMSK2
#define INT2_PCMSK		PCMSK0
#define INT1_PIN		PINC
#define INT2_PIN		PINA
#define ALL_INTS		10

//BIT-BANG I2C PINS (define in i2c.h)
#define SOFT_SCL		PORTA2	//INOUT
#define SOFT_SDA		PORTA1	//INOUT

/************************************************************************/
//ACCELEROMETER CONSTANTS: these all need to be translated to attributes
/************************************************************************/
#define ACCEL_ADDR				0x1C	// I2C address for first accelerometer
#define SCALE					0x08	// Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
#define DATARATE				0x05	// 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56
#define SLEEPRATE				0x03	// 0=50Hz, 1=12.5, 2=6.25, 3=1.56
#define ASLP_TIMEOUT 			600		// Sleep timeout value until SLEEP ODR if no activity detected 640ms/LSB
#define MOTION_THRESHOLD		16		// 0.063g/LSB for MT interrupt (16 is minimum to overcome gravity effects)
#define MOTION_DEBOUNCE_COUNT 	1		// IN LP MODE, TIME STEP IS 160ms/COUNT
#define I2C_RATE				100
#define NUM_AXIS 3
#define NUM_ACC_DATA (DATARATE * NUM_AXIS)


/************************************************************************/
//PROTOTYPES
/************************************************************************/
void toggle_led(uint8_t num_blinks, uint8_t milliseconds);
void control_leds(bool on, bool amber);
void shut_down_uss_leds(bool ce_led);
void init_accelerometer(void);
void initialize_pins(void);
bool check_moving(void);
long read_mcu_batt(void);
bool clear_acc_ints(void);
void disable_int(uint8_t pcie);
void enable_int(uint8_t pcie);
void deep_sleep_handler(bool still);
void setup(void);

/************************************************************************/
//GLOBAL VARIABLES
/************************************************************************/
static volatile bool got_slp_wake;
static volatile bool got_data_acc;
static volatile bool go_to_sleep;
static volatile bool driving;
static volatile uint8_t intSource;

static bool accel_on;
static unsigned long usec;
static float range;

static int16_t accelCount[3];  				// Stores the 12-bit signed value
MMA8452 accel = MMA8452(ACCEL_ADDR);
Ultrasonic ultrasonic;


//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Main loop
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
int main(void)
{
	setup();
	
    while(1)
    {
		//ACCELEROMETER DATA IS READY
		if (got_data_acc && accel_on)
		{
			got_data_acc = false;
			accel.readAccelData(accelCount);  // Read the x/y/z adc values, clears int
			
			//sequentially check for motion detection
			if((INT1_PIN & (1 << INT1_ACC)))
			{
				driving = check_moving();	//we will go through one more time before, maybe break out here?
				if(!driving)
				{
					shut_down_uss_leds(true);
				}
			}
			else
			{
				driving = true;
			}
			
			go_to_sleep = true;
			
			if(((1 << VBATT_OK) & VBATT_OK_PORT))
			{
				//enable power to sonar
				SONIC_PWR_PORT |= (1 << SONIC_PWR);
				_delay_us(SONIC_BRINGUP_TIME_uS);	//TODO: may need to adjust delay here to optimize bringup time
				
				range = 0;
				disable_int(ALL_INTS);
				usec = ultrasonic.timing(50000);
				range = ultrasonic.convert(usec, ultrasonic.CM);
				
				if(range < MAX_SONAR_DISTANCE && range > MIN_SONAR_DISTANCE)
				{
					control_leds(true, true);	// amber LED on, normal use

					//turn off the power to sonar only
					shut_down_uss_leds(false);
					
					_delay_ms(LED_ON_TIME);	//TODO: some sort of sleep mode during this time?
				}

				enable_int(ALL_INTS);
			}
			else
			{
				control_leds(true, false);	// if low battery, leave red light on
			}
			
			shut_down_uss_leds(((1 << VBATT_OK) & VBATT_OK_PORT));	// if low battery, leave red light on
		}
		
		//ACCELEROMETER CHANGED INTO SLEEP/AWAKE STATE
		if(got_slp_wake)
		{
			got_slp_wake = false;
			driving = check_moving();
			if(driving)
			{
				//handle_battery_mgmt();	//TODO: battery management
			}
			go_to_sleep = true;		//go into driving state mode
		}
		
		//UNIT NEEDS TO GO TO DEEP SLEEP
		if(go_to_sleep)
		{
			shut_down_uss_leds(true);
			deep_sleep_handler(driving);
		}
    }
}


//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Initialization function.  Configures IO pins, enables interrupts.
//#					Heartbeat checks the accelerometerS.
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void setup()
{
	
	//initialize the pins
	initialize_pins();
	_delay_ms(1500);
	shut_down_uss_leds(true);
	
	sei();								//ENABLE EXTERNAL INTERRUPT FOR WAKEUP
	got_slp_wake = false;
	got_data_acc = false;
	go_to_sleep = false;
	driving = false;	
	
	ADCSRA = 0;	//disable ADC
	
	if (accel.readRegister(WHO_AM_I) == 0x2A) 						// WHO_AM_I should always be 0x2A
	{
		accel_on = true;
	}
	else
	{
		accel_on = false;
		while(1){toggle_led(1, FAST_TOGGLE);}
	}
	
	init_accelerometer();
	_delay_ms(1500);
	toggle_led(2, SLOW_TOGGLE);
}


//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Toggles the RED LED specified number of times at specified frequency
//#
//# Parameters:		Number of blinks, length in ms of blink
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void toggle_led(uint8_t num_blinks, uint8_t milliseconds)
{
	LED2_DDR |= (1 << LED_CNTL2);
	
	for(int i = 0; i< num_blinks; i++)
	{
		control_leds(true, false);	//RED LED on at PWM defined brightness
		if(milliseconds == FAST_TOGGLE)
			_delay_ms(150);
		else
			_delay_ms(500);
			
		control_leds(false, false);	//RED LED off
		if(milliseconds == FAST_TOGGLE)
			_delay_ms(150);
		else
			_delay_ms(500);
	}
	
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	This checks to see if we are moving or not
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
bool check_moving()
{
	bool still = false;
	intSource= accel.readRegister(INT_SOURCE) & 0xFE; //we don't care about the data interrupt here
	accel.readRegister(FF_MT_SRC);	//CLEAR MOTION INTERRUPT

	switch(intSource)
	{
		case 0x84:		//MOTION AND SLEEP/WAKE INTERRUPT (if even possible?)
		case 0x80:		//SLEEP/WAKE INTERRUPT
		{
			uint8_t sysmod = accel.readRegister(SYSMOD);
			//accel.readRegister(FF_MT_SRC);	//CLEAR MOTION INTERRUPT

			if(sysmod == 0x02)    		//SLEEP MODE
			still = true;
			else if(sysmod == 0x01)  	//WAKE MODE
			still = false;
			break;
		}
		case 0x04:						//MOTION INTERRUPT
		default:
		break;
	}
	clear_acc_ints();			//clear interrupts at the end of this handler

	return (still ? false : true);
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	This powers down the ultrasonic and led power and turns off
//#					the battery reading voltage divider
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void shut_down_uss_leds(bool leds)
{
	if(leds)
	{
		//turn off the LED and led power
		control_leds(false, true);
	}
	//disable USS
	SONIC_PWR_PORT &= ~(1 << SONIC_PWR);
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Turns on or off the LED's using the specified PWM brightness
//#
//# Parameters:		on => true = LED on, false = LED off
//#					amber => true = amber LED, false = red LED
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void control_leds(bool on, bool amber)
{
	// make sure LED_CNTLx are outputs
	LED1_DDR |= (1 << LED_CNTL1);	// Output on LED_CNTL1
	LED2_DDR |= (1 << LED_CNTL2);	// Output on LED_CNTL2
	
	if(amber)
	{	
		// always shut down the RED LED
		TCCR1A = 0; // normal mode
		TCCR1B = 0;	// turn off PWM
		LED2_PORT &= ~(1 << LED_CNTL2);	// turn off the pin
		
		if(on)
		{
			TCCR0A = (1 << COM0A1) | (1 << WGM00);	// phase correct PWM mode
			OCR0B = PWM_PULSE_WIDTH;				// PWM pulse width
			TCCR0B = (1 << CS01);					// clock source = CS1[2:0] { 0 = off | 1 => /8 | 2 => /64 | 3 => /256 | 4 => /1024}, start PWM
		}
		else
		{
			TCCR0A = 0; // normal mode
			TCCR0B = 0;	// turn off PWM
			LED1_PORT &= ~(1 << LED_CNTL1);	// turn off the pin
		}
	}
	else
	{
		// always shut down the AMBER LED
		TCCR0A = 0; // normal mode
		TCCR0B = 0;	// turn off PWM
		LED1_PORT &= ~(1 << LED_CNTL1);	// turn off the pin
		
		if(on)
		{
			TCCR1A = (1 << COM1A1) | (1 << WGM10);		// phase correct PWM mode
			OCR1B = PWM_PULSE_WIDTH;					// PWM pulse width on OC1B
			TCCR1B = (1 << CS11);						// clock source = CS1[2:0] { 0 = off | 1 => /8 | 2 => /64 | 3 => /256 | 4 => /1024}, start PWM
		}
		else
		{
			TCCR1A = 0; // normal mode
			TCCR1B = 0;	// turn off PWM
			LED2_PORT &= ~(1 << LED_CNTL2);	// turn off the pin
		}
	}
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Initializes the accelerometer and enables the interrupts
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void init_accelerometer(){
	
	enable_int(ALL_INTS);
	
	accel.initMMA8452(SCALE, DATARATE, SLEEPRATE, ASLP_TIMEOUT, MOTION_THRESHOLD, MOTION_DEBOUNCE_COUNT);
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Sets up the pin configurations
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void initialize_pins()
{
	//outputs
	DDRA =  (1 << TRIGGER) | (1 << CE_REG) | (1 << LED_CNTL1) | (1 << SPARE) | (1 << SONIC_PWR);
	DDRB = (1 << LED_CNTL2);
	PORTA = 0;	
	PORTB = 0;
	
	//inputs
	DDRA &=  ~(1 << INT2_ACC);
	DDRB &=  ~(1 << ECHO_3V3);
	DDRC &=  ~((1 << INT1_ACC) | (1 << VBATT_OK));
	
	//input w/ pull-ups
	PORTA |= (1 << INT2_ACC);
	PORTC |= (1 << INT1_ACC);
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Puts the unit into sleep while enabling proper interrupts to
//#					exit sleep mode.  In normal mode, we want to sleep in between
//#					data ready acquisitions to maximize power.  When no motion is present,
//#					we only want to be woken up by BLE or movement again, not data
//#					ready.
//#
//# Parameters:		driving --> 	false = disable acc data interrupts
//#									true = enable acc data interrupts
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void deep_sleep_handler(bool driving)
{
	got_slp_wake = false;
	got_data_acc = false;
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	cli();
	if(driving)
	{
		enable_int(INT2_PCIE);
		disable_int(INT1_PCIE);
	}
	else
	{
		enable_int(INT1_PCIE);
		disable_int(INT2_PCIE);
	}
	if(!clear_acc_ints())
		return;
	sei();
	sleep_cpu();
	sleep_disable();
	go_to_sleep = false;
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Reads the specified registers to clear all interrupts for the
//#					accelerometer.
//#					INT_SOURCE | FF_MT_SRC | ACCELEROMETER DATA
//#
//# Parameters:		None
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
bool clear_acc_ints()
{
	if(accel.readRegister(INT_SOURCE) == ~0u)
		return false;
	if(accel.readRegister(FF_MT_SRC) == ~0u)
		return false;
	accel.readAccelData(accelCount);
		return true;
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Enables the corresponding interrupt bank.
//#
//# Parameters:		Interrupt bank to enable.  Default is all interrupts enabled.
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void enable_int(uint8_t pcie)
{
	switch(pcie)
	{
		case INT1_PCIE:
		{
			INT1_PCMSK |= (1 << INT1_PCINT);	//INT1_ACC Interrupt
			break;
		}
		case INT2_PCIE:
		{
			INT2_PCMSK |= (1 << INT2_PCINT);	//INT2_ACC Interrupt
			break;
		}
		default:
		{
			INT1_PCMSK |= (1 << INT1_PCINT);
			INT2_PCMSK |= (1 << INT2_PCINT);
			break;
		}
	}
	
	if((pcie == PCIE0) || (pcie == PCIE2))
		GIMSK |= (1 << pcie);
	else
		GIMSK |= ((1 << INT1_PCIE) | (1 << INT2_PCIE));
}


//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	Disables the corresponding interrupt bank.
//#
//# Parameters:		Interrupt bank to disable.  Default is all interrupts disabled.
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
void disable_int(uint8_t pcie)
{
	switch(pcie)
	{
		case PCIE2:
		{
			INT1_PCMSK &= ~(1 << INT1_PCINT);	//INT1_ACC Interrupt
			break;
		}
		case PCIE0:
		{
			INT2_PCMSK &= ~(1 << INT2_PCINT);	//INT2_ACC Interrupt
			break;
		}
		default:
		{
			INT1_PCMSK &= ~(1 << INT1_PCINT);
			INT2_PCMSK &= ~(1 << INT2_PCINT);
			break;
		}
	}
	
	if((pcie == PCIE0) || (pcie == PCIE2))
		GIMSK &= ~(1 << pcie);
	else
		GIMSK = 0;
	
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	ISR for the PCINT2 bus interrupt.  This is an external interrupt from
//#					the accelerometer that is triggered by filtered motion or if the
//#					accelerometer is entering or exiting sleep mode.
//#
//# Parameters:		Interrupt vector
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
ISR(PCINT2_vect)
{
	cli();
	if((INT1_PIN & (1 << INT1_ACC)))
	{
		got_slp_wake = true;
	}
	sei();
}

//#START_FUNCTION_HEADER//////////////////////////////////////////////////////
//#
//# Description: 	ISR for the PCINT0 bus interrupt.  This is an external interrupt from
//#					the accelerometer that is triggered by the accelerometer data being ready.
//#
//# Parameters:		Interrupt vector
//#
//# Returns: 		Nothing
//#
//#//END_FUNCTION_HEADER////////////////////////////////////////////////////////
ISR(PCINT0_vect)
{
	cli();
	if((INT2_PIN & (1 << INT2_ACC)))
	{
		got_data_acc = true;
		disable_int(INT1_PCIE);	//disable motion interrupt and sequential check instead
	}
	sei();
}