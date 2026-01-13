/*
 * Project.c
 *
 * Description:
 *	This program is designed for Atmega328P microcontroller.
 *	The program implements the following functions:
 *	  - If the on/off switch is on, sound is on and LED brightness is 
 *	    controlled via PWM.
 *	  - The frequency of the sound is transmitted to external terminal every 
 *		two seconds.
 *	  - The potentiometer controls the sound frequency. The frequency can also 
 *		be tuned via external terminal using characters '+' and '-' (e.g. '+' 
 *		means adding 10 Hz to the frequency).
 * 
 * Author : Artturi Isomaa
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdlib.h>

// Clock speed (Using 4MHz so that the Proteus simulation runs smoothly)
#define FOSC 4000000

// Baud rate for USART
#define BAUD 4800

// USART Baud Rate Register, equation from Atmega 328P datasheet USART example
#define MYUBRR FOSC/16/BAUD-1

// Initialization functions
void init(void);
void init_Timer0(void);
void init_Timer1(uint8_t freq);
void init_Timer2(void);
void init_ADC();
void init_USART(uint16_t ubrr);

// Interrupt functions
ISR(INT0_vect);
ISR(TIMER0_COMPA_vect);

// Other functions
uint32_t read_ADC(void);
void USART_transmit(unsigned char data);
unsigned char USART_receive(void);
void fall_asleep(void);
void update_frequency(uint32_t freq);
void handle_button_press(void);
void handle_system_on(void);
void handle_system_off(void);
void update_frequency_from_ADC(void);
void update_frequency_from_USART(void);
void transmit_frequency(void);

// The frequency of the sound
volatile uint32_t sound_freq = 1000;

// Millisecond counter updated by Timer0 Compare Match interrupt (CTC mode)	   
volatile uint32_t ms_counter = 0;

// System state flag. 0 = off (power-down), 1 = on.
volatile uint8_t system_on = 0;

// Indicates that the button interrupt (INT0) has triggered.
volatile uint8_t button_pressed = 0;

// Stores previous ADC value to detect change. At the start some impossible
// value
volatile uint16_t last_adc_value = 0xFFFF;

// Pulse width of the PWM signal (brightness of the LED)
volatile uint8_t pulse_width = 0;

// Direction of LED brightness change (1 = brighter, -1 = dimmer)
volatile int8_t pwm_direction = 1;


int main(void)
{
	init(); // Initialize system

	while (1)
	{
		// Handles the ON/OFF button presses.
		// Toggles the system_on flag if the button is pressed.
		handle_button_press();

		if (system_on) {
			// Changes the state of the system so that the system starts to 
			// operate normally: sounder, LED, ADC and USART turn on.
			// (Also Timers turn on)
			handle_system_on();
		} else {
			// Turn off the sounder and LED and go to sleep (power-down).
			// Also ADC and USART stop because timers don't work in power-down 
			// mode.
			handle_system_off();
		}
	}
}


// INT0 interrupt handler
ISR(INT0_vect) {
	// Debounce to prevent system from turning on immediately during
	// initialization
	_delay_ms(1);
	
	// Set button_pressed flag
	button_pressed = 1;
}


// Timer0 Compare Match interrupt handler
ISR(TIMER0_COMPA_vect)
{
	// Milliseconds update
	ms_counter += 4; // Add 4 ms on each Timer0 interrupt
	
	// Update LED brightness to create breathing effect
	pulse_width += pwm_direction; // Increase or decrease pulse width 
								  // (LED brightness)

	if (pulse_width == 0 || pulse_width == 255)
		pwm_direction = -pwm_direction; // Reverse direction at min/max 
									    // brightness

	OCR2B = pulse_width; // Apply new pulse width to PWM (sets LED brightness)
}


void init(void) {
	// Initialize on/off button
	DDRD &= ~(1<<PD2); // Port D pin 2 as input
	PORTD |= (1<<PD2);  // Activate internal pull-up for button
	
	// Initialize LED
	DDRD |= (1 << PD3); // Port D pin 3 as output
	
	// Initialize Timer0 for counting 2000 ms and implementing LED breathing 
	// effect
	init_Timer0();
	
	// Initialize timer1 for CTC
	init_Timer1(sound_freq);
	
	// Initialize timer2 for PWM
	init_Timer2();
	
	// Initialize USART
	init_USART(MYUBRR);
	
	// Initialize ADC
	init_ADC();
	
	// Initialize INT0 interrupt
	EICRA &= ~(1<<ISC01) & ~(1<<ISC00); // Low level generates an interrupt
	EIMSK |= (1<<INT0); // Enable INT0 interrupt
	
	// Enable global interrupts
	sei();
}


void init_Timer0(void)
{
	// Set CTC mode
	TCCR0A = (1 << WGM01);
	
	// Set prescaler 256 -> timer frequency = 4 MHz / 256 = 15625 Hz
	TCCR0B = (1 << CS02);
	
	// OCR0A = f_timer * t_desired - 1 
	// = 15625 * 0,004 - 1 = 62,5 - 1 = 61,5 -> ~62
	
	OCR0A = 62; // Set compare value for ~4 ms
	TIMSK0 = (1 << OCIE0A); // Enable Timer0 compare match A interrupt
}


void init_Timer1(uint8_t freq) {
	DDRB |= (1 << PB1); // Port B pin 1 (OC1A) as output
	
	// CTC mode (WGM12), prescaler = 8 (CS11)
	TCCR1B = (1 << WGM12) | (1 << CS11);
	
	// Set compare value for (FOSC / (2 * prescaler * freq)) - 1
	OCR1A = (FOSC / (2 * 8 * freq)) - 1; 
}


void init_Timer2(void) {
	// Configure Timer2 for 8-bit Fast PWM on OC2B (PD3)
	
	// Non-inverting mode (COM2B1), Fast PWM (WGM20 + WGM21)
	TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
	
	TCCR2B = (1 << CS21); // Prescaler 8
}


void init_ADC(void) {
	// Select reference voltage to AVCC, input channel = ADC0 (MUX3..0 = 0000)
	ADMUX = (1 << REFS0);
	
	/* 
	   Enable ADC and set prescaler to 64 (ADC clock = FOSC/64 = 62,5 kHz).
	   Datasheet: "...requires an input clock frequency between 
	   50kHz and 200kHz to get maximum resolution."
	*/
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}


uint32_t read_ADC(void) {
	ADCSRA |= (1 << ADSC); // Start conversion
	
	// Wait until conversion is complete (ADSC becomes 0)
	while (ADCSRA & (1 << ADSC));
	
	return ADC; // Return 10-bit ADC result (0–1023)
}


void init_USART(uint16_t ubrr)
{
	// Set baud rate
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	
	// Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	
	// Set frame format: 8 data, 1 stop bit, no parity
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
}


void USART_transmit(unsigned char data)
{
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1<<UDRE0)));
	
	// Put data into buffer, sends the data
	UDR0 = data;
}


unsigned char USART_receive(void)
{
	// Wait for data to be received
	while (!(UCSR0A & (1<<RXC0)));
	
	// Get and return received data from buffer
	return UDR0;
}


void fallAsleep(void) {
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set the power-down sleep mode
	cli(); // Disable interrupts
	sleep_enable(); // Set SE-bit
	sei(); // Enable interrupts
	sleep_cpu(); // SLEEP-instruction
	
	// Entry point after wake up
	
	sleep_disable(); // Reset SE-bit
}


void update_frequency(uint32_t freq) {
	// Update the global sound frequency value
	sound_freq = freq;
	
	/*
	   Recalculate Timer1 compare value for the new frequency.
	   Formula is derived from datasheet: 
	   OCR1A = FOSC / (2 * prescaler * frequency) - 1.
	   Timer1 is running with prescaler = 8.
	*/
	OCR1A = (FOSC / (2 * 8 * sound_freq)) - 1;
}


void handle_button_press(void) {
	if (button_pressed) {
		if (!(PIND & (1 << PD2))) {
			system_on ^= 1; // Toggle ON/OFF
			while (!(PIND & (1 << PD2))); // Wait for button release
		}
		button_pressed = 0; // Reset the flag
	}
}


void handle_system_on(void) {
	TCCR1A |= (1 << COM1A0); // Set sounder ON
	update_frequency_from_ADC(); // If ADC value is changed update the frequency
	
	// If frequency was changed via USART update frequency
	update_frequency_from_USART();
	
	transmit_frequency(); // Transmit the sound frequency via USART every two seconds
}


void handle_system_off(void) {
	TCCR1A &= ~(1 << COM1A0); // Turn off sounder
	
	// The rest of the functionality turns off automatically 
	// when the device enters power-down mode, as the timers are stopped.
	fallAsleep(); // Go to sleep (power-down)
}


void update_frequency_from_ADC(void) {
	uint32_t adc_value = read_ADC(); // Read current ADC value
	
	// Update the frequency of the sound only if the ADC has changed
	if ((adc_value - last_adc_value) != 0) {
		last_adc_value = adc_value;
		
		// Scaling the frequency between 50 and 1000 Hz
		uint32_t new_freq = 50 + ((adc_value * (1000 - 50)) / 1023);
		update_frequency(new_freq);
	}

}


void update_frequency_from_USART(void) {
	// Check if a character has been received via USART and
	// update frequency if so.
	if (UCSR0A & (1 << RXC0)) {
		char c = UDR0; // Read the received character
		
		// Increase the sound frequency
		if (c == '+' && sound_freq <= 990)
		update_frequency(sound_freq + 10);
		
		// Decrease the sound frequency
		else if (c == '-' && sound_freq > 59)
		update_frequency(sound_freq - 10);
	}
}


void transmit_frequency(void) {
	// Transmit the sound frequency via USART every two seconds
	if (ms_counter >= 2000) {
		char buffer[5];
		
		// Convert the sound frequency (int) to string
		itoa(sound_freq, buffer, 10);
		
		// Send the number over USART
		for (uint8_t i = 0; buffer[i] != '\0'; i++)
			USART_transmit(buffer[i]);

		// Send " Hz" and newline
		USART_transmit(' ');
		USART_transmit('H');
		USART_transmit('z');
		USART_transmit('\r');
		USART_transmit('\n');
		ms_counter = 0; // Reset 2-second timer
	}
}