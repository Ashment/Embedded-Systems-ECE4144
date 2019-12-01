
//////////////////////////
// EE4144 ADC Homework  //
//  David Hou | bh1762  //
//////////////////////////

///////////////////////////////////////////////////////////////////////
// 1. Below Code is commented.


void adc_init(void) {
	// Select the V_ref of ADC to Vcc
	ADMUX = (1<<REFS0);

	// 7 = 111. Set ADSRA to 1000 0111
	// Enables the ADC peripheral and set prescalar to 128
	ADCSRA = (1<<ADEN) | 7;
}

int readAdc(char chan) {

	// Set V_ref to Vcc.
	// Set ADC Channel to the last 4 bits of function parameter chan
	ADMUX = (1<<REFS0) | (chan & 0x0f);

	// Write to Start Conversion bit to begin ADC conversion.
	ADCSRA |= (1<<ADSC);

	// Wait for ADSC become 0 => Conversion is finished
	while (ADCSRA & (1<<ADSC));
	// Then, return the result value of the ADC conversion.
	return ADCW;
}


///////////////////////////////////////////////////////////////////////
// 2. Rewritten without register macros below.
// ADCSRA   at  0x7A
// ADMUX    at  0x7C 

// Declaring register addresses
unsigned char *mADCSRA = 0x7A;
unsigned char *mADMUX = 0x7C;
unsigned char *ResH = 0x79;
unsigned char *ResL = 0x78;


void adc_init(void) {
	// ADCSRA to 0100 0000
	*mADCSRA = (1<<6);

	// Set ADMUX to 1000 0111
	*mADMUX= (1<<7) | 0x0111;
}


int readAdc(char chan) {
	// ADMUX to 0100 chan[3:0]
	*mADMUX = (1<<6) | (chan & 0x0f);

	// Write to Start Conversion bit to begin ADC conversion.
	*mADCSRA |= (1<<6);

	// Wait for ADSC become 0 => Conversion is finished
	while (*mADCSRA & (1<<6));
	// Then, return the result value of the ADC conversion.
	return (int)((*ResL) | (*ResH<<8));
}


///////////////////////////////////////////////////////////////////////
// 3. Implementation Question

// Digital Inputs: PC0(A0), PC1(A1), PC2(A2)
// Digital Output: PD4(Pin4)
// PD4 should be HIGH when [A2 A1 A0] > 2. Otherwise, PD4 is LOW.

void setup() {
	// Set Port C to all Input. Set PD4 to output.
	DDRC = 0;
	DDRD = (1<<4);
}

void loop() {
	// Let InpsVal be set to the value of [A2 A1 A0]_binary.
	unsigned char InpsVal = (PORTC | 0x111);

	if(InpsVal > 2){
		PORTD |= (1<<4);
	}else{
		PORTD &= ~(1<<4);
	}

}

















