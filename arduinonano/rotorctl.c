#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "rotorctl.h"

int main(void) {
	setuppins();
	home();
	setuprotor();
	easycomm();
	return 0;
}

void easycomm() {
	usart_init();
	// Testing the usart
	while(1) {
		usart_transmit(usart_receive());
	}
}

void usart_init(void) {
	/* Setup serial port */
	// Setup speed
	UBRR0H = (uint8_t)(MYUBRR>>8);
	UBRR0L = (uint8_t)(MYUBRR);
	
	// Set frame format: 8N1
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
	
	// Enable receiver and transmitter
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
}

void usart_transmit(uint8_t data) {
	// Wait for empty transmit buffer
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

uint8_t usart_receive(void) {
	// Wait for new frame
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

void setuprotor(void) {
	TCCR1A = (1<<WGM12) | (1<<CS12); // CTC mode, prescaler 256
	OCR1AH = (uint8_t)(0); // Compare trigger after 8 counts
	OCR1AL = (uint8_t)(8);
	TIMSK1 |= 1<<OCIE1A; // Enable compare interrupt TIMER1_COMP_vect
}

/* This ISR is setup in setuprotor and called all 0.128ms.
   It reads the encoder positions and updates our knowledge of the rotor position.
   It also cares about engaging / disengaging the rotor and controlling the speed.
 */

ISR( TIMER1_COMPA_vect ) {
	// Read Azimuth position
	rotorstate.az_gray_old = rotorstate.az_gray;
	rotorstate.az_bin_old = rotorstate.az_bin;
	uint8_t az_a = (AZ_A_PIN & (1<<AZ_A_NUM))>>AZ_A_PIN;
	uint8_t az_b = (AZ_B_PIN & (1<<AZ_B_NUM))>>AZ_B_PIN;
	rotorstate.az_gray = az_a | (az_b<<1);
	rotorstate.az_bin = gray2bin[rotorstate.az_gray];
	
	// Read Elevation position
	rotorstate.el_gray_old = rotorstate.el_gray;
	rotorstate.el_bin_old = rotorstate.el_bin;
	uint8_t el_a = (EL_A_PIN & (1<<EL_A_NUM))>>EL_A_PIN;
	uint8_t el_b = (EL_B_PIN & (1<<EL_B_NUM))>>EL_B_PIN;
	rotorstate.el_gray = el_a | (el_b<<1);
	rotorstate.el_bin = gray2bin[rotorstate.el_gray];
	
	// Update Azimuth if rotor has moved
	if ((rotorstate.az_bin == 0 && rotorstate.az_bin_old == 3) || rotorstate.az_bin > rotorstate.az_bin_old) {
		// Encoder counting up
		rotorstate.az_movedir = AZ_ENCDIR;
		rotorstate.azsteps += AZ_ENCDIR;
	} else if (rotorstate.az_bin != rotorstate.az_bin_old) {
		// Encoder counting, but not up -> counting down
		rotorstate.az_movedir = -AZ_ENCDIR;
		rotorstate.azsteps -= AZ_ENCDIR;
	} // No else. We want to keep the old direction in memory.
	
	// Update Elevation if rotor has moved
	if ((rotorstate.el_bin == 0 && rotorstate.el_bin_old == 3) || rotorstate.el_bin > rotorstate.el_bin_old) {
		// Encoder counting up
		rotorstate.el_movedir = EL_ENCDIR;
		rotorstate.elsteps += EL_ENCDIR;
	} else if (rotorstate.el_bin != rotorstate.el_bin_old) {
		// Encoder counting, but not up -> counting down
		rotorstate.el_movedir = -EL_ENCDIR;
		rotorstate.elsteps -= EL_ENCDIR;
	} // No else. We want to keep the old direction in memory.
	
	rotorstate.tickcount += 1;
	if ( rotorstate.tickcount == (TICKCOUNT_MAX>>1)) {
		// half of max tickcount
		// renew az speed
		int16_t azdiff = rotorstate.azsteps_want - rotorstate.azsteps;
		if (azdiff == 0) {
			if (rotorstate.az_speed == 0) {
				// All well
			} else { 
				// We move, but have hit the target
				rotorstate.az_speed--;
			}
		} else if (sign(azdiff) == rotorstate.az_movedir) {
			// We are moving in the right direction
			if (abs(azdiff) >= speedsteps[rotorstate.az_speed+1]) {
				rotorstate.az_speed++;
			} else if (abs(azdiff) < speedsteps[rotorstate.az_speed]) {
				rotorstate.az_speed--;
			}
		} else {
			// We are not moving in the right direction
			if (rotorstate.az_speed != 0) {
				rotorstate.az_speed--;
			}
		}
		
		set_azspeed(rotorstate.az_speed);
		if (rotorstate.az_speed == 0) {
			set_azdir(sign(azdiff));
		}
	} else if (rotorstate.az_speed == 1) {
		// only check for precice hit at nearly no speed
		if (rotorstate.azsteps_want - rotorstate.azsteps == 0) {
			rotorstate.az_speed--;
			set_azspeed(rotorstate.az_speed);
		}
	}
	
	if (rotorstate.tickcount == TICKCOUNT_MAX) {
		// max tickcount
		rotorstate.tickcount = 0;
		// renew el speed
		int16_t eldiff = rotorstate.elsteps_want - rotorstate.elsteps;
		if (eldiff == 0) {
			if (rotorstate.el_speed == 0) {
				// All well
			} else { 
				// We move, but have hit the target
				rotorstate.el_speed--;
			}
		} else if (sign(eldiff) == rotorstate.el_movedir) {
			// We are moving in the right direction
			if (abs(eldiff) >= speedsteps[rotorstate.el_speed+1]) {
				rotorstate.el_speed++;
			} else if (abs(eldiff) < speedsteps[rotorstate.el_speed]) {
				rotorstate.el_speed--;
			}
		} else {
			// We are not moving in the right direction
			if (rotorstate.el_speed != 0) {
				rotorstate.el_speed--;
			}
		}
		
		set_elspeed(rotorstate.el_speed);
		if (rotorstate.el_speed == 0) {
			set_eldir(sign(eldiff));
		}
	} else if (rotorstate.el_speed == 1) {
		// only check for precice hit at nearly no speed
		if (rotorstate.elsteps_want - rotorstate.elsteps == 0) {
			rotorstate.el_speed--;
			set_elspeed(rotorstate.el_speed);
		}
	}
}

void home(void) {
	homeaz();
	homeel();
}

void homeaz(void) {
	if ((AZ_SW_PIN & (1<<AZ_SW_NUM))>>AZ_SW_NUM == SW_HIT_END) {
		set_azdir(AZ_CW*AZ_ENCDIR);
		set_azspeed(MAXSPEED);
		while ((AZ_SW_PIN & (1<<AZ_SW_NUM))>>AZ_SW_NUM == SW_HIT_END);
	} else {
		set_azdir(AZ_CCW*AZ_ENCDIR);
		set_azspeed(MAXSPEED);
		while ((AZ_SW_PIN & (1<<AZ_SW_NUM))>>AZ_SW_NUM != SW_HIT_END);
	}
	set_azspeed(0);
}

void homeel(void) {
	if ((EL_SW_PIN & (1<<EL_SW_NUM))>>EL_SW_NUM == SW_HIT_END) {
		set_eldir(EL_UP*EL_ENCDIR);
		set_elspeed(MAXSPEED);
		while ((EL_SW_PIN & (1<<EL_SW_NUM))>>EL_SW_NUM == SW_HIT_END);
	} else {
		set_eldir(EL_DN*EL_ENCDIR);
		set_elspeed(MAXSPEED);
		while ((EL_SW_PIN & (1<<EL_SW_NUM))>>EL_SW_NUM != SW_HIT_END);
	}
	set_elspeed(0);
}

void set_azspeed(uint8_t speed) {
	if (speed > 0) {
		AZ_BRK_PORT &= ~(1<<AZ_DIR_NUM);
	}
	//TODO: handle speed=0 special
	OCR0A = speedpresets[speed];
	if (speed == 0) {
		AZ_BRK_PORT |= (1<<AZ_DIR_NUM);
	}
}

void set_azdir(int8_t dir) {
	dir *= AZ_ENCDIR;
	if (dir <= 0) {
		AZ_DIR_PORT &= ~(1<<AZ_DIR_NUM);
    } else {
        AZ_DIR_PORT |= (1<<AZ_DIR_NUM);
    }
}

void set_elspeed(uint8_t speed) {
	if (speed > 0) {
		EL_BRK_PORT &= ~(1<<EL_DIR_NUM);
	}
	//TODO: handle speed=0 special
	OCR2A = speedpresets[speed];
	if (speed == 0) {
		EL_BRK_PORT |= (1<<EL_DIR_NUM);
	}
}

void set_eldir(int8_t dir) {
	dir *= EL_ENCDIR;
	if (dir <= 0) {
		EL_DIR_PORT &= ~(1<<EL_DIR_NUM);
    } else {
        EL_DIR_PORT |= (1<<EL_DIR_NUM);
    }
}

void setuppins(void) {
	// Configure inputs
	AZ_A_DDR &= ~(1<<AZ_A_NUM);
	AZ_A_PORT |= (1<<AZ_A_NUM);
	AZ_B_DDR &= ~(1<<AZ_B_NUM);
	AZ_B_PORT |= (1<<AZ_B_NUM);
	AZ_SW_DDR &= ~(1<<AZ_SW_NUM);
	AZ_SW_PORT |= (1<<AZ_SW_NUM);
	EL_A_DDR &= ~(1<<EL_A_NUM);
	EL_A_PORT |= (1<<EL_A_NUM);
	EL_B_DDR &= ~(1<<EL_B_NUM);
	EL_B_PORT |= (1<<EL_B_NUM);
	EL_SW_DDR &= ~(1<<EL_SW_NUM);
	EL_SW_PORT |= (1<<EL_SW_NUM);
	
	// Configure outputs
	AZ_DIR_DDR |= (1<<AZ_DIR_NUM);
	AZ_DIR_PORT &= ~(1<<AZ_DIR_NUM);
	AZ_BRK_DDR |= (1<<AZ_BRK_NUM);
	AZ_BRK_PORT |= (1<<AZ_DIR_NUM);
	AZ_PWM_DDR |= (1<<AZ_PWM_NUM);
	AZ_PWM_PORT &= ~(1<<AZ_PWM_NUM);
	EL_DIR_DDR |= (1<<EL_DIR_NUM);
	EL_DIR_PORT &= ~(1<<EL_DIR_NUM);
	EL_BRK_DDR |= (1<<EL_BRK_NUM);
	EL_BRK_PORT |= (1<<EL_DIR_NUM);
	EL_PWM_DDR |= (1<<EL_PWM_NUM);
	EL_PWM_PORT &= ~(1<<EL_PWM_NUM);
	
	/* Now all outputs should be turned off and the breaks fastend */
	
	/* Setup PWM for AZ/EL driver */
	// TIMER0 for AZ
	// non-inverting OC0A, Fast PWM, reset at OCR0A
	TCCR0A = (1<<COM0A1) | (1<<WGM01) | (1<<WGM00);
	// Clock divisor 1024 => ~all 10ms reset
	TCCR0B = (1<<CS02) | (1<<CS00);
	
	// TIMER2 for EL
	// non-inverting OC2A, Fast PWM, reset at OCR2A
	TCCR2A = (1<<COM2A1) | (1<<WGM21) | (1<<WGM20);
	// Clock divisor 1024 => ~all 10ms reset
	TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20);
}

int sign(int x) {
    return (x > 0) - (x < 0);
}

