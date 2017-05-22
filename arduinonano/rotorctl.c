#include <stdlib.h>
#include <string.h>
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
	while(1) {
		read_cmd();
		if (cmd_length < 2) {
			continue; // All commands are at least 2 char long
		}
		exec_cmd();
	}
}

void exec_cmd(void) {
	if (memcmp(cmd, "AZ", 2) == 0)
		cmd_az();
	else if (memcmp(cmd, "EL", 2) == 0)
		cmd_el();
	else if (memcmp(cmd, "VE", 2) == 0)
		cmd_ve();
}

void cmd_az(void) {
	// Is a degree supplied?
	if (cmd_length > 2) {
		/* Set new degree */
		// deg is in *10^1 to avoid floats
		uint32_t deg = decode_deg();
		if (deg <= 1800) {
			uint32_t steps = deg * AZ_SCALE;
			steps /= 10;
			rotorstate.azsteps_want = (uint16_t)steps;
		}
	}
	/* Return degree */
	
	uint32_t steps = (uint32_t)rotorstate.azsteps;
	steps *= 10;
	uint16_t deg = (uint16_t)(steps / AZ_SCALE);
	uint8_t degs[5];
	uint8_t length = encode_deg(deg, degs, 5);
	uint8_t retstr[] = "AZXXX.X";
	for (uint8_t i = 0; i < length; i++) {
		retstr[i+2] = degs[i];
	}
	retstr[length + 2] = ' ';
	usart_transmit_mult(retstr, length + 3);
}

void cmd_el(void) {
	// Is a degree supplied?
	if (cmd_length > 2) {
		/* Set new degree */
		// deg is in *10^1 to avoid floats
		uint32_t deg = decode_deg();
		if (deg <= 400) {
			uint32_t steps = deg * EL_SCALE;
			steps /= 10;
			rotorstate.elsteps_want = (uint16_t)steps;
		}
	}
	/* Return degree */
	uint32_t steps = (uint32_t)rotorstate.elsteps;
	steps *= 10;
	uint16_t deg = (uint16_t)(steps / EL_SCALE);
	uint8_t degs[5];
	uint8_t length = encode_deg(deg, degs, 5);
	uint8_t retstr[] = "ELXXX.X";
	for (uint8_t i = 0; i < length; i++) {
		retstr[i+2] = degs[i];
	}
	retstr[length + 2] = ' ';
	usart_transmit_mult(retstr, length + 3);
}

uint8_t encode_deg(uint16_t deg, uint8_t str[], uint8_t maxlength) {
	uint8_t idx = maxlength - 1;
	// Fill the buffer from the end
	while (1) {
		// Insert . after one decimal place
		if (idx == maxlength-2) {
			str[idx] = '.';
			idx--;
			continue;
		}
		uint8_t part = (uint8_t)(deg % 10);
		str[idx] = part + '0';
		deg /= 10;
		if (deg == 0 && idx < maxlength - 2) {
			break;
		}
		if (idx == 0) {
			break;
		}
		idx--;
	}
	// Move the buffer to the front
	for(uint8_t i = 0; i < maxlength - idx; i++) {
		str[i] = str[i+idx];
	}
	return maxlength - idx;
}

uint16_t decode_deg(void) {
	uint16_t deg = 0;
	uint8_t idx = 2;
	for (; idx < CMD_MAX; idx++) {
		// Non-numeric literal?
		if (cmd[idx] < '0' || cmd[idx] > '9') {
			break;
		}
		deg *= 10;
		deg += cmd[idx] - '0';
	}
	idx++;
	deg *= 10;
	// Valid number left?
	if (idx == CMD_MAX || cmd[idx] < '0' || cmd[idx] > '9' ) {
		return deg;
	}
	deg += cmd[idx] - '0';
	return deg;
}

void cmd_ve(void) {
	usart_transmit_mult(versionstr, versionstr_len);
}

// Read one command from uart into cmd array and return length
void read_cmd(void) {
	cmd_length = 0;
	while (1) {
		uint8_t c;
		c = usart_receive();
		
		// Convert lower to upper letters
		if ((c >= 'a') && (c <= 'z')) {
			c -= ('a'-'A');
		}
		
		// Filter out not-number and not-characters
		if ((c < '.') ||
			((c > '.') && (c < '0')) ||
		    ((c > '9') && (c < 'A')) ||
		    (c > 'Z')) {
			return; // Command fully read
		}
		
		cmd[cmd_length] = c;
		cmd_length++;
		
		// Maximum length read, cmd must end here
		if (cmd_length == CMD_MAX) {
			cmd_length--;
			return;
		}
	}
}

void usart_init(void) {
	/* Setup serial port */
	// Setup speed
	UBRR0H = (uint8_t)(MYUBRR>>8);
	UBRR0L = (uint8_t)(MYUBRR);
	
	// Set frame format: 8N1
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
	
	// Enable receiver and transmitter and receive interrupt
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
}

void usart_transmit(uint8_t data) {
	// Wait for empty transmit buffer
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void usart_transmit_mult(uint8_t data[], uint8_t length) {
	for (uint8_t i = 0; i<length; i++) {
		usart_transmit(data[i]);
	}
}

void usart_write(char string[]) {
	for (uint8_t i = 0; string[i]; i++) {
		usart_transmit((unsigned char)string[i]);
	}
	usart_transmit('\r');
	usart_transmit('\n');
}

uint8_t usart_receive(void) {
	// Wait for new frame
	while(buffread == buffwrite);
	uint8_t retval = buffer[buffread];
	buffread++;
	buffread %= BUFFSIZE;
	return retval;
}

ISR ( USART_RX_vect ) {
	buffer[buffwrite] = UDR0;
	buffwrite++;
	buffwrite %= BUFFSIZE;
}

void setuprotor(void) {
	TCCR1A = (1<<WGM12) | (1<<CS12); // CTC mode, prescaler 256
	OCR1AH = (uint8_t)(0); // Compare trigger after 8 counts
	OCR1AL = (uint8_t)(8);
	TIMSK1 |= 1<<OCIE1A; // Enable compare interrupt TIMER1_COMP_vect
	SREG |= (1<<SREG_I); // Global Interupt enable
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
		TCCR0A |= (1<<COM0A1); // Enable output
	}
	OCR0A = speedpresets[speed];
	if (speed == 0) {
		TCCR0A &= ~(1<<COM0A1); // Really disable output
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
		TCCR2A |= (1<<COM2A1); // Enable output
	}
	OCR2A = speedpresets[speed];
	if (speed == 0) {
		TCCR2A &= ~(1<<COM2A1); // Really disable output
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
	// non-inverting OC0A, Fast PWM, clear at OCR0A
	// Not enabeling OC0A here, done when setting speed
	TCCR0A = (1<<WGM01) | (1<<WGM00);
	// Clock divisor 256 => ~all 0.4ms reset
	TCCR0B = (1<<CS02);
	
	// TIMER2 for EL
	// non-inverting OC2A, Fast PWM, clear at OCR2A
	// Not enabeling OC2A here, done when setting speed
	TCCR2A = (1<<WGM21) | (1<<WGM20);
	// Clock divisor 256 => ~all 0.4ms reset
	TCCR2B = (1<<CS22) | (1<<CS21);
}

int sign(int x) {
    return (x > 0) - (x < 0);
}

