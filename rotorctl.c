#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "rotorctl.h"

int main(void) {
	usart_init();
	setuppins();
	usart_write("Homeing");
	home();
	usart_write("Homed");
	setuprotor();
	easycomm();
	return 0;
}

void easycomm() {
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
		deg += 3600;
		deg -= AZ_OFFSET;
		if (deg >= 3600) {
			deg -= 3600;
		}
		if (deg <= 1800) {
			uint32_t steps = deg * AZ_SCALE;
			steps /= 10;
			rotorstate.azsteps_want = steps;
		}
	}
	/* Return degree */
	
	int32_t steps = rotorstate.azsteps;
	steps *= 10;
	int32_t deg = (steps / AZ_SCALE);
	deg += AZ_OFFSET;
	if (deg >= 3600) {
		deg -= 3600;
	}
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
		deg += 3600;
		deg -= EL_OFFSET;
		if (deg >= 3600) {
			deg -= 3600;
		}
		if (deg <= 400) {
			uint32_t steps = deg * EL_SCALE;
			steps /= 10;
			rotorstate.elsteps_want = steps;
		}
	}
	/* Return degree */
	int32_t steps = rotorstate.elsteps;
	steps *= 10;
	int32_t deg = (steps / EL_SCALE);
	deg += EL_OFFSET;
	if (deg >= 3600) {
		deg -= 3600;
	}
	uint8_t degs[5];
	uint8_t length = encode_deg(deg, degs, 5);
	uint8_t retstr[] = "ELXXX.X";
	for (uint8_t i = 0; i < length; i++) {
		retstr[i+2] = degs[i];
	}
	retstr[length + 2] = ' ';
	usart_transmit_mult(retstr, length + 3);
}

uint8_t encode_deg(uint32_t deg, uint8_t str[], uint8_t maxlength) {
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

uint32_t decode_deg(void) {
	uint32_t deg = 0;
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
	usart_write(versionstr);
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
	UCSR0B |= (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
	SREG |= (1<<SREG_I); // Global interrupt enable
}

void usart_transmit(uint8_t data) {
	// Write data in the transmit buffer
	txbuffer[txbuffwrite] = data;
	txbuffwrite++;
	txbuffwrite %= TXBUFFSIZE;
	//Enable Interrupt
	UCSR0B |= (1<<UDRIE0);
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

ISR ( USART_UDRE_vect ) {
	if (txbuffread == txbuffwrite) {
		// Disable Interrupt
		UCSR0B &= ~(1<<UDRIE0);
		return;
	}
	UDR0 = txbuffer[txbuffread];
	txbuffread++;
	txbuffread %= TXBUFFSIZE;
}

uint8_t usart_receive(void) {
	// Wait for new frame
	while(rxbuffread == rxbuffwrite);
	uint8_t retval = rxbuffer[rxbuffread];
	rxbuffread++;
	rxbuffread %= RXBUFFSIZE;
	return retval;
}

ISR ( USART_RX_vect ) {
	rxbuffer[rxbuffwrite] = UDR0;
	rxbuffwrite++;
	rxbuffwrite %= RXBUFFSIZE;
}

void setuprotor(void) {
	TCCR1B = (1<<WGM12) | (1<<CS12); // CTC mode, prescaler 256
	OCR1A = (uint16_t)(8); // Compare trigger after 8 counts
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
	uint8_t az_a = (AZ_A_PIN & (1<<AZ_A_NUM))>>AZ_A_NUM;
	uint8_t az_b = (AZ_B_PIN & (1<<AZ_B_NUM))>>AZ_B_NUM;
	rotorstate.az_gray = az_a | (az_b<<1);
	rotorstate.az_bin = gray2bin[rotorstate.az_gray];
	//usart_transmit(rotorstate.az_gray + '0');
	//usart_transmit(rotorstate.az_bin + '0');
	//usart_write("");
	
	// Read Elevation position
	rotorstate.el_gray_old = rotorstate.el_gray;
	rotorstate.el_bin_old = rotorstate.el_bin;
	uint8_t el_a = (EL_A_PIN & (1<<EL_A_NUM))>>EL_A_NUM;
	uint8_t el_b = (EL_B_PIN & (1<<EL_B_NUM))>>EL_B_NUM;
	rotorstate.el_gray = el_a | (el_b<<1);
	rotorstate.el_bin = gray2bin[rotorstate.el_gray];
	//usart_transmit(rotorstate.el_gray + '0');
	//usart_transmit(rotorstate.el_bin + '0');
	//usart_write("");
	
	// Update Azimuth if rotor has moved
	if ((rotorstate.az_bin == 0 && rotorstate.az_bin_old == 3) ||
	    (rotorstate.az_bin == 1 && rotorstate.az_bin_old == 0) ||
	    (rotorstate.az_bin == 2 && rotorstate.az_bin_old == 1) ||
	    (rotorstate.az_bin == 3 && rotorstate.az_bin_old == 2)) {
		//usart_write("az cnt up");
		// Encoder counting up
		rotorstate.az_movedir = AZ_ENCDIR;
		rotorstate.azsteps += AZ_ENCDIR;
	} else if ((rotorstate.az_bin == 0 && rotorstate.az_bin_old == 1) ||
	           (rotorstate.az_bin == 1 && rotorstate.az_bin_old == 2) ||
	           (rotorstate.az_bin == 2 && rotorstate.az_bin_old == 3) ||
	           (rotorstate.az_bin == 3 && rotorstate.az_bin_old == 0)) {
		//usart_write("az cnt dn");
		// Encoder counting, but not up -> counting down
		rotorstate.az_movedir = -AZ_ENCDIR;
		rotorstate.azsteps -= AZ_ENCDIR;
	} else if ((rotorstate.az_bin == 0 && rotorstate.az_bin_old == 2) ||
	           (rotorstate.az_bin == 1 && rotorstate.az_bin_old == 3) ||
	           (rotorstate.az_bin == 2 && rotorstate.az_bin_old == 0) ||
	           (rotorstate.az_bin == 3 && rotorstate.az_bin_old == 1)) {
		// Must have skipped a step, this should not be able to happen
		usart_write("ALskippedaz");
	} // No else. We want to keep the old direction in memory.
	 
	// Update Elevation if rotor has moved
	if ((rotorstate.el_bin == 0 && rotorstate.el_bin_old == 3) ||
	    (rotorstate.el_bin == 1 && rotorstate.el_bin_old == 0) ||
	    (rotorstate.el_bin == 2 && rotorstate.el_bin_old == 1) ||
	    (rotorstate.el_bin == 3 && rotorstate.el_bin_old == 2)) {
		//usart_write("el cnt up");
		// Encoder counting up
		rotorstate.el_movedir = EL_ENCDIR;
		rotorstate.elsteps += EL_ENCDIR;
	} else if ((rotorstate.el_bin == 0 && rotorstate.el_bin_old == 1) ||
	           (rotorstate.el_bin == 1 && rotorstate.el_bin_old == 2) ||
	           (rotorstate.el_bin == 2 && rotorstate.el_bin_old == 3) ||
	           (rotorstate.el_bin == 3 && rotorstate.el_bin_old == 0)) {
		//usart_write("el cnt dn");
		// Encoder counting, but not up -> counting down
		rotorstate.el_movedir = -EL_ENCDIR;
		rotorstate.elsteps -= EL_ENCDIR;
	} else if ((rotorstate.el_bin == 0 && rotorstate.el_bin_old == 2) ||
	           (rotorstate.el_bin == 1 && rotorstate.el_bin_old == 3) ||
	           (rotorstate.el_bin == 2 && rotorstate.el_bin_old == 0) ||
	           (rotorstate.el_bin == 3 && rotorstate.el_bin_old == 1)) {
		// Must have skipped a step, this should not be able to happen
		usart_write("ALskippedel");
	} // No else. We want to keep the old direction in memory.
	
	// Keep care, not to hit the end
	if (rotorstate.azsteps > AZ_MAXSTEPS || rotorstate.azsteps < 0) {
		if (rotorstate.azsteps > AZ_MAXSTEPS + 2*AZ_SCALE || rotorstate.azsteps < AZ_SCALE * -2) { // Overshot massively, stop and reset
			set_azspeed(0);
			// Trigger Watchdog
			WDTCSR = (1<<WDCE) | (1<<WDE);
			while(1);
		}
		if (!(rotorstate.azsteps_want > 0 && rotorstate.azsteps_want <= AZ_MAXSTEPS)) { // Well, other than this shouldn't happen, but...
			rotorstate.azsteps_want = rotorstate.azsteps > AZ_MAXSTEPS ? AZ_MAXSTEPS : 0;
		}
	}

	if (rotorstate.elsteps > EL_MAXSTEPS || rotorstate.elsteps < 0) {
		if (rotorstate.elsteps > EL_MAXSTEPS + 2*EL_SCALE || rotorstate.elsteps < EL_SCALE * -2) { // Overshot massively, stop and reset
			set_elspeed(0);
			// Trigger Watchdog
			WDTCSR = (1<<WDCE) | (1<<WDE);
			while(1);
		}
		if (!(rotorstate.elsteps_want > 0 && rotorstate.elsteps_want <= EL_MAXSTEPS)) { // Well, other than this shouldn't happen, but...
			rotorstate.elsteps_want = rotorstate.elsteps > EL_MAXSTEPS ? EL_MAXSTEPS : 0;
		}
	}
	
	// Thermal Flag is not present on this board :(
	/*
	uint8_t az_therm = !((AZ_TH_PIN & (1<<AZ_TH_NUM))>>AZ_TH_NUM);
	uint8_t el_therm = !((EL_TH_PIN & (1<<EL_TH_NUM))>>EL_TH_NUM);
	if (az_therm) {
		if (!rotorstate.az_thermtrig) {
			usart_write("ALthermaz");
		}
		rotorstate.az_thermtrig = 1;
	} else {
		rotorstate.az_thermtrig = 0;
	}

	if (el_therm) {
		if (!rotorstate.el_thermtrig) {
			usart_write("ALthermel");
		}
		rotorstate.el_thermtrig = 1;
	} else {
		rotorstate.el_thermtrig = 0;
	}
	
	if (rotorstate.az_thermtrig || rotorstate.el_thermtrig) {
		rotorstate.az_speed = 0;
		rotorstate.el_speed = 0;
		set_azspeed(0);
		set_elspeed(0);
		//usart_write("ALthermshutdown");
		return;
	}
	*/
	
	rotorstate.tickcount += 1;
	if ( rotorstate.tickcount == (TICKCOUNT_MAX>>1)) {
		// half of max tickcount
		// renew az speed
		int32_t azdiff = rotorstate.azsteps_want - rotorstate.azsteps;
		if (int32abs(azdiff) <= 20) {
			//usart_write("diff 0");
			if (rotorstate.az_speed == 0) {
				// All well
			} else { 
				// We move, but have hit the target
				rotorstate.az_speed--;
			}
		} else if (sign(azdiff) == rotorstate.az_movedir) {
			// We are moving in the right direction
			//usart_write("diff, right dir");
			if (rotorstate.az_speed != MAXSPEED && int32abs(azdiff) >= speedsteps[rotorstate.az_speed+1]) {
				//usart_write("incspeed");
				rotorstate.az_speed++;
			} else if (int32abs(azdiff) < speedsteps[rotorstate.az_speed]) {
				//usart_write("decspeed");
				rotorstate.az_speed--;
			}
		} else {
			//usart_write("diff, wrong dir");
			// We are not moving in the right direction
			if (rotorstate.az_speed != 0) {
				//usart_write("decspeed");
				rotorstate.az_speed--;
			}
		}
		
		set_azspeed(rotorstate.az_speed);
		if (rotorstate.az_speed == 0) {
			//usart_write("setsign");
			set_azdir(sign(azdiff));
			rotorstate.az_movedir = sign(azdiff) * AZ_ENCDIR;
		}
	} else if (rotorstate.az_speed == 1) {
		// only check for precice hit at nearly no speed
		if (int32abs(rotorstate.azsteps_want - rotorstate.azsteps) <= 10) {
			rotorstate.az_speed--;
			set_azspeed(rotorstate.az_speed);
		}
	}
	
	if (rotorstate.tickcount == TICKCOUNT_MAX) {
		// max tickcount
		rotorstate.tickcount = 0;
		// renew el speed
		int32_t eldiff = rotorstate.elsteps_want - rotorstate.elsteps;
		//usart_transmit(eldiff + '0');
		//usart_write("");
		if (int32abs(eldiff) <= 20) {
			if (rotorstate.el_speed == 0) {
				// All well
			} else { 
				// We move, but have hit the target
				rotorstate.el_speed--;
			}
		} else if (sign(eldiff) == rotorstate.el_movedir) {
			// We are moving in the right direction
			if (rotorstate.el_speed != MAXSPEED && int32abs(eldiff) >= speedsteps[rotorstate.el_speed+1]) {
				rotorstate.el_speed++;
			} else if (int32abs(eldiff) < speedsteps[rotorstate.el_speed]) {
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
			rotorstate.el_movedir = sign(eldiff) * EL_ENCDIR;
		}
	} else if (rotorstate.el_speed == 1) {
		// only check for precice hit at nearly no speed
		if (int32abs(rotorstate.elsteps_want - rotorstate.elsteps) <= 10) {
			rotorstate.el_speed--;
			set_elspeed(rotorstate.el_speed);
		}
	}
}

void home(void) {
	homeaz();
	homeel();
	rotorstate.azsteps = 0;
	rotorstate.elsteps = 0;
}

void homeaz(void) {
	// Move out of the endswitch
	set_azdir(AZ_CW*AZ_ENCDIR);
	set_azspeed(MAXSPEED);
	while ((AZ_SW_PIN & (1<<AZ_SW_NUM))>>AZ_SW_NUM == SW_HIT_END);
	set_azspeed(0);
	
	// Move in the endswitch
	set_azdir(AZ_CCW*AZ_ENCDIR);
	set_azspeed(MAXSPEED);
	while ((AZ_SW_PIN & (1<<AZ_SW_NUM))>>AZ_SW_NUM != SW_HIT_END);
	set_azspeed(0);
	
	// Slowly move out of the endswitch
	set_azdir(AZ_CW*AZ_ENCDIR);
	set_azspeed(1);
	while ((AZ_SW_PIN & (1<<AZ_SW_NUM))>>AZ_SW_NUM == SW_HIT_END);
	set_azspeed(0);
}

void homeel(void) {
	// Move out of the endswitch
	set_eldir(EL_UP*EL_ENCDIR);
	set_elspeed(MAXSPEED);
	while ((EL_SW_PIN & (1<<EL_SW_NUM))>>EL_SW_NUM == SW_HIT_END);
	set_elspeed(0);
	
	// Move in the endswitch
	set_eldir(EL_DN*EL_ENCDIR);
	set_elspeed(MAXSPEED);
	while ((EL_SW_PIN & (1<<EL_SW_NUM))>>EL_SW_NUM != SW_HIT_END);
	set_elspeed(0);
	
	// Slowly move out of the endswitch
	set_eldir(EL_UP*EL_ENCDIR);
	set_elspeed(1);
	while ((EL_SW_PIN & (1<<EL_SW_NUM))>>EL_SW_NUM == SW_HIT_END);
	set_elspeed(0);
}

void set_azspeed(uint8_t speed) {
	//usart_write("setting speed");
	//usart_transmit(speed + '0');
	if (speed > 0) {
		//if (~(TCCR0A | ~(1<<COM0A1))) usart_write("unbreaking az");
		AZ_BRK_PORT &= ~(1<<AZ_BRK_NUM);
		TCCR0A |= (1<<AZ_PWM_EN); // Enable output
	}
	AZ_PWM_REG = speedpresets[speed];
	if (speed == 0) {
		//if (TCCR0A & (1<<COM0A1)) usart_write("really stopping az");
		TCCR0A &= ~(1<<AZ_PWM_EN); // Really disable output
		AZ_BRK_PORT |= (1<<AZ_BRK_NUM);
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
		EL_BRK_PORT &= ~(1<<EL_BRK_NUM);
		TCCR0A |= (1<<EL_PWM_EN); // Enable output
	}
	EL_PWM_REG = speedpresets[speed];
	//EL_PWM_REG = 255;
	if (speed == 0) {
		TCCR0A &= ~(1<<EL_PWM_EN); // Really disable output
		EL_BRK_PORT |= (1<<EL_BRK_NUM);
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
	// Disable Watchdog
	WDTCSR = (1<<WDCE) | (1<<WDE);
	WDTCSR = 0;
	
	// Configure inputs
	AZ_A_DDR &= ~(1<<AZ_A_NUM);
	AZ_A_PORT |= (1<<AZ_A_NUM);
	AZ_B_DDR &= ~(1<<AZ_B_NUM);
	AZ_B_PORT |= (1<<AZ_B_NUM);
	AZ_SW_DDR &= ~(1<<AZ_SW_NUM);
	AZ_SW_PORT |= (1<<AZ_SW_NUM);
	AZ_TH_DDR &= ~(1<<AZ_TH_NUM);
	AZ_TH_PORT |= (1<<AZ_TH_NUM);
	EL_A_DDR &= ~(1<<EL_A_NUM);
	EL_A_PORT |= (1<<EL_A_NUM);
	EL_B_DDR &= ~(1<<EL_B_NUM);
	EL_B_PORT |= (1<<EL_B_NUM);
	EL_SW_DDR &= ~(1<<EL_SW_NUM);
	EL_SW_PORT |= (1<<EL_SW_NUM);
	EL_TH_DDR &= ~(1<<EL_TH_NUM);
	EL_TH_PORT |= (1<<EL_TH_NUM);
	
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
	// TIMER0 for AZ and EL, COMP A for AZ, COMP B for EL
	// non-inverting, Fast PWM, clear at OCR0A / OCR0B
	// Not enabeling OC0A and OC0B here, done when setting speed
	TCCR0A = (1<<WGM01) | (1<<WGM00);
	// Clock divisor 256 => ~all 0.4ms reset
	TCCR0B = (1<<CS02);
}

int32_t sign(int32_t x) {
    return (x > 0) - (x < 0);
}

int32_t int32abs(int32_t x) {
	if (x > 0)
		return x;
	else
		return -x;
}
