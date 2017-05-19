#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "rotorctl.h"

int main(void) {
	setuppins();
	home();
	setuprotor();
	return 0;
}

void setuprotor(void) {
	TCCR0A = (1<<WGM01) | (1<<CS02); // CTC mode, prescaler 256
	OCR0A = (uint8_t)(8); // Compare trigger after 8 counts
	TIMSK0 |= 1<<OCIE0A; // Enable compare interrupt TIMER0_COMP_vect
}

/* This ISR is setup in setuprotor and called all 0.128ms.
   It reads the encoder positions and updates our knowledge of the rotor position.
   It also cares about engagind / disengaging the rotor.
 */

ISR( TIMER0_COMPA_vect ) {
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
	} else {
		rotorstate.az_movedir = 0;
	}
	
	// Update Elevation if rotor has moved
	if ((rotorstate.el_bin == 0 && rotorstate.el_bin_old == 3) || rotorstate.el_bin > rotorstate.el_bin_old) {
		// Encoder counting up
		rotorstate.el_movedir = EL_ENCDIR;
		rotorstate.elsteps += EL_ENCDIR;
	} else if (rotorstate.el_bin != rotorstate.el_bin_old) {
		// Encoder counting, but not up -> counting down
		rotorstate.el_movedir = -EL_ENCDIR;
		rotorstate.elsteps -= EL_ENCDIR;
	} else {
		rotorstate.el_movedir = 0;
	}
	
	// Check current position against our wishes
	int16_t azdiff = rotorstate.azsteps_want - rotorstate.azsteps;
	if (abs(azdiff) > 5) {
		if (rotorstate.az_movedir != sign(azdiff)) {
			engage_az(sign(azdiff)*AZ_ENCDIR);
		}
	} else {
		stop_az();
	}
	
	int16_t eldiff = rotorstate.elsteps_want - rotorstate.elsteps;
	if (abs(eldiff) > 5) {
		if (rotorstate.el_movedir != sign(eldiff)) {
			engage_el(sign(eldiff)*EL_ENCDIR);
		}
	} else {
		stop_el();
	}
}

void home(void) {
	homeaz();
	homeel();
}

void homeaz(void) {
	if ((AZ_SW_PIN & (1<<AZ_SW_NUM))>>AZ_SW_NUM == SW_HIT_END) {
		engage_az(AZ_CW);
		while ((AZ_SW_PIN & (1<<AZ_SW_NUM))>>AZ_SW_NUM == SW_HIT_END);
	} else {
		engage_az(AZ_CCW);
		while ((AZ_SW_PIN & (1<<AZ_SW_NUM))>>AZ_SW_NUM != SW_HIT_END);
	}
	stop_az();
}

void homeel(void) {
	if ((EL_SW_PIN & (1<<EL_SW_NUM))>>EL_SW_NUM == SW_HIT_END) {
		engage_el(EL_UP);
		while ((EL_SW_PIN & (1<<EL_SW_NUM))>>EL_SW_NUM == SW_HIT_END);
	} else {
		engage_el(EL_DN);
		while ((EL_SW_PIN & (1<<EL_SW_NUM))>>EL_SW_NUM != SW_HIT_END);
	}
	stop_el();
}

void engage_az(uint8_t dir) {
	if (dir == 0) {
		AZ_DIR_PORT &= ~(1<<AZ_DIR_NUM);
	} else {
		AZ_DIR_PORT |= (1<<AZ_DIR_NUM);
	}
	AZ_BRK_PORT &= ~(1<<AZ_DIR_NUM);
	AZ_PWM_PORT |= (1<<AZ_PWM_NUM);
}

void stop_az(void) {
	AZ_PWM_PORT &= ~(1<<AZ_PWM_NUM);
	AZ_BRK_PORT |= (1<<AZ_DIR_NUM);
}

void engage_el(uint8_t dir) {
	if (dir == 0) {
		EL_DIR_PORT &= ~(1<<EL_DIR_NUM);
	} else {
		EL_DIR_PORT |= (1<<EL_DIR_NUM);
	}
	EL_BRK_PORT &= ~(1<<EL_DIR_NUM);
	EL_PWM_PORT |= (1<<EL_PWM_NUM);
}

void stop_el(void) {
	EL_PWM_PORT &= ~(1<<EL_PWM_NUM);
	EL_BRK_PORT |= (1<<EL_DIR_NUM);
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
}

int sign(int x) {
    return (x > 0) - (x < 0);
}

