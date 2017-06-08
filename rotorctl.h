#ifndef ROTORCTL_H
#define ROTORCTL_H

/* Physical pins */

/* Encoder inputs and endswitches */

// AZ A: Pin D9
#define AZ_A_PORT PORTB
#define AZ_A_DDR DDRB
#define AZ_A_PIN PINB
#define AZ_A_NUM 1

// AZ B: Pin D8
#define AZ_B_PORT PORTB
#define AZ_B_DDR DDRB
#define AZ_B_PIN PINB
#define AZ_B_NUM 0

// AZ endswitch: D4
#define AZ_SW_PORT PORTD
#define AZ_SW_DDR DDRD
#define AZ_SW_PIN PIND
#define AZ_SW_NUM 4

// AZ thermal: D13
#define AZ_TH_PORT PORTB
#define AZ_TH_DDR DDRB
#define AZ_TH_PIN PINB
#define AZ_TH_NUM 5

// EL A: Pin D2
#define EL_A_PORT PORTD
#define EL_A_DDR DDRD
#define EL_A_PIN PIND
#define EL_A_NUM 2

// EL B: Pin D3
#define EL_B_PORT PORTD
#define EL_B_DDR DDRD
#define EL_B_PIN PIND
#define EL_B_NUM 3

// EL endswitch: D7
#define EL_SW_PORT PORTD
#define EL_SW_DDR DDRD
#define EL_SW_PIN PIND
#define EL_SW_NUM 7

// EL thermal: D10
#define EL_TH_PORT PORTB
#define EL_TH_DDR DDRB
#define EL_TH_PIN PINB
#define EL_TH_NUM 2

/* Driver outputs */

// AZ direction: A1
#define AZ_DIR_PORT PORTC
#define AZ_DIR_DDR DDRC
#define AZ_DIR_NUM 1

// AZ break: A0
#define AZ_BRK_PORT PORTC
#define AZ_BRK_DDR DDRC
#define AZ_BRK_NUM 0

// AZ PWM: D5 OC0B
#define AZ_PWM_PORT PORTD
#define AZ_PWM_DDR DDRD
#define AZ_PWM_NUM 5
#define AZ_PWM_EN COM0B1
#define AZ_PWM_REG OCR0B

// EL break: D11
#define EL_BRK_PORT PORTB
#define EL_BRK_DDR DDRB
#define EL_BRK_NUM 3

// EL direction: D12
#define EL_DIR_PORT PORTB
#define EL_DIR_DDR DDRB
#define EL_DIR_NUM 4

// EL PWM: D6 OC0A
#define EL_PWM_PORT PORTD
#define EL_PWM_DDR DDRD
#define EL_PWM_NUM 6
#define EL_PWM_EN COM0A1
#define EL_PWM_REG OCR0A

/* Serial port settings */
#define BAUD 57600
#define MYUBRR (F_CPU/16/BAUD-1) //Formula c&p from the datasheet

#define TXBUFFSIZE 50
uint8_t txbuffer[TXBUFFSIZE];
volatile uint8_t txbuffread, txbuffwrite;

#define RXBUFFSIZE 20
uint8_t rxbuffer[RXBUFFSIZE];
volatile uint8_t rxbuffread, rxbuffwrite;

/* Informations */

// When is the endswitch hit?
#define SW_HIT_END 0

// Directions
#define AZ_CW 1
#define AZ_CCW 0
#define EL_UP 1
#define EL_DN 0

// Direction, when the encoder counts upwards
// -1 or 1
#define AZ_ENCDIR 1
#define EL_ENCDIR 1

#define AZ_SCALE (int32_t)400 // 400 steps for 1 degree
#define EL_SCALE (int32_t)1600 // 1600 steps for 1 degree

#define AZ_MAXSTEPS (180 * AZ_SCALE)
#define EL_MAXSTEPS (40 * EL_SCALE)

// Offset to zero with one trailing 0. Only positve values.
// e.g. 180 => 1800
#define AZ_OFFSET 0
#define EL_OFFSET 0

char versionstr[] = "VErotorcontrol1.0";

// Max tickcounter, needed for speedcontrol
#define TICKCOUNT_MAX 100

// Max command length
// 10 should be enought
#define CMD_MAX 10
uint8_t cmd[CMD_MAX];
uint8_t cmd_length;

uint8_t gray2bin[4] = { 0b00, 0b01, 0b11, 0b10 };

/* Speed tables */
#define MAXSPEED 5
// Min difference to engage higher speed
uint16_t speedsteps[MAXSPEED+1] = { 0, 1, 10, 50, 200, 1000 };
// Speed for each difference step
uint16_t speedpresets[MAXSPEED+1] = {0, 20, 50, 100, 200, 255};

/* Variables */

typedef struct {
	// Current az/el in steps
	int32_t azsteps;
	int32_t elsteps;
	
	// Wanted az/el in steps
	int32_t azsteps_want;
	int32_t elsteps_want;
	
	/* moveticks is currently unused */
	// Ticks since last degree change
	//uint16_t az_moveticks;
	//uint16_t el_moveticks;
	//
	// As above, but old value
	// Usable as speed indicator
	//uint16_t az_moveticks_old;
	//uint16_t el_moveticks_old;
	
	// Current speedstep
	uint8_t az_speed;
	uint8_t el_speed;
	
	// Last move direction
	int8_t az_movedir;
	int8_t el_movedir;
	
	// Encoder as gray code
	uint8_t az_gray_old;
	uint8_t az_bin_old;
	uint8_t az_gray;
	uint8_t az_bin;
	uint8_t el_gray_old;
	uint8_t el_bin_old;
	uint8_t el_gray;
	uint8_t el_bin;
	
	// Tick counter, needed for speed control
	uint8_t tickcount;
} rotorstate_t;

rotorstate_t rotorstate;

int main(void);
void easycomm();
void exec_cmd(void);
void cmd_az(void);
void cmd_el(void);
uint8_t encode_deg(uint32_t, uint8_t[], uint8_t);
uint32_t decode_deg(void);
void cmd_ve(void);
void read_cmd(void);
void usart_init(void);
void usart_transmit(uint8_t);
void usart_transmit_mult(uint8_t[], uint8_t);
void usart_write(char[]);
uint8_t usart_receive(void);
void setuprotor(void);
void home(void);
void homeaz(void);
void homeel(void);
void set_azspeed(uint8_t);
void set_azdir(int8_t);
void set_elspeed(uint8_t);
void set_eldir(int8_t);
void setuppins(void);
int32_t sign(int32_t);
int32_t int32abs(int32_t);

#endif //ROTORCTL_H
