/* Focuser Driver
	Version 1.0 1st August 2018
      1. Receive a serial feed (38400 baud) of focuser commands via the telescope hub
      2. Translate receive commands into AMIS30543 functions (stepper focuser controller)
          2a. 128 microsteps
          2b. maximum 2070mA drive
      3. Power supply: 12 V, 3A
	  4. Drive 7 segment display and 3 LEDs
*/
/* Version Control --------------------------------------------------------------------------------------------------
	Version	Date		Description						Debug State		Debug Date
	1.1		27/10/2018	Release Mark					untested		27/10/2018
	1.1		16/12/2018	Load last position from EEPROM	untested
	1.1.2	03/01/2019	Revised ASCOM support
*/
#include <myeepromanything.h>
#define VERSION_RELEASE	1
#define VERSION_UPDATE	1
/* Functionality Checklist ------------------------------------------------------------------------------------------
	Flashing Run LED
	Flashing GPS LED
	Flashing HUB LED
	On switch on:
		Focuser moves to home (inwards until switch, then outwards slowly until switch)
		Display shows 0 position, then temperature and humidity
	On instruction focuser moves to specified position (Move to), display shows correct position
	On instruction focuser moves specified amount (Move), display shows correct position
	On instruction display turns off then on instruction on again
	Hub receives updates with correct information
*/

// compiler directives ----------------------------------------------------------------------------------------------
//#define DEBUG_OUTPUT
//#define DEBUG_RECEIVED_OUTPUT
//#define DEBUG_BEMF_OUTPUT						// console output bemf messages during main loop
//#define DEBUG_CURRENT_OUTPUT					// console output focuser current messages during main loop
//#define DEBUG_CALIBRATE_OUTPUT					// console output focuser messages during calibration
//#define DEBUG_CALIBRATE_STEP_OUTPUT				// console output step number and current during calibration steps
//#define DEBUG_MOVE_STEP_OUTPUT					// console output step number and current during move steps
//#define DEBUG_TEMPERATURE_OUTPUT				// console output the temperature and humidity
//#define DISPLAY_DEBUG							// console output 
//#define DISPLAY_DEBUG_GPS						// console output the GPS packets
// end of compiler directives ---------------------------------------------------------------------------------------
// definitions ------------------------------------------------------------------------------------------------------
#define amisStepPin			33
#define amisDIRPin			31
#define amisCSPin			29
#define amisCLRPin			27
#define amisERRPin			25
#define amisPORPin			23
#define amisSLAPin			A0
#define DHT22_PIN			44
#define display_DIN			11		// MAX7219 DATA pin		D11 green
#define display_CS			10		// MAX7219 CS/LOAD pin	D10 blue
#define display_CLK			12		// MAX7219 CLK pin		D12 yellow
#define focuser_Home_Pin	42
#define RUNNING_LED_pin		49		// 49 = Green
#define HUB_LED_pin			48		// 48 = Blue
#define GPS_LED_pin			46		// 46 = Red
#define PBSWITCHESPIN       A0            // push button switches
#define INLED               A1            // in and out leds
#define OUTLED              A2
#define BUZZERPIN           A3            // buzzer
#define TEMPPIN             2             // temperature probe on pin 2, use 4.7k pullup
#define HPSWPIN             12            // home position switch is on D12
#define postStepDelayUs		100

//#define focuser_to_HUB_Update_command	0x05
//#define focuser_Home_command			0x10
//#define focuser_Move_command			0x20	
//#define focuser_Move_to_command			0x25
//#define focuser_Display_on_command		0x30
//#define focuser_Display_off_command		0x35
//#define focuser_Halt_command			0xff

#define HOME				0
#define AWAY				48500L

#define ATHOME				0			// status of HOME switch when at home
#define NOTATHOME			1			// status of HOME switch when not at home
#define OUTWARDS			0
#define INWARDS		  		1
#define HUB_serial			Serial1
#define HUB_RXpin			18
#define HUB_TXpin			19
#define HUB_baud			38400
#define HUB_STX				0x02
#define HUB_ETX				0x03

#define GPS_serial			Serial3
#define GPS_BAUD			9600

#define Temperature_Humidity_error	0x1
#define Date_Time_error				0x2
#define Long_Lat_error				0x4
#define Altitude_error				0x8

#define Last_position_address 0x20

#define LED_FLASH_TIME		20           // number of milli seconds to light the LEDs

#define LCDUPDATESTEPCOUNT  15            // the number of steps moved which triggers an lcd update when moving, do not make too small
#define LCDPAGEDISPLAYTIME  25            // time in milliseconds that each lcd page is displayed for (2000-4000)
#define TEMPREFRESHRATE     1000L         // refresh rate between temperature conversions unless an update is requested via serial command
#define HOMESTEPS           200           // Prevent searching for home position switch never returning, this should be > than # of steps between closed and open

// ----------------------------------------------------------------------------------------------------------
// GLOBAL DEFINES
// DO NOT CHANGE


#define EEPROMSIZE          1024          // ATMEGA328P 1024 EEPROM
#define VALIDDATAFLAG       99            // valid eeprom data flag
#define SLOW                0             // motorspeeds
#define MED                 1
#define FAST                2
#define STEP1               1             // step modes
#define STEP2               2
#define STEP4				4
#define STEP8				8
#define STEP16				16
#define STEP32				32
#define STEP64				64
#define STEP128				128

#define MOTORPULSETIME      5             // requires minimum 5uS pulse to step
#define MOVINGIN            0
#define MOVINGOUT           1
#define FOCUSERUPPERLIMIT   2000000000L   // arbitary focuser limit up to 2000000000
#define FOCUSERLOWERLIMIT   1024L         // lowest value that maxsteps can be
#define DEFAULTSTEPSIZE     50.0          // This is the default setting for the step size in microns
#define MINSTEPSIZE         0.001         // this is the minimum step size in microns
#define EEPROMWRITEINTERVAL 10000L        // interval in milliseconds to wait after a move before writing settings to EEPROM, 10s
#define HPSWOPEN            HIGH
#define HPSWCLOSED          LOW
#define DISPLAYPAGETIMEMAX  40
#define DISPLAYPAGETIMEMIN  20


// includes ---------------------------------------------------------------------------------------------------------
#include <TimeLib.h>
#include <Time.h>
#include <SPI.h>
#include <AMIS30543.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MicroNMEA.h>
#include <MemoryFree.h>
//#include <EEPROM.h>
#include <myEEPROM.h>
#include <Bounce2.h>  
#include "D:/Dropbox/Motor_Controller/Ascom_Development/Telescope_Hub/HUB_Commands.h"


// prototypes -------------------------------------------------------------------------------------------------------
bool Check_Hub_Message(void);
void Parse_Hub_Incoming_Message(void);
void Parse_Incoming_gps_packet(void);
void send_update_to_HUB(void);
void EnableDriver();
char readCUR(void);
void nextStep(int, int);
void DisableDriver(void);
void resetDriver(void);
void writeReg(char,char);
char readReg(char);
char readStatusReg(char);
void error(void);
// variables --------------------------------------------------------------------------------------------------------
union sg_long_union {
  unsigned long sg_ulong;
  unsigned int  sg_int[2];
  unsigned char sg_chars[4];
};
union sg_double_union {
	double sg_double;
	unsigned char sg_chars[4];
};
union sg_int_union {
  char sg_chars[2];
  int sg_int;
};
typedef struct {
  unsigned char header;             // [0] STX
  unsigned char command_number;     // [1] Command Number
  unsigned int focuser_parameter;		// [2 - 3]
  unsigned int focuser_direction;	// [4 - 5]
  unsigned char footer;             // [6] ETX
} HUB_incoming_message_structure;
#define HUB_incoming_message_length 7
union HUB_Incoming_Message {
  HUB_incoming_message_structure hub_incoming_message;
  unsigned char sg_chars[HUB_incoming_message_length];
};
HUB_Incoming_Message incoming_hub_message;
typedef struct {
  unsigned char header;				// [0] STX
  unsigned char command_number;		// [1] Command Number
  unsigned char focuser_status;		// [2]
  unsigned int focuser_position;	// [3 - 4]
  double temperature;				// [5 - 8]
  double humidity;					// [9 - 12]
  unsigned char year;				// [13] year
  unsigned char month;				// [14] month
  unsigned char day;				// [15] day
  unsigned char hour;				// [16] hour
  unsigned char minute;				// [17] minute
  unsigned char second;				// [18] second
  double latitude;					// [19,22] latitude
  double longitude;					// [23,26] longitude
  double altitude;					// [27,30] altitude (above mean sea level
  unsigned char footer;				// [31] ETX
} HUB_outgoing_message_structure;
#define HUB_outgoing_message_length 32
union HUB_Outgoing_Message {
  HUB_outgoing_message_structure hub_outgoing_message;
  unsigned char sg_chars[HUB_outgoing_message_length];
};
HUB_Outgoing_Message outgoing_hub_message;
enum regAddr { WR  = 0x0, CR0 = 0x1, CR1 = 0x2, CR2 = 0x3, CR3 = 0x9, SR0 = 0x4, SR1 = 0x5, SR2 = 0x6, SR3 = 0x7, SR4 = 0xA };
volatile bool hub_incoming_message_available = false;
volatile bool incoming_gps_packet_available = false;      // if true there us a gps packet to parse
volatile char gps_c = 0;
unsigned char hub_outptr = 0;
unsigned char hub_inptr = 0;
unsigned char hub_inbuffer[0xff];
unsigned char hub_string_ptr = 0;


struct config_t {
	int validdata;								// if this is 99 then data is valid
	long position;								// last focuser position
	long maximum_step;							// max steps
	double step_size;							// the step size in microns, ie 7.2, minimum value is 0.001 microns
	double step_mode;							// stepping mode, full, half, 1/4, 1/8. 1/16. 1/32 [1.2.4.8.16.32]
	double reverse_direction;					// reverse direction
	double coil_power;							// coil pwr
	double temperature_mode;					// temperature display mode, Celcius=1, Fahrenheit=0
	double pagedisplaytime;						// refresh rate of display - time each page is displayed for
	double step_size_enabled;					// if 1, controller returns step size
	double seven_segment_update_on_move;		// update position on lcd when moving
	double temperature_compensation_enabled;	// indicates if temperature compensation is enabled
	double temperature_coefficient;				// steps per degree temperature coefficient value
	double delay_after_move;					// delay after movement is finished
	double backlash_steps_in;					// number of backlash steps to apply for IN moves
	double backlash_steps_out;					// number of backlash steps to apply for OUT moves
	double focuser_direction;					// keeps track of last focuser move direction
	double backlash_in_enabled;					// enable or disable backlash compensation for IN
	double backlash_out_enabled;				// enable or disable backlash compensation for OUT
	double tc_direction;
} myfocuser;

long previous_millis = 0;
unsigned int current_address = 0;


long last_status_update = 0;
long last_display_update = 0;
unsigned int microsteps_target = 0;
unsigned char microsteps_direction = 0;
unsigned int microsteps_pos = 0;
double current_temperature = 0;
double current_humidity = 0;
unsigned int current_position = 0;
unsigned int last_position = 0;					// will be read from EEPROM address 0x20
unsigned char current_status = 0;
unsigned char last_focuser_Home_switch_state = 0;
unsigned char focuser_Home_switch_state = 0;
long last_focuser_Home_switch_time = 0;
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
char i = 0;
long gps_time_LED_On = 0;
bool gps_status = false;
long running_time_LED_On = (long) 0;
bool running_status = false;
long hub_time_LED_On = 0;
bool hub_status = false;
char gps_nmeaBuffer[100];
int gps_year = 0;
int gps_month = 0;
int gps_day = 0;
int gps_hour = 0;
int gps_minute = 0;
int gps_second = 0;
int gps_valid = 0;
long gps_latitude_long = 0;
double gps_latitude = 0;
char latitude_temp[10];
long gps_longitude_long = 0;
double gps_longitude = 0;
char longitude_temp[10];
long gps_altitude_long = 0;
double gps_altitude = 0;
char altitude_temp[10];
bool gps_timedate_valid = false;
bool gps_longlat_valid = false;
bool gps_altitude_valid = false;
enum {
	display_REG_DECODE = 0x09,
	display_REG_INTENSITY = 0x0A,
	display_REG_SCANLIMIT = 0x0B,
	display_REG_SHUTDOWN = 0x0C,
	display_REG_DISPTEST = 0x0F,
};
enum { OFF = 0, ON = 1 };
bool display_flag = false;
char position[10];
char display_string[9];
const byte DP = 0b10000000;
const byte t = 0b00001111;
const byte h = 0b00010111;
const byte P = 0b01100111;
const byte dash = 0b00000001;
const byte E = 0b01001111;
const byte R = 0b01110111;
const byte S = 0b01011011;
const byte O = 0b01111110;
const byte space = 0b00000000;
bool focuser_display_on = true;				// default to true - display ON
int timer1_counter;
int home_position_switchstate;
int homeposflag;
// instantiations -----------------------------------------------------------------------------------------------------
AMIS30543 stepper;
DHT_Unified dht(DHT22_PIN, DHT22);
sensors_event_t event;
sensor_t sensor;
MicroNMEA gps_nmea(gps_nmeaBuffer, sizeof(gps_nmeaBuffer));
HardwareSerial& gps_serial = GPS_serial;
Bounce home_position_switchbounce = Bounce();             // setup debouncer for hp switch
// ISRs ---------------------------------------------------------------------------------------------------------------
void serialEvent1() { // Hub Serial Port Interrupt Service Routine
	while (HUB_serial.available()) {
		hub_inbuffer[hub_inptr++] = (unsigned char) HUB_serial.read();    // add the received character to the buffer and increment pointer
	}
}
void serialEvent3() { // gps Serial Port Interrupt Service Routine, NEMA Packet Handler
	while (GPS_serial.available()) {
		gps_c = GPS_serial.read();
		if (gps_nmea.process(gps_c)) {
			incoming_gps_packet_available = true;
		}
	}
}
ISR(TIMER1_OVF_vect) {						// interrupt service routine 
	TCNT1 = timer1_counter;					// preload timer
	send_update_to_SEVEN_SEGMENT();
}
// setup --------------------------------------------------------------------------------------------------------------
void setup() {
	byte found = 0;
	unsigned int data_size = 0;
	unsigned int number_locations = 0;

	Serial.begin(38400);				// start the console serial port at 38400 baud
	pinMode(13, OUTPUT);				// define the yellow led pin as an output
	digitalWrite(13, LOW);				// Turn off the yellow LED.
	digitalWrite(amisPORPin, HIGH);		// set the amisPORPin high (not active) before we open it
	pinMode(amisPORPin, OUTPUT);		// define the amis POR pin as an output
	pinMode(amisERRPin, INPUT);			// define the amis ERR pin as an input
	digitalWrite(amisCLRPin, LOW);		// set the amisCLRPin low before we open it
	pinMode(amisCLRPin, OUTPUT);		// define the amisCLRPin as an output
	digitalWrite(amisCSPin, HIGH);		// set the amisCSPin high (active low) before we open it
	pinMode(amisCSPin, OUTPUT);			// define the amisCSPin as an output
	digitalWrite(amisDIRPin, OUTWARDS);	// set the amisDIRPin CW (0) before we open it
	pinMode(amisDIRPin, OUTPUT);		// define the amisDIRPin as an output
	digitalWrite(amisStepPin, LOW);		// set the amisStepPin low before we open it
	pinMode(amisStepPin, OUTPUT);		// define the amisStepPin (STEP) as an output
	pinMode(focuser_Home_Pin, INPUT);
	pinMode(DHT22_PIN, INPUT);			// define the DHT11 pin as an input
	digitalWrite(GPS_LED_pin,LOW);
	pinMode(GPS_LED_pin, OUTPUT);
	digitalWrite(RUNNING_LED_pin,LOW);
	pinMode(RUNNING_LED_pin, OUTPUT);
	digitalWrite(HUB_LED_pin,LOW);
	pinMode(HUB_LED_pin, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(display_DIN, OUTPUT);
	pinMode(display_CS, OUTPUT);
	pinMode(display_CLK, OUTPUT);
	digitalWrite(display_CS, HIGH);
// initialise temperature and humidity sensor ---------------------------------------------------------------------------------------
	dht.begin();
	dht.temperature().getSensor(&sensor);
	dht.humidity().getSensor(&sensor);
// initialise GPS sensor -----------------------------------------------------------------------------------------------------------
	gps_serial.begin(GPS_BAUD, SERIAL_8N1);       // initialise the gps serial port
	gps_serial.flush();
	gps_nmea.setUnknownSentenceHandler(gps_printUnknownSentence);
	MicroNMEA::sendSentence(gps_serial, "$PORZB");										// Clear the list of messages which are sent.
	MicroNMEA::sendSentence(gps_serial, "$PORZB,GGA,1,GLL,1,GSV,1,GSA,1,RMC,1,VTG,1");    // Process only listed messages.
	MicroNMEA::sendSentence(gps_serial, "$PNVGNME,2,9,1");
// initialise motor controller -----------------------------------------------------------------------------------------------------
	SPI.begin();							// begin the MOSI and MISO bus (used to communicate with the amis20543)
	stepper.init(amisCSPin);				// tell the amis driver where it's CS pin is
	EnableDriver();							// initialise and enable the amis controller
	digitalWrite(13, HIGH);					// Turn on the yellow LED.
	position[9] = NULL;						// make the char array into a string by inserting a string terminator
	resetDisplay();							// reset the MAX7219 display
// initialize timer1 ---------------------------------------------------------------------------------------------------------------
	noInterrupts();							// disable all interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	// Set timer1_counter to the correct value for our interrupt interval
	//timer1_counter = 64911;   // preload timer for .01 second interrupt (65536-16MHz/256/100Hz)
	//timer1_counter = 64286;   // preload timer for .02 second interrupt (65536-16MHz/256/50Hz)
	//timer1_counter = 34286;   // preload timer for .5 second interrupt (65536-16MHz/256/2Hz)
	timer1_counter = 3036;		// preload timer for 1 second interrupt (625536-16MHz/256/1Hz)
	TCNT1 = timer1_counter;		// preload timer
	TCCR1B |= (1 << CS12);		// 256 prescaler 
	TIMSK1 |= (1 << TOIE1);		// enable timer overflow interrupt
	interrupts();				// enable all interrupts

	current_address = 0;                          // start at 0 if not found later
	found = 0;
	data_size = sizeof(myfocuser);
	number_locations = EEPROMSIZE / data_size;
	for (int lp1 = 0; lp1 < number_locations; lp1++)
	{
		int address = lp1 * data_size;
		EEPROM_readAnything(address, myfocuser);
		if (myfocuser.validdata == VALIDDATAFLAG) // check to see if the data is valid
		{
			current_address = address;                       // data was erased so write some default values
			found = 1;
		}
	}
	if (found == 1) {
		// set the focuser back to the previous settings
		// done after this in one hit
		// mark current eeprom address as invalid and use next one
		// each time focuser starts it will read current storage, set it to invalid, goto next location and
		// write values to there and set it to valid - so it doesnt always try to use same locations over and
		// over and destroy the eeprom
		// using it like an array of [0-nlocations], ie 100 storage locations for 1k EEPROM
		EEPROM_readAnything(current_address, myfocuser);
		myfocuser.validdata = 0;
		writeEEPROMNow();                       // update values in EEPROM
		current_address += data_size;                // goto next free address and write data
		// bound check the eeprom storage and if greater than last index [0-EEPROMSIZE-1] then set to 0
		if (current_address >= (number_locations * data_size))
			current_address = 0;
		myfocuser.validdata = VALIDDATAFLAG;
		writeEEPROMNow();                       // update values in EEPROM
	}
	else {
		set_focuser_defaults(); 	// set defaults because not found
	}
	myfocuser.temperature_compensation_enabled = 0;        // disable temperature compensation on startup else focuser will auto adjust whilst focusing!

// range check focuser variables
	if (myfocuser.maximum_step < FOCUSERLOWERLIMIT)
		myfocuser.maximum_step = FOCUSERLOWERLIMIT;
	if (myfocuser.position < 0)
		myfocuser.position = 0;
	else if (myfocuser.position > myfocuser.maximum_step)
		myfocuser.position = myfocuser.maximum_step;
	if (myfocuser.step_size < 0)
		myfocuser.step_size = 0;
	else if (myfocuser.step_size > (double) DEFAULTSTEPSIZE)
		myfocuser.step_size = (double) DEFAULTSTEPSIZE;
	if (myfocuser.delay_after_move > 250)
		myfocuser.delay_after_move = 250;
	myfocuser.focuser_direction = myfocuser.focuser_direction & 0x01;
	move_direction = myfocuser.focuser_direction;
	if (myfocuser.temperature_coefficient > 200)
		myfocuser.temperature_coefficient = 200;

#ifdef DEBUG
	Serial.print("myfocuser.position = ");
	Serial.println(myfocuser.position);
#endif
	current_position = myfocuser.position;
	target_position = myfocuser.position;

	if (!myfocuser.coil_power)
		clear_output();
	myfocuser.step_mode = myfocuser.step_mode & 0x01;
	motor_speed = FAST;
	saved_motor_speed = FAST;
	update_motor_speed_delay();
	write_now = 1;                             // ensure validated values are saved

	Home();									// Position the focuser to home
	Move_to(Get_LastPosition());			// move to the last position
	send_update_to_SEVEN_SEGMENT();
} // end of setup -----------------------------------------------------------------------------------------------------
// main loop ----------------------------------------------------------------------------------------------------------
void loop() {
	if (Check_Hub_Message() == true) Parse_Hub_Incoming_Message();	// if a hub message has arrived, parse it
	if (millis() > (last_status_update + 1000)) {					// send an update to the HUB every second
		if (focuser_display_on == true) {							// flash the running led
			running_status = true;
			running_time_LED_On = millis();
			digitalWrite(RUNNING_LED_pin, HIGH);
		}
		last_status_update = millis();
		send_update_to_HUB();										// send an update message to the HUB
	}
	if (millis() > (last_display_update + 2000)) {					// send an update to the 7 Segment every 2 seconds
		last_display_update = millis();
		if (focuser_display_on == true) {
			send_update_to_SEVEN_SEGMENT();
		}
		else {
			set_register(display_REG_SHUTDOWN, OFF);				// turn off display
		}
	}
	if (incoming_gps_packet_available == true) Parse_Incoming_gps_packet();       // if a gps packet has arrived parse it
	if ((gps_status == true) && (millis() >= gps_time_LED_On + LED_FLASH_TIME)) {	// switch the gps led off
		digitalWrite(GPS_LED_pin, LOW);
		gps_status = false;
	}
	if ((running_status == true) && (millis() >= running_time_LED_On + LED_FLASH_TIME)) {	// switch the running led off
		digitalWrite(RUNNING_LED_pin, LOW);
		running_status = false;
	}
	if ((hub_status == true) && (millis() >= hub_time_LED_On + LED_FLASH_TIME)) {	// switch the hub led off
		digitalWrite(HUB_LED_pin, LOW);
		hub_status = false;
	}
	if (queue.count() >= 1) {                 // check for serial command
		processCommand();
	}
	digitalWrite(INLED, 0);                // turn off the IN/OUT LEDS and BUZZER
	digitalWrite(OUTLED, 0);
	digitalWrite(BUZZERPIN, 0);
	if (myfocuser.temperature_compensation_enabled == 0) {
		int PBVal = readpbswitches(PBSWITCHESPIN);
		if (PBVal != 0) {
			delay(50);                             // wait small delay in case user is holding pb down
			PBVal = readpbswitches(PBSWITCHESPIN);
			if (PBVal != 0) {
				switch (PBVal) {                      // now check the pbval using a switch for 1 2 and 3
					case 1:                             // toggle sw1 is ON and 2 is off
														// move IN
						move_direction = MOVINGIN;
						myfocuser.focuser_direction = move_direction;
						move_started = 1;
						is_moving = 1;
						target_position--;
						if (target_position < 0) target_position = 0;
						update_position_seven_segment();
						break;
					case 2:                             // toggle sw2 is ON and SW1 is OFF
														// move OUT
						move_direction = MOVINGOUT;
						myfocuser.focuser_direction = move_direction;
						move_started = 1;
						is_moving = 1;
						target_position = target_position + 1;
						if (target_position > myfocuser.max_step) target_position = myfocuser.max_step;
						update_position_seven_segment();
						break;
					case 3:                             // toggle sw1 and sw2 are ON
						digitalWrite(BUZZERPIN, 1);       // turn on buzzer
						while (read_pb_switches(PBSWITCHESPIN) == 3)  // wait for pb to be released
							;
						current_position = 0;
						target_position = 0;
						is_moving = 0;
						digitalWrite(BUZZERPIN, 0);       // turn off buzzer
						break;
					default:
															// do nothing
						break;
				} // end of switch
			}
		}
	}
	if (myfocuser.temperature_compensation_enabled == 0) {
		if (jogging == 1) {
			move_started = 1;
			is_moving = 1;
			if (jogging_direction == 0) {				// move IN
				move_direction = MOVINGIN;
				myfocuser.focuser_direction = move_direction;
				move_started = 1;
				target_position--;
				if (target_position < 0)
					target_position = 0;
				update_position_seven_segment();
			}else{										// move OUT
				move_direction = MOVINGOUT;
				myfocuser.focuser_direction = move_direction;
				move_started = 1;
				target_position++;
				if (target_position > myfocuser.max_step)
					target_position = myfocuser.max_step;
				update_position_seven_segment();
			}
		}
	}
	if (target_position != current_position) { 	// Move the position by a single step if target <> current position
		is_moving = 1;                        // focuser is moving
		flag_eeprom_update();
		if (motor_speed_change == 1) {
			long nearing_home_position = current_position - target_position; 			// Slow down if approaching home position
			nearing_home_position = abs(nearing_home_position);
			if (nearing_home_position < tsw_threshold) {
				motor_speed = SLOW;                           // slow
				update_motor_speed_delay();
			}
		}
		if (targetPosition < currentPosition) {	// Going Anticlockwise to lower position
			anticlockwise();
			current_position--;
		}
		if (targetPosition > currentPosition) { 		// Going Clockwise to higher position}
			clockwise();
			current_position++;
		}

		hpswbounce.update();            // we need to call update to read home position switch state, no interrupts are used
		hpswstate = hpswbounce.read();  // reads the home position switch state

		// if switch state = CLOSED and currentPosition != 0
		// need to back OUT a little till switch opens and then set position to 0
		if ((hpswstate == HPSWCLOSED) && (currentPosition != 0)) {
			is_moving = 1;
			// need to back OUT a little till switch opens and then set position to 0
			settohome();
			is_moving = 0;
		}
		// else if switch state = CLOSED and Position = 0
		// need to back OUT a little till switch opens and then set position to 0
		else if ((hpswstate == HPSWCLOSED) && (currentPosition == 0)) {
			is_moving = 1;
			// need to back OUT a little till switch opens and then set position to 0
			set_to_home();
			is_moving = 0;
		}
		// else if switchstate = OPEN and Position = 0
		// need to move IN a little till switch CLOSES then
		else if ((hpswstate == HPSWOPEN) && (currentPosition == 0)) {
			is_moving = 1;
			// need to move IN a little till switch CLOSES then
			move_to_home();
			// need to back OUT a little till switch opens and then set position to 0
			set_to_home();
			is_moving = 0;
		}

		if (myfocuser.seven_segment_update_on_move == 1) { 		// check if seven segment needs updating during move
			update_count++;
			if (updatecount > SEVENSEGMENTUPDATESTEPCOUNT) {
				update_position_seven_segment();
				update_count = 0;
			}
		}
		update_motor_speed_delay();
		delay(motor_speed_delay);  // required else stepper will not move
	}
	else {
		is_moving = 0; 					// focuser has reached target, focuser is NOT moving now, move is completed
										// clear state of home position
		hpswbounce.update();            // we need to call update to read home position switch state, no interrupts are used
		hpswstate = hpswbounce.read();  // clear any 0 readings
		motor_speed = saved_motor_speed;       // restore original motorSpeed
		update_motor_speeddelay();
		long currentMillis = millis();   		// see if the display needs updating
		if (((currentMillis - old_display_timestamp_not_moving) > (myfocuser.page_display_time * 100)) || (current_millis < old_display_timestamp_not_moving))
		{
			old_display_timestamp_not_moving = current_millis;    // update the timestamp
			seven_segment.clear();
		}
		long temperature_now = millis();
			// see if the temperature needs updating - done automatically every 5s
		if (((temperature_now - last_temperature_conversion) > TEMPREFRESHRATE) || (temperature_now < last_temperature_conversion)) {
			last_temperature_conversion = millis();      // update
			if (request_temperature_flag == 0) {
				read_temperature();
				request_temperature_flag = 1;
			}else{
				request_temperature();
				request_temperature_flag = 0;
			}
		} // end of check to see if it is time to get new temperature reading
		// check for temperature compensation;
		if (myfocuser.temperature_compensation_enabled == 1) {
			if (tc_started == 0) {
				tc_started = 1;
				start_temperature_value = ch1tempval;
			}
			// if temperature has changed by 1 degree
			double temperature_change = start_temperature_value - ch1tempval;
			if (temperature_change >= 1) {
				// move the focuser by the required amount
				// this should move focuser inwards
				new_position = target_position - myfocuser.temperature_coefficient;
				// rangecheck target
				if (new_position < 0)
					new_position = 0;
				if (new_position > myfocuser.max_step)
					new_position = myfocuser.max_step;
				move_started = 1;
				target_position = new_position;
				tc_started = 0;                  // indicate that temp compensation was done
			} // end of check for tempchange >=1
		} // end of check for tempcompenabled == 1)
		if (write_now == 1) {	// is it time to update EEPROM settings?
			// decide if we have waited 10s (value of EEPROMWRITEINTERVAL) after the last myfocuser key variable update, if so, update the EEPROM
			long current_millis = millis();
			if (((current_millis - previous_millis) > EEPROMWRITEINTERVAL) || (current_millis < previous_millis)) {
				myfocuser.validdata = VALIDDATAFLAG;
				myfocuser.position = current_position;
				writeEEPROMNow();                   // update values in EEPROM
				write_now = 0;
				previous_millis = current_millis;     // update the timestamp
			}
		}
		if (!myfocuser.coil_power) clear_output();
	}
} 
// Get last position --------------------------------------------------------------------------------------------------
int Get_LastPosition(void) {
	int lastposition = 0;
	EEPROM.get(Last_position_address, lastposition);	// read the last position from EEPROM
	if ((lastposition < 1) || (lastposition > AWAY)) {	// check for invalid positions
		// error - bad position, so default to home
		lastposition = 0;
		store_position_in_eeprom(0);
	}
	return lastposition;
}
// send update to SEVEN_SEGMENT ---------------------------------------------------------------------------------------
void send_update_to_SEVEN_SEGMENT() {
	char position_s[5];
	noInterrupts();							// disable all interrupts
	dtostrf(current_position, 5, 0, position_s);
	sprintf(display_string, "%s ", position_s);
	display(display_string);
	interrupts();             // enable all interrupts
}
// parse incoming GPS packet-------------------------------------------------------------------------------------------
void Parse_Incoming_gps_packet(void) {
	incoming_gps_packet_available = false;	// clear the packet arrived flag
	char * packet_ID = gps_nmea.getMessageID();
#ifdef DISPLAY_DEBUG_GPS
		for (i=0; i<3; i++) {
		    Serial.print(packet_ID[i]);
		}
		Serial.println();
#endif
	if (gps_nmea.isValid() == false) return;
	if (focuser_display_on == true) {
		digitalWrite(GPS_LED_pin, HIGH);      // light an LED when a packet received
		gps_status = true;
		gps_time_LED_On = millis();
	}
		//-- GPGLL ------------------------------------------------------------------------------------------------
		/*
		GLL Geographic Position – Latitude / Longitude
			          1      2     3      4      5      6 7 
					,  |      |     |      |      |      | | 
			$--GLL, llll.ll, a, yyyyy.yy, a, hhmmss.ss, A*hh
			1) Latitude 
			2) N or S(North or South) 
			3) Longitude 
			4) E or W(East or West) 
			5) Time(UTC) 
			6) Status A - Data Valid, V - Data Invalid 
			7) Checksum
		*/
	if (strcmp(packet_ID, "GLL") == 0) {
#ifdef DISPLAY_DEBUG_GPS
		Serial.println("GLL detected");
#endif
		gps_timedate_valid = true;
		gps_hour = (int)gps_nmea.getHour();
		if ((gps_hour < 0) || (gps_hour > 23)) gps_timedate_valid = false;
		gps_minute = (int)gps_nmea.getMinute();
		if ((gps_minute < 0) || (gps_minute > 59)) gps_timedate_valid = false;
		gps_second = (int)gps_nmea.getSecond();
		if ((gps_second < 0) || (gps_second > 59)) gps_timedate_valid = false;
		if (gps_timedate_valid == false) {
			error(Date_Time_error);
			current_status |= Date_Time_error;
		}
		else {
			current_status ^= Date_Time_error;
		}
		gps_longlat_valid = true;
		gps_latitude_long = gps_nmea.getLatitude();
		gps_latitude = gps_latitude_long / 1000000.;
		if ((gps_latitude < -90.00) || (gps_latitude > 90.00)) 	gps_longlat_valid = false;
		gps_longitude_long = gps_nmea.getLongitude();
		gps_longitude = gps_longitude_long / 1000000.;
		if ((gps_longitude < -180.00) || (gps_longitude > 180.00)) gps_longlat_valid = false;
		if (gps_longlat_valid == false) {
			error(Long_Lat_error);
			current_status |= Long_Lat_error;
		}
		else {
			current_status ^= Long_Lat_error;
		}
	}
	/* -- GGA ---------------------------------------------------------------------------------------------------------
		GGA Global Positioning System Fix Data.Time, Position and fix related data for a GPS receiver
		           1         2      3     4      5  6  7    8    9  10  11  12   13   14  15
		           |         |      |     |      |  |  |    |    |   |   |   |   |     |  | 
		$--GGA, hhmmss.ss, llll.ll, a, yyyyy.yy, a, x, xx, x.x, x.x, M, x.x, M, x.x, xxxx*hh
		1) Time(UTC) 
		2) Latitude 
		3) N or S(North or South) 
		4) Longitude 
		5) E or W(East or West) 
		6) GPS Quality Indicator, 0 - fix not available, 1 - GPS fix, 2 - Differential GPS fix 
		7) Number of satellites in view, 00 - 12 
		8) Horizontal Dilution of precision 
		9) Antenna Altitude above / below mean - sea - level(geoid) 
		10) Units of antenna altitude, meters 
		11) Geoidal separation, the difference between the WGS - 84 earth ellipsoid and mean - sea - level(geoid), "-" means mean - sea - level below ellipsoid 
		12) Units of geoidal separation, meters 
		13) Age of differential GPS data, time in seconds since last SC104 type 1 or 9 update, null field when DGPS is not used 
		14) Differential reference station ID, 0000 - 1023 
		15) Checksum
		*/
	if (strcmp(packet_ID, "GGA") == 0) {
#ifdef DISPLAY_DEBUG_GPS
		Serial.println("GGA detected");
#endif
		gps_altitude_valid = true;
		gps_nmea.getAltitude(gps_altitude_long);
		gps_altitude = gps_altitude_long / 1000.;
		if ((gps_altitude < 0) || (gps_altitude > 9000.)) gps_altitude_valid = false;
		if (gps_altitude_valid == false) {
			error(Altitude_error);
			current_status |= Altitude_error;
		}
		else {
			current_status ^= Altitude_error;
		}
	}
		// -- RMC -----------------------------------------------------------------------------------------------------------  
		//		Serial.println("RMC detected");
		/*
		RMC Recommended Minimum Navigation Information         
		           1     2     3   4    5     6  7   8    9  10 11 12
				   |     |     |   |    |     |  |   |    |  |   |  | 
		$--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a*hh
		1) Time (UTC) 
		2) Status, V = Navigation receiver warning 
		3) Latitude 
		4) N or S 
		5) Longitude 
		6) E or W 
		7) Speed over ground, knots 
		8) Track made good, degrees true 
		9) Date, ddmmyy 
		10) Magnetic Variation, degrees 
		11) E or W 
		12) Checksum
		*/
	if (strcmp(packet_ID, "RMC") == 0) {
		gps_timedate_valid = true;
		gps_year = (int)gps_nmea.getYear();
		if ((gps_year < 0) || (gps_year > 99)) gps_timedate_valid = false;
		gps_month = (int)gps_nmea.getMonth();
		if ((gps_month < 0) || (gps_month > 12)) gps_timedate_valid = false;
		gps_day = (int)gps_nmea.getDay();
		if ((gps_day < 0) || (gps_day > 31)) gps_timedate_valid = false;
		gps_hour = (int)gps_nmea.getHour();
		if ((gps_hour < 0) || (gps_hour > 23)) gps_timedate_valid = false;
		gps_minute = (int)gps_nmea.getMinute();
		if ((gps_minute < 0) || (gps_minute > 59)) gps_timedate_valid = false;
		gps_second = (int)gps_nmea.getSecond();
		if ((gps_second < 0) || (gps_second > 59)) gps_timedate_valid = false;
		if (gps_timedate_valid == false) {
			error(Date_Time_error);
			current_status |= Date_Time_error;
		}
		else {
			current_status ^= Date_Time_error;
		}
		gps_longlat_valid = true;
		gps_latitude_long = gps_nmea.getLatitude();
		gps_latitude = gps_latitude_long / 1000000.;
		if ((gps_latitude < -90.00) || (gps_latitude > 90.00)) gps_longlat_valid = false;
		gps_longitude_long = gps_nmea.getLongitude();
		gps_longitude = gps_longitude_long / 1000000.;
		if ((gps_longitude < -180.00) || (gps_longitude > 180.00)) gps_longlat_valid = false;
		if (gps_longlat_valid == false) {
			error(Long_Lat_error);
			current_status |= Long_Lat_error;
		}
		else {
			current_status ^= Long_Lat_error;
		}
	}
#ifdef DISPLAY_DEBUG_GPS
	Serial.print("timedate valid:");
	Serial.print(gps_timedate_valid, DEC);
	Serial.print(" Date:");
	Serial.print((int)gps_year);
	Serial.print('-');
	Serial.print((int)gps_month);
	Serial.print('-');
	Serial.print((int)gps_day);
	Serial.print(", Time:");
	Serial.print((int)gps_hour);
	Serial.print(':');
	Serial.print((int)gps_minute);
	Serial.print(':');
	Serial.println((int)gps_second);
	Serial.print("loglat valid:");
	Serial.print(gps_longlat_valid, DEC);
	Serial.print(" Latitude (deg): ");
	Serial.println(gps_latitude, 6);
	Serial.print("Longitude (deg): ");
	Serial.println(gps_longitude, 6);
	Serial.print("Altitude (m): ");
	Serial.println(gps_altitude, 3);
	Serial.print("freeMemory()=");
	Serial.println(freeMemory());
	Serial.println("-----------------------");
#endif
	gps_nmea.clear();
} // end of Parse gps data
// catch unrequired GPS packets----------------------------------------------------------------------------------------
void gps_printUnknownSentence(MicroNMEA& gps_nmea) {
	Serial.println();
	Serial.print("Unknown sentence: ");
	Serial.println(gps_nmea.getSentence());
	gps_nmea.clear();
}
// check temperature --------------------------------------------------------------------------------------------------
void Get_Temperature() {
	// Get temperature event and print its value.
	sensors_event_t event;
	dht.temperature().getEvent(&event);
	if (isnan(event.temperature)) {
		error(Temperature_Humidity_error);
		current_status |= Temperature_Humidity_error;
	}
	else {
		current_temperature = event.temperature;
		current_status ^= Temperature_Humidity_error;
#ifdef DEBUG_TEMPERATURE_OUTPUT
		Serial.print("Temperature: ");
		Serial.print(event.temperature);
		Serial.println(" *C");
#endif
	}
	// Get humidity event and print its value.
	dht.humidity().getEvent(&event);
	if (isnan(event.relative_humidity)) {
		error(Temperature_Humidity_error);
		current_status |= Temperature_Humidity_error;
	}
	else {
		current_humidity = event.relative_humidity;
		current_status ^= Temperature_Humidity_error;
#ifdef DEBUG_TEMPERATURE_OUTPUT
		Serial.print("Humidity: ");
		Serial.print(event.relative_humidity);
		Serial.println("%");
#endif
	}
}
// --------------------------------------------------------------------------------------------------------------------
bool Check_Home() {
	unsigned char reading = (unsigned char) digitalRead(focuser_Home_Pin);
	if (reading != last_focuser_Home_switch_state) {
		last_focuser_Home_switch_time = millis();
	}
	if ((millis() - last_focuser_Home_switch_time) > debounceDelay) {
		if (reading != focuser_Home_switch_state) {
			focuser_Home_switch_state = reading;
			if (focuser_Home_switch_state == (unsigned char) ATHOME) {
				current_status = 0;
				return true;
			}
			else {
				current_status = 9;
				return false;
			}
		}
	}
	last_focuser_Home_switch_state = reading;
}
// --------------------------------------------------------------------------------------------------------------------
void Check_BEMF() {
  double bemf_reading = ((double)analogRead(amisSLAPin & 0xff));					// read the bemf pin
  double bemf_voltage = (double) bemf_reading * (double) 5.0;
#ifdef DEBUG_BEMF_OUTPUT
  Serial.print(millis(), DEC);
  Serial.print("\tfocuser BEMF (SLA):");
  Serial.println(bemf_voltage, DEC);
#endif
}
// --------------------------------------------------------------------------------------------------------------------
bool Check_Hub_Message(void) {
	char thisbyte;
	while (hub_outptr != hub_inptr) {									 // check altitude serial buffer for data
		thisbyte = hub_inbuffer[hub_outptr++];							// take a character from the input buffer and increment pointer
		if ((thisbyte == (char)HUB_STX) && (hub_string_ptr == 0)) {     // look for the STX, but only if the output string is empty
#ifdef DEBUG_RECEIVED_OUTPUT
			Serial.print(millis(), DEC);
			Serial.print("\tSTX Received: (");
			Serial.print(hub_string_ptr, DEC);
			Serial.println(") ");
#endif
			incoming_hub_message.sg_chars[hub_string_ptr++] = HUB_STX;    // store the STX and increment the string pointer
			hub_incoming_message_available = false;
		} else {
			if ((thisbyte == (char)HUB_ETX) && (hub_string_ptr == HUB_incoming_message_length-1)) {	// character was not an STX check for ETX
#ifdef DEBUG_RECEIVED_OUTPUT
				Serial.print(millis(), DEC);
				Serial.print("\tETX Received: (");
				Serial.print(hub_string_ptr, DEC);
				Serial.println(") ");
#endif
				incoming_hub_message.sg_chars[hub_string_ptr] = HUB_ETX;  // save the ETX and indicate packet available
				hub_string_ptr = 0;                                       // zero the string pointer
				hub_incoming_message_available = true;
			} else {
#ifdef DEBUG_RECEIVED_OUTPUT
				Serial.print(millis(), DEC);
				Serial.print("\tCharacter Received: (");
				Serial.print(hub_string_ptr, DEC);
				Serial.print(") ");
				Serial.println(thisbyte, DEC);
#endif
				incoming_hub_message.sg_chars[hub_string_ptr++] = thisbyte; // Not a valid STX or a valid ETX so save it and increment string pointer
			}
		}
	} // end of while hub outptr != inptr
	return hub_incoming_message_available;
}
// --------------------------------------------------------------------------------------------------------------------
void Parse_Hub_Incoming_Message() {
	unsigned long microsteps_target = 0;
	hub_incoming_message_available = false;
	if (focuser_display_on == true) {
		hub_status = true;
		hub_time_LED_On = millis();
		digitalWrite(HUB_LED_pin, HIGH);
	}
#ifdef DEBUG_OUTPUT
		Serial.print(millis(), DEC);
		Serial.println("\tPacket Received from HUB");
#endif
		switch (incoming_hub_message.hub_incoming_message.command_number) {
			//----------------------------------------------------------------------------------------------------
		case 0: // get current focuser position
			sendresponse(current_position);
			break;
		case 1: // get motor moving status - 01 if moving, 00 otherwise
			sendresponse(isMoving);
			break;
		case 2: // get motor controller status - Controller Response to "connected"
			sendresponse(OK);
			break;
		case 3: // get firmware version string
			sendresponse(firmware_version);
			break;
		case 6: // get temperature as a double XXXX
			sendresponse(current_temperature);
			break;
		case 8: // get MaxStep, returns XXXXXX
			sendresponse(myfocuser.maximum_step);
			break;
		case 10: // get MaxIncrement, returns xxxxxx
			sendresponse(myfocuser.maximum_increment);
			break;
		case 11: // get coil pwr setting (00 = coils released after move, 01 = coil pwr on after move)
			sendresponse(myfocuser.coil_power_setting);
			break;
		case 13: // get reverse direction setting, 00 off, 01 on
			sendresponse(myfocuser.reverse_direction);
			break;
		case 24: // get state of Temperature Compensation, 0=disabled, 1=enabled
			sendresponse(myfocuser.temperature_compensation_state);
			break;
		case 25: // get if Temperature Compensation available 0=No, 1=Yes
			sendresponse(OK);
			break;
		case 26: // get Temperature Coefficient (in steps per degree)
			sendresponse(myfocuser.temperature_coefficient);
			break;
		case 29: // get stepmode, returns XX#
			sendresponse(myfocuser.stepmode);
			break;
		case 32: // get if stepsize is enabled in controller (1 or 0, 0/1)
			sendresponse(myfocuser.step_size_enabled);
			break;
		case 33: // get step size in microns (if enabled by controller)
			sendresponse(myfocuser.step_size);
			break;
		case 34: // get the time that an LCD screen is displayed for (in milliseconds, eg 2500 = 2.5seconds
			sendresponse(myfocuser.pagedisplaytime);
			break;
		case 37: // get Display status 0=disabled, 1=enabled
			sendresponse(display_enabled);
			break;
		case 38: // get Temperature mode 1=Celsius, 0=Fahrenheight
			sendresponse(myfocuser.tempmode);
			break;
		case 39: // get the new motor position (target) XXXXXX
			sendresponse(target_position);
			break;
		case 43: // get motorspeed (0-3)
			sendresponse(motor_speed);
			break;
		case 45: // get tswthreshold - value for which stepper slows down at end of its move
			sendresponse(tswthreshold);
			break;
		case 47: // get if motorspeedchange enabled/disabled
			sendresponse(motor_speed_change);
			break;
		case 62: // get update of position on lcd when moving (00=disable, 01=enable)
			sendresponse(myfocuser.lcd_update_on_move);
			break;
		case 63: // get status of home position switch (0=open, 1=closed)
			home_position_switchbounce.update();                // we need to call update to read home position switch state, no interrupts are used
			sendresponse(home_position_switchbounce.read();;                 // home switch activated
			break;
		case 66: // get jogging state enabled/disabled
			sendresponse(jogging_state);
			break;
		case 68: // get jogging direction 0=IN, 1=OUT
			sendresponse(jogging_direction);
			break;
		case 41:  // Troubleshooting only
			sendresponse(null);
			break;
		case 72: // gets DelayAfterMove
			sendresponse(myfocuser.delay_after_move);
			break;
		case 74: // get backlash in enabled status
			sendresponse(myfocuser.backlash_in_enabled);
			break;
		case 76:  // get backlash OUT enabled status
			sendresponse(myfocuser.backlash_out_enabled);
			break;
		case 78: // get number of backlash steps IN
			sendresponse(myfocuser.backlashsteps_in);
			break;
		case 80: // get number of backlash steps OUT
			sendresponse(myfocuser.backlashsteps_out);
			break;
		case 87: // get temp comp direction 1=IN
			sendresponse(myfocuser.tcdirection);
			break;
		case 5:  // Set new target position to xxxxxx (and focuser initiates immediate move to xxxxxx)
			target_position = incoming_hub_message.hub_incoming_message.focuser_parameter;
			if (target_position < current_position) {
				move_direction = MOVINGIN;
			}
			else {
				move_direction = MOVINGOUT;
			}
			isMoving = true;
			break;
		case 28: // home the motor to position 0
			if (myfocuser.tempcompenabled == 0) {
				if (cmdval == 28) {
					newPos = 0;                   // if this is a home then set target to 0
				}
				else {
					pos = decstr2long(param);     // else set target to a move command
					newPos = pos;
				}
				isMoving = true;
				if (target_position < current_position) {
					move_direction = MOVINGIN;
				}
				else {
					move_direction = MOVINGOUT;
				}
				Serial.print("- Current Position = "); Serial.print(currentPosition); Serial.print("#");
				Serial.print("- Target Position = "); Serial.print(newPos); Serial.print("#");
				Serial.print("- Previous direction = "); Serial.print(myfocuser.focuserdirection); Serial.print("#");
				Serial.print("- New Direction = "); Serial.print(movedirection); Serial.print("#");
														// determine if a change in direction has taken place
				if (move_direction != myfocuser.focuser_direction) {
					Serial.print("- Applying backlash#");
					long tmppos = newPos;
														// apply backlash because moving in opposite direction
					if (movedirection == MOVINGIN) {
						Serial.print("- Backlash Steps IN="); Serial.print(myfocuser.backlashsteps_in); Serial.print("#");
														// apply IN backlash steps
						for (int steps = 0; steps < myfocuser.backlashsteps_in; steps++) {
							anticlockwise();
							delayMicroseconds(MOTORPULSETIME);
							tmppos--;
							if (tmppos <= 0) {
								newPos = 0;
								break;
							}
						}
					} else {
						Serial.print("- Backlash steps OUT="); Serial.print(myfocuser.backlashsteps_out); Serial.print("#");
														// apply OUT backlash steps
						for (int steps = 0; steps < myfocuser.backlashsteps_out; steps++) {
							clockwise();
							delayMicroseconds(MOTORPULSETIME);
							tmppos++;
							if (tmppos >= myfocuser.maxstep) {
								newPos = myfocuser.maxstep;
								break;
							}
						}
					}
					myfocuser.focuser_direction = move_direction;
				}
				Serial.print("- 2s delay before move#");
				delay(2000);
														// rangecheck target
				if (newPos < 0)
					newPos = 0;
				if (newPos > myfocuser.maxstep)
					newPos = myfocuser.maxstep;
				movestarted = 1;
				Serial.print("- Move to position "); Serial.print(newPos); Serial.print("#");
				targetPosition = newPos;
				updatepositionlcd();
				flageepromupdate();
			}
			break;
		case 7: // set MaxStep
			myfocuser.max_step = incoming_hub_message.hub_incoming_message.focuser_parameter);
			if (myfocuser.max_step > FOCUSERUPPERLIMIT)            // range check the new value for maxSteps
				myfocuser.max_step = FOCUSERUPPERLIMIT;
			if (myfocuser.max_step < FOCUSERLOWERLIMIT)            // avoid setting maxSteps too low
				myfocuser.max_step = FOCUSERLOWERLIMIT;
			flageepromupdate();
			break;
		case 12: // set coil pwr 0=release pwr after move, 1=keep power on after move
			myfocuser.coil_power = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x01;
			flageepromupdate();
			break;
		case 14: // set reverse direction setting 0=normal, 1=reverse
			myfocuser.ReverseDirection = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x01;
			flageepromupdate();
			break;
		case 15: // set MotorSpeed, 00 = Slow, 01 = Med, 02 = Fast
			motor_speed = saved_motor_speed = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x03;
			updatemotorSpeedDelay();
			break;
		case 16: // set display to Celsius
			myfocuser.temperature_mode = 1;
			flageepromupdate();
			break;
		case 17: // set display to Fahrenheit
			myfocuser.temperature_mode = 0;
			flageepromupdate();
			break;
		case 18: // set the return of user specified stepsize 0=OFF(default), 1=ON - reports what user specified as stepsize
			myfocuser.stepsizeenabled = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x01;
			flageepromupdate();
			break;
		case 19:  // :19xxxx#  None   set the step size value - double type, eg 2.1
			double temp_step_size = (double)incoming_hub_message.hub_incoming_message.focuser_parameter.toFloat();
			if (temp_step_size < MINSTEPSIZE)
				tempstepsize = DEFAULTSTEPSIZE;       // set default maximum stepsize
			myfocuser.step_size = temp_step_size;
			flageepromupdate();
			break
		case 22:  // :22xxx#    None    set the temperature compensation value to xxx
			double paramval = incoming_hub_message.hub_incoming_message.focuser_parameter;
			if (paramval < 0)
				paramval = 0;
			else if (paramval > 200)
				paramval = 200;
			myfocuser.temperature_compensation = (byte)paramval;     // save setting in EEPROM
			flageepromupdate();
			break;
		case 23:  // set the temperature compensation ON (1) or OFF (0)
			myfocuser.tempcompenabled = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x01;
			flageepromupdate();
			break;
		case 30:  // set stepmode (1=Full, 2=Half)
			myfocuser.step_mode = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x03;
			setstepmode(myfocuser.step_mode);
			update_motor_speed_delay();
			flageepromupdate();
			break;
		case 31:  // set current motor position to xxxxxx (does not move, updates currentpos and targetpos to xxxxxx)
			new_position = incoming_hub_message.hub_incoming_message.focuser_parameter;
										// rangecheck target
			if (new_position < 0)
				new_position = 0;
			if (new_position > myfocuser.max_step)
			new_position = myfocuser.max_step;
			isMoving = 0;
			current_position = target_position = new_position;
			flageepromupdate();
			break;
		case 35:  // set length of time an LCD page is displayed for in milliseconds
			position = incoming_hub_message.hub_incoming_message.focuser_parameter;
			if (position < DISPLAYPAGETIMEMIN)           // bounds check to 2000-4000 2s-4s
				position = DISPLAYPAGETIMEMIN;
			if (position > DISPLAYPAGETIMEMAX)
				position = DISPLAYPAGETIMEMAX;
			myfocuser.page_display_time = position;
			flageepromupdate();
			break;
		case 36:
		// :360#    None    Disable Display
		// :361#    None    Enable Display
			display_enabled = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x01;
			if (displayenabled == 0) {
				lcd.noDisplay();
			} else {
				lcd.display();
			}
			break;
		case 40: // reset Arduino controller
			software_Reboot();
			break;
		case 44: // set motorspeed threshold when moving - switches to slowspeed when nearing destination
			paramval = incoming_hub_message.hub_incoming_message.focuser_parameter;
			if (paramval < 50)                  // range check
				paramval = 50;
			else if (paramval > 200)
				paramval = 200;
			tswthreshold = (byte)paramval;
			break;
		case 46: // Enable/Disable motorspeed change when moving
			motorspeedchange = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x01;
			break;
		case 48: // Save settings to EEPROM
			myfocuser.valid_data = VALIDDATAFLAG;
			myfocuser.position = currentPosition;
			writeEEPROMNow();
			writenow = 0;
			break;
		case 61: // set update of position on lcd when moving (00=disable, 01=enable)
			myfocuser.lcdupdateonmove = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x01;
			flageepromupdate();
			break;
		case 64: // move a specified number of steps
			isMoving = 1;
			movestarted = 1;
			target_position = currentPosition + incoming_hub_message.hub_incoming_message.focuser_parameter;
			// rangecheck target
			if (target_position < 0)
				target_position = 0;
			if (target_position > myfocuser.max_step)
				target_position = myfocuser.max_step;
			flageepromupdate();
			break;
		case 65: // set jogging state enable/disable
			jogging = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x01;
			break;
		case 67:  // :67#     None    Set jogging direction, 0=IN, 1=OUT
			joggingDirection = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x01;
			break;
		case 42: // Reset focuser defaults
			current_address = 0;
			set_focuser_defaults();
			current_position = myfocuser.position;
			target_position = myfocuser.position;
			break;
		case 71: // set DelayAfterMove in milliseconds
			paramval = incoming_hub_message.hub_incoming_message.focuser_parameter;
												// bounds check to 0-250
			if (paramval < 0)
				paramval = 0;
			if (paramval > 250)
				paramval = 250;
			myfocuser.DelayAfterMove = (byte)paramval;
			flageepromupdate();
			break;
		case 73: // Disable/enable backlash IN (going to lower focuser position)
			myfocuser.backlash_in_enabled = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x01;
			flageepromupdate();
			break;
		case 75: // Disable/enable backlash OUT (going to lower focuser position)
			myfocuser.backlash_out_enabled = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x01;
			flageepromupdate();
			break;
		case 77: // set backlash in steps
			paramval = incoming_hub_message.hub_incoming_message.focuser_parameter;
			if (paramval < 0)                   // range check
				paramval = 0;
			myfocuser.backlash_steps_in = (byte)paramval;
			flageepromupdate();
			break;
		case 79: // set backlash OUT steps
			paramval = incoming_hub_message.hub_incoming_message.focuser_parameter;
			if (paramval < 0)                   // range check
				paramval = 0;
			myfocuser.backlash_steps_out = (byte)paramval;
			flageepromupdate();
			break;
		case 88: // set temp comp direction 1=IN
			myfocuser.tc_direction = incoming_hub_message.hub_incoming_message.focuser_parameter & 0x01;
			flageepromupdate();
			break;
//----------------------------------------------------------------------------------------------------
		case Focuser_Home_command: {
			Home();
			break;
		} // end of case home
		case Focuser_Halt_command: {
			DisableDriver();
			EnableDriver();
			break;
		} // end of case halt
		case Focuser_Move_to_command: {
			microsteps_target = incoming_hub_message.hub_incoming_message.focuser_parameter;
			Move_to(microsteps_target);
			break;
		} // end of case move_to
		case Focuser_Move_command: {
			microsteps_target = incoming_hub_message.hub_incoming_message.focuser_parameter;
			microsteps_direction = incoming_hub_message.hub_incoming_message.focuser_direction;
			Move(microsteps_target, microsteps_direction);
			break;
		} // end of case move
		case Focuser_Display_On_command: {
			focuser_display_on = true;
			break;
		}
		case Focuser_Display_Off_command: {
			focuser_display_on = false;
			break;
		}
	} // end of switch
} // end of Parse
// Move_to ------------------------------------------------------------------------------------------------------------
void Move_to(unsigned int target) {
	if (target > (unsigned int)AWAY) target = (unsigned int)AWAY;	// apply limits
	if (target < (unsigned int)HOME) target = (unsigned int)HOME;	// apply limits
	if (target > current_position) {				// target is greater than current location, go outwards
		stepper.setDirection(OUTWARDS);
		do {
			cli();
			nextStep(OUTWARDS);
			sei();
		} while (current_position < target);		// step and decrement current position = target
	}
	if (current_position > target) {				// target is less than current location, go inwards
		stepper.setDirection(INWARDS);
		do {
			cli();
			nextStep(INWARDS);
			sei();
		} while (current_position > target);
	}
	store_position_in_eeprom(current_position);
} // end of move_to
// Move ---------------------------------------------------------------------------------------------------------------
void Move(unsigned int steps, int direction) {
	unsigned int target_position = 0;
	if (direction == (int)OUTWARDS) {						// go outwards
		target_position = current_position + steps;
		if (target_position > (unsigned int)AWAY) target_position = (unsigned int)AWAY;   // limit movement to away
		stepper.setDirection(OUTWARDS);
		do {
			cli();
			nextStep(OUTWARDS);
			sei();
		} while (current_position != target_position);		// step and decrement current position = target
	} else {												// go inwards
		target_position = current_position - steps;
		if (target_position < (unsigned int)HOME) target_position = (unsigned int)HOME;		// limit movement to home
		stepper.setDirection(INWARDS);
		do {
			cli();
			nextStep(INWARDS);
			sei();
		} while (current_position != target_position);
	}
	store_position_in_eeprom(current_position);
} // end of move_to
// Home - ------------------------------------------------------------------------------------------------------------=
void Home(void) {
	//	resetDriver();
#ifdef DEBUG_OUTPUT
	Serial.print(millis(), DEC);
	Serial.println(" HOME command issued");
#endif	
	stepper.setCurrentMilliamps(2000);
	stepper.enableDriver();
	if (digitalRead(focuser_Home_Pin) == ATHOME) {              // at home, so slowly get off home position by moving forwards
#ifdef DEBUG_OUTPUT
		Serial.print(millis(), DEC);
		Serial.println(" ATHOME, Switch high, moving outwards at MicrosStep128");
#endif
		stepper.setDirection(OUTWARDS);							// default to outwards direction
		do {
			cli();
			nextStep(OUTWARDS);
			sei();
		} while (digitalRead(focuser_Home_Pin) == ATHOME);     // stop when home switch opens
	}
	else {                                       // not home, so move backwards until home 
#ifdef DEBUG_OUTPUT
		Serial.print(millis(), DEC);
		Serial.println(" NOTATHOME, Switch low, moving inwards at MicroSteps32");
#endif
		stepper.setDirection(INWARDS);							// direction
		stepper.setStepMode(AMIS30543::MicroStep32);
		do {
			cli();
			nextStep(INWARDS);
			sei();
		} while (digitalRead(focuser_Home_Pin) == NOTATHOME);
#ifdef DEBUG_OUTPUT
		Serial.print(millis(), DEC);
		Serial.println(" ATHOME, Switch high, moving outwards");
#endif
		stepper.setStepMode(AMIS30543::MicroStep128);
		stepper.setDirection(OUTWARDS);							// default to outwards direction											                      // home reached, so slowly get off home position by moving forwards
		do {
			cli();
			nextStep(OUTWARDS);
			sei();
		} while (digitalRead(focuser_Home_Pin) == NOTATHOME);	// stop when home switch opens
	}
	current_position = 0.0;									// HOME
	stepper.setDirection(OUTWARDS);							// default to outwards direction
	store_position_in_eeprom(current_position);
#ifdef DEBUG_OUTPUT
	Serial.print(millis(), DEC);
	Serial.print(" HOME, Current Position ");
	Serial.println(current_position,DEC);
#endif
}
void Set_to_Home()  {  // Switch is CLOSED
	int save = motor_speed;  // save speed setting and set to slow
	motor_speed = SLOW;
	update_motor_speed_delay();
	// recheck state of Home position and keep going till it opens
	home_position_switch_bounce.update();            // we need to call update to read home position switch state, no interrupts are used
	home_position_switch_state = home_position_switch_bounce.read();  // reads the home position switch state
	while (home_position_switch_state == home_position_switch_CLOSED) {
		clockwise();    // take one step OUT
		delay(MOTORPULSETIME);
		// recheck state of Home position and keep going till it opens
		home_position_switch_bounce.update();            // we need to call update to read home position switch state, no interrupts are used
		home_position_switch_state = home_position_switch_bounce.read();  // reads the home position switch state
	}
	target_position = 0;   // home switch is now open
	current_position = 0;
	motor_speed = save;    // restore motorSpeed
	update_motor_speed_delay();
}
void movetohome() { // focuser is at 0 and switch still OPEN
	int save = motor_speed;  // save speed setting and set to slow
	motor_speed = SLOW;
	update_motor_speed_delay();
	// try to compensate for positional difference in home position switch
	home_position_switch_bounce.update();            // we need to call update to read home position switch state, no interrupts are used
	home_position_switch_state = home_position_switch_bounce.read();  // reads the home position switch state
	while (home_position_switch_state == home_position_switch_OPEN)
	{
		anticlockwise();    // take one step IN
		delay(MOTORPULSETIME);
		// recheck state of Home position and keep going till it closes
		home_position_switch_bounce.update();            // we need to call update to read home position switch state, no interrupts are used
		home_position_switch_state = home_position_switch_bounce.read();  // reads the home position switch state
	}
	motor_speed = save;  // restore motorSpeed
	update_motor_speed_delay();
}
void software_Reboot() {
	asm volatile ("jmp 0");  // jump to the start of the program
}
void anticlockwise() { // Move stepper anticlockwise
	(!myfocuser.Reverse_Direction) ? digitalWrite(OUTLED, 1) : digitalWrite(INLED, 1);
	(!myfocuser.Reverse_Direction) ? mystepper.step(-1) : mystepper.step(1);
	delayMicroseconds(MOTORPULSETIME);
	(!myfocuser.ReverseDirection) ? digitalWrite(OUTLED, 0) : digitalWrite(INLED, 0);
}
void clockwise() { // Move stepper clockwise
	(!myfocuser.Reverse_Direction) ? digitalWrite(INLED, 1) : digitalWrite(OUTLED, 1);
	(!myfocuser.Reverse_Direction) ? mystepper.step(1) : mystepper.step(-1);
	delayMicroseconds(MOTORPULSETIME);
	(!myfocuser.ReverseDirection) ? digitalWrite(INLED, 0) : digitalWrite(OUTLED, 0);
}

// set the microstepping mode
void setstepmode(byte stepmode) {
	if (stepmode == STEP1) {
		mystepper.SetSteppingMode(SteppingMode::FULL);
		myfocuser.stepmode = STEP1;
	} 
	else if (stepmode == STEP2)	{
		mystepper.SetSteppingMode(SteppingMode::HALF);
		myfocuser.stepmode = STEP2;
	} 
	else if (step_mode == STEP4) {
		mystepper.SetSteppingMode(SteppingMode::FULL);
		myfocuser.stepmode = STEP4;
	}
	else if (step_mode == STEP8) {
		mystepper.SetSteppingMode(SteppingMode::FULL);
		myfocuser.stepmode = STEP8;
	}
	else if (step_mode == STEP16) {
		mystepper.SetSteppingMode(SteppingMode::FULL);
		myfocuser.stepmode = STEP16;
	}
	else if (step_mode == STEP32) {
		mystepper.SetSteppingMode(SteppingMode::FULL);
		myfocuser.stepmode = STEP32;
	}
	else if (step_mode == STEP64) {
		mystepper.SetSteppingMode(SteppingMode::FULL);
		myfocuser.stepmode = STEP64;
	}
	else if (step_mode == STEP128) {
		mystepper.SetSteppingMode(SteppingMode::FULL);
		myfocuser.stepmode = STEP128;
	}
}

void update_motor_speed_delay() {
	switch (motor_speed) {
		case SLOW: // slow
			switch (myfocuser.step_mode) {
				case STEP1: // full steps
					motor_speed_delay = SLOWDELAY1;
					motor_speed_RPM = motorSpeedSlowRPM;
					break;
				case STEP2: // 
					motor_speed_delay = SLOWDELAY2;
					motor_speed_RPM = motorSpeedSlowRPM * 2;
					break;
				case STEP4: // 
					motor_speed_delay = SLOWDELAY4;
					motor_speed_RPM = motorSpeedSlowRPM * 4;
					break;
				case STEP8: // 
					motor_speed_delay = SLOWDELAY8;
					motor_speed_RPM = motorSpeedSlowRPM * 8;
					break;
				case STEP16: //
					motor_speed_delay = SLOWDELAY16;
					motor_speed_RPM = motorSpeedSlowRPM * 16;
					break;
				case STEP32: //
					motor_speed_delay = SLOWDELAY32;
					motor_speed_RPM = motorSpeedSlowRPM * 32;
					break;
				case STEP64: //
					motor_speed_delay = SLOWDELAY64;
					motor_speed_RPM = motorSpeedSlowRPM * 64;
					break;
				case STEP128: //
					motor_speed_delay = SLOWDELAY128;
					motor_speed_RPM = motorSpeedSlowRPM * 128;
					break;
			}
			break;
		case MED: // medium
			switch (myfocuser.step_mode) {
			case STEP1: // full steps
				motor_speed_delay = MEDDELAY1;
				motor_speed_RPM = motorSpeedMedRPM;
				break;
			case STEP2: // 
				motor_speed_delay = MEDDELAY2;
				motor_speed_RPM = motorSpeedMedRPM * 2;
				break;
			case STEP4: // 
				motor_speed_delay = MEDDELAY4;
				motor_speed_RPM = motorSpeedMedRPM * 4;
				break;
			case STEP8: // 
				motor_speed_delay = MEDDELAY8;
				motor_speed_RPM = motorSpeedMedRPM * 8;
				break;
			case STEP16: //
				motor_speed_delay = MEDDELAY16;
				motor_speed_RPM = motorSpeedMedRPM * 16;
				break;
			case STEP32: //
				motor_speed_delay = MEDDELAY32;
				motor_speed_RPM = motorSpeedMedRPM * 32;
				break;
			case STEP64: //
				motor_speed_delay = MEDDELAY64;
				motor_speed_RPM = motorSpeedMedRPM * 64;
				break;
			case STEP128: //
				motor_speed_delay = MEDDELAY128;
				motor_speed_RPM = motorSpeedMedRPM * 128;
				break;
			}
		break;
	case FAST: // fast
		switch (myfocuser.step_mode {
			case STEP1: // full steps
				motor_speed_delay = FASTDELAY1;
				motor_speed_RPM = motorSpeedFastRPM;
				break;
			case STEP2: // 
				motor_speed_delay = FASTDELAY2;
				motor_speed_RPM = motorSpeedFastRPM * 2;
				break;
			case STEP4: // 
				motor_speed_delay = FASTDELAY4;
				motor_speed_RPM = motorSpeedFastRPM * 4;
				break;
			case STEP8: // 
				motor_speed_delay = FASTDELAY8;
				motor_speed_RPM = motorSpeedFastRPM * 8;
				break;
			case STEP16: //
				motor_speed_delay = FASTDELAY16;
				motor_speed_RPM = motorSpeedFastRPM * 16;
				break;
			case STEP32: //
				motor_speed_delay = FASTDELAY32;
				motor_speed_RPM = motorSpeedFastRPM * 32;
				break;
			case STEP64: //
				motor_speed_delay = FASTDELAY64;
				motor_speed_RPM = motorSpeedFastRPM * 64;
				break;
			case STEP128: //
				motor_speed_delay = FASTDELAY128;
				motor_speed_RPM = motorSpeedFastRPM * 128;
				break;
		}
		break;
	}
	mystepper.set_speed(motor_speed_RPM);      // update the motor speed
}
void writeEEPROMNow() {
	EEPROM_writeAnything(current_address, myfocuser);    // update values in EEPROM
}

void set_focuser_defaults() {
	myfocuser.validdata = VALIDDATAFLAG;
	myfocuser.max_step = 10000L;
	myfocuser.position = 5000L;
	myfocuser.coil_power = 1;
	myfocuser.reverse_direction = 0;
	myfocuser.step_mode = 1;                             // full stepping
	myfocuser.page_display_time = DISPLAYPAGETIMEMIN;
	myfocuser.step_size_enabled = 0;                  // default state is step size OFF
	myfocuser.step_size = DEFAULTSTEPSIZE;
	myfocuser.temperature_mode = 1;                          // default is celsius
	myfocuser.temperature_compensation_enabled = 0;
	myfocuser.temperature_coefficient = 0;
	myfocuser.tc_direction = 1;
	myfocuser.seven_segment_update_on_move = 0;
	myfocuser.delay_after_move = 0;
	myfocuser.backlash_steps_in = 0;
	myfocuser.backlash_steps_out = 0;
	myfocuser.focuser_direction = MOVINGIN;
	myfocuser.backlash_in_enabled = 0;
	myfocuser.backlash_out_enabled = 0;
	writeEEPROMNow();                                   // update values in EEPROM
	is_moving = 0;
	move_started = 0;
}
void store_position_in_eeprom(int position_now) {
	EEPROM.update(Last_position_address, (position_now / 0xff));
	EEPROM.update(Last_position_address, (position_now % 0xff));
}
// send update to HUB -------------------------------------------------------------------------------------------------
void send_update_to_HUB() {
	unsigned char i = 0;
	unsigned char status = 0;
	Get_Temperature();
	outgoing_hub_message.hub_outgoing_message.header = HUB_STX;									// [0] STX
	outgoing_hub_message.hub_outgoing_message.command_number = Focuser_Update_command;	// [1]	
	outgoing_hub_message.hub_outgoing_message.focuser_status = current_status;					// [2] status
	outgoing_hub_message.hub_outgoing_message.focuser_position = current_position;				// [3 - 4]
	outgoing_hub_message.hub_outgoing_message.temperature = current_temperature;				// [5 - 8]
	outgoing_hub_message.hub_outgoing_message.humidity = current_humidity;						// [9 - 12]
	outgoing_hub_message.hub_outgoing_message.day = (unsigned char) gps_day;					// [13]
	outgoing_hub_message.hub_outgoing_message.month = (unsigned char) gps_month;				// [14]
	outgoing_hub_message.hub_outgoing_message.year = (unsigned char) gps_year;					// [15]
	outgoing_hub_message.hub_outgoing_message.hour = (unsigned char) gps_hour;					// [16]
	outgoing_hub_message.hub_outgoing_message.minute = (unsigned char) gps_minute;				// [17]
	outgoing_hub_message.hub_outgoing_message.second = (unsigned char) gps_second;				// [18]
	outgoing_hub_message.hub_outgoing_message.latitude = gps_latitude;							// [19,22]
	outgoing_hub_message.hub_outgoing_message.longitude = gps_longitude;						// [23,26]
	outgoing_hub_message.hub_outgoing_message.altitude = gps_altitude;							// [27,30]
	outgoing_hub_message.hub_outgoing_message.footer = HUB_ETX;									// [31] ETX
	for (i = 0; i < HUB_outgoing_message_length; i++) {
		HUB_serial.write(outgoing_hub_message.sg_chars[i]);
	}
}
// --------------------------------------------------------------------------------------------------------------------
void gps_HardwareReset() {
	while (gps_serial.available()) { // Empty input buffer
		gps_serial.read();
	}
	while (gps_serial.available()) {
		gps_c = gps_serial.read();
		if (gps_nmea.process(gps_c)) return;
	}
}
// Controller Subroutines ---------------------------------------------------------------------------------------------
void EnableDriver() {
#ifdef DEBUG_OUTPUT
	Serial.print(millis(), DEC);
	Serial.println("\tEnable Driver started");
#endif
	resetDriver();
	writeReg(WR, 0b01111000);	// set WDEN = 0, WDT = 8,													Watchdog timer duration = 288mS
	writeReg(CR0, 0b00010100);	// set SM = 0, CUR = 2070mA,												Current 2070mA
	writeReg(CR1, 0b00000010);	// set DIRCTRL = 0, StepP = 0, PWMF = 0, PWMJ = 0, EMC = 10					if DIR = 0 CW motion, if DIR = 1 CCW motion, StepP rise fast
	writeReg(CR2, 0b00010000);  // set MOTEN = 0, SLP = 0, SLAG = 0, SLAT = 0, Speed load angle gain = 0.5, Speed load angle is transparent
	writeReg(CR3, 0b00000001);  // ESM = 1, stepmode 1/128
#ifdef DEBUG_OUTPUT
	Serial.print(millis(), DEC);
#endif
	if (readReg(WR) != 0b01111000) {
#ifdef DEBUG_OUTPUT
		Serial.println("\tWriting or reading WR failed; driver power might be off.");
#endif
		error(2);
	} else {
#ifdef DEBUG_OUTPUT
		Serial.println("\tWriting or reading WR succeeded.");
#endif
	}
#ifdef DEBUG_OUTPUT
	Serial.print(millis(), DEC);
#endif
	if (readReg(CR0) != 0b00010100) {
#ifdef DEBUG_OUTPUT
		Serial.println("\tWriting or reading CR0 failed.");
#endif
		error(3);
	} else {
#ifdef DEBUG_OUTPUT
		Serial.println("\tWriting or reading CR0 succeeded.");
#endif
	}
#ifdef DEBUG_OUTPUT
	Serial.print(millis(), DEC);
#endif
	if (readReg(CR1) != 0b00000010) {
#ifdef DEBUG_OUTPUT
		Serial.println("\tWriting or reading CR1 failed.");
#endif
		error(4);
	} else {
#ifdef DEBUG_OUTPUT
		Serial.println("\tWriting or reading CR1 succeeded.");
#endif
	}
#ifdef DEBUG_OUTPUT
	Serial.print(millis(), DEC);
#endif
	if (readReg(CR2) != 0b00010000) {
#ifdef DEBUG_OUTPUT
		Serial.println("\tWriting or reading CR2 failed.");
#endif
		error(5);
	} else {
#ifdef DEBUG_OUTPUT
		Serial.println("\tWriting or reading CR2 succeeded.");
#endif
	}
#ifdef DEBUG_OUTPUT
	Serial.print(millis(), DEC);
#endif
	if (readReg(CR3) != 0b00000001) {
#ifdef DEBUG_OUTPUT
		Serial.println("\tWriting or reading CR3 failed.");
#endif
		error(6);
	} else {
#ifdef DEBUG_OUTPUT
		Serial.println("\tWriting or reading CR3 succeeded.");
#endif
	}
	if (!readReg(WR) || !readReg(CR0) || !readReg(CR1) || !readReg(CR2) || !readReg(CR3)) {
#ifdef DEBUG_OUTPUT
		Serial.print("WR:");
		Serial.println(readReg(WR), BIN);
		Serial.print("CR0:");
		Serial.println(readReg(CR0), BIN);
		Serial.print("CR1:");
		Serial.println(readReg(CR1), BIN);
		Serial.print("CR2:");
		Serial.println(readReg(CR2), BIN);
		Serial.print("CR3:");
		Serial.println(readReg(CR3), BIN);
		Serial.println("\tResetSettings failed.");
#endif
		error(7);
	} else {
#ifdef DEBUG_OUTPUT
		Serial.println("\tResetSettings succeeded.");
#endif
	}
	stepper.enableDriver();
#ifdef DEBUG_OUTPUT
	Serial.print(millis(), DEC);
#endif
	if (stepper.driver.readReg(CR2) != 0x80) {
#ifdef DEBUG_OUTPUT
		Serial.println("\tError: Enable Driver failed.");
#endif
		error(8);
	} else {
#ifdef DEBUG_OUTPUT
		Serial.println("\tEnable Driver Succeeded.");
#endif
	}
}
void DisableDriver() {
#ifdef DEBUG_OUTPUT
	Serial.print(millis(), DEC);
	Serial.println("\tDisable Driver started");
#endif
	stepper.disableDriver();
#ifdef DEBUG_OUTPUT
	Serial.print(millis(), DEC);
#endif
	if (stepper.driver.readReg(CR2) != 0x00) {
#ifdef DEBUG_OUTPUT
		Serial.println("\tError: Disable Driver failed.");
#endif
		error(9);
	} else {
#ifdef DEBUG_OUTPUT
		Serial.println("\tDisable Driver succeeded");
#endif
	}
}
void writeReg(char address, char value) {
	stepper.driver.writeReg(address, value);
}
char readReg(char address) {
	return stepper.driver.readReg(address);
}
char readStatusReg(char address) {
	return readReg(address) & 0x7F;
}
char readCUR(void) {
	return readReg(CR0) & 0b11111;
}
void resetDriver(void) {
	Serial.print(millis(), DEC);
	Serial.println("\tresetDriver");
	digitalWrite(amisCLRPin, HIGH);
	delay(1);
	digitalWrite(amisCLRPin, LOW);
	delay(1);
	stepper.resetSettings();
	delay(1);
	Serial.print(millis(), DEC);
	Serial.println("\tFinished resetDriver");
}
void nextStep(int my_direction) {		  // The Step minimum high pulse width is 2 microseconds.
	int stepper_increment = 0;
	if ((my_direction == INWARDS) && (Check_Home() == true) || ((my_direction == OUTWARDS) && (current_position > AWAY))) {
		return;													// dont do anything
	}
	digitalWrite(amisStepPin, HIGH);	// send the step pulse to the motor driver
	delayMicroseconds(3);
	digitalWrite(amisStepPin, LOW);
	delayMicroseconds(3);
	delayMicroseconds(postStepDelayUs);
	if (my_direction == OUTWARDS) {
		current_position += stepper_increment;
	}
	else {
		current_position -= stepper_increment;
	}
}
void error(int error_number) {
	char error_number_s[2];
	dtostrf(error_number, 2, 0, error_number_s);
	sprintf(display_string, "%s", error_number_s);
	Serial.print("Error: ");
	Serial.println(display_string);
	error_display(display_string);
}
void set_register(byte reg, byte value) {   // ... write a value into a max7219 register See MAX7219 Datasheet, Table 1, page 6
	digitalWrite(display_CS, LOW);
	shiftOut(display_DIN, display_CLK, MSBFIRST, reg);
	shiftOut(display_DIN, display_CLK, MSBFIRST, value);
	digitalWrite(display_CS, HIGH);
}
void display(String thisString) { // ... display on the 7-segment display
	set_register(display_REG_SHUTDOWN, OFF);  // turn off display
	set_register(display_REG_SCANLIMIT, 7);   // scan limit 8 digits
	set_register(display_REG_DECODE, 0b00011111);		// 
	if ((thisString.charAt(0) < 0x30) || (thisString.charAt(0) > 0x39)) thisString.setCharAt(0, 0x30);
	if ((thisString.charAt(1) < 0x30) || (thisString.charAt(1) > 0x39)) thisString.setCharAt(1, 0x30);
	if ((thisString.charAt(2) < 0x30) || (thisString.charAt(2) > 0x39)) thisString.setCharAt(2, 0x30);
	if ((thisString.charAt(3) < 0x30) || (thisString.charAt(3) > 0x39)) thisString.setCharAt(3, 0x30);
	if ((thisString.charAt(4) < 0x30) || (thisString.charAt(4) > 0x39)) thisString.setCharAt(4, 0x30);
	set_register(1, thisString.charAt(4));  //  9
	set_register(2, thisString.charAt(3));	//  9
	set_register(3, thisString.charAt(2));	//  9
	set_register(4, thisString.charAt(1));	//  9
	set_register(5, thisString.charAt(0));	//  9
	set_register(6, dash);					//  
	set_register(7, P);					//  
	set_register(8, space);						//  P
	set_register(display_REG_SHUTDOWN, ON);   // Turn on display
}
void error_display(String thisString) { // ... display on the 7-segment display, thisString is the error number
	set_register(display_REG_SHUTDOWN, OFF);  // turn off display
	set_register(display_REG_SCANLIMIT, 7);   // scan limit 8 digits
	set_register(display_REG_DECODE, 0b00000011);		// 
	if ((thisString.charAt(0) < 0x30) || (thisString.charAt(0) > 0x39)) thisString.setCharAt(0, 0x30);
	if ((thisString.charAt(1) < 0x30) || (thisString.charAt(1) > 0x39)) thisString.setCharAt(1, 0x30);
	if ((thisString.charAt(3) < 0x30) || (thisString.charAt(3) > 0x39)) thisString.setCharAt(3, 0x30);
	if ((thisString.charAt(4) < 0x30) || (thisString.charAt(4) > 0x39)) thisString.setCharAt(4, 0x30);
	if ((thisString.charAt(8) < 0x30) || (thisString.charAt(8) > 0x39)) thisString.setCharAt(8, 0x30);
	if ((thisString.charAt(9) < 0x30) || (thisString.charAt(9) > 0x39)) thisString.setCharAt(9, 0x30);
	set_register(1, thisString.charAt(1));
	set_register(2, thisString.charAt(0));
	set_register(3, dash);
	set_register(4, R);		// R  
	set_register(5, O);     // O
	set_register(6, R);     // R
	set_register(7, R);		// R
	set_register(8, E);     // E
	set_register(display_REG_SHUTDOWN, ON);   // Turn on display
}
void resetDisplay() {
	set_register(display_REG_SHUTDOWN, OFF);   // turn off display
	set_register(display_REG_DISPTEST, OFF);   // turn off test mode
	set_register(display_REG_INTENSITY, 0x0D); // display intensity
}
