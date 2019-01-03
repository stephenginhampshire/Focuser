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
#include <EEPROM.h>
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
  unsigned int focuser_target;		// [2 - 3]
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
// instantiations -----------------------------------------------------------------------------------------------------
AMIS30543 stepper;
DHT_Unified dht(DHT22_PIN, DHT22);
sensors_event_t event;
sensor_t sensor;
MicroNMEA gps_nmea(gps_nmeaBuffer, sizeof(gps_nmeaBuffer));
HardwareSerial& gps_serial = GPS_serial;
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
			microsteps_target = incoming_hub_message.hub_incoming_message.focuser_target;
			Move_to(microsteps_target);
			break;
		} // end of case move_to
		case Focuser_Move_command: {
			microsteps_target = incoming_hub_message.hub_incoming_message.focuser_target;
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
