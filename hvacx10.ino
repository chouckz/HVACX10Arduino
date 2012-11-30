/* HVAC and Home automation control by Igor Demenchuk  v 2.1*/

#include <SPI.h>
#include <Ethernet.h>
#include <WebServer.h>
#include <DHT.h>
#include <stdio.h>
#include <OneWire.h>
#include <EEPROMex.h>
#include <RTClib.h>
#include <Wire.h>
#include <x10.h>
#include <Time.h> //http://www.arduino.cc/playground/Code/Time
#include <Timezone.h> //https://github.com/JChristensen/Timezone

#define _Digole_Serial_UART_  //OLED display To tell compiler compile the special communication only, 
#define LCDCol 16 //OLED display 
#define LCDRow 5//OLED display 

#include <DigoleSerial.h> //OLED display 
#include <Adafruit_BMP085.h> //Pressure sensor

// ID of the settings block
#define CONFIG_VERSION "hv21"

// Tell where to store your config data in EEPROM
#define memoryBase 256

 ///RTC_Millis RTC; //RTC emulation sinc with PC USB time
RTC_DS1307 RTC; //RTC board is connected

Adafruit_BMP085 bmp; //Pressure sensor

DigoleSerialDisp mydisp(&Serial, 9600); //OLED display //UART:Pin 1(TX)on arduino to RX on module

// no-cost stream operator as described at 
// http://sundial.org/arduino/?page_id=119
template<class T>
inline Print &operator <<(Print &obj, T arg)
{ obj.print(arg); return obj; }





//x//Adafruit_PCD8544 display = Adafruit_PCD8544(18, 17, 16, 15, 14);
//Adafruit_PCD8544 display = Adafruit_PCD8544(CLK, DIN, D/C,CE(S), RST);
// CHANGE THIS TO YOUR OWN UNIQUE VALUE
static uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xAF, 0xFE, 0xDD };

// CHANGE THIS TO MATCH YOUR HOST NETWORK
static uint8_t ip[] = { 192, 168, 1, 210 };
static uint8_t gateway[] = { 192, 168, 1, 1 };
byte mask[] = { 255, 255, 255, 0 };

const unsigned char fonts[] = {6, 10, 18, 51, 120, 123}; //OLED display fonts

#define PREFIX "/hvac"  // WEB server URL prefix eg http://192.168.1.210/hvac


/*UNO PINs assigment 
		* - sytem use
Digital
0*  system Serial  / INdoor DHT22 temp and humidity sensor
1*  system Serial / OUTdoor DHT22 temp and humidity sensor
2 - IN/OUT OneWire temp sensors
3 -OUT Garage DOOR relay
4* - Ethernet shild SD card  
5 - OUT Taking control over regular termostat relay
6 - OUT HVAC Y compressor relay
7 - OUT HVAC W aux heat relay
8 - OUT HVAC O heat/cool heat pump valve relay (ON is cooling)
9 -  OUT HVAC G fan relay
10* - Ethernet shild
11* - Ethernet shild
12* - Ethernet shild
13* - Ethernet shild


Analog
0 - HVAC G fan MONITOR
1 - HVAC O heat/cool heat pump valve(ON is cooling) MONITOR
2 - HVAC W aux heat relay MONITOR
3 - HVAC Y compressor MONITOR
4 - I2C interface (Baro pressure sensor and RTC)
5 - I2C interface (Baro pressure sensor and RTC)
*/

/* MEGA PINs assigment 
		* - sytem use
Digital
0*  system Serial 
1*  system Serial 
2 - IN/OUT OneWire temp sensors
3 -OUT Garage DOOR relay
4* - Ethernet shild SD card  
5 - OUT Taking control over regular termostat relay
6 - OUT HVAC Y compressor relay
7 - OUT HVAC W aux heat relay
8 - OUT HVAC O heat/cool heat pump valve relay (ON is cooling)
9 -  OUT HVAC G fan relay
10* - Ethernet shild
11* - Ethernet shild
12* - Ethernet shild
13* - Ethernet shild

20 - I2C interface (Baro pressure sensor and RTC)(NEW from UNO!)
21 - I2C interface (Baro pressure sensor and RTC)(NEW from UNO!)

26 - x10 Serial data transmit pin(NEW from UNO!)
28 - x10 Serial  zero crossing pin(NEW from UNO!)
30 - Serial out for external Display
32 - INdoor DHT22 temp and humidity sensor (NEW from UNO!)
34 - OUTdoor DHT22 temp and humidity sensor(NEW from UNO!)

53 - x10 Serial data receive pin NOT USED by the library!!!(NEW from UNO!)

Analog
0 - HVAC G fan MONITOR
1 - HVAC O heat/cool heat pump valve(ON is cooling) MONITOR
2 - HVAC W aux heat relay MONITOR
3 - HVAC Y compressor MONITOR
4 - (NEW from UNO!)
5 - (NEW from UNO!)
*/


// On the Ethernet Shield, CS is pin 4. Note that even if it's not 
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
//Digital pins assigment
const int chipSelect = 4; //SD
const int door_pin = 3; //PIN to triger relay for Garage door remote click 
const int control_pin = 5;  // PIN to triger relay for Taking control over regular termostat 
const int y_comp_pin = 6; //PIN to triger relay for  HVAC Y compressor
const int w_aux_pin = 7; //PIN to triger relay for HVAC W aux heat
const int o_valve_pin = 8;//PIN to triger relay for HVAC O heat/cool heat pump valve relay (ON is cooling, some HVAC are opposite OFF  is cooling)
const int g_fan_pin = 9;//PIN to triger relay for  HVAC G fan
//
const int indoor_sensor_pin = 34; //DHT22 temp and humidity sensor 
const int outdoor_sensor_pin = 32;//DHT22 temp and humidity sensor 
//const int disp_pin = 30; //OLED display serial TX
const int OneWire_pin = 2; // all OneWire temp sensors common data line
//Analog pins assigment
//FIXED!!! 4 - I2C interface in UNO
//FIXED!!! 5 - I2C interface in UNO
const int y_comp_mon_pin =  A3; // monitoring Input for  Y HVAC system
const int w_aux_mon_pin = A2; // monitoring Input for  W HVAC system
const int o_valve_mon_pin = A1; // monitoring Input for  O HVAC system (High is cooling, some HVAC are opposite Low  is cooling)
const int g_fan_mon_pin = A0; // monitoring Input for  G (FAN) HVAC system

const int rxPin = 53; // NOT USED  x10 serial interface data receive pin
const int txPin = 28; //x10 serial interface  data transmit pin
const int zcPin = 26; //x10 serial interface  zero crossing pin
const int rpm_ir= 4; //Wind Speed Meter corecponding interrupt number for a proper pin for Mega2560	(IR 0..5) cresponding interrupt s are (2	3	21	20	19	18); uno just 0 and 1 (pins the same)

int active = 0; // 0 or 1 . Currently active if state is active it meens AC or other hardware is realy ON and running now and we are controllong HVAC 
int s_state = 0; //Actual state acording to monitoring inputs of 24V on proper lines 0=stop (maybe active),1 = cooling, 2=heat, 3= auxheat, 4= ventilation

int door_delay = 100; //delay between open the relay and trasmit RF signal to the door opener and closing it. the time the button is pressed only for this number of miliseconds. lower value can save battary life. Battery can be replaced by directly powering remote by 3.3v from Arduino 
int affect_delay =100;//delay to proceed with taking HVAC control or applying changes in miliseconds

float indoor_temp = 0;
float outdoor_temp = 0;
float plenum_temp = 0; //sensor inside heat excahge cahmber so we can monitor it and shut down system if it is too hot or cold
int indoor_Humidity = 0;
int outdoor_Humidity = 0;
const int monitor_sh = 700; //threshold value. if value on analog pin is > is 1 at HVAC state monitoring. Actual valut need to be observed in you power setup enviroment. 
bool do_check_state = false; //we chack the real voltage on output to HVAC in order to shutdown the system if state mismatch
float heat_pump_sr_temp = 0.0;//temperature C below what to use AUX heat
unsigned long ltime; //Last time to follow intervals of tasks initiation 
unsigned long at; //to mesure time for each step, can be skipped

unsigned long last_control_start_time = 0; //in Arduino milliseconds
unsigned long off_time; //Arduino milliseconds when to stop taking HVAC control
long temp_read_interval = 10; //time between all 10 sensor read in seconds
long time_left =0; //time left till giving up the controls
#define kn_coof 1.7 //coof to translate RPM to KNOTS is individual for each metering device and shoould be calibrated before implemented
float kn = 0; //wind speed in knots
int gust_kn_max, gust_kn_min;
unsigned int rpmcount = 0; //temp variable for counting RPM via interrupt
bool dox10 = true; //temporary variable to let Arduino change(click) only one x10 plug at a time

int g = 0; //actual voltage on monitoring HVAC control lines
int y = 0;//actual voltage on  monitoring HVAC control lines
int o = 0;//actual voltage on monitoring HVAC control lines
int w = 0;//actual voltage on monitoring HVAC  control lines

	int hourz ; //Current hours 24h format  (without 0 in front if <10)
	int lhour =0; //hour when last time we did onece per hour action
	int minutez ; //Current minuts (without 0 in front if <10)
	int yearz; //Current year
	int monthz; //Current Month number (without 0 in front if <10)
	int dayz;//Current day (without 0 in front if <10)
unsigned long utime; //Current Unix time from RTC board

//Semsor IDs дулу MAC address are unioque to each sensor! You should chage it when putting/replacing  sensors 
//use this siple programm to get your sensors IDs
byte plenum_sensor[8] = {0x28, 0x05, 0x82, 0x11, 0x04, 0x00, 0x00, 0x94};//plenum sensor inside furnese
byte sensor_G1_R1[8] = {0x28, 0x83, 0x8D, 0x11, 0x04, 0x00, 0x00, 0xA3};//Bathroom
byte sensor_G1_R2[8] = {0x28, 0x2A, 0x2D, 0xC7, 0x03, 0x00, 0x00, 0x28};//Green bedroom 
byte sensor_G1_R3[8] = {0x28, 0x06, 0x17, 0xC7, 0x03, 0x00, 0x00, 0xDA};//Yellow bedroom 
byte sensor_G1_R4[8] = {0x28, 0xFF, 0x18, 0xC7, 0x03, 0x00, 0x00, 0x51};//Blue bedroom 
byte sensor_G3_R1[8] = {0x28, 0x9A, 0xBE, 0x50, 0x04, 0x00, 0x00, 0xCB};//Guest bedroom 
byte sensor_G3_R2[8] = {0x28, 0x6D, 0xB5, 0x50, 0x04, 0x00, 0x00, 0x4C};//Lab

 char x10house = A; //x10 house id
 char x10units[17][2] = {{ALL_LIGHTS_ON,'0'}, {UNIT_1,'0'}, {UNIT_2,'0'}, {UNIT_3,'0'}, {UNIT_4,'0'}, {UNIT_5,'0'}, {UNIT_6,'0'}, {UNIT_7,'0'}, {UNIT_8,'0'}, {UNIT_9,'0'}, {UNIT_10,'0'}, {UNIT_11,'0'}, {UNIT_12,'0'}, {UNIT_13,'0'}, {UNIT_14,'0'}, {UNIT_15,'0'}, {UNIT_16,'0'}}; //UNIT ID, Current state ON/OFF
float tempArray[][5] = {{0,0,0,0,0}, {0},{0, 0,0},{0}}; //temperatueres array in all rooms   if group have a room first is a avarage temp in all rooms in the group
 int x10plugs[][4]=		 {{1,2,3,4}, {5},{6,7},{8}}; //units numbers(as per x10units[x][0]) at certain location UNIT_XX 
int x10pools[17]={0,0,0,0,0,12,0,0,13,0,0,0,0,0,0,0,0}; // pools of x10 plugs in the same room so if it is too cold inside or outside we turn on all units until only one can maintain needed temp

bool dday = false; //if now is day according to x10 settings of day hours
 bool x10connected =true; //if x10 serial module is not connected and you try to send commands there Arduino will fereaze, so you need to dislable it when not connected or you dont want to see x10 actions.  
 bool OLED =true; // If OLED display is connected

#define infoL 7 //size of information slot for each room or group

struct StoreStruct {
	char version[5];   // This is for mere detection if they are your settings
	int state; //0=stop (maybe active),1 = cooling, 2=heat, 3= auxheat, 4= ventilation
	float set_temp ; //control over HVAC is taken	long time_left; //time left till giving up the controls
	int off_delay;// //taking control time out in minuts
	unsigned long x10_override_off;// x10 Override time out (when to turn off in lunux time RTC.now()
	int x10_override;// x10 Unit ID  
	unsigned long last_temp_read_time ;
	float delta_temp; //temperature difference that we consider as a margin when making a decision to turn on or off. Temp should be lower then (current - Delta) to turn On and higher then (current + Delta) to turn OFF. If it is between +- Delta we don't do anything
	float pool_temp; // avtivate the pools if OUTSIDE temp lower then pool_temp
	float pool_factor; // avtivate the pools if current room/group temp lower then (NEEDED TEMP - (pool_factor * delta_temp))
	char x10heating[10+infoL*10]; //main array of x10 heating settings "	'106001000g111718r111718r211718r311718r411718g211718g311718r111718r211718g411718'
	long bar[25]; //array of pressure values for the last 24 periods(hours)
} settings = {
    CONFIG_VERSION,
    0,0,0,0,0, 0, 0.5,5,4,
	"906001900g191515r132315r222220r332215r422220g232217g391515r132115r201515g432323" ,
	/* from first char left to right
	 x - type of general state 0 - all always OFF, 1- all always ON, 9 -SPLIT all acording to scedule and groups/rooms settings
	 xx - 2 chars for hour of day to begin
	 xx- 2 chars for min of day to begin
	 xx - 2 chars for hour of night to begin
	 xx- 2 chars for min of night to begin
		Gx group/flour number (1-9) 
		x - type of wanted group/room state 0 - all always OFF, 1- all always ON, 2 active day only, 2 active day and nigt,  9 -SPLIT(for groups with rooms only), 
		xx - 2 chars  of desired remperature during day
		xx - 2 chars  of desired remperature during night
				Rx room number (1-9) 
				x - type of current group/room state 0 - all always OFF, 1- all always ON, 3 active day only, 2 active day and nigt,
				xx - 2 chars  of desired remperature during day
				xx - 2 chars  of desired remperature during night
			...	
	*/
	{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

};

#define barr 16 //resolution to encode and send Bar Pressure levels to GUI/ 16 levels - is hex pressure coding system to report to web GUI

char x10State[9+(infoL+4)*10];	
	/*		"906001900g1228991515r1227032315r2239022222r3227032215r4220022221g2219122219g3163991515r1164002115r2163001515g4224032223"
	from first char left to right
	 x - type of general state 0 - all always OFF, 1- all always ON, 9 -SPLIT all acording to scedule and groups/rooms settings
	 xx - 2 chars for hour of day to begin
	 xx- 2 chars for min of day to begin
	 xx - 2 chars for hour of night to begin
	 xx- 2 chars for min of night to begin
		Gx group/flour number (1-9) 
		xxx - 3 chars  of current temperature without decimal splitter so 22.8 becomes 228
		x - type of actual current group/room state 0 -  OFF, 1- ON,   9 -SPLIT(for groups with rooms only), 
		x - type of wanted group/room state 0 - all always OFF, 1- all always ON, 2 active day only, 2 active day and nigt,  9 -SPLIT(for groups with rooms only), 
		xx - 2 chars  of desired remperature during day
		xx - 2 chars  of desired remperature during night
				Rx room number (1-9) 
				xxx - 3 chars  of current temperature without decimal splitter so 22.7 becomes 227
				x - type of actual current room state 0 -  OFF, 1- ON
				x - type of wanted group/room state 0 - all always OFF, 1- all always ON, 3 active day only, 2 active day and nigt,
				xx - 2 chars  of desired remperature during day
				xx - 2 chars  of desired remperature during night
			...	
	x - первый  знак  это тип состояния 
	 xx - 2 знака время( час) включения 
	 xx- 2 знака минуты включения 
	 xx - 2 знака время( час) выключения
	 x- 2 знака минуты выключения 
	 Gn это номер группы (для меня этаж)
	 xxx - 3 знака текущей (средней по всем комнатам) температуры в группе/этаже
	 х- знак  это тип состояния для всей группы. 0 - отопление не включено, 1 - включено. Он имеет значение если в группе нет комнат.  
	 х - знак  это тип режима  для всей группы 
	 xx - 2 знака нужной температуры днём для всей группы
	 xx - 2 знака нужной температуры ночью для всей группы
	 Rn - номер комнаты в группе
	 xxx- 3 знака текущей температуры в комнате
	 х- знак  это тип состояния для комнаты. 0 - отопление не включено, 1 - включено. 
	 х - знак  это тип режима для этой комнаты
	 xx - 2 знака нужной температуры днём для этой комнаты
	 xx - 2 знака нужной температуры ночью для этой комнаты
	 */ 

char tempArr[9+infoL*10];//temporary Array to work with char arrays like strings as intensive use of Strings crushes Arduino
 char barState[25] = 	"888888888888888888888888";	//Array of bars for Pressure chart 0 is minimun, F is maximun first is 24 old last is current
 int sensor_errors[16] = 	{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //count the number of consec eroors within a sensor and reset to 0 when good reading
 int max_errors = 10; //Maximum number of consec eroors allowed before reporting an error
char errors[256] =""; ///  errors string to send errors to web client 
byte pos =1; //scroling star for OLED display between hours and mins
bool x10active= false; //only if any of x10 plugs is ON it is true 

StoreStruct memsettings; //temporary structure to compare with before we use serrings from EEPROM

int configAdress=0; //location in EEPROM

WebServer webserver(PREFIX, 80); //Init web server


// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT in_sensor(indoor_sensor_pin, DHTTYPE);
DHT out_sensor(outdoor_sensor_pin, DHTTYPE);

OneWire  ds(OneWire_pin);  //Init OneWire sensor

int sensor_id = 1; //to go along all sensors and features settings one by one 

bool ok  = false; //loading/recovering proper settings from EPROM

//set your time zone and DAyTime Rule
 TimeChangeRule usEdt = {"EDT", Second, Sun, Mar, 2, -240}; //UTC - 4 hours
 TimeChangeRule usEst = {"EST", First, Sun, Nov, 2, -300}; //UTC - 5 hours
 Timezone usEastern(usEdt, usEst);

 TimeChangeRule *tcr; //pointer to the time change rule, use to get TZ abbrev
 time_t utc, local;

 void formCmd(WebServer &server, WebServer::ConnectionType type, char *, bool) //main function to parse web server GET and POST requests
{
    server.httpSuccess(); 
	// if we're handling a GET or POST, we can output our data here.For a HEAD request, we just stop after outputting headers. 
  if (type == WebServer::HEAD)
    return;
	
  if (type == WebServer::POST)
  {
	errors[0] = 0; //clean up error string
	if (OLED) mydisp.clearScreen(); //CLear screen
    bool repeat;
    char name[32], value[200];
    //data_log_String = ltime + " " ;
	bool save = false;
    do //parse POST params
    {
      repeat = server.readPOSTparam(name,32, value,200);

		if  (strcmp(name, "garage") == 0)// && (atoi(value) == 1))
			{
				digitalWrite(door_pin, HIGH);
				delay(door_delay); 
				digitalWrite(door_pin, LOW);
             }
		if  (strcmp(name, "x10") == 0)// && (atoi(value) == 1))
			{
			 	inversex10(atoi(value));
              }

		if  (strcmp(name, "set_temp") == 0) 
			{
				settings.set_temp = atof(value);

                        }
		if  (strcmp(name, "off_delay") == 0) 
			{
                                 settings.off_delay = atoi(value);
		      }
		if  (strcmp(name, "upd_interval") == 0) 
			{
                                 temp_read_interval = atoi(value);
								//debug//Serial.print("Refresh interval Adjusted to: ");    
								//debug//Serial.println(temp_read_interval );    
		      }
		if  (strcmp(name, "delta_temp") == 0) 
			{
                                 settings.delta_temp = atof(value);

		      }

			  
		if  (strcmp(name, "ntp") == 0) 
			{

				unsigned long ntp = strtoul(value, NULL, 0);//+	1349000000 000;		;
				if (ntp >1349000000)
			  	   	{
					//debug//Serial.print("Time Adjusted: ");    
					//debug//Serial.println(ntp );    
				//RTC.begin(DateTime( usEastern.toLocal(atol(value), &tcr)));///if RTC disconected 
					RTC.adjust(DateTime( usEastern.toLocal(ntp, &tcr)));;
					}
									//	RTC.adjust(DateTime( ntp));
			 } 
		if  (strcmp(name, "poolt") == 0) 
			{

				settings.pool_temp = atof(value);
			 } 

		 if  (strcmp(name, "poolf") == 0) 
			{

				settings.pool_factor = atof(value);
			 } 
			 
		if  (strcmp(name, "x10_override_off") == 0) 
			{
				settings.x10_override_off = strtoul(value, NULL, 0);
				if (settings.x10_override_off > utime)
					save = true;
			 } 

		 if  (strcmp(name, "x10_override") == 0) 
			{
				settings.x10_override = atoi(value);
			} 

		if  (strcmp(name, "xs") == 0) 
			{
				memset(&settings.x10heating[0], 0, sizeof(settings.x10heating));
			  for (byte i=0;i<sizeof(settings.x10heating);i++){
				settings.x10heating[i]=value[i];
					}
					//strcpy(settings.x10heating ,value);
                                // settings.x10heating = value;
								// x10heatingS = value;
								// Serial.println(sizeof(settings.x10heating));
								//Serial.println(value);
								//Serial.println(":");
					save = true;
					parsex10();
		      }		
		if  (strcmp(name, "state") == 0) 
			{
				last_control_start_time = ltime;
				settings.state = atoi(value);
				save = true;
				implement_changes();
			}
      			  
    } while (repeat);
	if (save) {
		//new_config = true;
		saveConfig(); 
	//	new_config = true;
		save = false;
		//debug//Serial.println("Saving Config updates to:");  
		//debug//Serial.println(settings.x10heating);    //to_log("New command recived: state = " + String(state) + " set_temp = " + String(set_temp,2) + " off_delay = " + String(off_delay) + " upd_interval = " + String(temp_read_interval))
	   } 
	server << "200 OK";
    return;
  }

if (type == WebServer::GET)
  {
 server << "<hvac i=\""<<indoor_temp;
  server <<"\" o=\""<<outdoor_temp;
   server  <<"\" ih=\""<<indoor_Humidity;
  server   <<"\" oh=\""<<outdoor_Humidity;
   server  <<"\" p=\""<<plenum_temp;
	server << "\" s=\""<<settings.state;
	server <<"\" c=\""<<s_state;
   server <<"\" a=\""<<active;
   server <<"\" t=\""<<settings.set_temp;
   server  <<"\" l=\""<<time_left ;//<<"\"/>";
    server  <<"\" xs=\""<<x10State;//heatingStateS;
	server  <<"\" xa=\""<<x10active;//heatingStateS;
	server  <<"\" d=\""<<dday;
    server  <<"\" b=\""<<barState;//Bar pressure graph 0-f levels
	server  <<"\" w=\""<<kn;//Wind
	server  <<"\" g=\""<<gust_kn_max;//Wind
	server  <<"\" poolt=\""<<settings.pool_temp;
	server  <<"\" poolf=\""<<settings.pool_factor;
	server  <<"\" x10_override_off=\""<<settings.x10_override_off;
	server  <<"\" x10_override=\""<<settings.x10_override;
	server  <<"\" pr=\""<<settings.bar[24] ;
	server  <<"\" e=\""<<errors;
	server  <<"\" n=\""<<utime<<"\"/>";
  }
}


void calc_time_left()  //for HVAC control
{
  if (settings.off_delay > 0)
   { off_time = last_control_start_time + settings.off_delay *60000;
    time_left = off_time - ltime;
    if (ltime > off_time) time_left = 4294967295 -ltime + off_time;
    time_left /= 60000; //to minutes
   }
   else time_left = -1;
}

void read_sensors(int i)
{
	float t;
	int r;
	  		switch (i) 
				{
				 case 1:  //IN DOOR DH22
					// Reading temperature or humidity takes about 250 milliseconds!
					// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
					indoor_temp = in_sensor.readTemperature();
					indoor_Humidity = in_sensor.readHumidity();

					  // check if returns are valid, if they are NaN (not a number) then something went wrong!
					  if (isnan(indoor_temp) || isnan(indoor_Humidity)) {
							sensor_errors[i] ++;
							if (sensor_errors[i] >= max_errors) 
								{
									indoor_temp = indoor_Humidity = 0;
									if (OLED) 
									{
										mydisp.setPrintPos(0, 4);	
										mydisp.setFont(fonts[1]);	
										mydisp.println( "In DHT22 fault");
										mydisp.setFont(fonts[6]);	
									}
									//debug//strcat (errors,"In DH22 fault! ");			
									strcpy(errors,"In DHT22 fault! ");	 errors[20] =0;
								}
					} else {
  						tempArray[1][0] = indoor_temp;
						sensor_errors[i] = 0; 
					}
					//debug//Serial.print("	Indoor:"); 
					//debug//Serial.print(tempArray[1][0]);		
					//debug//Serial.print(" C  %"); 
					//debug//Serial.println(indoor_Humidity);		
				break;
				case 2: //OUT DOOR DH22
					// Reading temperature or humidity takes about 250 milliseconds!
					// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
					outdoor_temp = out_sensor.readTemperature();
					outdoor_Humidity = out_sensor.readHumidity();

					  // check if returns are valid, if they are NaN (not a number) then something went wrong!
					  if (isnan(outdoor_temp) || isnan(outdoor_Humidity)) {
							sensor_errors[i] ++;
							if (sensor_errors[i] >= max_errors) 
								{
									outdoor_temp = outdoor_Humidity = 0;
									if (OLED) 
									{
										mydisp.setPrintPos(0, 4);	
										mydisp.setFont(fonts[1]);	
										mydisp.println( "Out DHT22 fault");
										mydisp.setFont(fonts[6]);	
									}
									//debug//strcat (errors,"In DH22 fault! ");			
									strcpy(errors,"Out DHT22 fault! ");	 errors[20] =0;
								}
					} else {
  						sensor_errors[i] = 0; 
					}
					//debug//	Serial.print("	Outdoor:"); Serial.print(outdoor_temp);	Serial.print(" C  %"); Serial.println(outdoor_Humidity);		
				break;
				case 3: 
					   t = getTemperature(plenum_sensor,0);
					   if (t != 0.0) 
							{	
								plenum_temp = t;
								sensor_errors[i] = 0; 
							}
							else 
								{
									sensor_errors[i] ++;
									if (sensor_errors[i] >= max_errors) 
										{
											plenum_temp = 0;		
											
											if (OLED) 
											{
												mydisp.setPrintPos(0, 4);	
												mydisp.setFont(fonts[1]);	
												mydisp.println("Plenum sen fault");
												mydisp.setFont(fonts[6]);	
											}
											//debug//strcat (errors,"Pl fault! ");	
											strcpy(errors,"Plenum sensor fault");	 errors[20] =0;
										}
								}
								
					  if ((plenum_temp > 50) or (plenum_temp <3)) 	//Security Check: plenum is not too  hot or freasing
						{
							settings.state = 0; 
							if (OLED) 
							{
								mydisp.setPrintPos(0, 4);	
								mydisp.setFont(fonts[1]);	
								mydisp.println("Plenum too Cold | Hot");
								mydisp.setFont(fonts[6]);	
								}
							strcpy(errors,"Pl too Cold or Hot");	 errors[20] =0;
							//debug//strcat (errors,"Pl too! ");	
							saveConfig();
							 //debug//Serial.println("Plenum too Cold or too Hot!"); 
							//Serial.println("Saving Config");		
						}
						//debug//Serial.print("	Plenum:"); 
						//debug//Serial.print(plenum_temp);		
						//debug//Serial.println(" C"); 
				break;
				case 4: 
					   t = getTemperature(sensor_G1_R1,0);
					   if (t != 0.0) 
							{	
								tempArray[0][1] = t; 
								tempArray[0][0] = 	(tempArray[0][1] + tempArray[0][2] + tempArray[0][3] +	tempArray[0][4] )/4;
								sensor_errors[i] = 0; 
							}
							else 
								{
									sensor_errors[i] ++;
									if (sensor_errors[i] >= max_errors) 
										{
											tempArray[0][1] = t; 	
											//debug//strcat (errors,"G1-R1 fault! ");	
											strcpy(errors,"G1-R1 fault!");	 errors[12] =0;
										}
								}
						//debug//Serial.print("	Bathroom:"); 
						//debug//Serial.print(t);		
						//debug//Serial.println(" C"); 

				break;
				case 5: 				
					   t = getTemperature(sensor_G1_R2,0);
						if (t != 0.0) 
							{	
								tempArray[0][2]  = t; 
								tempArray[0][0] = 	(tempArray[0][1] + tempArray[0][2] + tempArray[0][3] +	tempArray[0][4] )/4;
								sensor_errors[i] = 0; 
							}
							else 
								{
									sensor_errors[i] ++;
									if (sensor_errors[i] >= max_errors) 
										{
											tempArray[0][2] = t; 	
											strcpy(errors,"G1-R2 fault!");	 errors[12] =0;
											//debug//strcat (errors,"G1-R2 fault! ");	
										}
								}
						//debug//Serial.print("	Yellow:"); 
						//debug//Serial.print(t);		
						//debug//Serial.println(" C"); 
				break;
				case 6: 
					   t = getTemperature(sensor_G1_R3,0);
					   if (t != 0.0) 
							{	
								tempArray[0][3] = t; 
								tempArray[0][0] = 	(tempArray[0][1] + tempArray[0][2] + tempArray[0][3] +	tempArray[0][4] )/4;
								sensor_errors[i] = 0; 
							}
							else 
								{
									sensor_errors[i] ++;
									if (sensor_errors[i] >= max_errors) 
										{
											tempArray[0][3] = t; 	
											//debug//strcat (errors,"G1-R3 fault! ");	
											strcpy(errors,"G1-R3 fault!");	 errors[12] =0;
										}
								}
					   //debug//Serial.print("	Green:"); 
						//debug//Serial.print(t);		
						//debug//Serial.println(" C"); 

				break;
				case 7: 
					   t = getTemperature(sensor_G1_R4,0);
					   if (t != 0.0) 
							{	
								tempArray[0][4] = t; 
								tempArray[0][0] = 	(tempArray[0][1] + tempArray[0][2] + tempArray[0][3] +	tempArray[0][4] )/4;
								sensor_errors[i] = 0; 
							}
							else 
								{
									sensor_errors[i] ++;
									if (sensor_errors[i] >= max_errors) 
										{
											tempArray[0][4] = t; 	
											//debug//strcat (errors,"G1-R4 fault! ");	
											strcpy(errors,"G1-R4 fault!");	 errors[12] =0;
											}
								}
						//debug//Serial.print("	Blue:"); 
						//debug//Serial.print(t);		
						//debug//Serial.println(" C"); 

					   break;
				case 8: 
					   t = getTemperature(sensor_G3_R1,0);
					   if (t != 0.0) 
							{	
								tempArray[2][1] = t; 
								tempArray[2][0] = (tempArray[2][1] + tempArray[2][2] )/2; 
								sensor_errors[i] = 0; 
							}
							else 
								{
									sensor_errors[i] ++;
									if (sensor_errors[i] >= max_errors) 
										{
											tempArray[2][1] = t; 	
											//debug//strcat (errors,"G3-R1 fault! ");	
											strcpy(errors,"G3-R1 fault!");	 errors[12] =0;
										}
								}
						//debug//Serial.print("	Lab:"); 
						//debug//Serial.print(t);		
						//debug//Serial.println(" C"); 

				break;
				case 9: 
					   t = getTemperature(sensor_G3_R2,0);
					   if (t != 0.0) 
							{	
								tempArray[2][2] = t; 
								tempArray[2][0] = (tempArray[2][1] + tempArray[2][2] )/2; 
								sensor_errors[i] = 0; 
							}
							else 
								{
									sensor_errors[i] ++;
									if (sensor_errors[i] >= max_errors) 
										{
											tempArray[2][2] = t; 	
											strcpy(errors,"G3-R2 fault!");	 errors[12] =0;											
											//debug//strcat (errors,"G3-R2 fault! ");	
										}
								}
						//debug//Serial.print("	Guest:"); 
						//debug//Serial.print(t);		
						//debug//Serial.println(" C"); 
					break;
				case 10: 
				    delay(50);
					t= bmp.readTemperature();
					if ((t >0) and (t<100)) 
							{	
								tempArray[3][0] = t; 
								sensor_errors[i] = 0; 
							}
							else 
								{
									sensor_errors[i] ++;
									if (sensor_errors[i] >= max_errors) 
										{
											tempArray[3][0] = t; 	
											strcpy(errors,"BR-T  fault!");	 errors[12] =0;													
											//debug//strcat (errors,"BR-T fault! ");	
										}
								}
					//debug//Serial.print("	Basement:");
					//debug//Serial.print(t);		
					//debug//Serial.print("C,  "); 
					//debug//t = bmp.readPressure();
					//Serial.println(sizeof(settings.bar));
					//r = static_cast<int> (t < 0 ? t - 0.5 : t + 0.5);
					//m//if ((t >80000) and (t<110000))  settings.bar[24] = long(t);
					//if (settings.bar[36] >0)  bar_graph();
					//debug//Serial.print("Pressure = ");
					//debug//Serial.println(t);
													
				break;
				case 11: 
					implement_changes();///////////////////////////////////////////////////////////////////////////////////////////////////////////////
					//debug//Serial.println(" Setting HVAC..."); 
				break;
				case 12: 		
					get_wind();
				break;		  
				case 13: 
					parsex10();
				break;
				default:
					if (OLED) drawOLED();
					sensor_id =0;
				break;
				}
 }
 
  void get_wind()
  {
 					rpmcount = 0;		
					attachInterrupt(rpm_ir, rpm_fun, RISING);
					delay(1000);
					detachInterrupt(0) ;
					kn = (float)(kn + (float)rpmcount/kn_coof)/2;
					if (kn > gust_kn_max) gust_kn_max = kn;
					if (kn < gust_kn_min) gust_kn_min = kn;
 }
 
 void drawOLED() //update OLED display
 {
 				float dec;
					mydisp.setPrintPos(0, 0,_TEXT_);
					mydisp.setFont(fonts[6]);
	
					if (hourz <10) mydisp.print("0");
					mydisp.print(hourz);
					mydisp.setPrintPos(3, 0,_TEXT_);
					//mydisp.print(":");
					if (minutez <10) mydisp.print("0");
					mydisp.print(minutez); 
					if (dday) 
					{	mydisp.setMode('~');			
						mydisp.drawBox(0, 0, 15, 11);		
						mydisp.drawBox(24, 0, 15, 11);		
						mydisp.setMode('C');
					}
					mydisp.print("   ");
					mydisp.setPrintPos(5, 0,_TEXT_);
					mydisp.setFont(fonts[0]);
					mydisp.print(" ");
					mydisp.setFont(fonts[1]);	
				
					if (abs(outdoor_temp) < 10) mydisp.print(' ');
					if (outdoor_temp < 0) mydisp.print('-');
					if (outdoor_temp > 0) mydisp.print('+');
					mydisp.setFont(fonts[6]);		

				/*	if (outdoor_temp == 0) 
						{
							mydisp.print('-');
							mydisp.setFont(fonts[0]);
							mydisp.print(" ");
							mydisp.setFont(fonts[1]);
							mydisp.print(" ");
						}
					else */
						{
							mydisp.print(abs(int(outdoor_temp))); 
							mydisp.setFont(fonts[0]);
							dec = (float)(outdoor_temp-int(outdoor_temp))*10;
							if (int(dec)== 0) 
							{
								mydisp.print(" ");
								mydisp.setFont(fonts[1]);
								mydisp.print(" ");
							}
								else 
									{
										mydisp.print(".");
										mydisp.setFont(fonts[1]);
										mydisp.print(abs(dec),0);
									}		
						}
					mydisp.print(" ");			
					mydisp.setFont(fonts[6]);	

					if (gust_kn_max <10) mydisp.print(" ");
					//if (kn== 0) 
					//	{
					//		mydisp.print('-');
					//	}
					//else 
						//{
							mydisp.print(gust_kn_max); 
					//}

					mydisp.setFont(fonts[0]);		
					mydisp.print("k");	
					mydisp.setFont(fonts[1]);		
					mydisp.print(" ");						
				//	mydisp.println(int(bar[24]*0.0075+0.5));
				//	mydisp.setFont(fonts[0]);		
			//		mydisp.print("mm");	
					mydisp.setFont(fonts[6]);						
					mydisp.setPrintPos(13,0);
					mydisp.print(outdoor_Humidity); 
					mydisp.setFont(fonts[1]);			
					mydisp.print("%");		
					mydisp.setFont(fonts[6]);	
					mydisp.nextTextLine();
////////////////LINE 2////////////////
					if (tempArray[0][1] <10) mydisp.print(" ");
					if (tempArray[0][1] == 0) 
						{
							mydisp.print('-');
							mydisp.setFont(fonts[0]);
							mydisp.print(" ");
							mydisp.setFont(fonts[1]);
							mydisp.print(" ");
						}
					else 
						{
							mydisp.print(int(tempArray[0][1])); 
							mydisp.setFont(fonts[0]);
							dec = (float)(tempArray[0][1]-int(tempArray[0][1]))*10;
							//dec = int(dec);
							//dec = int((float)(tempArray[0][1]-int(tempArray[0][1]))*10);
							if (int(dec)== 0) 
							{
								mydisp.print(" ");
								mydisp.setFont(fonts[1]);
								mydisp.print(" ");
							}
								else 
									{
										mydisp.print(".");
										mydisp.setFont(fonts[1]);
										mydisp.print(dec,0);
									}
					}
					if (x10units[x10plugs[0][0]][1] == '0') 	mydisp.setColor(0);
					mydisp.drawBox(16, 13, 10, 2);	
					mydisp.setColor(1);
					mydisp.setFont(fonts[0]);									

					mydisp.print(" Y");
					mydisp.setFont(fonts[6]);		
					if (tempArray[0][2] <10) mydisp.print(" ");
					if (tempArray[0][2] == 0) 
						{
							mydisp.print('-');
							mydisp.setFont(fonts[0]);
							mydisp.print(" ");
							mydisp.setFont(fonts[1]);
							mydisp.print(" ");
						}
					else 
						{
							mydisp.print(int(tempArray[0][2])); 
							mydisp.setFont(fonts[0]);
							dec = (float)(tempArray[0][2]-int(tempArray[0][2]))*10;
							if (int(dec)== 0) 
							{
								mydisp.print(" ");
								mydisp.setFont(fonts[1]);
								mydisp.print(" ");
							}
								else 
									{
										mydisp.print(".");
										mydisp.setFont(fonts[1]);
										mydisp.print(dec,0);
									}
					}
					if (x10units[x10plugs[0][1]][1] == '0') 	mydisp.setColor(0);
					mydisp.drawBox(50, 13, 10, 2);		
					mydisp.setColor(1);
					mydisp.setFont(fonts[0]);									

					mydisp.print(" G");
					mydisp.setFont(fonts[6]);		
					if (tempArray[0][3] <10) mydisp.print(" ");
					if (tempArray[0][3] == 0) 
						{
							mydisp.print('-');
							mydisp.setFont(fonts[0]);
							mydisp.print(" ");
							mydisp.setFont(fonts[1]);
							mydisp.print(" ");
						}
					else 
						{
								mydisp.print(int(tempArray[0][3])); 
							mydisp.setFont(fonts[0]);
							dec = (float)(tempArray[0][3]-int(tempArray[0][3]))*10;
							if (int(dec)== 0) 
							{
								mydisp.print(" ");
								mydisp.setFont(fonts[1]);
								mydisp.print(" ");
							}
								else 
									{
										mydisp.print(".");
										mydisp.setFont(fonts[1]);
										mydisp.print(dec,0);
									}
						}
					if (x10units[x10plugs[0][2]][1]== '0') 	mydisp.setColor(0);
					mydisp.drawBox(84, 13, 10, 2);		
					mydisp.setColor(1);
					mydisp.setFont(fonts[0]);									

					mydisp.print(" B");
					mydisp.setFont(fonts[6]);		
					if (tempArray[0][4] <10) mydisp.print(" ");
					if (tempArray[0][4] == 0) 
						{
							mydisp.print('-');
							mydisp.setFont(fonts[0]);
							mydisp.print(" ");
							mydisp.setFont(fonts[1]);
							mydisp.print(" ");
						}
					else 
						{
							mydisp.print(int(tempArray[0][4])); 
							mydisp.setFont(fonts[0]);
							dec = (float)(tempArray[0][4]-int(tempArray[0][4]))*10;
							if (int(dec)== 0) 
							{
								mydisp.print(" ");
								mydisp.setFont(fonts[1]);
								mydisp.print(" ");
							}
								else 
									{
										mydisp.print(".");
										mydisp.setFont(fonts[1]);
										mydisp.print(dec,0);
									}
						}
					if (x10units[x10plugs[0][3]][1] == '0') 	mydisp.setColor(0);
					mydisp.drawBox(118, 13, 10, 2);		
					mydisp.setColor(1);
					mydisp.setFont(fonts[6]);		
  
////////LINE 3/////////////////	
					mydisp.setPrintPos(0, 2);		

					mydisp.setFont(fonts[0]);	
					mydisp.print(' ');
					mydisp.setFont(fonts[1]);	
					mydisp.print('@');	
					mydisp.setFont(fonts[6]);		
					if (tempArray[1][0] <10) mydisp.print(' ');
					if (tempArray[1][0] == 0) 
						{
							mydisp.print('-');
							mydisp.setFont(fonts[0]);
							mydisp.print(" ");
							mydisp.setFont(fonts[1]);
							mydisp.print(" ");
						}
					else 
						{
							mydisp.print(int(tempArray[1][0])); 
							mydisp.setFont(fonts[0]);
							dec = (float)(tempArray[1][0]-int(tempArray[1][0]))*10;
							if (int(dec)== 0) 
							{
								mydisp.print(" ");
								mydisp.setFont(fonts[1]);
								mydisp.print(" ");
							}
								else 
									{
										mydisp.print(".");
										mydisp.setFont(fonts[1]);
										mydisp.print(dec,0);
									}

						}
						if (x10units[x10plugs[1][0]][1]== '0') 	mydisp.setColor(0);
					mydisp.drawBox(26, 26, 10, 2);		
					mydisp.setColor(1);					

					mydisp.print(" ");

					mydisp.setFont(fonts[0]);	
					mydisp.print(" ");
					mydisp.setFont(fonts[1]);	
					mydisp.print("L");
					mydisp.setFont(fonts[6]);		
					if (tempArray[2][1] <10) mydisp.print(" ");
					if (tempArray[2][1] == 0) 
						{
							mydisp.print('-');
							mydisp.setFont(fonts[0]);
							mydisp.print(" ");
							mydisp.setFont(fonts[1]);
							mydisp.print(" ");
						}
					else 
						{
							mydisp.print(int(tempArray[2][1])); 
							mydisp.setFont(fonts[0]);
							dec = (float)(tempArray[2][1]-int(tempArray[2][1]))*10;
							if (int(dec)== 0) 
							{
								mydisp.print(" ");
								mydisp.setFont(fonts[1]);
								mydisp.print(" ");
							}
								else 
									{
										mydisp.print(".");
										mydisp.setFont(fonts[1]);
										mydisp.print(dec,0);
									}

					//mydisp.setMode('C');
						}
						if (x10units[x10plugs[2][0]][1]  == '0') 	mydisp.setColor(0);
							mydisp.drawBox(68, 26, 10, 2);	
					mydisp.setColor(1);							
					mydisp.print(" ");
					mydisp.setFont(fonts[0]);	
					mydisp.print(' ');
					mydisp.setFont(fonts[1]);	
					mydisp.print("G");

					mydisp.setFont(fonts[6]);		
					if (tempArray[2][2] <10) mydisp.print(" ");
					if (tempArray[2][2] == 0) 
						{
							mydisp.print('-');
							mydisp.setFont(fonts[0]);
							mydisp.print(" ");
							mydisp.setFont(fonts[1]);
							mydisp.print(" ");
						}
					else 
						{
							mydisp.print(int(tempArray[2][2])); 
							mydisp.setFont(fonts[0]);
							dec = (float)(tempArray[2][2]-int(tempArray[2][2]))*10;
							if (int(dec)== 0) 
							{
								mydisp.print(" ");
								mydisp.setFont(fonts[1]);
								mydisp.print(" ");
							}
								else 
									{
										mydisp.print(".");
										mydisp.setFont(fonts[1]);
										mydisp.print(dec,0);
									}

						}
						if (x10units[x10plugs[2][1]][1] == '0') 	mydisp.setColor(0);
							mydisp.drawBox(110, 26, 10, 2);		
					mydisp.setColor(1);
					mydisp.setFont(fonts[6]);		
					mydisp.nextTextLine();			  
//////////////LINE 4///////////////////////
					
					mydisp.setPrintPos(1, 3);
					mydisp.setFont(fonts[1]);		

					if ((plenum_temp - tempArray[1][0])>=0 )	mydisp.print('+');
					mydisp.println((plenum_temp - tempArray[1][0]) ,0);
					if ((plenum_temp - tempArray[1][0])<10) mydisp.print(' ');

					mydisp.setFont(fonts[0]);	
					mydisp.print(" B");
					mydisp.setFont(fonts[6]);	
					if (tempArray[3][0] <10) mydisp.print(" ");
					mydisp.print(int(tempArray[3][0])); 
					mydisp.setFont(fonts[0]);
					dec = (float)(tempArray[3][0]-int(tempArray[3][0]))*10;
					if (int(dec)== 0) 
					{
						mydisp.print(" ");
						mydisp.setFont(fonts[1]);
						mydisp.print(" ");
					}
						else 
							{
								mydisp.print(".");
								mydisp.setFont(fonts[1]);
								mydisp.print(dec,0);
							}
					if (x10units[x10plugs[3][0]][1] == '0') 	mydisp.setColor(0);
					mydisp.drawBox(50, 39, 10, 2);		
					mydisp.setColor(1);
					mydisp.print(" ");
					mydisp.setFont(fonts[6]);		
					
					//mydisp.println(int(bar[24]*0.0075+0.5));
					mydisp.println(int(settings.bar[24]*0.0075+0.5));
					mydisp.setFont(fonts[0]);		
					mydisp.print("mm");	
					mydisp.setFont(fonts[6]);		
					mydisp.setPrintPos(13, 3);
					mydisp.print(indoor_Humidity); 
					mydisp.setFont(fonts[1]);		
					mydisp.print("%");	
					mydisp.setFont(fonts[6]);	
 }
 
void rotate() 	//show animated rotating star on display between hours and minutes
{
	switch (pos) 
		{
			case 1: 
				mydisp.drawStr(2, 0, "|");
			break;
			case 2: 
				mydisp.drawStr(2, 0, "/");
			break;
			case 3: 
				mydisp.drawStr(2, 0, "-");
			break;
			case 4: 
				mydisp.drawStr(2, 0, "\\");
			break;			
			case 5: 
				mydisp.drawStr(2, 0, "|");
			break;
			case 6: 
				mydisp.drawStr(2, 0, "/");
			break;			
			case 7: 
				mydisp.drawStr(2, 0, "-");
			break;
			case 8: 
				mydisp.drawStr(2, 0, "\\");
				pos =0;
			break;	
		}
			pos++;
		if (dday) 
					{	mydisp.setMode('~');	
						mydisp.drawBox(16, 0, 7, 11);		
						mydisp.setMode('C');
					}
		if (pos % 2)
		{
					if (settings.x10heating[0] == '9')			
						{
							mydisp.drawStr(0, 3,"X");
							 mydisp.digitalOutput(0B00000010);
						}
		}
		else
		{
		int cs =settings.state;
		if (cs ==0) cs = s_state;
					switch (cs) 
					//switch (state) 
							{
							 case 1: 
								mydisp.drawStr(0, 3,"C");
								 mydisp.digitalOutput(0B00000100); // RGB LED Blue -> 100
							break;
							case 2: 
								mydisp.drawStr(0, 3,"H");
								   mydisp.digitalOutput(0B00000011);// RGB LED Green + Red = Yellow -> 011
							break;
							case 3: 
								mydisp.drawStr(0, 3,"A");
								 mydisp.digitalOutput(0B00000001);// RGB LED Red -> 001
							break;
							case 4: 
								mydisp.drawStr(0, 3,"V");
								   mydisp.digitalOutput(0B00000111);// RGB LED Blue + Green + Red = White -> 111
							break;
							default:
								if (settings.x10heating[0] == '9')			
									{
										mydisp.drawStr(0, 3,"X");
										 mydisp.digitalOutput(0B00000010);// RGB LED Green-> 010
									}
							break;
							} //g switch
		}
		if ((active == 1)  or (x10active)) 
			{
				mydisp.setMode('~');
				mydisp.drawBox(0, 39,7, 11);		
				mydisp.setMode('C');	
			}
			
}

void bar_graph() //build a char array for Pressure graph
{
					//debug//Serial.print("Pressure = ");
					//debug//Serial.println(settings.bar[24]);
					long bmax = settings.bar[24];
					long bmin = settings.bar[24];
					int i;
					float x ;
					int r ;
					char cbar[2] ;
					
					for (i=1;i<25;i++)
					  {
							if (settings.bar[i] >0)
							{
								settings.bar[i-1] = settings.bar[i];
								if (settings.bar[i]<bmin)  bmin = settings.bar[i];
								if (settings.bar[i]>bmax)  bmax = settings.bar[i];

								}
						}
					float unit = (float)(bmax-bmin)/(barr-1); //barr levels - is hex pressure coding system to report to web GUI
						
					for (i=0;i<24;i++)
					  {
					  	if (settings.bar[i] >0)
							{
								x = (float)(settings.bar[i]-bmin)/unit;
								r = floor(x+0.5); 
								itoa (r,cbar,16);
								barState[i] =cbar[0];
							}
						}
}

void implement_changes() //implement HVAC 
{
  calc_time_left();
  if (((ltime > off_time) && (off_time > last_control_start_time)) || 
        (((ltime > off_time) && (ltime < last_control_start_time))&& (settings.off_delay > 0)))
      { 			
		   if (settings.state != 0)
		   {
				settings.state = 0; 
				if (OLED) 
				{
					mydisp.setPrintPos(0, 4);	
					mydisp.setFont(fonts[1]);	
					mydisp.print(hourz);
					mydisp.print(":");
					mydisp.print(minutez);
					mydisp.println(" HVAC ST by time");
					mydisp.setFont(fonts[6]);	
				}
				active = 0; 
				saveConfig();
			}
		}
      
  if  (settings.state == 0) 
         {
            active = 0;
			do_check_state = false; 
            delay(affect_delay); 
            digitalWrite(y_comp_pin, LOW);
            digitalWrite(o_valve_pin, LOW);
            digitalWrite(w_aux_pin, LOW);
            digitalWrite(g_fan_pin, LOW);
            delay(affect_delay); 
            digitalWrite(control_pin, LOW);
         }      
   else
      {
        if  (settings.state == 1)
         {
           if  (indoor_temp > (settings.set_temp + settings.delta_temp)) 
           {
            delay(affect_delay); 
            digitalWrite(control_pin, HIGH);
            delay(affect_delay); 
            digitalWrite(o_valve_pin, HIGH);
            delay(affect_delay); 
            digitalWrite(y_comp_pin, HIGH);
            digitalWrite(g_fan_pin, HIGH);
            digitalWrite(w_aux_pin, LOW);
            active = 1;
            }
           if  (indoor_temp < (settings.set_temp - settings.delta_temp))
            {
              delay(affect_delay); 
              digitalWrite(control_pin, HIGH);
              delay(affect_delay); 
              digitalWrite(y_comp_pin, LOW);
              digitalWrite(o_valve_pin, HIGH);
              digitalWrite(g_fan_pin, LOW);
              digitalWrite(w_aux_pin, LOW);
              active = 0;
            }
          }
       
         if  (settings.state == 2) 
        {
           if  ((settings.set_temp  - settings.delta_temp) > indoor_temp) 
           {
            delay(affect_delay); 
            digitalWrite(control_pin, HIGH);
            delay(affect_delay); 
            digitalWrite(o_valve_pin, LOW);
            delay(affect_delay); 
            digitalWrite(y_comp_pin, HIGH);
            digitalWrite(g_fan_pin, HIGH);
            digitalWrite(w_aux_pin, LOW);
            active = 1;
           }
           if  ((settings.set_temp  + settings.delta_temp) < indoor_temp) 
            {
              delay(affect_delay); 
              digitalWrite(control_pin, HIGH);
              digitalWrite(y_comp_pin, LOW);
              digitalWrite(o_valve_pin, LOW);
              digitalWrite(g_fan_pin, LOW);
              digitalWrite(w_aux_pin, LOW);
              active = 0;
            }
          }
         
         if  (settings.state == 3) 
         {
           if  ((settings.set_temp  - settings.delta_temp) > indoor_temp) 
           {
            delay(affect_delay); 
            digitalWrite(control_pin, HIGH);
            delay(affect_delay); 
            digitalWrite(o_valve_pin, LOW);
            delay(affect_delay); 
            digitalWrite(y_comp_pin, LOW);
            digitalWrite(w_aux_pin, HIGH);
            digitalWrite(g_fan_pin, HIGH);
            active = 1;
           }
           if  ((settings.set_temp  + settings.delta_temp) < indoor_temp) 
            {
              delay(affect_delay); 
              digitalWrite(control_pin, HIGH);
              digitalWrite(o_valve_pin, LOW);
              delay(affect_delay); 
              digitalWrite(y_comp_pin, LOW);
              digitalWrite(g_fan_pin, LOW);
              digitalWrite(w_aux_pin, LOW);
              active = 0;
            }
          }
        if  (settings.state == 4) 
         {
            delay(affect_delay); 
            digitalWrite(control_pin, HIGH);
            delay(affect_delay); 
            digitalWrite(y_comp_pin, LOW);
            digitalWrite(o_valve_pin, LOW);
            digitalWrite(w_aux_pin, LOW);
            digitalWrite(g_fan_pin, HIGH);
            active = 1;
           }
	}
	
	get_state();
	if (active == 1)	 //security checks if actual vlotage on the controls is the same as desired to STOP in case of any relays, connections  or other failues.
		{
			if ((s_state != settings.state) and (do_check_state)) 
				{
					char tc[2];
					if (OLED) 
					{
						mydisp.setPrintPos(0, 4);	
						mydisp.setFont(fonts[1]);	
						itoa (s_state,tc,10);
						mydisp.print(hourz);
						mydisp.print(":");
						mydisp.print(minutez);
						mydisp.print(" ");
						mydisp.println(tc);
						mydisp.println(" insted ");
						itoa (settings.state,tc,10);
						mydisp.println(tc);
						mydisp.println(" !");
						mydisp.setFont(fonts[6]);		
					}
					settings.state = 0; //system should be stopped if actual state mesured mismatch set state 
					//saveConfig();
				}
				do_check_state = true; 
		}
}

void get_state() //check actual HVAC lines 24VAC state 
{
s_state = 0;
int g = analogRead(g_fan_mon_pin);
int y = analogRead(y_comp_mon_pin);
int o = analogRead(o_valve_mon_pin);
int w = analogRead(w_aux_mon_pin);
/*Serial.print("G=");
Serial.print(g);
Serial.print(" Y=");
Serial.print(y);
Serial.print(" O=");
Serial.print(o);
Serial.print(" W=");
Serial.println(w);
						mydisp.setPrintPos(0, 4);	
						//debug//strcat (errors,"SetTo=");	
						mydisp.setFont(fonts[1]);	
						mydisp.print("G");
						mydisp.print(g);
						mydisp.print(" Y");
						mydisp.print(y);
						mydisp.print(" O");
						mydisp.print(o);
						mydisp.print(" W");
						mydisp.print(w);	
						mydisp.setFont(fonts[6]);		*/

if (g>monitor_sh) s_state = 4;
if (y>monitor_sh)
  {
   if (o>monitor_sh) { s_state = 1; }
   else { s_state = 2; }
  }
if (w>monitor_sh) s_state = 3;
}

float getTemperature(byte* address, byte type_s){ //read OneWire sensor
  byte i;
  byte present = 0;
  //byte type_s = 0;
  byte data[12];
  byte addr[8];
  int tr;
  float celsius, fahrenheit;
  //reset the bus
  ds.reset();
  //select our sensor
  ds.select(address);
  //CONVERT T function call (44h) which puts the temperature into the scratchpad
  ds.write(0x44,1);
  //sleep a second for the write to take place
  delay(1000);
    //reset the bus
  ds.reset();
  //select our sensor
  ds.select(address);
  //read the scratchpad (BEh)
  ds.write(0xBE);
  for (byte i=0;i<9;i++){
    data[i] = ds.read();
  }
   // convert the data to actual temperature

  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;   //fahrenheit = celsius * 1.8 + 32.0;
    if ((celsius< 1000) and (celsius != 85))	return celsius;  //if sensor is disconected it shows 4000c value /// sometimes getting this 85deg, not sure why. if sensor is conected and you still getting a lot of errors decrese pull up resistor (i have to use 2.3kOhm) 
  else return 0.0;
}

int freeRam () { //check free RAM
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void inversex10(int unit) //Inverse current value of specific x10 UNIT usefull for ON/OFF light 
{
	if  (x10units[unit][1] == '1') 	
			postx10(unit,OFF);
		else 
			postx10(unit,ON);
}

void sendx10(int j, int kk, char action) //send x10 command to all of specific Group/ room. If it is not empty group ot will send same command to all rooms in it  
{
	if (dox10)
		{
							int lr = (sizeof(x10plugs[0])/sizeof(x10plugs[0][0]));	
							int lg = (sizeof(x10plugs)/sizeof(x10plugs[0]));				

							if (x10connected) x10.beginTransmission(x10house);//x10house
							if (j>0) { lg = j;j--;}
							if (kk >0) 
								{
									lr =kk;kk--; 
								}
							Serial.print(" (");
							for( j=j; j<lg ; j++ ) 
								{
									for(int k=kk;k<lr;k++) 
										{
											if  (x10plugs[j][k] !=NULL)// ((strcmp(unit, "")) != 0) 
												{											

													int p =x10plugs[j][k];
														if ((x10connected)and (dox10)) x10.write(x10units[p][0]);//	x10.write(UNIT_1);	
													if (action == ON) 
														{
															if (x10units[p][1]== '0') dox10 = false;
															x10units[p][1]= '1';
														}
														else  
														{	
															if (x10units[p][1]== '1') dox10 = false;
															x10units[p][1] = '0';
														}
													Serial.print("UNIT_");
													Serial.print(p); 
													Serial.print(",");
												}
										} 
								} 
							if (x10connected) x10.write(action);	
							Serial.print(") ");
							if (action == ON) 
								{
									Serial.print("ON");
								}
								else 
								{
									Serial.print("OFF");
								}
							if (x10connected) x10.endTransmission();	
		} //dox10
}

void postx10(int unit, char action) //post individual ON/OFF action to specific x10 UNIT
{
	if (x10connected) 
		{					
			x10.beginTransmission(x10house);//x10house	
			x10.write(x10units[unit][0]);//	
			x10.write(action);	
			Serial.print("UNIT_");
			Serial.print(unit);			
			if (action == ON)
				{
					x10units[unit][1]= '1';Serial.println(" ON");
				}
				else  
				{
					x10units[unit][1] = '0'; Serial.println(" OFF");
				}
			x10.endTransmission();	
		}
			Serial.print("UNIT_");
			Serial.print(unit);			
			if (action == ON)
				{
					Serial.println(" ON");
				}
				else  
				{
					Serial.println(" OFF");
				}
}

void refreshUnit(const char* c, int size, int g, int r, bool a, 	int s) //recurrent function to check if certain Room or Group should be turned on or off including pools 
{		
char cur[size] ;cur[0]=0; //= c; 		
strcat (cur,c);	

		if (a)
			{
				//Serial.print("G=");		Serial.print(g);		Serial.print(" ");		Serial.print("R=");		Serial.println(r);
				a = false; 
				tempArr[0] = cur[2];		tempArr[1] 	=0;			
				s = atoi(tempArr);//cur.substring(2,3).toInt();//master state
				//tempArr = subStr(cur,3,2);
				strncpy(tempArr, cur+3, 2); tempArr[2] =0;
				int dtemp =  atoi(tempArr) ;
				strncpy(tempArr, cur+5, 2); tempArr[2] =0;
				int ntemp = atoi(tempArr);
				float xtemp = ntemp;
				if (dday) xtemp = dtemp;
				int uid;
				if (r == 0) uid =x10plugs[g-1][r];
					else uid = x10plugs[g-1][r-1];
													switch (s) 
															{
																case 2: 
																	if (dday) 
																		{	
																			if  (tempArray[g-1][r] < (dtemp - settings.delta_temp))
																				s = 1;
																			if  (tempArray[g-1][r] > (dtemp + settings.delta_temp))
																				s = 0;
																			}
																		else //night
																			{
																				if  (tempArray[g-1][r] < (ntemp - settings.delta_temp))
																					s = 1;
																				if  (tempArray[g-1][r] > (ntemp + settings.delta_temp))
																					s = 0;
																			}
																break;
																case 3: 
																	if (dday) 
																		{
																			if  (tempArray[g-1][r] < (dtemp - settings.delta_temp))
																				s = 1;
																			if  (tempArray[g-1][r] > (dtemp + settings.delta_temp))
																				s = 0;
																		}
																		else
																			s = 0;
																	break;
																case 9: 
																		a = true;
																break;																			
															} //g switch

							if ((s==2) or (s==3)) //if temp = needed +- delta we need to keep old state
								{
									s = 0;
									if (x10units[uid][1] == '1')  s = 1;
								}		
	
							if (uid== settings.x10_override)//Overriding
								{
									
								if (((r==0) and (x10plugs[g-1][r+1] ==0)) or (r>0))//Overriding
								{
									if ( utime<	settings.x10_override_off ) 
										s = 1;
										else
												{
													settings.x10_override = 0;
													settings.x10_override_off = 0;
												}
									}
								}
						 	if (s ==1)		//final submit  and pools
								{
									sendx10(g,r,ON);
									x10active=true;	
									
									if  ((outdoor_temp < settings.pool_temp) or (xtemp > (tempArray[g-1][r] + settings.delta_temp * settings.pool_factor) ))
										{

											if (x10pools[uid] != 0)  postx10(x10pools[uid],ON);
										}
								}
						 	if (s ==0)		
								{
									sendx10(g,r,OFF); 
									if (x10pools[uid] != 0)  postx10(x10pools[uid],OFF);
								}
					} //if a						
				int i = getposition(cur,size, 'r',1);		
				char tc[infoL]; 
				strncpy(tc, cur, 2); 	tc[2] =0;
				strcat (x10State,tc);		
				int f =floor(tempArray[g-1][r]*10 + 0.5); 		
				if (f  < 0)	f += 1000;																		
				if (f  < 10)	strcat (x10State,"0");	
				if (f  < 100)	strcat (x10State,"0");	
				itoa (f,tc,10);
				strcat (x10State,tc);	
								
				itoa (s,tc,10);
				strcat (x10State,tc);		

				strncpy(tc, cur+2, 5); 	tc[5] =0;
				strcat (x10State,tc);	
				if (i >0)
					{
						int rr =1;
						int j;
						while (i < size)
							{
								j = i;								
								i = getposition(cur, size, 'r',i+1);		
								if (i == -1) i = size;
								strncpy(tempArr, cur+j, i-j); tempArr[i-j] =0;
								refreshUnit(tempArr,i-j+1,g,rr,a,s);
								rr++;
							}	
					} //(r==0)
}


void parsex10(){ 
	x10active= false;
	dox10 = true;
	bool a = false;//to take action or not(just browse through all tree
		
			if (settings.x10_override >13) 
				{
				if  ( utime <	settings.x10_override_off )
					postx10(settings.x10_override,ON);
				else 
					{ 	
						postx10(settings.x10_override,OFF);
						settings.x10_override = 0;
						settings.x10_override_off = 0;
					}
				}									
				int ctime = hourz*60+minutez;		

				strncpy(tempArr, settings.x10heating+1, 2); tempArr[2] =0;
				int dtime =  atoi(tempArr)*60 ;
				strncpy(tempArr, settings.x10heating+3, 2); tempArr[2] =0;
				dtime += atoi(tempArr);
				strncpy(tempArr, settings.x10heating+5, 2); tempArr[2] =0;
				int ntime  =  atoi(tempArr)*60 ;
				strncpy(tempArr, settings.x10heating+7, 2); tempArr[2] =0;
				ntime += atoi(tempArr);
				dday = false;
				if (((dtime<ntime) and (ctime >= dtime) and (ctime < ntime)) or ((dtime > ntime) and ((ctime >= dtime) or (ctime < ntime))) )
						{	
							dday = true;	//debug//Serial.println("-DAY-");	
						}
				tempArr[0] = settings.x10heating[0];	tempArr[1] 	=0;	
				int general_system_mode = atoi(tempArr);// inString.substring(0,1).toInt(); //current master state

				switch (general_system_mode) 
					{
						case 1: 
							sendx10(0,0,ON);
						break;
						case 0: 
							sendx10(0,0,OFF);
						break;
						case 9: 
							a = true;
						break;	
					}// switch 	
				int i = getposition(settings.x10heating, sizeof(settings.x10heating), 'g',1);		
				strncpy(x10State, settings.x10heating, i);	x10State[i] = 0;			
				int g =1;	
				int j;
	
				while (i <sizeof(settings.x10heating))
					{
						j = i;								
						i = getposition(settings.x10heating, sizeof(settings.x10heating), 'g',i+1);		
						if (i == -1) i = sizeof(settings.x10heating);
						strncpy(tempArr, settings.x10heating+j, i-j); tempArr[i-j] =0;
						refreshUnit(tempArr,i-j+1,g,0,a,general_system_mode);
						g++;
					} //while	
}

int getposition(const char* a, int arr_size, char to_find, int from) //do the same as IndexOf for Strings
{
    int pos = -1;
    for(int i = from; i < arr_size; ++i)
    {
        if(a[i] == to_find)
        {
            pos = i;
            break;
        }
    }
    return pos;
}


bool loadConfig() {
	//debug//Serial.print("Config restored from EEPROM");
  EEPROM.readBlock(configAdress, memsettings);
  return (strcmp(memsettings.version, CONFIG_VERSION) == 0) ;
}

void saveConfig() {
	EEPROM.writeBlock(configAdress, settings);
}

 void rpm_fun() //used by interrupt to count RPM
 {
   rpmcount++;
   //Each rotation, this interrupt function is run twice
 }

void loop()
{
  ltime = millis();
  if ((ltime >= (settings.last_temp_read_time + temp_read_interval * 100)) or (ltime < settings.last_temp_read_time ))
  {
  	//debug//Serial.print("FreeRam:");
    //debug//Serial.println(freeRam());	
		if (sensor_id ==1)
			{
				DateTime now = RTC.now();	 
				utime = now.unixtime();
				hourz = now.hour();
				minutez =  now.minute(); 
				yearz = now.year();
				monthz =  now.month(); 
				dayz = now.day();
			}
	//debug//at = millis();
	//debug//Serial.print(sensor_id);
	//debug//Serial.println(":");
	if ((active == 0)  and (!(x10active))) 
		mydisp.digitalOutput(0B00000000); //turn off RGB LED connected to OLED display
		
    read_sensors(sensor_id);
	if (OLED) rotate();
	//debug//Serial.print(millis()-at);    
	//debug//Serial.println("ms");
    settings.last_temp_read_time = ltime;
	sensor_id++;
	if (lhour != hourz) //every hour check and update pressure and wind Gusts
	{
		gust_kn_min = 999;
		gust_kn_max = 0;
		get_wind();
  		delay(60); //delay for proper I2C readings for two devices on one bus		
		float t = bmp.readPressure();
		if ((t >80000) and (t<110000)) 
				{	
					settings.bar[24] = long(t);
					sensor_errors[0] = 0; 
				}
				else 
					{
						sensor_errors[0] ++;
						if (sensor_errors[0] >= max_errors) 
							{
								strcpy(errors,"BR-P  fault!");	 errors[12] =0;			
							}
					}
		//debug//Serial.print("Pressure = ");
		//debug//Serial.println(t);
		bar_graph();	
		saveConfig();
		lhour = hourz;
	}

  }
  char buff[64];
  int len = 64;

  // process incoming connections one at a time forever 
  webserver.processConnection(buff, &len);

 }

void setup()
{ 

  Serial.begin(9600);	 
    	//debug//Serial.print("FreeRam:");
    //debug//Serial.println(freeRam());
  Serial.print("=== Starting :");
  pinMode(door_pin, OUTPUT);
  pinMode(control_pin, OUTPUT);
  pinMode(y_comp_pin, OUTPUT);
  pinMode(o_valve_pin, OUTPUT);
  pinMode(w_aux_pin, OUTPUT);
  pinMode(g_fan_pin, OUTPUT);

  Wire.begin();    

	EEPROM.setMemPool(memoryBase, EEPROMSizeMega); //Set memorypool base to 32, assume Arduino Uno board
	configAdress  = EEPROM.getAddress(sizeof(StoreStruct)); // Size of config object
	ok = loadConfig();
  
  if (ok)  {	settings = memsettings;} 		
  else 
		saveConfig();

    RTC.begin(); //if RTC conected 
	///RTC.begin(DateTime(__DATE__, __TIME__));//if RTC module not conected , get time from USB

 	bmp.begin();
	in_sensor.begin();
	out_sensor.begin();
	//debug//x10connected = false;
	  if (x10connected) x10.begin(rxPin, txPin, zcPin);

	 Ethernet.begin(mac,ip,gateway,mask);  
	webserver.setDefaultCommand(&formCmd);
	webserver.begin();

	if (OLED) 
	{
		mydisp.begin();
		mydisp.displayConfig(0);    //set config display ON, 0=off
		mydisp.disableCursor(); //disable cursor, enable cursore use: enableCursor();
		mydisp.clearScreen(); //CLear screen
	}
}



