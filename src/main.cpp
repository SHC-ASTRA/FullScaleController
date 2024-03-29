// Imports!
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <MicroNMEA.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <ADC.h>
#include <ADC_util.h>
#include <VescUart.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//*******************************
// RGB Strip Configuration
//*******************************
#define LED_PIN 2
#define LED_COUNT 64
#define SERIAL_TX_LED 0
#define SERIAL_RX_LED 1
#define BORDER_START_LED 4
#define BORDER_END_LED 63

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// RGB Strip Helped Functions
void clear_strip(uint32_t color);
void set_single_color(int pin, uint32_t color);
// RGB Strip Color Constants
const uint32_t serial_tx_color = strip.Color(255, 0, 0, 0);
const uint32_t serial_rx_color = strip.Color(255, 128, 0);
const uint32_t clear_color = strip.Color(0, 0, 0);
const uint32_t border_color = strip.Color(255, 0, 127);
const uint32_t teleop_color = strip.Color(0, 0, 255);
const uint32_t autonomous_color = strip.Color(255, 0, 0);
const uint32_t goal_color = strip.Color(0, 255, 0);
bool isGoal = false;

//*******************************
// GPS Module Configuration
//*******************************
#define GPSSerial Serial6
SoftwareSerial mySerial =  SoftwareSerial(10, 11);
#define GPSSerial2 mySerial
Adafruit_GPS GPS(&GPSSerial);
Adafruit_GPS GPS2(&GPSSerial2);
void setupAdafruitGPS();
void publishGPSData();

//*******************************
// Motor Drivers Configuration
//*******************************
int frleftMotorSpd = 0;
int frrightMotorSpd = 0;
int bkleftMotorSpd = 0;
int bkrightMotorSpd = 0;
float maxPower = 0.5;
float frRightPower = 0;
float frLeftPower = 0;
float bkRightPower = 0;
float bkLeftPower = 0;
int MAX_SPEED = 6000;
u_int32_t lastPacketTime = 0;

VescUart VESC1; // Front Right
VescUart VESC2; // Front Left
// VESC 3 (Back Right) is disabled due to a hardware fault
VescUart VESC4; // Back Left
VescUart VESC5; // AUX1 (Used for Back Right)
// VESC 6 (Aux2) is not needed

void calculateTankControlSpeeds(float magnitude, float direction, float speed);
void calculateMotorSpeeds(float magnitude, float direction, float speed);

//*******************************
// Battery Voltage Reader Config
//*******************************
float cells = 8;				   // Number of battery cells present
float minBatVoltage = 3.4 * cells; // Below this voltage motors should refuse to function
float maxBatVoltage = 4.2 * cells; // Battery voltage that should be considered fully charged
float fullAmpHours = 20;

float batteryVoltage;
float batteryCharge;

u_int32_t lastBatteryTime = 0;
void publishBatteryData();

//*******************************
// Command Handling
//*******************************
#define JSerial Serial4
// #define JSerial Serial
void parseCommand(String command);
void setMotors(float magnitude, float direction);

byte msgCount = 0;		  // count of outgoing messages
byte localAddress = 0x42; // address of this device
byte destination = 0x69;  // destination to send to
long lastSendTime = 0;	  // last send time
int interval = 1000;	  // interval between sends
bool ackCommand = false;  // acknowledge reception of last command
bool enableJetson = true;

struct dataPacket
{
	float latitude;
	float longitude;
	float battvoltage;
	uint8_t mode;
	unsigned long elapsed_time;
	unsigned long last_jetson_packet_time;
	int left_wheels_rpm;
	int right_wheels_rpm;
	uint8_t autonomous_status;
	float dist_to_goal;
	bool command_ack;
};

struct commandPacket
{
	bool halt;
	bool ignore_jetson;
	bool command_drive;
	int left_rpm_command;
	int right_rpm_command;
	bool change_frequency;
	char lora_frequency;
	bool command_navigate;
	float command_latitude;
	float command_longitude;
};

void s()
{
	JSerial.print("status;");
}

void setup()
{
	// LED Ring
	strip.begin();
	strip.setBrightness(255);

	clear_strip(clear_color);

	// Serial Communications
	JSerial.begin(115200);

	// I2C - For GPS Module
	Wire.begin();
	Wire.setClock(400000);

	// GPS Module
	setupAdafruitGPS();

	// Setup VESC Motors
	Serial8.begin(115200);
	Serial7.begin(115200);
	Serial2.begin(115200);
	Serial5.begin(115200);

	VESC1.setSerialPort(&Serial8);
	VESC2.setSerialPort(&Serial7);
	VESC4.setSerialPort(&Serial2);
	VESC5.setSerialPort(&Serial5);

	if (VESC1.getVescValues())
	{
		s();
		JSerial.println("VESC 1 Connected!");
		s();
		JSerial.println(VESC1.data.id);
		s();
		JSerial.println(VESC1.data.rpm);
		s();
		JSerial.println(VESC1.data.inpVoltage);
		s();
		JSerial.println(VESC1.data.ampHours);
		s();
		JSerial.println(VESC1.data.tachometerAbs);
	}
	else
	{
		s();
		JSerial.println("No VESC1");
	}
	if (VESC2.getVescValues())
	{
		s();
		JSerial.println("VESC 2 Connected!");
		s();
		JSerial.println(VESC2.data.id);
		s();
		JSerial.println(VESC2.data.rpm);
		s();
		JSerial.println(VESC2.data.inpVoltage);
		s();
		JSerial.println(VESC2.data.ampHours);
		s();
		JSerial.println(VESC2.data.tachometerAbs);
	}
	else
	{
		s();
		JSerial.println("No VESC2");
	}
	if (VESC4.getVescValues())
	{
		s();
		JSerial.println("VESC 4 Connected!");
		s();
		JSerial.println(VESC4.data.id);
		s();
		JSerial.println(VESC4.data.rpm);
		s();
		JSerial.println(VESC4.data.inpVoltage);
		s();
		JSerial.println(VESC4.data.ampHours);
		s();
		JSerial.println(VESC4.data.tachometerAbs);
	}
	else
	{
		s();
		JSerial.println("No VESC4");
	}
	if (VESC5.getVescValues())
	{
		s();
		JSerial.println("VESC 5 Connected!");
		s();
		JSerial.println(VESC5.data.id);
		s();
		JSerial.println(VESC5.data.rpm);
		s();
		JSerial.println(VESC5.data.inpVoltage);
		s();
		JSerial.println(VESC5.data.ampHours);
		s();
		JSerial.println(VESC5.data.tachometerAbs);
	}
	else
	{
		s();
		JSerial.println("No VESC5");
	}

	// Give VESC time to boot up
	delay(1000);

	// Tell VESC to send us its data
	VESC1.getVescValues();

	// Read battery voltage
	batteryVoltage = VESC1.data.inpVoltage; // Read voltage directly from VESC
	batteryCharge = (batteryVoltage - minBatVoltage) / (maxBatVoltage - minBatVoltage);

	clear_strip(clear_color);
}

void loop()
{
	GPS.read();
	GPS2.read();
	// Process GPS
	if (GPS.newNMEAreceived())
	{
		GPS.parse(GPS.lastNMEA());

		if (GPS.lastSentence[0] == 'G' && GPS.lastSentence[1] == 'G' && GPS.lastSentence[2] == 'A')
		{
			// publishGPSData();
		}
	}
	// Process GPS
	if (GPS2.newNMEAreceived())
	{
		GPS2.parse(GPS2.lastNMEA());
		GPS.parse(GPS.lastNMEA());
		GPS.parse(GPS2.lastNMEA());

		if (GPS2.lastSentence[0] == 'G' && GPS2.lastSentence[1] == 'G' && GPS2.lastSentence[2] == 'A')
		{
			publishGPSData();
		}
	}

	if (millis() - lastBatteryTime > 1000)
	{
		// Tell VESC to send us its data
		VESC1.getVescValues();

		// Read battery voltage
		batteryVoltage = VESC1.data.inpVoltage; // Read voltage directly from VESC
		batteryCharge = (batteryVoltage - minBatVoltage) / (maxBatVoltage - minBatVoltage);

		lastBatteryTime = millis();
		publishBatteryData();
	}

	if (JSerial.available())
	{
		String command = JSerial.readStringUntil('\n');
		parseCommand(command);
	}

	if (millis() - lastSendTime > interval)
	{
		dataPacket myPacket;
		myPacket.battvoltage = batteryVoltage;
		myPacket.elapsed_time = millis();
		myPacket.last_jetson_packet_time = lastPacketTime;
		myPacket.latitude = GPS.latitudeDegrees;
		myPacket.longitude = GPS.longitudeDegrees;
		myPacket.left_wheels_rpm = frleftMotorSpd;
		myPacket.right_wheels_rpm = frrightMotorSpd;
		myPacket.autonomous_status = 0;
		myPacket.command_ack = ackCommand;
		ackCommand = false;
		myPacket.dist_to_goal = 0;
		myPacket.mode = 0;

		lastSendTime = millis();
	}

	if (millis() - lastPacketTime > 200)
	{
		frleftMotorSpd = 0;
		frrightMotorSpd = 0;
		bkleftMotorSpd = 0;
		bkrightMotorSpd = 0;
	}

	// VESC1.setRPM(frrightMotorSpd);
	// VESC2.setRPM(frleftMotorSpd);
	// VESC4.setRPM(bkleftMotorSpd);
	// VESC5.setRPM(bkrightMotorSpd);

	VESC1.setDuty((float)frrightMotorSpd / (float)MAX_SPEED);
	VESC2.setDuty((float)frleftMotorSpd / (float)MAX_SPEED);
	VESC4.setDuty((float)bkleftMotorSpd / (float)MAX_SPEED);
	VESC5.setDuty((float)bkrightMotorSpd / (float)MAX_SPEED);
}

void parseCommand(String command)
{
	String exec = command.substring(0, command.indexOf(';'));
	if (exec.equals("set_motors") && enableJetson)
	{
		lastPacketTime = millis();

		int firstCommma = command.indexOf(',');
		int lastComma = command.lastIndexOf(',');
		String mag = command.substring(exec.length() + 1, firstCommma);
		String dir = command.substring(firstCommma + 1, lastComma);
		String speed_str = command.substring(lastComma + 1);

		float magnitude = strtof(mag.c_str(), NULL);
		float direction = strtof(dir.c_str(), NULL);
		float speed = strtof(speed_str.c_str(), NULL);

		calculateTankControlSpeeds(magnitude, direction, speed);
	}
	else if (exec.equals("set_motor") && enableJetson)
	{
		lastPacketTime = millis();

		int firstComma = command.indexOf(',');
		String motor = command.substring(exec.length() + 1, firstComma);
		String power_str = command.substring(firstComma + 1);
		float power = strtof(power_str.c_str(), NULL);

		if (motor.equals("max_power"))
		{
			maxPower = power;
		}
		else if (motor.equals("front_right"))
		{
			frRightPower = power;
		}
		else if (motor.equals("front_left"))
		{
			frLeftPower = power;
		}
		else if (motor.equals("back_right"))
		{
			bkRightPower = power;
		}
		else if (motor.equals("back_left"))
		{
			bkLeftPower = power;
		}

		else if (motor.equals("right"))
		{
			frRightPower = power;
			bkRightPower = power;
		}
		else if (motor.equals("left"))
		{
			frLeftPower = power;
			bkLeftPower = power;
		}

		frrightMotorSpd = (int)constrain(frRightPower * MAX_SPEED * maxPower, -MAX_SPEED, MAX_SPEED);
		frleftMotorSpd = (int)constrain(frLeftPower * MAX_SPEED * maxPower, -MAX_SPEED, MAX_SPEED);
		bkrightMotorSpd = (int)constrain(bkRightPower * MAX_SPEED * maxPower, -MAX_SPEED, MAX_SPEED);
		bkleftMotorSpd = (int)constrain(bkLeftPower * MAX_SPEED * maxPower, -MAX_SPEED, MAX_SPEED);

		if (abs(frleftMotorSpd) < 1000)
		{
			frleftMotorSpd = 0;
		}
		if (abs(frrightMotorSpd) < 1000)
		{
			frrightMotorSpd = 0;
		}
		if (abs(bkleftMotorSpd) < 1000)
		{
			bkleftMotorSpd = 0;
		}
		if (abs(bkrightMotorSpd) < 1000)
		{
			bkrightMotorSpd = 0;
		}
	}
	else if (exec.equals("signal_teleop"))
	{
		clear_strip(teleop_color);
	}
	else if (exec.equals("signal_autonomous"))
	{
		clear_strip(autonomous_color);
	}
	else if (exec.equals("signal_goal"))
	{
		isGoal = !isGoal;

		if (isGoal)
			clear_strip(goal_color);
		else
			clear_strip(clear_color);
	}
	else if (exec.equals("signal_idle"))
	{
		clear_strip(clear_color);
	}
}

void calculateTankControlSpeeds(float left, float right, float speed)
{
	float scale = MAX_SPEED * speed;
	frleftMotorSpd = (int)constrain(scale * (left), -MAX_SPEED, MAX_SPEED);
	frrightMotorSpd = (int)constrain(scale * (right), -MAX_SPEED, MAX_SPEED);
	bkleftMotorSpd = (int)constrain(scale * (left), -MAX_SPEED, MAX_SPEED);
	bkrightMotorSpd = (int)constrain(scale * (right), -MAX_SPEED, MAX_SPEED);

	if (abs(frleftMotorSpd) < 1000)
	{
		frleftMotorSpd = 0;
	}
	if (abs(frrightMotorSpd) < 1000)
	{
		frrightMotorSpd = 0;
	}
	if (abs(bkleftMotorSpd) < 1000)
	{
		bkleftMotorSpd = 0;
	}
	if (abs(bkrightMotorSpd) < 1000)
	{
		bkrightMotorSpd = 0;
	}
}

void calculateMotorSpeeds(float magnitude, float direction, float speed)
{
	float scale = MAX_SPEED * speed;
	frleftMotorSpd = (int)constrain(scale * (-magnitude - direction), -MAX_SPEED, MAX_SPEED);
	frrightMotorSpd = (int)constrain(scale * (-magnitude + direction), -MAX_SPEED, MAX_SPEED);
	bkleftMotorSpd = (int)constrain(scale * (-magnitude - direction), -MAX_SPEED, MAX_SPEED);
	bkrightMotorSpd = (int)constrain(scale * (-magnitude + direction), -MAX_SPEED, MAX_SPEED);

	if (abs(frleftMotorSpd) < 1000)
	{
		frleftMotorSpd = 0;
	}
	if (abs(frrightMotorSpd) < 1000)
	{
		frrightMotorSpd = 0;
	}
	if (abs(bkleftMotorSpd) < 1000)
	{
		bkleftMotorSpd = 0;
	}
	if (abs(bkrightMotorSpd) < 1000)
	{
		bkrightMotorSpd = 0;
	}

	JSerial.print("status;");
	JSerial.print(frleftMotorSpd);
	JSerial.print(" ");
	JSerial.print(frrightMotorSpd);
	JSerial.print(" ");
	JSerial.print(bkleftMotorSpd);
	JSerial.print(" ");
	JSerial.print(bkrightMotorSpd);
}

void clear_strip(uint32_t color)
{
	for (int i = 0; i < LED_COUNT; i++)
	{
		strip.setPixelColor(i, color);
	}
	strip.show();
}

void set_single_color(int pin, uint32_t color)
{
	strip.setPixelColor(pin, color);
	strip.show();
}

void publishBatteryData()
{
	JSerial.print("battery;");

	JSerial.print("v=");
	JSerial.print(batteryVoltage);

	JSerial.print(",c=");
	JSerial.println(batteryCharge);
}

void setupAdafruitGPS()
{
	// 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
	GPS.begin(9600);
	// uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	// uncomment this line to turn on only the "minimum recommended" data
	// GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
	// For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
	// the parser doesn't care about other sentences at this time
	// Set the update rate
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 1 Hz update rate
	// For the parsing code to work nicely and have time to sort thru the data, and
	// print it out we don't suggest using anything higher than 1 Hz

	// Request updates on antenna status, comment out to keep quiet
	GPS.sendCommand(PGCMD_ANTENNA);
	// 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
	GPS2.begin(9600);
	// uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
	GPS2.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	// uncomment this line to turn on only the "minimum recommended" data
	// GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
	// For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
	// the parser doesn't care about other sentences at this time
	// Set the update rate
	GPS2.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 1 Hz update rate
	// For the parsing code to work nicely and have time to sort thru the data, and
	// print it out we don't suggest using anything higher than 1 Hz

	// Request updates on antenna status, comment out to keep quiet
	GPS2.sendCommand(PGCMD_ANTENNA);
}

void publishGPSData()
{
	JSerial.print("gps;");

	JSerial.print("time=");
	uint8_t hms = GPS.hour; // Print the hours
	JSerial.print(hms);

	JSerial.print(F(":"));
	hms = GPS.minute; // Print the minutes
	JSerial.print(hms);

	JSerial.print(F(":"));
	hms = GPS.seconds; // Print the seconds
	JSerial.print(hms);

	JSerial.print(F("."));
	unsigned long millisecs = GPS.milliseconds; // Print the milliseconds
	JSerial.print(millisecs);

	JSerial.print(",lat=");
	long latitude = GPS.latitude_fixed; // Print the latitude
	JSerial.print(latitude);

	JSerial.print(",long=");
	long longitude = GPS.longitude_fixed; // Print the longitude
	JSerial.print(longitude);

	long altitude = GPS.altitude; // Print the height above mean sea level
	JSerial.print(",alt=");
	JSerial.print(altitude);

	long ground_speed = GPS.speed;
	JSerial.print(",ground_speed=");
	JSerial.print(ground_speed);

	long motion_heading = GPS.angle;
	JSerial.print(",motion_heading=");
	JSerial.print(motion_heading);

	long horizontal_accuracy = 3;
	JSerial.print(",horizontal_accuracy=");
	JSerial.println(horizontal_accuracy);
}
