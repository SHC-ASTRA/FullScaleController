// Imports!
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <MicroNMEA.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>
#include <ADC.h>
#include <ADC_util.h>
#include <VescUart.h>
#include <Adafruit_GPS.h>


//*******************************
// RGB Strip Configuration
//*******************************
#define LED_PIN 2
#define LED_COUNT 64
#define SERIAL_TX_LED 0
#define SERIAL_RX_LED 1
#define LORA_TX_LED 2
#define LORA_RX_LED 3
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
Adafruit_GPS GPS(&GPSSerial);
void setupAdafruitGPS();
void publishGPSData();

//*******************************
// Motor Drivers Configuration
//*******************************
int frleftMotorSpd = 0;
int frrightMotorSpd = 0;
int bkleftMotorSpd = 0;
int bkrightMotorSpd = 0;    
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
float cells = 8;                   // Number of battery cells present
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
void parseCommand(String command);
void setMotors(float magnitude, float direction);

// Sensor Reading Values
// Config Variables

void s() 
{
    Serial.print("status;");
}

void setup()
{
    // LED Ring
    strip.begin();
    strip.setBrightness(255);

    clear_strip(clear_color);
    
    // Serial Communications
    Serial.begin(115200);

    while(!Serial && millis() < 15000)
    {
        delay(10);
    }
    
    // LoRA
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


    if ( VESC1.getVescValues() ) 
    {
        s(); Serial.println("VESC 1 Connected!");
        s(); Serial.println(VESC1.data.id);
        s(); Serial.println(VESC1.data.rpm);
        s(); Serial.println(VESC1.data.inpVoltage);
        s(); Serial.println(VESC1.data.ampHours);
        s(); Serial.println(VESC1.data.tachometerAbs);
    }
    else
    {
        s(); Serial.println("No VESC1");
    }
    if ( VESC2.getVescValues() ) 
    {
        s(); Serial.println("VESC 2 Connected!");
        s(); Serial.println(VESC2.data.id);
        s(); Serial.println(VESC2.data.rpm);
        s(); Serial.println(VESC2.data.inpVoltage);
        s(); Serial.println(VESC2.data.ampHours);
        s(); Serial.println(VESC2.data.tachometerAbs);
    }
    else
    {
        s(); Serial.println("No VESC2");
    }
    if ( VESC4.getVescValues() ) 
    {
        s(); Serial.println("VESC 4 Connected!");
        s(); Serial.println(VESC4.data.id);
        s(); Serial.println(VESC4.data.rpm);
        s(); Serial.println(VESC4.data.inpVoltage);
        s(); Serial.println(VESC4.data.ampHours);
        s(); Serial.println(VESC4.data.tachometerAbs);
    }
    else
    {
        s(); Serial.println("No VESC4");
    }
    if ( VESC5.getVescValues() ) 
    {
        s(); Serial.println("VESC 5 Connected!");
        s(); Serial.println(VESC5.data.id);
        s(); Serial.println(VESC5.data.rpm);
        s(); Serial.println(VESC5.data.inpVoltage);
        s(); Serial.println(VESC5.data.ampHours);
        s(); Serial.println(VESC5.data.tachometerAbs);
    }
    else
    {
        s(); Serial.println("No VESC5");
    }

    // Give VESC time to boot up
    delay(1000);

     // Tell VESC to send us its data
    VESC1.getVescValues();

    //Read battery voltage
    batteryVoltage = VESC1.data.inpVoltage; // Read voltage directly from VESC
    batteryCharge = (batteryVoltage - minBatVoltage) / (maxBatVoltage - minBatVoltage);

    clear_strip(clear_color);
}

void loop()
{ 
    GPS.read();
    // Process GPS
    if (GPS.newNMEAreceived())
    {
        GPS.parse(GPS.lastNMEA());
        publishGPSData();
    }

    if (millis() - lastBatteryTime > 1000)
    {
        // Tell VESC to send us its data
        VESC1.getVescValues();

        //Read battery voltage
        batteryVoltage = VESC1.data.inpVoltage; // Read voltage directly from VESC
        batteryCharge = (batteryVoltage - minBatVoltage) / (maxBatVoltage - minBatVoltage);

        lastBatteryTime = millis();
        publishBatteryData();
    }

    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        parseCommand(command);
    }

    if (millis() - lastPacketTime > 200)
    {
        frleftMotorSpd = 0;
        frrightMotorSpd = 0;
        bkleftMotorSpd = 0;
        bkrightMotorSpd = 0;
    }


    VESC1.setRPM(frrightMotorSpd);
    VESC2.setRPM(frleftMotorSpd);
    VESC4.setRPM(bkleftMotorSpd);
    VESC5.setRPM(bkrightMotorSpd);
}

void parseCommand(String command)
{
    String exec = command.substring(0, command.indexOf(';'));
    if (exec.equals("set_motors"))
    {
        lastPacketTime = millis();

        int firstCommma = command.indexOf(',');
        int lastComma = command.lastIndexOf(',');
        String mag = command.substring(exec.length()+1, firstCommma);
        String dir = command.substring(firstCommma+1, lastComma);
        String speed_str = command.substring(lastComma+1);
        
        float magnitude = strtof(mag.c_str(), NULL);
        float direction = strtof(dir.c_str(), NULL);
        float speed = strtof(speed_str.c_str(), NULL);

        calculateTankControlSpeeds(magnitude, direction, speed);

    } else if (exec.equals("signal_teleop"))
    {
        clear_strip(teleop_color);
    } else if (exec.equals("signal_autonomous"))
    {
        clear_strip(autonomous_color);
    } else if (exec.equals("signal_goal"))
    {
        isGoal = !isGoal;

        if (isGoal)
            clear_strip(goal_color);
        else
            clear_strip(clear_color);
    } else if (exec.equals("signal_idle"))
    {
        clear_strip(clear_color);
    }
}

void calculateTankControlSpeeds(float left, float right, float speed)
{
    int MAX_SPEED = 6000;
    float scale = MAX_SPEED * speed;
    frleftMotorSpd = (int)constrain(scale*(left), -MAX_SPEED, MAX_SPEED);
    frrightMotorSpd = (int)constrain(scale*(right), -MAX_SPEED, MAX_SPEED);
    bkleftMotorSpd = (int)constrain(scale*(left), -MAX_SPEED, MAX_SPEED);
    bkrightMotorSpd = (int)constrain(scale*(right), -MAX_SPEED, MAX_SPEED);

    if (abs(frleftMotorSpd) < 1000) {
      frleftMotorSpd = 0;
    }
    if (abs(frrightMotorSpd) < 1000) {
      frrightMotorSpd = 0;
    }
    if (abs(bkleftMotorSpd) < 1000) {
      bkleftMotorSpd = 0;
    }
    if (abs(bkrightMotorSpd) < 1000) {
      bkrightMotorSpd = 0;
    }
}

void calculateMotorSpeeds(float magnitude, float direction, float speed)
{
    int MAX_SPEED = 6000;
    float scale = MAX_SPEED * speed;
    frleftMotorSpd = (int)constrain(scale*(-magnitude - direction), -MAX_SPEED, MAX_SPEED);
    frrightMotorSpd = (int)constrain(scale*(-magnitude + direction), -MAX_SPEED, MAX_SPEED);
    bkleftMotorSpd = (int)constrain(scale*(-magnitude - direction), -MAX_SPEED, MAX_SPEED);
    bkrightMotorSpd = (int)constrain(scale*(-magnitude + direction), -MAX_SPEED, MAX_SPEED);

    if (abs(frleftMotorSpd) < 1000) {
      frleftMotorSpd = 0;
    }
    if (abs(frrightMotorSpd) < 1000) {
      frrightMotorSpd = 0;
    }
    if (abs(bkleftMotorSpd) < 1000) {
      bkleftMotorSpd = 0;
    }
    if (abs(bkrightMotorSpd) < 1000) {
      bkrightMotorSpd = 0;
    }

    Serial.print("status;");
    Serial.print(frleftMotorSpd);
    Serial.print(" ");
    Serial.print(frrightMotorSpd);
    Serial.print(" ");
    Serial.print(bkleftMotorSpd);
    Serial.print(" ");
    Serial.print(bkrightMotorSpd);
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
    Serial.print("battery;");

    Serial.print("v=");
    Serial.print(batteryVoltage);

    Serial.print(",c=");
    Serial.println(batteryCharge);
}

void setupAdafruitGPS()
{
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);
}

void publishGPSData()
{
    Serial.print("gps;");

    Serial.print("time=");
    uint8_t hms = GPS.hour; // Print the hours
    Serial.print(hms);

    Serial.print(F(":"));
    hms = GPS.minute; // Print the minutes
    Serial.print(hms);

    Serial.print(F(":"));
    hms = GPS.seconds; // Print the seconds
    Serial.print(hms);

    Serial.print(F("."));
    unsigned long millisecs = GPS.milliseconds; // Print the milliseconds
    Serial.print(millisecs);

    Serial.print(",lat=");    
    long latitude = GPS.latitudeDegrees; // Print the latitude
    Serial.print(latitude);

    Serial.print(",long=");
    long longitude = GPS.longitudeDegrees; // Print the longitude
    Serial.print(longitude);

    long altitude = GPS.altitude; // Print the height above mean sea level
    Serial.print(",alt=");
    Serial.print(altitude);

    long ground_speed = GPS.speed;
    Serial.print(",ground_speed=");
    Serial.print(ground_speed);

    long motion_heading = GPS.angle;
    Serial.print(",motion_heading=");
    Serial.print(motion_heading);

    long horizontal_accuracy = 3;
    Serial.print(",horizontal_accuracy=");
    Serial.println(horizontal_accuracy);
}