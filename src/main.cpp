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

//*******************************
// GPS Module Configuration
//*******************************
SFE_UBLOX_GNSS myGNSS;

boolean GNSS_enable = true;

long latitude_mdeg = 0;
long longitude_mdeg = 0;
int numSats = 0;

u_int32_t lastGPSTime = 0;

void publishPVTdata(UBX_NAV_PVT_data_t);

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
    
    // for (int i = 0; i < 6000; i++)
    // {
    //     delay(10);
    //     //While waiting for the serial port to open, make a rainbow smiley face that changes color progressively faster and faster
    //     uint32_t rgbcolor = strip.gamma32(strip.ColorHSV((i*500)%65536, 255, 255));
    //     clear_strip(rgbcolor);
    //     strip.show();
    // }

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
    if (!myGNSS.begin())
    {
        Serial.println("status;Ublox GPS not detected at default I2C address. Please check wiring.");
        GNSS_enable = false;
    } 
    else 
    {
        myGNSS.setI2COutput(COM_TYPE_UBX);                 //Set the I2C port to output UBX only (turn off NMEA noise)
        myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

        myGNSS.setNavigationFrequency(2); //Produce two solutions per second

        myGNSS.setAutoPVTcallback(&publishPVTdata);
    }

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
    batteryCharge = VESC1.data.ampHours/fullAmpHours; // Read amp hours from VESC

    clear_strip(clear_color);
}

void loop()
{

    // Process GPS
    if (millis() - lastGPSTime > 250 && GNSS_enable)
    {
        lastGPSTime = millis();
        myGNSS.checkUblox();     // Check for the arrival of new data and process it.
        myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
    }

    if (millis() - lastBatteryTime > 1000)
    {
        // Tell VESC to send us its data
        VESC1.getVescValues();

        //Read battery voltage
        batteryVoltage = VESC1.data.inpVoltage; // Read voltage directly from VESC
        batteryCharge = VESC1.data.ampHours/fullAmpHours; // Read amp hours from VESC
        
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

        calculateMotorSpeeds(magnitude, direction, speed);

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

void publishPVTdata(UBX_NAV_PVT_data_t ubxDataStruct)
{
    set_single_color(SERIAL_TX_LED, serial_tx_color);

    Serial.print("gps;");

    Serial.print("time=");
    uint8_t hms = ubxDataStruct.hour; // Print the hours
    Serial.print(hms);

    Serial.print(F(":"));
    hms = ubxDataStruct.min; // Print the minutes
    Serial.print(hms);

    Serial.print(F(":"));
    hms = ubxDataStruct.sec; // Print the seconds
    Serial.print(hms);

    Serial.print(F("."));
    unsigned long millisecs = ubxDataStruct.iTOW % 1000; // Print the milliseconds
    Serial.print(millisecs);

    Serial.print(",lat=");    
    long latitude = ubxDataStruct.lat; // Print the latitude
    Serial.print(latitude);

    Serial.print(",long=");
    long longitude = ubxDataStruct.lon; // Print the longitude
    Serial.print(longitude);

    long altitude = ubxDataStruct.hMSL; // Print the height above mean sea level
    Serial.print(",alt=");
    Serial.print(altitude);

    long ground_speed = ubxDataStruct.gSpeed;
    Serial.print(",ground_speed=");
    Serial.print(ground_speed);

    long motion_heading = ubxDataStruct.headMot;
    Serial.print(",motion_heading=");
    Serial.print(motion_heading);

    long horizontal_accuracy = ubxDataStruct.hAcc;
    Serial.print(",horizontal_accuracy=");
    Serial.println(horizontal_accuracy);

    set_single_color(SERIAL_TX_LED, clear_color);
}

void publishBatteryData()
{
    set_single_color(SERIAL_TX_LED, serial_tx_color);

    Serial.print("battery;");

    Serial.print("v=");
    Serial.print(batteryVoltage);

    Serial.print(",c=");
    Serial.println(batteryCharge);

    set_single_color(SERIAL_TX_LED, clear_color);
}