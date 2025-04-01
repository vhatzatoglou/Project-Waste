//#include <ADXL345.h>
//I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "ACS712.h"
#include <Adafruit_INA219.h>
#include <Arduino.h>
#include <SensirionI2CSen5x.h>

uint16_t error;
char errorMessage[256];
// Read Measurement
float massConcentrationPm1p0;
float massConcentrationPm2p5;
float massConcentrationPm4p0;
float massConcentrationPm10p0;
float ambientHumidity;
float ambientTemperature;
float vocIndex;
float noxIndex;

Adafruit_INA219 ina219;
ACS712  ACS(36, 3.3, 4095, 185);
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
 
const char DEVICE_NAME[] = "mpu6050";


#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define TINY_GSM_MODEM_SIM7000
//#define TINY_GSM_MODEM_SIM7070
#define SerialAT Serial2
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
  
// #define DUMP_AT_COMMANDS

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "iot.1nce.net"; //SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";
#include <TinyGsmClient.h>
#include <ESP32_Servo.h>
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
 
#include "timestamp32bits.h"

//#include "Zanshin_BME680.h" // Include the BME680 Sensor library
#include <LinkedList.h>
#include "helper.cpp"
#include <ArduinoJson.h>
#include "ESP32httpUpdate.h"
#include <Time.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "FS.h"
#include "SPIFFS.h"
#include <Wire.h>
#include "PN532_I2C.h"
#include "PN532.h"
#include "NfcAdapter.h"

#include <Ticker.h>
//#include <ADXL345.h>
#include "AES.h"
#include "Base64.h"
#include <VL6180X.h>

VL6180X Cover_sensor;
AES aes;

// Our AES key. Same in NodeJS but in hex bytes
byte key[] = {0x53, 0x40, 0x66, 0x65, 0x62, 0x30, 0x74, 0x54, 0x68, 0x65, 0x33, 0x21, 0x21, 0x21, 0x21, 0x40};
//ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
TaskHandle_t Task1;
TaskHandle_t Task2;
int Sen1=0,Sen2=0,Sen3=0,Sen4=0,Sen5=0,Sen6=0,Sen7=0;
String str, CCID;
int i;
int we;
int gf=0;
int lc = 0;
int gr = -1;
int System_Tag = 0;
byte stopWorking = 0;
long open_cover_time = millis();
long start_time = millis();
long start_timeAQ = millis();
float Working_time = millis();
float Working_time_connecting = 0;
float Working_time_posting = 0;
bool isConnected = false;
float mAh = 0.0;
float lat=0.0,  lon=0.0;
int chgTimes = 0;
int chgTimesNew = 2;
int p, DAY1, MONTH_, YEAR_, YEAR1, Seconds_, Hours_, Minutes_, hourOfDay, DAY_OF_WEEK, hasModemSIM, contentLength;
boolean GPRS_on;
int sp=0;
bool reply = false;
bool checkObstacle=false;
//int ADXL345 = 0x53;
int Left_Range=0x52;
int Right_Range=0x54;
int Cover_Range=0x32;
float X_out, Y_out, Z_out;  // Outputs
int CoverStatus=1;
int Servo_i=75;
enum _LockerStatus
{
  Locker_Opened,
  Locker_Closed

};
enum _LockerStatusN
{
  Locker_Closed_,
  Locker_Opened_

};
enum _tagType
{
  SystemTag,
  Citizen_Valid,
  Citizen_inValid,
  Garbage_CollectorTag,
  MaintenanceTag,
  BinbotID
};

enum _BinStatus
{
  keno,
  Operative,
  BinIsFull,
  Cover_Open,
  Maintenance,
  Weight_Full,
  Error,
  Battery_low,
  Empty,
  Delivered,
  In_warehouse,
  Garbage_collection,
  ProblemCoverNotOpening,
  AlarmisOn,
  ProblemCoverIsOpentoLong,
  NewLocationDetected,
  Violation_Tilt

};
enum _BinOrientation
{
  Bin_Up,
  Bin_Down
};
enum _parseState
{
  PS_DETECT_MSG_TYPE,

  PS_IGNORING_COMMAND_ECHO,

  PS_HTTPACTION_TYPE,
  PS_HTTPACTION_RESULT,
  PS_HTTPACTION_LENGTH,

  PS_HTTPREAD_LENGTH,
  PS_HTTPREAD_CONTENT
};

int parseState = PS_DETECT_MSG_TYPE;
char buffer[256];
byte pos;
String httpContent = "";
String result = "0";
int binstatus = 0;
int binOrientation=0;
//OUTPUT
#define DONE_PIN 18        //OUTPUT
#define ALARM_PIN 13       //OUTPUT
#define GREEN_PIN 23       //OUTPUT
#define LOCKER_PIN 33      //OUTPUT TRIGGER LOCKER
#define SCALE_ONOFF_PIN 14 //OUTPUT TRIGGER TO OPEN INDICATOR
#define SERVO_PIN 05

//SCALE COMMUNICATION
#define RXSCALE_PIN 15
#define TXSCALE_PIN 17
#define PIN_DTR 25
#define NFC_RST_PIN 19
// LilyGO T-SIM7000G Pinout
#define UART_BAUD 115200
#define PIN_DTR 25
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4
#define LED_PIN 12
//INPUT
#define STATUSSCALE_PIN 34 //INPUT LOCKER
// #define SENSOR1 32         //INPUT
// #define SENSOR2 35         //INPUT
//INPUT I2C
#define SDA_PIN 21
#define SCL_PIN 22
#define ADC_PIN 36

#define BUZZER_CHANNEL 0

#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

SensirionI2CSen5x sen5x;

Servo myservo; // create servo object to control a servo

LinkedList<String> Datalist = LinkedList<String>();
LinkedList<String> LostDatalist = LinkedList<String>();
LinkedList<int> LostData = LinkedList<int>();
LinkedList<double> WeightList = LinkedList<double>();
LinkedList<int> CoverRange = LinkedList<int>();
LinkedList<Tag *> Taglist = LinkedList<Tag *>();
PN532_I2C pn532_i2c(Wire);
NfcAdapter nfc = NfcAdapter(pn532_i2c);
String tagId = "";
String tagId1 = "";
String userId = "";
byte nuidPICC[4];
String dt;
int hasSIM = 0;
int formatF = 0;
int Garbagecollection = 0;
int Mainntanace = 0;
int BinFull = 0;
int ultime = 0, mustUnlock = 0;
const char *ntpServer = "pool.ntp.org";
const long gmtOfSPIFFSet_sec = 0;
const int daylightOfSPIFFSet_sec = 3600;
int countId;
int enablePost = 0;
String SameTagID;
int tagExist = 0;
String FirstBoot;
String DSN;
String URL;
String f;
int wa = 0;
int scaleIsConnected = 0;
long lock_time = 0;
int step, Lockstatus = 0,TagPresent=0;
byte scale_status = 0;
long delayTime;
boolean hasServo = true;
double weight, weightF, weightA;
long working_period = millis();
long nfc_period = millis();
float batterylevel, voltage;
String APs="",APs_="";
int mVperAmp = 100;           // this the 5A version of the ACS712 -use 100 for 20A Module and 66 for 30A Module
double Total_mA = 0;
float current_mA = 0;
float power_mW = 0;
int battery_time=0;
int NewVersion=0;
int I2cbusy=0;
void store_Tag(int type, String Id)
{

  Serial.println("store_Tag ");
  SPIFFS.begin();
  String a, s;
  File f = SPIFFS.open("/Tags", "w");
  StaticJsonDocument<200> doc;
  int f1 = 0;

  for (int i = 0; i < Taglist.size(); i++)
  {
    if (type == BinbotID && Taglist.get(i)->tagId == Id) // binbot id tag if scan 10 times the same
    {
      Taglist.get(i)->type = (int)BinbotID;
      Serial.println("Type :  BinbotID");
      f1 = 1;
    }
    else if (type == MaintenanceTag && Taglist.get(i)->tagId == Id) // binbot id tag if scan 10 times the same
    {
      Taglist.get(i)->type = (int)MaintenanceTag;
      Serial.println("Type :  Maintenance");
      f1 = 1;
    }
    else if (type == Garbage_CollectorTag && Taglist.get(i)->tagId == Id) // binbot id tag if scan 10 times the same
    {
      Taglist.get(i)->type = (int)Garbage_CollectorTag;
      Serial.println("Type :  Garbage_CollectorTag");
      f1 = 1;
    }
    else if (type == Citizen_Valid && Taglist.get(i)->tagId == Id && Taglist.get(i)->type != (int)Garbage_CollectorTag && Taglist.get(i)->type != (int)MaintenanceTag)
    {
      Taglist.get(i)->type = (int)Citizen_Valid;
      Serial.println("Type : " + String(type));
      f1 = 1;
    }
    else if (type == Citizen_inValid && Taglist.get(i)->tagId == Id && Taglist.get(i)->type != (int)Garbage_CollectorTag && Taglist.get(i)->type != (int)MaintenanceTag)
    {
      Taglist.get(i)->type = (int)Citizen_inValid;
      Serial.println("Type : " + String(type));
      f1 = 1;
    }
    if (f1 == 1)
    {

      break;
    }
  }
  if (f1 == 0)
  {
    Tag *tag = new Tag();
    tag->type = type;
    tag->tagId = Id;
    Taglist.add(tag);
    Serial.println("Store : Tag " + Id);
    s = "";
  }
  for (int i = 0; i < Taglist.size(); i++)
  {
    doc["type"] = Taglist.get(i)->type;
    doc["Id"] = Taglist.get(i)->tagId;
    serializeJson(doc, s);
    Serial.println(s);
    f.println(s);
    s = "";
  }

  f.close();
  s = "";
  SPIFFS.end();
}
void restore_Tags()
{
  SPIFFS.begin();
  String s;
  Taglist.clear();
  File f = SPIFFS.open("/Tags", "r");

  while (f.available())
  {
    s = f.readStringUntil('\n');
    Serial.println("restore tag :" + s);
    if (s == "")
    {
      f.close();
      return;
    }
    StaticJsonDocument<200> doc;
    deserializeJson(doc, s);
    const char *t = doc["Id"];
    String tg = reinterpret_cast<const char *>(t);

    Tag *tag = new Tag();
    tag->type = doc["type"];
    tag->tagId = tg;
    //Serial.println("tag :" + "Type:" + String(doc["type"] )+ " id: " + String(doc["Id"]));
    if (!tg.isEmpty())
      Taglist.add(tag);
  }
  f.close();
  SPIFFS.end();
}
int pos1 = 0;
 
void lockByServo()
{
 // myservo.attach(SERVO_PIN); 
  //  if ( Lockstatus==1)
  //   return;
 //if (CoverStatus ==Locker_Closed)
    if (CoverStatus ==Locker_Closed)
    while (Servo_i<75)
    {
      Servo_i++;
      myservo.write(Servo_i); //turn servo by 1 degrees
      delay(15);        //delay for smoothness
       
    }
  //  for(int i=3; i<=85; i+=1)
  //   {
  //     Serial.println("locking : " + String(i));
  //     myservo.write(i); //turn servo by 1 degrees
  //     delay(20);        //delay for smoothness
  //   }
  // delay(1000);
  // myservo.detach();
  //  Lockstatus=1;

  // for (pos1 = 0; pos1 <= 180; pos1 += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo.write(pos1);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15ms for the servo to reach the position
  // }
   checkObstacle=true;
   ultime=0;
   
  // storeSettings();
}
void beepDelay(int period = 1000)
{
  int stTime = millis();
  // int p=millis();
  while (millis() - stTime < period)
  {
    pinMode(ALARM_PIN, OUTPUT);
    digitalWrite(ALARM_PIN, HIGH);
    delay(50);
  }

  pinMode(ALARM_PIN, INPUT);
  delay(200);
}
void UnlockByServo()
{
  //myservo.attach(SERVO_PIN); 
  //delay(1000);
  
 
  if (ultime == 0)
  {
    Serial.println("Unlocking");
    beepDelay(500);
     ultime++; 
    if (ultime >0)
    while (Servo_i>10)
    {
      Servo_i--;
      myservo.write(Servo_i); //turn servo by 1 degrees
      delay(15);        //delay for smoothness
      ultime++;
    }
  }
 // storeSettings();
 
}


void NFC_RST()
{
  Serial.println("NFC Restart");
  pinMode(NFC_RST_PIN, OUTPUT);
  digitalWrite(NFC_RST_PIN, LOW);
  delay(1000);
  pinMode(NFC_RST_PIN, INPUT);
}

 

float getVPP()
{
  float result;
  int readValue;                // value read from the sensor
  int maxValue = 0;             // store max value here
  int minValue = 4096;          // store min value here ESP32 ADC resolution
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 1000) //sample for 1 Sec
   {
       readValue = analogRead(ADC_PIN);
       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           /*record the minimum sensor value*/
           minValue = readValue;
       }
   }
   
   // Subtract min from max
   result = ((maxValue - minValue) * 3.3)/4096.0; //ESP32 ADC resolution 4096
      
   return result;
 }
 int b1=0;
void check_battery()
{
  if (   I2cbusy==1  ||  (Sen2==0)  ) return;
  I2cbusy=1;
  if (millis() - battery_time > 1000 &&   I2cbusy==0 )
  {
      I2cbusy=1;
      Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
      Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
      current_mA = ina219.getCurrent_mA();
       if (current_mA!=INFINITY)
      Total_mA=Total_mA+current_mA;
      if (current_mA>1500)
      UnlockByServo();
      double bat=(30000*3600-Total_mA)*100/(30000*3600);
      Serial.println("Battery % :" + String(bat));
      batterylevel=bat;
      battery_time=millis();
      I2cbusy=0;
  }
  I2cbusy=0;


}

void check_battery_()
{
  int analogValue = analogRead(ADC_PIN);

  Serial.println("analogValue : " + String(analogValue));

  batterylevel = 100 * analogValue / 4095;

  Serial.println("batterylevel : " + String(batterylevel));
}
void calc_battery()
{
  batterylevel = (float)(30000.00 - mAh) * 100 / 30000.00;
  Serial.println("Calculated Batterylevel : " + String(batterylevel));
}

void sendGSM(String msg, int waitMs = 100)
{

  SerialAT.println(msg);
  // delay(waitMs);
  delay(waitMs);
  char b;
  int64_t timestamp1 = millis();
  while (SerialAT.available())
  {

    b = SerialAT.read();
    buffer[pos++] = b;
    Serial.print(String(b));
    parseATText(b);
  }
  msg = "";
}

void modemPowerOn()
{
  // pinMode(PWR_PIN, OUTPUT);
  // digitalWrite(PWR_PIN, LOW);
  // delay(500);
  // digitalWrite(PWR_PIN, HIGH);

  pinMode(PWR_PIN, OUTPUT); 
  digitalWrite(PWR_PIN, HIGH); 
  delay(500); 
  digitalWrite(PWR_PIN, LOW);
}

void modemPowerOff()
{

  modem.gprsDisconnect();
  SerialMon.println(F("GPRS disconnected"));
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(1500); //Datasheet Ton mintues = 1.2S
  digitalWrite(PWR_PIN, LOW);
  Serial.println(F("***********************************************************"));
  Serial.println(F("                   Modem is Power off"));
  Serial.println(F("***********************************************************\n"));
}
void enableGPS()
{
    // Set Modem GPS Power Control Pin to HIGH ,turn on GPS power
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+CGPIO=0,48,1,1");
   if (modem.waitResponse(10000L) != 1) {
       DBG("Set GPS Power HIGH Failed");
   }
    modem.enableGPS();
}

void disableGPS()
{
    // Set Modem GPS Power Control Pin to LOW ,turn off GPS power
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+CGPIO=0,48,1,0");
    if (modem.waitResponse(10000L) != 1) {
        DBG("Set GPS Power LOW Failed");
    }
    modem.disableGPS();
}
 

void modemRestart()
{
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}
void modem_on1()
{
  // Set-up modem  power pin
  Serial.println("\nStarting Up Modem...");
  pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, HIGH);
  Working_time_connecting = millis();
  modemPowerOn();
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  String res;

  Serial.println("========INIT========");

  if (!modem.init())
  {
    // modemRestart();
    delay(2000);
    Serial.println("Failed to restart modem, attempting to continue without restarting");
  }

  Serial.println("========SIMCOMATI======");
  modem.sendAT("+SIMCOMATI");
  modem.waitResponse(1000L, res);
  res.replace(GSM_NL "OK" GSM_NL, "");
  Serial.println(res);
  res = "";
  Serial.println("=======================");

  Serial.println("=====Preferred mode selection=====");
  modem.sendAT("+CNMP?");
  if (modem.waitResponse(1000L, res) == 1)
  {
    res.replace(GSM_NL "OK" GSM_NL, "");
    Serial.println(res);
  }
  res = "";
  Serial.println("=======================");

  Serial.println("=====Preferred selection between CAT-M and NB-IoT=====");
  modem.sendAT("+CMNB?");
  if (modem.waitResponse(1000L, res) == 1)
  {
    res.replace(GSM_NL "OK" GSM_NL, "");
    Serial.println(res);
  }
  res = "";
  Serial.println("=======================");

  String name = modem.getModemName();
  Serial.println("Modem Name: " + name);

  String modemInfo = modem.getModemInfo();
  Serial.println("Modem Info: " + modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3)
  {
    modem.simUnlock(GSM_PIN);
  }

  for (int i = 0; i < 2; i++)
  {
    uint8_t network[] = {
        2,  /*Automatic*/
        13, /*GSM only*/
        38, /*LTE only*/
        51  /*GSM and LTE only*/
    };
    Serial.printf("Try %d method\n", network[i]);
    modem.setNetworkMode(13);
    delay(3000);
   isConnected = false;
    int tryCount = 60;
    while (tryCount>0)
    {
      int16_t signal = modem.getSignalQuality();
      Serial.print("Signal: ");
      Serial.print(signal);
      Serial.print(" ");
      Serial.print("isNetworkConnected: ");
      isConnected = modem.isNetworkConnected();
      Serial.println(isConnected ? "CONNECT" : "NO CONNECT");
      Serial.println("tryCount" +  String(tryCount));
      if (isConnected)
      {
        break;
      }
       tryCount--;
      delay(1000);
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    if (isConnected)
    {
      break;
    }
  }
  digitalWrite(LED_PIN, LOW);

  Serial.println();
 

  Serial.println("=====Inquiring UE system information=====");
  modem.sendAT("+CPSI?");
  if (modem.waitResponse(1000L, res) == 1)
  {
    res.replace(GSM_NL "OK" GSM_NL, "");
    Serial.println(res);
  }

  Serial.println("/**********************************************************/");
  Serial.println("After the network test is complete, please enter the  ");
  Serial.println("AT command in the serial terminal.");
  Serial.println("/**********************************************************/\n\n");

  // --------TESTING GPRS--------
  Serial.println("\n---Starting GPRS TEST---\n");
  Serial.println("Connecting to: " + String(apn));
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    delay(10000);
    Working_time_connecting = 0;
    return;
  }

  Serial.print("GPRS status: ");
  if (modem.isGprsConnected())
  {
    Serial.println("connected");
    Working_time_connecting = millis() - Working_time_connecting;
  }
  else
  {
    Working_time_connecting = millis() - Working_time_connecting;
    Serial.println("not connected");
  }

  String ccid = modem.getSimCCID();
  Serial.println("CCID: " + ccid);

  String imei = modem.getIMEI();
  Serial.println("IMEI: " + imei);

  String cop = modem.getOperator();
  Serial.println("Operator: " + cop);

  IPAddress local = modem.localIP();
  Serial.println("Local IP: " + String(local));

  int csq = modem.getSignalQuality();
  Serial.println("Signal quality: " + String(csq));

  SerialAT.println("AT+CPSI?"); //Get connection type and band
  delay(500);
  if (SerialAT.available())
  {
    String r = SerialAT.readString();
    Serial.println(r);
  }

   
}

void modem_on()
{
  
  if (isConnected) return;
  modem.init();
  String res;
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  Serial.println("========INIT========");
  
  if (!modem.init())
  {
    // modemRestart();
    delay(2000);
    Serial.println("Failed to restart modem, attempting to continue without restarting");
  }

  // Serial.println("========SIMCOMATI======");
  // modem.sendAT("+SIMCOMATI");
  // modem.waitResponse(1000L, res);
  // res.replace(GSM_NL "OK" GSM_NL, "");
  // Serial.println(res);
  // res = "";
  // Serial.println("=======================");

  // Serial.println("=====Preferred mode selection=====");
  // modem.sendAT("+CNMP?");
  // if (modem.waitResponse(1000L, res) == 1)
  // {
  //   res.replace(GSM_NL "OK" GSM_NL, "");
  //   Serial.println(res);
  // }
  // res = "";
  Serial.println("=======================");

  // Serial.println("=====Preferred selection between CAT-M and NB-IoT=====");
  // modem.sendAT("+CMNB?");
  // if (modem.waitResponse(1000L, res) == 1)
  // {
  //   res.replace(GSM_NL "OK" GSM_NL, "");
  //   Serial.println(res);
  // }
  // res = "";
  // Serial.println("=======================");

  // String name = modem.getModemName();
  // Serial.println("Modem Name: " + name);

  // String modemInfo = modem.getModemInfo();
  // Serial.println("Modem Info: " + modemInfo);

  // // Unlock your SIM card with a PIN if needed
  // if (GSM_PIN && modem.getSimStatus() != 3)
  // {
  //   modem.simUnlock(GSM_PIN);
  // }

// for (int i = 0; i < 2; i++)
  {
    // uint8_t network[] = {
    //     2,  /*Automatic*/
    //     13, /*GSM only*/
    //     38, /*LTE only*/
    //     51  /*GSM and LTE only*/
    // };
    // Serial.printf("Try %d method\n", network[i]);
    modem.setNetworkMode(13);
    //delay(3000);
     
    int tryCount = 60;
    while (tryCount>0)
    {
     
      if (scaleIsConnected == 0)
      if (Serial1.available())
      Serial1.readStringUntil('\n');
      int16_t signal = modem.getSignalQuality();
      Serial.print("Signal: ");
      Serial.print(signal);
      Serial.print(" ");
      Serial.print("isNetworkConnected: ");
      isConnected = modem.isNetworkConnected();
      Serial.println(isConnected ? "CONNECT" : "NO CONNECT");
        Serial.println("tryCount" +  String(tryCount));
      if (isConnected)
      {
        break;
        
      }
       tryCount--;
      delay(1000);
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    if (isConnected)
    {
     // break;
    }
  }
  digitalWrite(LED_PIN, LOW);
 
 

  // Serial.println("=====Inquiring UE system information=====");
  // modem.sendAT("+CPSI?");
  // if (modem.waitResponse(1000L, res) == 1)
  // {
  //   res.replace(GSM_NL "OK" GSM_NL, "");
  //   Serial.println(res);
  // }

  // Serial.println("/**********************************************************/");
  // Serial.println("After the network test is complete, please enter the  ");
  // Serial.println("AT command in the serial terminal.");
  // Serial.println("/**********************************************************/\n\n");

  // --------TESTING GPRS--------
  Serial.println("\n---Starting GPRS TEST---\n");
  Serial.println("Connecting to: " + String(apn));
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    delay(1000);
    Working_time_connecting = 0;
    return;
  }

  Serial.print("GPRS status: ");
  if (modem.isGprsConnected())
  {
    Serial.println("connected");
    Working_time_connecting = millis() - Working_time_connecting;
  }
  else
  {
    Working_time_connecting = millis() - Working_time_connecting;
    Serial.println("not connected");
  }

  String ccid = modem.getSimCCID();
  Serial.println("CCID: " + ccid);

  // String imei = modem.getIMEI();
  // Serial.println("IMEI: " + imei);

  // String cop = modem.getOperator();
  // Serial.println("Operator: " + cop);

  // IPAddress local = modem.localIP();
  // Serial.println("Local IP: " + String(local));

  // int csq = modem.getSignalQuality();
  // Serial.println("Signal quality: " + String(csq));

  // SerialAT.println("AT+CPSI?"); //Get connection type and band
  // delay(500);
  // if (SerialAT.available())
  // {
  //   String r = SerialAT.readString();
  //   Serial.println(r);
  // }

  getRTC();
}

void initCLTS()
{
  if (hasModemSIM > 0) //CHECK IF MODEM IS EXISTING
  {

    delay(1000);
    SerialAT.println("AT+CLTS=0;&W"); // Configuring TEXT mode
    updateSerial();
    delay(1000);
    SerialAT.println("AT+CLTS?");
    updateSerial();
  }
}

void getRTC()
{
  YEAR1 = 0;

  String DAY_, MONTH_, YEAR_, Seconds_, Hours_, Minutes_;
  // if ((millis()-rtcm)<60000)
  // return;
  Serial.println("getFreeHeap " + String(ESP.getFreeHeap()));
  hasModemSIM = 1;
  if (hasModemSIM > 0) //&& WiFi.status()!= WL_CONNECTED
  {

    {
      //delay(1000);
      SerialAT.println("AT+CNTPCID=1");
      updateSerial();
      SerialAT.println("AT+CNTP=\"time1.google.com\",0");
      updateSerial();
      SerialAT.println("AT+CNTP");
      delay(200);
      updateSerial();
      initCLTS();
      delay(200);
      Serial.println("CCLK :");
      SerialAT.println("AT+CCLK?");

      String datetimeSerialAT = updateSerial();
      int len = datetimeSerialAT.indexOf('\"');
      int a = datetimeSerialAT.lastIndexOf('\"');
      datetimeSerialAT = datetimeSerialAT.substring(a + 1, len);
      datetimeSerialAT.replace("\"", "");

      if (datetimeSerialAT.substring(0, 2).toInt() >= 19)
      {
        DAY_ = datetimeSerialAT.substring(6, 8);
        DAY1 = datetimeSerialAT.substring(6, 8).toInt();
        Serial.println("DAY1 :" + String(DAY1));
        MONTH_ = datetimeSerialAT.substring(3, 5);
        if (datetimeSerialAT.substring(0, 2).toInt() < 30)
          YEAR_ = String(datetimeSerialAT.substring(0, 2).toInt() + 2000);
        else
          YEAR_ = String(datetimeSerialAT.substring(0, 2).toInt());
        Hours_ = String(datetimeSerialAT.substring(9, 11).toInt());
        // if (datetimeSerialAT.substring(9, 11).toInt()==0)
        //  Hours=String(23);
        // else
        // {
        //  Hours=String(datetimeSerialAT.substring(9, 11).toInt());
        // }

        Minutes_ = datetimeSerialAT.substring(12, 14);
        Seconds_ = datetimeSerialAT.substring(15, 17);
        hourOfDay = Hours_.toInt();
        // hourOfDay = Minutes.toInt();
        // (hourOfDay=hourOfDay-27);

        datetimeSerialAT = "";
        Serial.println("Hours: " + Hours_);
        String m, d;
        m = String(MONTH_);
        d = String(DAY_);
        if (MONTH_.length() < 2)
          m = String("0") + String(MONTH_);
        if (DAY_.length() < 2)
          d = String("0") + String(DAY_);
        dt = String(YEAR_) + "-" + String(m) + "-" + String(d) + " " + String(Hours_) + ":" + String(Minutes_) + ":" + String(Seconds_);
        Serial.println("DT:" + dt);
        m = "";
        d == "";
        // if (datetimeSerialAT.substring(0, 2).toInt() >= 19)
        // if (p == 1  )
        //  StoreRTC(dt);
        p = 0;
      }
    }
  }
}

String updateSerial(String s)
{
  String str;
  delay(50);

  int64_t timestamp1 = millis();
  while (SerialAT.available())
  {

    str += SerialAT.readString();
    str.replace(s, "");
    str.trim();

    Serial.println(str); //Forward what Software Serial received to Serial Port
  }
  return str;
}
String updateSerial()
{
  String str = "";
  delay(50);
  int64_t timestamp1 = millis();
  while (SerialAT.available())
  {

    str += SerialAT.readString();
    str.trim();

    Serial.println(str); //Forward what Software Serial received to Serial Port
  }
  return str;
}

boolean GPRS_setup()
{
  boolean ret = false;

  return ret;
}

void GPRS_close()
{

  //if (smar2alarm.activityAlarm ==0)
  {

    SerialAT.println("AT+SAPBR=0,1"); //close PDP
    updateSerial();

    Serial.println("CLOSE GPRS");
    GPRS_on = false;

    SerialAT.println("AT+CMGF=1"); // Configuring TEXT mode
    updateSerial();

    SerialAT.println("AT+CNMI=1,2,0,0,0");
    updateSerial();
    // SerialAT.println("AT+CFUN=0");
  }
}

void beep(int times)
{

  for (int i = 0; i < times; i++)
  {

    pinMode(ALARM_PIN, OUTPUT);
    digitalWrite(ALARM_PIN, HIGH);
    delay(500);
    pinMode(ALARM_PIN, INPUT);
    delay(1000);
  }
}

String msg; // "{\"uid\":\"de8f0hg7573174f6b9\", \"gtp\":2, \"gwg\":16.2, \"tcd\":\"qwertyihjjkzcxzx\",\"scd\":\"XRB-SGR-40B9\",\"dts\":\"2002/10/06 16:01:30\",\"tmp\":30.2,\"hmt\":60.2,\"ver\":11.2,\"btr\":5.0,\"sst\":1}";
String encrypt(String msg)
{
  char b64data[msg.length() * 2 + 50];
  byte cipher[msg.length() * 2 + 50];
  byte iv[N_BLOCK] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};

  base64_encode(b64data, (char *)iv, N_BLOCK);
  String IV_base64 = String(b64data);
  Serial.println(" IV b64: " + IV_base64);

  int b64len = base64_encode(b64data, (char *)msg.c_str(), msg.length());

  Serial.println(" The lenght is:  " + String(msg.length()));

  // Encrypt! With AES128, our key and IV, CBC and pkcs7 padding
  aes.do_aes_encrypt((byte *)b64data, b64len, cipher, key, 128, iv);

  Serial.println("Cipher size: " + String(aes.get_size()));

  base64_encode(b64data, (char *)cipher, aes.get_size());
  Serial.println("Encrypted data in base64: " + String(b64data));

  // decrypt(b64data, IV_base64, 500);
  return String(b64data);
}
void decrypt(String b64data, String IV_base64, int lsize)
{
  char data_decoded[lsize];
  char iv_decoded[lsize];
  byte out[lsize];
  char temp[lsize];
  b64data.toCharArray(temp, lsize);
  base64_decode(data_decoded, temp, b64data.length());
  //base64_decode(data_decoded, temp, b64data.length());
  IV_base64.toCharArray(temp, lsize);
  base64_decode(iv_decoded, temp, IV_base64.length());
  aes.do_aes_decrypt((byte *)data_decoded, lsize, out, key, 128, (byte *)iv_decoded);
  char message[msg.length()];
  base64_decode(message, (char *)out, aes.get_size());
  for (int i = 0; i < aes.get_size(); i++)
  {
    char curChar = (char)message[i];
    if (curChar != '}')
      temp[i] = curChar;
    else
    {
      temp[i] = curChar;
      temp[i + 1] = '\0';
      break;
    }
  }
  String result = String((char *)temp);
  Serial.println(result);
}

void setup_aes()
{
  aes.set_key(key, sizeof(key)); // Get the globally defined key
}
void resetBuffer()
{

  memset(buffer, 0, sizeof(buffer));
  pos = 0;
}

String parseATText(byte b)
{

  // buffer[pos++] = b;

  if (pos >= sizeof(buffer))
    resetBuffer(); // just to be safe

  switch (parseState)
  {
  case PS_DETECT_MSG_TYPE:
  {
    if (b == '\n')
      resetBuffer();
    else
    {
      if (pos == 3 && strcmp(buffer, "AT+") == 0)
      {
        parseState = PS_IGNORING_COMMAND_ECHO;
      }
      else if (b == ':')
      {
        //Serial.print("Checking message type: ");
        //Serial.println(buffer);

        if (strcmp(buffer, "+HTTPACTION:") == 0)
        {
          Serial.println("Received HTTPACTION");
          parseState = PS_HTTPACTION_TYPE;
        }
        else if (strcmp(buffer, "+HTTPREAD:") == 0)
        {
          Serial.println("Received HTTPREAD");
          parseState = PS_HTTPREAD_LENGTH;
        }
        resetBuffer();
      }
    }
  }
  break;

  case PS_IGNORING_COMMAND_ECHO:
  {
    if (b == '\n')
    {

      Serial.print("Ignoring echo: ");
      Serial.println(buffer);
      parseState = PS_DETECT_MSG_TYPE;
      resetBuffer();
    }
  }
  break;

  case PS_HTTPACTION_TYPE:
  {
    if (b == ',')
    {
      Serial.print("HTTPACTION type is ");
      Serial.println(buffer);

      parseState = PS_HTTPACTION_RESULT;
      resetBuffer();
    }
  }
  break;

  case PS_HTTPACTION_RESULT:
  {
    if (b == ',')
    {

      Serial.print("HTTPACTION result is ");
      Serial.println(buffer);
      result = String(buffer);
      parseState = PS_HTTPACTION_LENGTH;
      resetBuffer();
    }
  }
  break;

  case PS_HTTPACTION_LENGTH:
  {
    if (b == '\n')
    {
      Serial.print("HTTPACTION length is ");
      Serial.println(buffer);
      // now request content

      //  SerialAT.print("AT+HTTPREAD=0,");
      //  Serial.println(buffer);
      //  delay(1000);
      //
      parseState = PS_DETECT_MSG_TYPE;
      resetBuffer();
    }
  }
  break;

  case PS_HTTPREAD_LENGTH:
  {
    if (b == '\n')
    {
      contentLength = atoi(buffer);

      Serial.print("HTTPREAD length is ");
      Serial.println(contentLength);

      Serial.print("HTTPREAD content: ");

      parseState = PS_HTTPREAD_CONTENT;
      resetBuffer();
    }
  }
  break;

  case PS_HTTPREAD_CONTENT:
  {
    // for this demo I'm just showing the content bytes in the serial monitor
    Serial.write(b);

    contentLength--;
    // Serial.println(buffer);

    if (contentLength <= 0)
    {

      // all content bytes have now been read

      httpContent = String(buffer);
      Serial.println("httpContent:->" + httpContent);
      parseState = PS_DETECT_MSG_TYPE;
      resetBuffer();
    }
  }
  break;
  }
}

void blinkGreen(void *pvParameters)
{
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  int p = 1;
  for (;;)
  {
    vTaskDelay(100);
    
   
   //if (CoverStatus ==Locker_Closed )
    
    {
     if (millis() - start_time > 8000 && wa < 6)
      {
        readscale1();

      }
     if (millis() - start_timeAQ > 200)
     {
     // AirPollution();
      start_timeAQ=millis();
     }
    }
    //if (tagId != "")
    {
      ReadAccelerometer();
      CalculateCoverRange();
    }
    if (scaleIsConnected == 0)
    if (Serial1.available())
    Serial1.readStringUntil('\n');
   //Serial.println("Weigh Readings : " + Serial1.readStringUntil('\n')); 
   
  }
  
  if ( CoverStatus==Locker_Closed  && p ==3)  lockByServo();

  if (CoverStatus==Locker_Closed  && (TagPresent == 0) || we == 1 )
  {
    if (we == 1 || gf>0)
      p = 3;

    if ((millis() - delayTime) < 500 / p && (millis() - delayTime) > 0)
    {
      pinMode(GREEN_PIN, OUTPUT);
      digitalWrite(GREEN_PIN, LOW);
    }
    if ((millis() - delayTime) > 500 / 2 && (millis() - delayTime) < 999 / 2)
    {
      pinMode(GREEN_PIN, INPUT);
    }
    if ((millis() - delayTime) > 1000 / p)
    {
      delayTime = millis();
      pinMode(GREEN_PIN, OUTPUT);
      digitalWrite(GREEN_PIN, LOW);
    }
  }
  else
  {
    pinMode(GREEN_PIN, OUTPUT);
    digitalWrite(GREEN_PIN, LOW);
    if ( wa >= 6 || Garbagecollection == 1)  modem_on();
  }
}

boolean ScanAddress(int address)
{

  byte error;
   //Serial.println("Scanning...");
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
       
      return true;
    }
    return false;
   
  }
}

boolean postlog(String payload)

{
  boolean ret = false;
  //GPRS_close();

  // if (DTC == 1)
  // Checkinternet();

  //
  // Serial.println("getFreeHeap " + String(ESP.getFreeHeap()));
  // wait for WiFi connection
  delay(100);

  WiFi.begin("TESLA1", "!9653711!");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");

  // Init and get the time

  if ((WiFi.status() == WL_CONNECTED))
  {

    HTTPClient http;
    Serial.print("[HTTP] begin...\n");

    //String url = URL + endpoint;

    String url = "http://device.scalebot.gr/api/Garb/DeviceSendBinInfo";
    delay(1000);
    Serial.println(url);
    payload = encrypt(payload);
    payload = "{\"request_data\":\"" + payload + "\"}";
    Serial.println(payload);
    http.setTimeout(15000);
    //  for (int i = 0; i < Datalist.size(); i++)
    {

      if (http.begin(url))
      {
        http.addHeader("Content-Type", "application/json");

        http.addHeader("app_type", "dSG46bbpUPuYxYpR2Lu9uA==");
        //
        //   Serial.println(smar2alarmDatalist.get(i)->data);
        int httpCode = http.POST(payload);

        if (httpCode > 0)
        {

          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
          {
            String payload = http.getString();
            ret = true;
            Serial.println(payload);
            httpContent = payload;
            payload = "";
          }
        }
        else
        {
          Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }
      }
      else
      {
        Serial.printf("[HTTP} Unable to connect\n");
      }
      http.end();
    }

    url = "";
  }
  else
  {
  }

  return ret;

  // ESP.deepSleep(30e6, WAKE_RF_DEFAULT); //sleep for 900 seconds
  //delay(5*60000);
}
void openScale()
{
   
  pinMode(SCALE_ONOFF_PIN, OUTPUT);
  digitalWrite(SCALE_ONOFF_PIN, HIGH);
  delay(200);
  Serial.println("openScale ");
  pinMode(SCALE_ONOFF_PIN, INPUT);
}
void openScaleBin()
{

  pinMode(SCALE_ONOFF_PIN, OUTPUT);
  digitalWrite(SCALE_ONOFF_PIN, HIGH);

  Serial.println("************* OpenScale ***************** ");
}

void CloseScaleBin()
{

  pinMode(SCALE_ONOFF_PIN, INPUT);

  Serial.println("************* CloseScale ***************** ");
}

int checklist(int l)
{
  for (int i = 0; i < LostDatalist.size(); i++)
  {
    if (LostDatalist.get(i) == String(l))
      return 0;
  }
  return 1;
}
boolean secure_postlog_gprs()
{

  //GPRS_enable();
  httpContent = "";
  result = "";
  boolean ret = false;
  int lastStringLength, r;
  String str, payload;

  String url = "http://device.scalebot.gr/api/Garb/DeviceSendBinInfo";
  String url1="http://3v.device.informula.gr/api/Garb/DeviceUpdateBinInfo";

  //delay(1000);
  for (int i = 0; i < Datalist.size(); i++)
  {
    if (checklist(i) == 1)
    {

      httpContent = "";
      payload = Datalist.get(i);
      if (payload=="")
      return true;
      if (payload.indexOf("sst") > 0)
        url = "http://device.scalebot.gr/api/Garb/DeviceSendInfo";
      else if (payload.indexOf("b_c") > 0)
        url = "http://device.scalebot.gr/api/Garb/DeviceUpdateBinInfo";
      else
        url = "http://device.scalebot.gr/api/Garb/DeviceSendBinInfo";

      payload.replace("timestamp", dt);
      // payload.replace("5240A149","code_mix1");//test

      Serial.println(url);
      payload.replace("\"[","[");
      payload.replace("]\"","]");
      payload.replace("\\","");
      //payload.replace("Bin_scale_V3","Bin_scale_Ver3");
      Serial.println(payload);
      payload = encrypt(payload);
 
       
      payload = "{\"request_data\":\"" + payload + "\"}";
      Serial.println(payload);
      lastStringLength = payload.length();
       
      sendGSM("AT+HTTPINIT", 50);
      sendGSM("AT+HTTPPARA=\"URL\",\"" + url + "\"", 50);
      sendGSM("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
      //sendGSM("AT+SHCONF=\"app_type\",\"dSG46bbpUPuYxYpR2Lu9uA==\"");
      sendGSM("AT+HTTPDATA=" + String(lastStringLength) + ",15000\r", 2000);
      sendGSM(payload, 2000);
      sendGSM("ATE0&W", 100);
      sendGSM("AT+HTTPACTION=1", 5000);
      int i1;
      Serial.println("Result --> " + result);
      if (result == "200,")
      {
        ret = true;
        r = 0;
        SerialAT.println("AT+HTTPREAD");
        sendGSM("AT+HTTPREAD", 2000);
        Serial.println("httpContent:-> " + httpContent);
        if (httpContent != "0" && httpContent != "1002" && httpContent != "4008")
        {
          // LostDatalist.add(i);
          // store_LostDatalist();
        }
        else if (httpContent == "4008")
        {
          StaticJsonDocument<200> doc;
          deserializeJson(doc, Datalist.get(i));
          const char *t = doc["tcd"];
          String tg = reinterpret_cast<const char *>(t);
          Serial.println("Citizen_inValid  Tagid:" + tg);
          store_Tag(Citizen_inValid, tg);
          LostDatalist.add(String(i));
          store_LostDatalist(i1);
        }
        else
        {
          LostDatalist.add(String(i));
          store_LostDatalist(i1);
        }
      }
      Serial.println("payload httpContent:-> " + httpContent);
      sendGSM("ATE1&W", 50);
      sendGSM("AT+HTTPTERM\r");
      delay(500);
      // beep(1);
    }
    httpContent = "";
    payload = "";
    result = "";
    //PostTime = millis();
  }
  return ret;
}

boolean secure_postlog_gprs1()
{

  //GPRS_enable();
  httpContent = "";
  result = "";
  boolean ret = false;
  int lastStringLength, r;
  String str, payload;
  String url = "http://device.scalebot.gr/api/Garb/DeviceSendBinInfo";

  delay(3000);
  for (int i = 0; i < Datalist.size(); i++)
  {

    payload = Datalist.get(i);
    payload.replace("timestamp", dt);
    Serial.println(payload);
    payload = encrypt(payload);
    payload.replace("timestamp", dt);
    payload = "{\"request_data\":\"" + payload + "\"}";
    lastStringLength = payload.length();

    sendGSM("AT+HTTPINIT", 50);
    sendGSM("AT+HTTPPARA=\"URL\",\"" + url + "\"", 50);
    sendGSM("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
    //sendGSM("AT+SHCONF=\"app_type\",\"dSG46bbpUPuYxYpR2Lu9uA==\"");
    sendGSM("AT+HTTPDATA=" + String(lastStringLength) + ",15000\r", 2000);
    sendGSM(payload, 2000);
    sendGSM("ATE0&W", 100);
    sendGSM("AT+HTTPACTION=1", 5000);
    int i1;
    Serial.println("Result --> " + result);

    if (result == "200,")
    {
      ret = true;
      r = 0;
      SerialAT.println("AT+HTTPREAD");
      sendGSM("AT+HTTPREAD", 2000);
      Serial.println("Http:" + httpContent);
      // if (httpContent!=0)
      // LostDatalist.add(payload);
    }

    Serial.println("payload httpContent" + httpContent);
    if (result == "604," || result == "603," || result == "601," || result == "")
    {
      r++;
      if (r == 3)
      {

        updateSerial();
        GPRS_setup();
        SerialAT.println("AT+CNTPCID=1");
        SerialAT.println("AT+CNTP=\"time1.google.com\",0");
        delay(1000);
        updateSerial();
        SerialAT.println("AT+CNTP");
        delay(1000);
        updateSerial();
        initCLTS();
        delay(1000);
      }
      // return ret;
    }
    sendGSM("ATE1&W", 50);
    sendGSM("AT+HTTPTERM\r");
    delay(3000);
  }

  payload = "";
  result = "";
  //PostTime = millis();
  return ret;
}

void GPSPowerOn()
{
  sendGSM("AT+CGNSPWR=1", 2000);
  for (int i = 0; i < 20; i++)
    sendGSM("AT+CGNSINF", 500);
}

String s;
void store_Datalist(int g)
{
  SPIFFS.begin();
  s = "";
  const size_t CAPACITY = JSON_ARRAY_SIZE(250);
  StaticJsonDocument<CAPACITY> doc;
  Serial.println("Store Datalist_" + String(g));
  JsonArray array;
  File f;
  f = SPIFFS.open("/Datalist_" + String(g), "w");
  if (!f)
  {

    Serial.println("file open failed");
  }
  else
  {
    array = doc.to<JsonArray>();
    int o;

    for (int i = 0; i < Datalist.size(); i++)
    {

      array.add((Datalist.get(i)));
      // if (i%10==0 && Datalist.size()>=10)
      // {
      //         serializeJson(doc, s);
      //         Serial.println("Store Tag data" + s);
      //         f.println(s);
      //         doc.clear();
      //         array = doc.to<JsonArray>();
      //         s = "";
      //         o++;
      // }
    }

    serializeJson(doc, s);
    if (s != "")
    {

      Serial.println("Store Log data_" + String(g) + s);
      f.println(s);
      doc.clear();
      s = "";
    }
  }
  Serial.println("List Size" + String(Datalist.size()));
  f.close();
  doc.clear();
  Datalist.clear();
  s = "";
}

void restore_Datalist(int g)
{

  s = "";
  //ESP.getMaxAllocHeap()
  const size_t CAPACITY = JSON_ARRAY_SIZE(250);
  Datalist.clear();
  Serial.println("/Datalist_" + String(g));
  StaticJsonDocument<CAPACITY> doc;
  File f;

  f = SPIFFS.open("/Datalist_" + String(g), "r");
  if (f)
  {

    while (f.available())
    {

      s = f.readStringUntil('\n');
      s.trim();

      Serial.println("Restore Datalist " + s);

      DeserializationError error = deserializeJson(doc, s);

      Serial.println("getFreeHeap " + String(ESP.getFreeHeap()));
      if (error)
      {
        //Serial.println(F("Failed to read gravityList"));
      }
      else
      {
        JsonArray array = doc.as<JsonArray>();
        for (JsonVariant v : array)
        {
          if (!v.isNull())
            Datalist.add(v.as<String>());
        }
        doc.clear();
      }
      doc.clear();
      s = "";
    }

    f.close();
  }
}
void store_LostDatalist(int g)
{
  SPIFFS.begin();
  s = "";
  const size_t CAPACITY = JSON_ARRAY_SIZE(250);
  StaticJsonDocument<CAPACITY> doc;
  Serial.println("Store Send Datalist");
  JsonArray array;
  File f;
  f = SPIFFS.open("/LostDatalist_" + String(g), "w");
  if (!f)
  {

    Serial.println("file open failed");
  }
  else
  {
    array = doc.to<JsonArray>();
    {
      for (int i = 0; i < LostDatalist.size(); i++)
      {
        array.add((LostDatalist.get(i)));
      }

      serializeJson(doc, s);
      Serial.println("Store Send Datalist" + s);
      f.println(s);
      doc.clear();
      s = "";
    }
    // Serial.println("file Size" + String(f.size()));
    f.close();
    doc.clear();
    s = "";
  }
}

void restore_LostDatalist(int g)
{

  s = "";
  const size_t CAPACITY = JSON_ARRAY_SIZE(250);
  LostDatalist.clear();
  Serial.println("Size CAPACITY " + String(CAPACITY));
  StaticJsonDocument<CAPACITY> doc;
  File f;

  f = SPIFFS.open("/LostDatalist_" + String(g), "r");
  if (f)
  {

    while (f.available())
    {

      s = f.readStringUntil('\n');
      s.trim();

      Serial.println("Restore Already post Datalist " + s);

      DeserializationError error = deserializeJson(doc, s);

      Serial.println("getFreeHeap " + String(ESP.getFreeHeap()));
      if (error)
      {
        //Serial.println(F("Failed to read gravityList"));
      }
      else
      {
        JsonArray array = doc.as<JsonArray>();
        for (JsonVariant v : array)
        {

          LostDatalist.add(v.as<String>());
        }
        doc.clear();
      }
      doc.clear();
      s = "";
    }

    f.close();
  }
}

void store_Taglist()
{
  s = "";
  const size_t CAPACITY = JSON_ARRAY_SIZE(100);
  StaticJsonDocument<CAPACITY> doc;
  Serial.println("Store Taglist");
  Serial.println("Size " + String(Taglist.size()));
  JsonArray array;
  File f;
  f = SPIFFS.open("/Taglist", "w");
  array = doc.to<JsonArray>();
  {
    for (int i = 0; i < Taglist.size(); i++)
    {
      array.add((Taglist.get(i)));
    }

    serializeJson(doc, s);
    Serial.println("Store Tag data" + s);
    f.println(s);
    doc.clear();
    s = "";
  }
  //Serial.println("file Size" + String(f.size()));
  f.close();
  doc.clear();
  s = "";
}

void store_frsboot()
{

  StaticJsonDocument<300> doc;
  Serial.println("Save reboot data");
  doc["firstboot"] = String(dt);

  File f = SPIFFS.open("/firstboot", "w");
  if (serializeJson(doc, f) == 0)
  {
    Serial.println(F("Failed to write to file"));
  }

  f.close();
  doc.clear();
}
void PutDataBuff()
{
  // if(tagId!="")
  {
    getSettings();
    s = "";
    Serial.println("getFreeHeap " + String(ESP.getFreeHeap()));
    //getRTC();
    StaticJsonDocument<200> doc;

    doc["gwg"] = weightF;      //,----- garbage_weight (decimal)
    doc["tcd"] = tagId;        //: "qwertyuiopasdfghjjkzcxzxvxcbcnvmbn",----- tag_code (string)
    doc["scd"] = DSN;          //: "qwertyuiopasdfghjjkzcxzxvxcbcnvmbn",------ scale_code(string)
    doc["dts"] = "timestamp";  //: "8/24/2022 6:01:28 PM"",------ system datetime for encryption(string)
    doc["btr"] = batterylevel; // battery

    serializeJson(doc, s);
    Serial.println(s);
    Datalist.add(s);
    gr++;
    store_Datalist(gr);
    storeGr();
    if (Datalist.size() >= 10)
    {

      gr++;
      storeGr();
      Datalist.clear();
    }

    doc.clear();
    s = "";
  }
  // step = 0;
}

void beepD(int delay)
{
  // pinMode(GREEN_PIN, OUTPUT);
  // digitalWrite(GREEN_PIN, LOW);
  // tone(BUZZER_PIN, 600, delay, BUZZER_CHANNEL);
  // noTone(BUZZER_PIN, BUZZER_CHANNEL);
  // pinMode(GREEN_PIN, INPUT);
}

void Green()
{

  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, LOW);
}

void printLocalTime()
{
  struct tm timeinfo;
  char dt[20];
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
  strftime(dt, 20, "%Y-%m-%d %H:%M:%S", &timeinfo);
  Serial.println("time:" + String(dt));
  Serial.println();
}
void restore_frsboot()
{

  StaticJsonDocument<300> doc;
  Serial.println("restore firstboot data");
  File f = SPIFFS.open("/firstboot", "r");
  if (f.available())
  {

    DeserializationError error = deserializeJson(doc, f);
    if (error)
    {
      Serial.println(F("Failed to read file Serial, using default configuration"));
    }
    else
    {
      const char *t = doc["firstboot"];
      FirstBoot = reinterpret_cast<const char *>(t);
      Serial.println(FirstBoot);
    }
    f.close();
  }
  else
  {
    store_frsboot();
  }

  doc.clear();
}
byte cn;
boolean checkWeight()
{
  cn = 0;
  weight = 0;
  if (WeightList.size() >= 30)
  {
   /// Serial.println("weight size" + String(WeightList.size()));
    for (int i = 20; i < 30; i++)
    {
       Serial.println("weight list:  " + String(WeightList.get(i)));
      // if (WeightList.get(WeightList.size() - 1) == WeightList.get(i))
      //   cn++;
      // else
      // {
      //   // cn--;
      // }

      // if (cn == 10)
      //   break;
      cn++;
      weight = weight + WeightList.get(i);
    }
    weight = weight / cn;
    // if (cn >= 10 && WeightList.size() >= 30)
    {

      // weight = WeightList.get(WeightList.size() - 1);
      WeightList.clear();
      Serial.println("weight -->" + String(weight));

      return true;
    }
  }

  return false;
}

void readscale()
{
 // modemPowerOn();
  String str;
  scaleIsConnected = 1;
  delay(100);
  while (Serial1.available())
  {
     
    str += Serial1.readStringUntil('\n');
    str.trim();
    // Serial.println("Weight:" + str); //Forward what Software Serial received to Serial Port
    // ParseSerialData(str,";");
    str.replace("kg", "");
    str.replace("+", "");
    str.replace("wn", "");
   
    weight = str.toDouble();
    WeightList.add(weight);
    if (WeightList.size() > 30)
      WeightList.remove(0);
   // Serial.println("Weight:" + str);
  
  // if (checkWeight() == true && weight ==0.0000) //test
  // if (millis() - start_time > 5000 && digitalRead(STATUSSCALE_PIN) == Locker_Closed) //change
  // {
  //    weightF = 10;
  //    if (we == 0)
  //    PutDataBuff();
  //    we = 1;
  // }
  if (checkWeight() == true) //&& digitalRead(STATUSSCALE_PIN) == Locker_Closed
  {
    if (weightA == weightA && (weight+0.9)>weightA)
      weightF = abs(abs(weight) - abs(weightA));
    else
      weightF = abs(weight);
    if (we == 0 && weightF > 0.3 && weight > 0.3 )
    {
      we = 1;
     //*****************************   Κλείσε την ζυγαρία  ***********************************
      CloseScaleBin();
     // modem_on();
      PutDataBuff();
       
      //beep(1);
      beepDelay(200);
      Serial.println("Weight true:" + String(weightF));
      open_cover_time = millis();
      return;
    }

 
  }
   return;
  }
   
}

void readscale1()
{
  String str;
  scaleIsConnected = 0;
  //delay(100);
  while (Serial1.available())
  {

    scaleIsConnected = 1;
    str += Serial1.readStringUntil('\n');
    str.trim();
    // Serial.println("Weight:" + str); //Forward what Software Serial received to Serial Port
    // ParseSerialData(str,";");f
    str.replace("kg", "");
    str.replace("+", "");
    str.replace("wn", "");
    weight = str.toDouble();
  if (weight == weight )
    {
      Serial.println("Weight:" + String(weight));
      if (wa>0 && weight>0.00) WeightList.add(weight);
      wa++;
      Serial.println("A:" + String(wa));
    }
    if (WeightList.size() > 10)
      WeightList.remove(0);
    Serial.println("WeightA:" + str);
    return;
  }
}

void ParseSerialData(String text, String splitChar)
{

  text.trim();
  int len = text.length();
  int firstClosingBracket = text.indexOf(';');
  int secondClosingBracket = text.indexOf(';', firstClosingBracket + 1);
  int thirdClosingBracket = text.indexOf(';', secondClosingBracket + 1);
  String s1 = text.substring(0, firstClosingBracket);
  String s2 = text.substring(firstClosingBracket + 1, secondClosingBracket);
  String s3 = text.substring(secondClosingBracket + 1, thirdClosingBracket);
  String s4 = text.substring(thirdClosingBracket + 1, len);
  Serial.println("s1:" + s1);
  Serial.println("s2:" + s2);
  Serial.println("s3:" + s3);
  Serial.println("s4:" + s4);
}

void checkFormat()
{

  delay(1000);
  SPIFFS.begin();
  if (!SPIFFS.exists("/formatComplete.txt"))
  {

    Serial.println("Please wait 30 secs for SPIFFS to be formatted");
    SPIFFS.format();

    delay(30000);
    Serial.println("Spiffs formatted");
    File f = SPIFFS.open("/formatComplete.txt", "w");
    if (!f)
    {

      Serial.println("file open failed");
    }
    else
    {

      f.println("Format Complete");

      formatF = 1;
    }
    f.close();
  }
  else
  {

    Serial.println("SPIFFS is formatted.");
  }
  //  while (!testsenors())
  //     delay(50);
}
void getSerial()
{
  SPIFFS.begin();
  StaticJsonDocument<300> doc;

  File f = SPIFFS.open("/Serial", "r");
  if (f.available())
  {

    DeserializationError error = deserializeJson(doc, f);
    if (error)
    {

      Serial.println(F("Failed to read file Serial, using default configuration"));
    }
    else
    {
      Serial.println("==========================================================");
      Serial.println(F("BinBot is Initialized "));

      const char *SER = doc["DSN"];
      DSN = reinterpret_cast<const char *>(SER);
      Serial.println("DSN : " + DSN);
      Serial.println("==========================================================");
    }

    f.close();
    doc.clear();
  }
}
int get_distance(int addr) {
  unsigned short dist = 0;
  Wire.beginTransmission(addr);
  Wire.write(0);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(addr, 2);
  if (Wire.available() != 2)
    return -1;
  dist = Wire.read() << 8;
  dist |= Wire.read();
  if (dist<0)
  dist=0;
  return dist;
}

void CheckBinIsFull()
{

}

void getSettings()
{
  SPIFFS.begin();
  StaticJsonDocument<300> doc;

  File f = SPIFFS.open("/Settings", "r");
  if (f.available())
  {

    DeserializationError error = deserializeJson(doc, f);
    if (error)
    {

      Serial.println(F("Failed to read file Serial, using default configuration"));
    }
    else
    {
      Serial.println("==========================================================");
      Serial.println(F("BinBOT settings "));

      BinFull = doc["BFL"];
      Serial.println("BinFull : " + String(BinFull));
      Total_mA = doc["MAH"];
      if (Total_mA==INFINITY)
      Total_mA=0;
      Serial.println("mAh : " + String(Total_mA));
      chgTimes = doc["CHG"];
      Serial.println("chgTimes : " + String(chgTimes));
    //  Servo_i= doc["SRV"];
      Serial.println("Servo_i : " + String(Servo_i));
      Serial.println("==========================================================");
    }

    f.close();
    doc.clear();
    calc_battery();
  }
}

void initialize_Scale(String DSN)
{
  SPIFFS.begin();

  {
    Serial.println((DSN));
    StaticJsonDocument<50> js;
    js["DSN"] = DSN;
    File f = SPIFFS.open("/Serial", "w");
    if (serializeJson(js, f) == 0)
    {
      Serial.println(F("Failed to write to file"));
    }
    else
    {
      Serial.println(F("Success to write to file"));
    }
    f.close();
  }

  //store_Tag(System,"731EC49A");
}
void storeSettings()
{

  SPIFFS.begin();
  {

    StaticJsonDocument<300> js;
    js["BFL"] = BinFull;
    js["MAH"] = Total_mA;
    js["CHG"] = chgTimes;
    js["SRV"] =Servo_i;
    File f = SPIFFS.open("/Settings", "w");
    if (serializeJson(js, f) == 0)
    {
      Serial.println(F("Failed to write to file"));
    }
    else
    {
      Serial.println(F("Success to write to file"));
    }
    f.close();
  }
}

void storeGr()
{

  {
    SPIFFS.begin();
    StaticJsonDocument<50> js;
    js["GR"] = gr;
    File f = SPIFFS.open("/Gr", "w");
    if (serializeJson(js, f) == 0)
    {

      Serial.println(F("Failed to write to file"));
    }
    else
    {
    }
    f.close();
  }
}
void getGr()
{

  {
    SPIFFS.begin();
    StaticJsonDocument<50> js;

    File f = SPIFFS.open("/Gr", "r");
    DeserializationError error = deserializeJson(js, f);
    if (error)
    {

      Serial.println(F("Failed to read "));
    }
    else
    {
      gr = js["GR"];
      Serial.println("GR: " + String(gr));
    }
    f.close();
  }
}
int dist1 =0;
void CalculateCoverRange()
{ 
   check_battery();
  if (   I2cbusy==1  ||  (Sen1==0)  ) return;
      I2cbusy=1;
      for (int i = 0; i < 2 ; i++)
      { 
        dist1 = Cover_sensor.readRangeSingleMillimeters();
       
        delay(50);
       if ( dist1 >0)
        {
            CoverRange.add(dist1); 
            if (CoverRange.size() > 2)
              CoverRange.remove(0);
          if (CoverRange.size() == 2)
          {
            dist1 =0;
            for (int i = 0; i < CoverRange.size() ; i++)
                dist1 =dist1+ CoverRange.get(i);
            dist1=dist1/CoverRange.size() ;
            CoverRange.clear();
            CheckCoverSTatus();
          }
        }
      }
       Serial.println("Cover is   : " + String(dist1));
       I2cbusy=0;
}

int checkClosed=0;
int checkOpened=0;
int CheckCoverSTatus()
{
    
  // Serial.println("**** Check Cover****");
  if (dist1<20 &&  CoverStatus!=Locker_Closed)
    { 
       Serial.println("Cover is  Closed : " + String(dist1));
       checkClosed++;
       if (checkClosed==4)
       {
         CoverStatus=Locker_Closed;
          checkClosed=0;
          checkOpened=0;
       }

       return CoverStatus;
    }
   if (dist1>25 &&  CoverStatus==Locker_Closed )
   {
     Serial.println("Cover is  Opened : "  + String(dist1));
         checkOpened++;
       if (checkOpened==2)
       {
          CoverStatus=Locker_Opened;
           checkOpened=0;
           checkClosed=0;
       }
     return   CoverStatus;  
   }
 


}

boolean checkFullBin()
{
  if (   I2cbusy==1 || Sen3==0 || Sen4==0)  return false;
  I2cbusy=1;
  if (Sen3==0) 
   int dist1 = get_distance(Left_Range);//Left
   int dist2 = get_distance(Right_Range);//Right
   for (int i = 0; i < 20; i++)
   {
    dist1 =dist1+ get_distance(Left_Range);
    dist2 =dist2+ get_distance(Right_Range);
    delay(50);
   }
   dist1=dist1/21;
   dist2=dist2/21;
  Serial.println("**** CheckFullBin ****");
  if (dist1>90)
    Serial.println("SENSOR1 EMPTY  : " + String(dist1));
  else
    Serial.println("SENSOR1 FULL : "  + String(dist1));

  if (dist2>90)
     Serial.println("SENSOR2 EMPTY : "  + String(dist2));
  else
    Serial.println("SENSOR2 FULL : "  + String(dist2));
    I2cbusy=0;
  if ( dist1<90 &&  dist2<90)
    return true;
  else
    return false;
}

void unlock()
{
   UnlockByServo(); 
 
  //bin is opened
  if (CoverStatus ==Locker_Opened)
  {
    pinMode(LOCKER_PIN, INPUT);
    if ( Garbagecollection == 0) 
    Serial.println("Bin is opened by the citizen");
    //if (scaleIsConnected == 0 && Garbagecollection == 0) // open scale if citizen is here
    // openScaleBin();
    //  scale_status = 1;
    Lockstatus = 1;
    mustUnlock = 0;
    TagPresent=0;
    start_time = millis();
  }
}

boolean checkTags()
{
  int tagExist = 0;
  if (tagId == "5240A149---")
  {
    if (System_Tag == 0)
    {
      System_Tag = 1;
      Serial.println(" LEARNING MODE ON");
    }
    else
    {
      System_Tag = 0;
      Serial.println(" LEARNING MODE OFF");
    }

    //return true; //test
    return false;
  }

  for (int i = 0; i < Taglist.size(); i++)
  {

    if (Taglist.get(i)->tagId == tagId)
    {
      tagExist = 1;
      if (SameTagID == tagId)
      {
        countId++;
        // beepDelay(500);
      }
      else
        SameTagID = tagId;
      if (Taglist.get(i)->type == Garbage_CollectorTag) // Tag Αποκομιδής άδειασμα κάδου
      {
        // beep(2);
        Garbagecollection = 1;
        Serial.println("Tag Apokomids adeiasma kadou");
        break;
      }
      if (Taglist.get(i)->type == MaintenanceTag) // Tag Συντήρησης
      {
        Mainntanace = 1;
        beep(2);
        Garbagecollection = 0;
        break;
      }
      if (Taglist.get(i)->type == Citizen_inValid)
      {
        Serial.println("Citizen_inValid");
        // done();
        // return false;
      }
      Serial.println("CountId " + String(countId));
    }
  }
  //STORE BINBOT TAGID  ACCORDING TO ITS COUNTER
  if (tagExist == 1 && countId >= 10 && countId >= 15 && System_Tag == 1)
  {

    store_Tag(BinbotID, tagId);
    initialize_Scale(tagId);
    DSN = tagId;
    Mainntanace = 0;
    Garbagecollection = 0;
    //beepDelay(500);
  }

  if (tagExist == 1 && countId >= 6 && countId < 10 && System_Tag == 1)
  {
    store_Tag(MaintenanceTag, tagId);

    //beepDelay(1000);
  }
  if (tagExist == 1 && countId >= 3 && countId < 6 && System_Tag == 1)
  {
    store_Tag(Garbage_CollectorTag, tagId);
    //beepDelay(1000);
  }
  if (tagExist == 0)
  {
    store_Tag(Citizen_Valid, tagId);
    // beepDelay(1000);
    Serial.println("Citizen_Valid!!!!");
  }
  if (countId >= 15)
    countId = 0;

  return true;
}

void readNFC()
{
  if (   I2cbusy==1 ||   (Sen6==0) ) return;
  I2cbusy=1;
  // Serial.println("Nfc read" );
  if (nfc.tagPresent())
  {
    NfcTag tag = nfc.read();
    tag.print();
    nfc_period = millis();
    tagId = tag.getUidString();
    tagId.replace(" ", "");
    open_cover_time = millis();
    working_period = millis();
   
    // if (CoverStatus ==Locker_Opened)
    //   beepDelay(250);
    if (Lockstatus == 1) //checkTags()==false && //TEST
      step = 1;
    we = 0;
    //if (Lockstatus == 0)
    TagPresent=1; 
  }
  else
  {
  }
  delay(50);
  I2cbusy=0;
}

void CheckNFC()
{
  // Serial.println("Nfc read" );
  if (tagId != "" &&  mustUnlock == 0)
  {
      CalculateCoverRange();
 
    {
      if (checkTags())
        //if (mustUnlock == 0)
        {
          if ((Garbagecollection == 0 && Mainntanace == 0)) // !checkFullBin()  && // && CoverStatus ==Locker_Closed
            mustUnlock = 1;

        if (tagId1!=tagId)
           tagId1=tagId;
          // beep(1);
          String covr;
          if (CoverStatus ==Locker_Opened)
            covr = "Opened";
          else
            covr = "Closed";
          Serial.println("BIN is:" + covr);
          step = 0;
          if (Garbagecollection == 1 || Mainntanace == 1) // unlock without open scale for gargage collection
          {
            //if (Garbagecollection == 1)
            mustUnlock = 1;
            CloseScaleBin();
            Green();
            unlock();
            Serial.println("Bin is self unlocked");
            start_time = millis();
            beep(2);
            return;
          }

          if ( covr = "Closed" && (Garbagecollection == 0 && Mainntanace == 0))
          {
            //Check if bin is full
            // if (checkFullBin() && BinFull == 0) //disabled
            // {
            //   BinFull = 1;
            //   //pinMode(GREEN_PIN, INPUT);
            //   Serial.println("**** Bin Is Full ****");
            //   //BEEP 2 TIMES TO INDICATE BIN IS FULL
            //   // beep(6);
            //   StaticJsonDocument<250> doc;
            //   binstatus = BinIsFull;
            //   doc["sst"] = binstatus;
            //   doc["scd"] = DSN;
            //   doc["dts"] = "timestamp";
            //   doc["btr"] = batterylevel;
            //   serializeJson(doc, s);
            //   Serial.println(s);
            //   Datalist.add(s);
            //   gr++;
            //   storeGr();
            //   store_Datalist(gr);

            //   // if (Garbagecollection != 1 && Mainntanace != 1)
            //   // {
            //   //   enablePost=1;
            //   //   transmit_data();
            //   //   done();
            //   // }
            //   return;
            // }
            // if (!checkFullBin())
            // {
            //   BinFull = 0;
            //   storeSettings();
            // }
            // else
            // {
            //    BinFull =1;
            //   storeSettings();
            // }

            //Green();
            // unlock();
            Serial.println("Please Wait Unlocking Bin");
            return;
          }
        }
    }
  }
  else
  {
  }
  delay(50);
}

void done()
{

  stopWorking = 1;
  Serial.println("Working_time:" + String((float)((float)Working_time / 10000.00)));
  Serial.println("Working_time_connecting:" + String((float)((float)Working_time_connecting / 10000.00)));
  Serial.println("Working_time_posting:" + String((float)((float)Working_time_posting / 10000.00)));
  if (Working_time > 0)
    mAh = mAh + (float)((Working_time / 10000.00) * 1.7);
  Serial.println("Calculation Consumption:" + String(mAh));
  if (Working_time_connecting > 0)
    mAh = mAh + (float)((Working_time_connecting / 10000.00) * 1.4);
  Serial.println("Calculation Consumption:" + String(mAh));
  if (Working_time_posting > 0)
    mAh = mAh + (float)((Working_time_posting / 10000.00) * 1.1);
  Serial.println("Calculation Consumption:" + String(mAh));

  Working_time = 0;
  Working_time_connecting = 0;
  Working_time_posting = 0;
  Serial.println("SYSTEM POWER OFF");
  storeSettings();

  SPIFFS.end();
  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, LOW);
  pinMode(DONE_PIN, OUTPUT);
  digitalWrite(DONE_PIN, HIGH);
}
void AlarmOn()
{
  Serial.println("******************* ALARM IS ON ************************");
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, HIGH);
  StaticJsonDocument<250> doc;
  binstatus = AlarmisOn;
  doc["sst"] = binstatus;
  doc["scd"] = DSN;
  doc["dts"] = "timestamp";
  doc["btr"] = batterylevel;
  serializeJson(doc, s);
  Serial.println(s);
  Datalist.add(s);
  gr++;
  storeGr();
  store_Datalist(gr);
  enablePost = 1;
  transmit_data();
  done();
}

void AlarmOFF()
{
  SPIFFS.end();
  pinMode(ALARM_PIN, INPUT);
}

 
void transmit_data()
{
 
  Working_time = millis() - Working_time;
  if (gr >= 0) //   && enablePost==1
  {
    while  ( isConnected==false && gf<60)
    {
     delay(500);
     gf++;
    }
    gf=0;
   while (dt=="" && gf<20)
     {
       getRTC();
       gf++;
     }
    Serial.println("BinBot has began the trasmitting procedure");
    //*****************************   Κλείσε την ζυγαρία  *************************************
    CloseScaleBin();

    //***************  Immediately open modem to establish internet connection  ****************
    // modemPowerOn();
    //*******************************************************************************************
    // SerialAT.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX); //Modem port

    // modem_on(); 
    //****************************    Έλεγχσς αν ο κάδος γέμισε  ****************************
    if (checkFullBin() && BinFull == 0)
    {
      BinFull = 1;
      storeSettings();
      //pinMode(GREEN_PIN, INPUT);
      Serial.println("**** Bin Is Full ****");
      //BEEP 2 TIMES TO INDICATE BIN IS FULL
      // beep(6);
      StaticJsonDocument<250> doc;
      binstatus = BinIsFull;
      doc["sst"] = binstatus;
      doc["scd"] = DSN;
      doc["dts"] = "timestamp";
      doc["btr"] = batterylevel;
      serializeJson(doc, s);
      Serial.println(s);
      Datalist.add(s);
      gr++;
      storeGr();
      store_Datalist(gr);
      enablePost = 1;
    }
    
    int r;
    File f;
    Working_time_posting = millis();
    delay(1000);
    SPIFFS.begin();
    if (isConnected)
    for (int i1 = 0; i1 < 50; i1++)
    {
      if (SPIFFS.exists("/Datalist_" + String(i1)))
      {
        f = SPIFFS.open("/Datalist_" + String(i1), "r");
        Serial.println("/Datalist_" + String(i1));
        if (f)
        {

          restore_Datalist(i1);
          //restore_LostDatalist(i1);
          if (Datalist.size() > 0)
            secure_postlog_gprs();
          while (LostDatalist.size() != Datalist.size() && r < 20)
          {
            pinMode(LED_PIN, OUTPUT);
            digitalWrite(LED_PIN, LOW);
            secure_postlog_gprs();
            pinMode(LED_PIN, INPUT);
            r++;
          }

          r = 0;
          LostDatalist.clear();
          Datalist.clear();
          SPIFFS.remove("/Datalist_" + String(i1));
          SPIFFS.remove("/LostDatalist_" + String(i1));
          Serial.println("Remove /Datalist_" + String(i1));
        }
        f.close();
      }
    }
   if (isConnected)
   {
    gr = -1;
    storeGr();
   }
   // modemPowerOff();

    Working_time_posting = millis() - Working_time_posting;
    SPIFFS.end();
   // delay(1000);
   // done();
  }

 // done();
  SPIFFS.end();
}

void checkVersion()
{
  beepDelay(2000);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println();
  Serial.println();
  Serial.println();

  for (uint8_t t = 4; t > 0; t--)
  {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    // beepDelay(500);
  }

  // wait for WiFi connection
  if ((WiFi.status() == WL_CONNECTED))
  {

    Serial.println("Downloading new version ....");
    t_httpUpdate_return ret = ESPhttpUpdate.update("http://device.scalebot.gr/api/Garb/GetDeviceVersion/1/1.0");

    ESPhttpUpdate.rebootOnUpdate(false);
    Serial.println(F("\n---- ota update done ----\n"));
    switch (ret)
    {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      // beepDelay(5000);
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      // beepDelay(1000);
      // if (CoverStatus ==Locker_Closed)
      //   lockByServo();
      // Mainntanace = 0;
      // ESP.restart();
      break;
    }
  }
}

// void checkVersion()
// {

//   SPIFFS.begin();
//   String URL = "http://device.scalebot.gr/api/Garb/GetDeviceVersion/1/1.0";
//   if ((WiFi.status()== WL_CONNECTED))

//     WiFiClient client;
//     HTTPClient http;

//     Serial.print("[HTTP] begin...\n");

//     Serial.println(URL);
//     String url = URL + "/systemcheck";
//     if (http.begin(client, url))
//     { // HTTP
//       http.addHeader("Content-Type", "application/json");
//       StaticJsonDocument<300> js;
//       Serial.println("==========================================================");

//       // Serial.println((DSN));
//       // js["DSN"] ="";// DSN;
//       // js["ADV"] ="";// ADV;

//       String jsonStr;
//       serializeJson(js, jsonStr);F

//       Serial.println(jsonStr);
//       int httpCode = http.POST(jsonStr);
//       jsonStr="";
//       if (httpCode > 0)
//       {

//         // if (Debug_ == 1)
//         //   Serial.printf("[HTTP] POST... code: %d\n", httpCode);

//         if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
//         {
//           String payload = http.getString();
//           payload.replace("https","http");
//           // if (Debug_ == 1)
//           Serial.println(payload);
//           DeserializationError error = deserializeJson(js, payload);
//           if (error)
//           {

//               Serial.println(F("Failed to read file checkVersion, using default configuration"));
//             return;
//           }

//           if (js["RES"] == 0)
//           {

//             int ver = js["ADV"];
//             const char *dl = js["ADU"];

//             Serial.println(dl);
//             Serial.println(ver);
//             Serial.println("==========================================================");

//             if (ADV < ver)
//             {

//               Serial.println(F("Downloading new version ...."));
//               for (int i = 0; i < 20; i++)

//               // ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
//               // ESPhttpUpdate.rebootOnUpdate(true);
//                ESPhttpUpdate.rebootOnUpdate(false);
//                t_httpUpdate_return ret = ESPhttpUpdate.update(client, dl);
//                delay(500);

//                Serial.println(F("\n---- ota update done ----\n"));
//               if (ret == HTTP_UPDATE_OK)
//               {

//                   Serial.println("HTTP_UPDATE_OK");
//                 File f = SPIFFS.open("/Config", "w");
//                 if (serializeJson(js, f) == 0)
//                 {

//                     Serial.println(F("Failed to write to file"));
//                 }
//                 else
//                   Serial.println(F("Success to write to file"));

//                 f.close();

//                 ESP.restart();
//               }
//               switch (ret)
//               {
//               case HTTP_UPDATE_FAILED:

//                   Serial.println("HTTP_UPDATE_FAILD Error");
//                 break;

//               case HTTP_UPDATE_NO_UPDATES:

//                   Serial.println("HTTP_UPDATE_NO_UPDATES");
//                 break;

//               case HTTP_UPDATE_OK:

//                 break;
//               }
//               delay(50);
//             }
//           url="";
//           url="";
//           }
//           payload = "";
//           js.clear();
//         }
//       }
//       else
//       {

//           Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
//       }

//       http.end();
//     }
//     else
//     {

//         Serial.printf("[HTTP} Unable to connect\n");
//     }
//   }
// }
void SetAccelerometer1()
{
  // adxl.powerOn();

  // //set activity/ inactivity thresholds (0-255)
  // adxl.setActivityThreshold(75); //62.5mg per increment
  // adxl.setInactivityThreshold(75); //62.5mg per increment
  // adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  // //look of activity movement on this axes - 1 == on; 0 == off 
  // adxl.setActivityX(1);
  // adxl.setActivityY(1);
  // adxl.setActivityZ(1);
 
  // //look of inactivity movement on this axes - 1 == on; 0 == off
  // adxl.setInactivityX(1);
  // adxl.setInactivityY(1);
  // adxl.setInactivityZ(1);
 
  // //look of tap movement on this axes - 1 == on; 0 == off
  // adxl.setTapDetectionOnX(0);
  // adxl.setTapDetectionOnY(0);
  // adxl.setTapDetectionOnZ(1);
 
  // //set values for what is a tap, and what is a double tap (0-255)
  // adxl.setTapThreshold(50); //62.5mg per increment
  // adxl.setTapDuration(15); //625us per increment
  // adxl.setDoubleTapLatency(80); //1.25ms per increment
  // adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  // //set values for what is considered freefall (0-255)
  // adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  // adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  // //setting all interrupts to take place on int pin 1
  // //I had issues with int pin 2, was unable to reset it
  // adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  // adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  // adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  // adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  // adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  // //register interrupt actions - 1 == on; 0 == off  
  // adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  // adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  // adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  // adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  // adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
}

void SetAccelerometer()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    if (accelgyro.testConnection())
    {
      NewVersion=1;
    }

}

void Read_GPS_LatLon()
{
  
    modem_on();
    if (modem.isGprsConnected())
    {
    getRTC();
    enableGPS(); 
    Serial.println("Start positioning . Make sure to locate outdoors.");
    Serial.println("The blue indicator light flashes to indicate positioning.");
    int tries=0;
    while (1 && tries<350) {
        if (modem.getGPS(&lat, &lon)) {
            Serial.println("The location has been locked, the latitude and longitude are:");
            Serial.print("latitude:"); Serial.println(String(lat,8));
            Serial.print("longitude:"); Serial.println(String(lon,8));
            tries++;
            break;
        }
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(1000);
    }

    disableGPS();
    }
}
long acc_time=0;
void ReadAccelerometer()
{
 
  if (NewVersion==0 ||    I2cbusy==1 || Sen5==0) return;
  I2cbusy=1;
   // read raw accel/gyro measurements from device
 accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 accAngleX = (atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
 accAngleY = (atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
 while (millis() - acc_time >5000 )
  {
    acc_time=millis();
    Serial.println("accAngleY:"   + String (accAngleY)); 
    Serial.println("accAngleX:" + String (accAngleX)); 
    readGaslevel();
  }
  I2cbusy=0;
}


void AirPollution() {
  if (   I2cbusy==1 ||   (Sen7==0) ) return;
  I2cbusy=1;
  error = sen5x.readMeasuredValues(
      massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
      massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
      noxIndex);
      I2cbusy=0;
  if (error) {
      Serial.print("Error trying to execute readMeasuredValues(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } else {
      
      Serial.print("Pm2p5:");
      Serial.print(massConcentrationPm2p5);
       Serial.print("\t");
      Serial.print("Pm10p0:");
      Serial.print(massConcentrationPm10p0);
      Serial.print("\t");
      Serial.print("Humidity:");
      if (isnan(ambientHumidity)) {
          Serial.print("n/a");
      } else {
          Serial.print(ambientHumidity);
      }
      Serial.print("\t");
      Serial.print("Temperature:");
      if (isnan(ambientTemperature)) {
          Serial.print("n/a");
      } else {
          Serial.print(ambientTemperature);
      }
      Serial.print("\t");
      Serial.print("VocIndex:");
      if (isnan(vocIndex)) {
          Serial.print("n/a");
      } else {
          Serial.print(vocIndex);
      }
      Serial.print("\t");
      Serial.print("NoxIndex:");
      if (isnan(noxIndex)) {
          Serial.println("n/a");
      } else {
          Serial.println(noxIndex);
      }
  }
   

}

void readGaslevel()
{
  int val=0;
  val=analogRead(32);//Read Gas value
  Serial.println("Gas level:" + String(val));//Print
//Serial.println(val,DEC);
val=(val*100/4096);
 
//Serial.println("Gas level:" + String(val));//Print
}
int violationTime=0;
void ProceessAccelerometer()
{
  //if (NewVersion==0) 
  return;
  if (abs(accAngleY)<60.0  &&  violationTime==0 &&  binOrientation==Bin_Up )
         { 
           // violationTime=0;  
            binOrientation=Bin_Down;  
            violationTime=millis(); 
            Serial.println("***********  Bin is Down   **************"); 
         } 

    if ( (abs(accAngleY)>60.0) &&  binOrientation!=Bin_Up )
       {
        binOrientation=Bin_Up;
        violationTime=0;
         Serial.println("***********  Bin is Up   **************"); 
       } 

  //***************************** ΑΠΟΚΟΜΙΔΗ IS ON ******************************
  if ( binOrientation==Bin_Down)   
   {
      // UnlockByServo();
      if (Garbagecollection==0)
      {
        Garbagecollection=1;
        tagId="F9A8E904";
        Serial.println("*************** GARBAGE COLLECTION IN PROGRESS *******************"); 
        mustUnlock = 1;
        CloseScaleBin();
        unlock();
        start_time = millis();
        beep(1);
      }
   }
    

  //****************************   VIOLATION DETECT  ****************************

   if (  binOrientation==Bin_Down  &&   (millis()-violationTime) > 120000  &&  violationTime>0 ) 
         {
                  if (!checkSamePlace())
                    prepare_Wifi_Gps_Data(Violation_Tilt);
                  else
                  {
                      StaticJsonDocument<250> doc;
                      doc["b_c"]=DSN;
                      doc["b_s"]=16;
                      doc["dts"]="timestamp";
                      if (lat>0)
                      {
                          doc["lat"]=String(lat,8);
                          doc["lng"]=String(lon,8);
                      }
                      serializeJson(doc, s);
                      Serial.println(s);
                      Datalist.add(s);
                      gr++;
                      storeGr();
                      store_Datalist(gr);
                  } 
                  pinMode(GREEN_PIN, INPUT);
                  Serial.println("**** VIOLATION DETECTED ****");
                  //BEEP 2 TIMES TO INDICATE BIN IS FULL
                  beep(2);
                  transmit_data();
                  storeSettings();
                  done();
                  return;
            }
 }

void ReadAccelerometer1()
 {
//   	//Boring accelerometer stuff   
// 	int x,y,z;  
// 	adxl.readXYZ(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z
		
// 	double xyz[3];
// 	adxl.getAcceleration(xyz);
// 	ax = xyz[0];
// 	ay = xyz[1];
// 	az = xyz[2];
// 	Serial.print("X=");
// 	Serial.println(ax);
//   if ( abs(ax)>0.60 && checkFullBin()) // ΑΠΟΚΟΜΙΔΗ IS ON 
//   {
//    // UnlockByServo();
//     beep(2);
//     Garbagecollection=1;
//     tagId="F9A8E904";
//  }

  //****************************   VIOLATION DETECT  ****************************
   if ( abs(ax)>0.60 &&  !checkFullBin()  ) //
          {
            int tries=0;
            while (  tries<30) {
              if (abs(ax)<0.20 )
               {
                beep(2);
                Garbagecollection=1;
                tagId="F9A8E904";
                break;
               }
              tries++;
            }
            if (abs(ax)>0.60 )
            {
                  if (!checkSamePlace())
                    prepare_Wifi_Gps_Data(Violation_Tilt);
                  pinMode(GREEN_PIN, INPUT);
                  Serial.println("**** VIOLATION DETECTED ****");
                  //BEEP 2 TIMES TO INDICATE BIN IS FULL

                  beep(2);
                  transmit_data();
                  storeSettings();
                  done();
                  return;
            }
          }

  //Serial.println(" g");
	 Serial.print("Y=");
	 Serial.println(ay);
  //Serial.println(" g");
	//Serial.print("Z=");
	//Serial.print(az);
  //Serial.println(" g");
//	Serial.println("**********************");
}
void test()
{
  lockByServo();
  beep(2);
  while (1==1)
  {
    checkFullBin();
    //delay(1000);
    CheckCoverSTatus();
    
  }
  SPIFFS.begin();
  for (int i1 = 0; i1 < 100; i1++) //test
  {
    if (SPIFFS.exists("/Datalist_" + String(i1)))
    {

      {

        SPIFFS.remove("/Datalist_" + String(i1));
        SPIFFS.remove("/LostDatalist_" + String(i1));
        Serial.println("Remove /Datalist_" + String(i1));
      }
    }
  }
  gr = -1;
  storeGr();
  Datalist.clear();
  if (CoverStatus ==Locker_Opened)
  {
    pinMode(LOCKER_PIN, OUTPUT);
    digitalWrite(LOCKER_PIN, HIGH);
    delay(500);
    pinMode(LOCKER_PIN, INPUT);
  }
  // StaticJsonDocument<250> doc;

  //         binstatus = Garbage_collection;

  //         doc["sst"] = binstatus;
  //         doc["scd"] =  "bin_scale";
  //         doc["dts"] = "timestamp";
  //         doc["btr"] = batterylevel;
  //         serializeJson(doc, s);
  //         Serial.println(s);
  //         Datalist.add(s);
  //         gr++;
  //         storeGr();
  //         store_Datalist(gr);
  //         transmit_data();
  // return;
  weightF = 1.1;
  DSN = "bin_scale5";
  tagId = "AA4EF8A5";

  Serial.println("getFreeHeap " + String(ESP.getFreeHeap()));

  weightF = 1.1;
  PutDataBuff();
  weightF = 2.1;

  // PutDataBuff();
  transmit_data();

  checkFullBin();
  //readNFC();
  UnlockByServo();
  delay(2000);
  //readscale();
  pinMode(DONE_PIN, INPUT);
  //done();
  delay(1000);
  pinMode(DONE_PIN, INPUT);
  //done();

  delay(60000);
  return;
}
String printMacAddress(byte mac[6]) {
  // the MAC address of your Wifi shield
  
  // Serial.print(mac[0],HEX);
  // Serial.print(":");
  // Serial.print(mac[1],HEX);
  // Serial.print(":");
  // Serial.print(mac[2],HEX);
  // Serial.print(":");
  // Serial.print(mac[3],HEX);
  // Serial.print(":");
  // Serial.print(mac[4],HEX);
  // Serial.print(":");
  // Serial.print(mac[5],HEX);

  return (String(mac[0],HEX) + ":" + String(mac[1],HEX) + ":" + String(mac[2],HEX) + ":" + String(mac[3],HEX) + ":" + String(mac[4],HEX) + ":" + String(mac[5],HEX));
}
void scanWifi_()
{
  WiFi.mode(WIFI_STA);
  delay(5000);
  int tries = 0;

  while (WiFi.status() != WL_CONNECTED && tries < 10)
  {
    Serial.print('.');
    tries++;
    delay(1000);
  }

  Serial.println("Setup done");
  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0)
  {
    Serial.println("no networks found");
  }
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      
      //Serial.println(WiFi.BSSIDstr(i));
      String BSr;
      BSr=WiFi.BSSIDstr(i);
      BSr.substring(BSr.indexOf(":")-2,BSr.length());
      Serial.println("->" +BSr);     

      delay(10);
    }
  }
  Serial.println("");

  // Wait a bit before scanning again
}



void openWifi()
{
    // Bring up the WiFi connection
    WiFi.mode( WIFI_STA );
    //delay( 5000 );
}
void closeWifi()
{
   WiFi.mode( WIFI_OFF );
   
}

 
void store_Aps()
 {
  SPIFFS.begin();
  File f = SPIFFS.open("/Aps", "w");
  Serial.println("store Aps :" + APs);
  f.println(APs);
  f.close();
  SPIFFS.end();
  Serial.println("store Aps1 :" + APs);
}
void restore_Aps()
 {
  SPIFFS.begin();
  File f = SPIFFS.open("/Aps", "r");
   
   while (f.available())
    {
      APs_= f.readStringUntil('\n');
    }
  Serial.println("restore Aps :" +APs_);
  f.close();
  SPIFFS.end();
}

boolean scanWifi(String s="")
{
  
  APs="";
  openWifi();
  delay(4000);
  String ssid;
  int32_t rssi;
  uint8_t encryptionType;
  uint8_t* bssid;
  int32_t channel;
  bool hidden;
  int scanResult;

  Serial.println(F("Starting WiFi scan..."));

  scanResult = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);

  if (scanResult == 0) {
    Serial.println(F("No networks found"));
  } else if (scanResult > 0) {
    Serial.printf(PSTR("%d networks found:\n"), scanResult);

    // Print unsorted scan results
    for (int8_t i = 0; i < scanResult; i++) {

  
    if (ssid=="BinBot")  
    {
       Mainntanace=1;
      {

            Serial.print("Maintenance is on. Wifi commection");
            WiFi.mode(WIFI_STA);
            //openwifi binbot; download firmware
          // WiFi.begin("Safebot", "s@f3b0t1");
             WiFi.begin("BinBot", "B1^80T");
            int tries = 0;

            delay(5000);
            while (WiFi.status() != WL_CONNECTED && tries < 60)
            {
              Serial.print('.');
              tries++;
              delay(1000);
            }

            if (WiFi.status() == WL_CONNECTED)
            {
              Mainntanace = 0;
              checkVersion();
            }
            else
            {
              mAh = 0;
              storeSettings();
            }

            Mainntanace = 0;
            return false;
          }

    }
    

    if (i >5) break;
      WiFi.getNetworkInfo(i, ssid, encryptionType, rssi, bssid, channel);

      Serial.printf(PSTR("  %02d: [CH %02d] [%02X:%02X:%02X:%02X:%02X:%02X] %ddBm %c  %s\n"),
                    i,
                    channel,
                    bssid[0], bssid[1], bssid[2],
                    bssid[3], bssid[4], bssid[5],
                    rssi,
                    (encryptionType == WIFI_AUTH_OPEN) ? ' ' : '*',
                     
                    ssid.c_str());
            String mac=printMacAddress(bssid) ;                  
            if (APs=="") 
               APs= "{\"ss\":"  + String("\"") +  String(rssi) + "\","  + "\"mac\":" + "\"" + mac  + "\"}"; 
            
            else
            APs=APs + ",{\"ss\":" + String("\"") +  String(rssi) +"\","  + "\"mac\":" + "\"" + mac  +"\"}"; 
            if ( s.indexOf(mac) >0 && s!="")
            {
                Serial.println(" - Same Place stop logging");
                closeWifi();
                sp=0;
               return true;
            }
      delay(0);
    }
   
  } else {
    Serial.printf(PSTR("WiFi scan error %d"), scanResult);
  }
    
    delay(10);
 
    Serial.println("New place logging->" + APs);
    store_Aps();
    closeWifi();
    sp=1;
    return false;
     
}
void prepare_Wifi_Gps_Data(int binstatus)
{
      // if (APs=="")// Not detection of wifi AP => Enable GPS 
       Read_GPS_LatLon(); 
       String s = "";
        
       StaticJsonDocument<250> doc;
       doc["b_c"]=DSN;
       doc["b_s"]=binstatus; 
       doc["dts"]="timestamp";
      //  if (APs!="")
      //  doc["w"]= "[" + APs + "]";
       if (lat>0)
       {
          doc["lat"]=String(lat,8);
          doc["lng"]=String(lon,8);
       }
       serializeJson(doc, s);
       Serial.println(s);
       Datalist.add(s);
       gr++;
       storeGr();
       store_Datalist(gr);
}
 
boolean checkSamePlace()
{
  restore_Aps();
  if (APs_!="" )
  {
       Serial.println(APs_);
      if ( scanWifi(APs_))
       {
          return true;
       }
    
  }
  else
  {
   scanWifi("");
   
  }
  
  
  return false;
}
void printModuleVersions() {
  uint16_t error;
  char errorMessage[256];

  unsigned char productName[32];
  uint8_t productNameSize = 32;

  error = sen5x.getProductName(productName, productNameSize);

  if (error) {
      Serial.print("Error trying to execute getProductName(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } else {
      Serial.print("ProductName:");
      Serial.println((char*)productName);
  }

  uint8_t firmwareMajor;
  uint8_t firmwareMinor;
  bool firmwareDebug;
  uint8_t hardwareMajor;
  uint8_t hardwareMinor;
  uint8_t protocolMajor;
  uint8_t protocolMinor;

  error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                           hardwareMajor, hardwareMinor, protocolMajor,
                           protocolMinor);
  if (error) {
      Serial.print("Error trying to execute getVersion(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } else {
      Serial.print("Firmware: ");
      Serial.print(firmwareMajor);
      Serial.print(".");
      Serial.print(firmwareMinor);
      Serial.print(", ");

      Serial.print("Hardware: ");
      Serial.print(hardwareMajor);
      Serial.print(".");
      Serial.println(hardwareMinor);
  }
}

void printSerialNumber() {
  uint16_t error;
  char errorMessage[256];
  unsigned char serialNumber[32];
  uint8_t serialNumberSize = 32;

  error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
  if (error) {
      Serial.print("Error trying to execute getSerialNumber(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } else {
      Serial.print("SerialNumber:");
      Serial.println((char*)serialNumber);
  }
}
void initSen55()
{
  sen5x.begin(Wire);
  error = sen5x.deviceReset();
  if (error) {
      Serial.print("Error trying to execute deviceReset(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  }
  if (error) {
    Serial.print("Error trying to execute deviceReset(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
}

// Print SEN55 module information if i2c buffers are large enough
#ifdef USE_PRODUCT_INFO
printSerialNumber();
printModuleVersions();
#endif

float tempOffset = 0.0;
error = sen5x.setTemperatureOffsetSimple(tempOffset);
if (error) {
    Serial.print("Error trying to execute setTemperatureOffsetSimple(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
} else {
    Serial.print("Temperature Offset set to ");
    Serial.print(tempOffset);
    Serial.println(" deg. Celsius (SEN54/SEN55 only");
}

// Start Measurement
error = sen5x.startMeasurement();
if (error) {
    Serial.print("Error trying to execute startMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
}
}
 
void CheckDevices()
{
Serial.println("******* CheckDevices  **********");
 if ( ScanAddress(0x29)) {Serial.println("I2C Cover Sensor OK");Sen1=1;} else Serial.println("I2C Cover Sensor NOT OK");
 if ( ScanAddress(0x40)) {Serial.println("I2C Current Sensor OK");Sen2=1;} else Serial.println("I2C Current Sensor NOT OK");
 if ( ScanAddress(0x52)) {Serial.println("I2C Left Sensor OK"); Sen3=3;} else Serial.println("I2C Left Sensor NOT OK");
 if ( ScanAddress(0x54)) {Serial.println("I2C Right Sensor OK"); Sen4=1;} else Serial.println("I2C Right Sensor NOT OK");
 if ( ScanAddress(0x68)) {Serial.println("I2C MCU Sensor OK");Sen5=1;} else Serial.println("I2C MCU Sensor NOT OK");
 if ( nfc.begin()) {Serial.println("I2C NFC Reader OK");Sen6=1;}  else Serial.println("I2C NFC Reader NOT OK");
 if ( ScanAddress(0x69)) {Serial.println("Air Quality Sensor OK"); Sen7=1;}  else Serial.println("Air Quality Sensor NOT OK");
 
 Serial.println("********************************");
}

void setup(void)
{
  Wire.begin(); 
  //******* Open Scale to calibrate  **********
  
  Serial1.begin(9600, SERIAL_8N1, RXSCALE_PIN, TXSCALE_PIN, TXSCALE_PIN); //Scale port
  Serial.begin(115200);
  openScaleBin();
  closeWifi();

 // openWifi();
   
  //********************************************
  
  Working_time = millis();
  pinMode(DONE_PIN, OUTPUT);
  digitalWrite(DONE_PIN, LOW);
  pinMode(STATUSSCALE_PIN, INPUT); //read BIN  cover status
  // pinMode(SENSOR1, INPUT);
  // pinMode(SENSOR2, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  myservo.attach(SERVO_PIN); 
  initSen55(); 
  
  Cover_sensor.init();
  Cover_sensor.configureDefault();
  Cover_sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  Cover_sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  Cover_sensor.setTimeout(500);
  Cover_sensor.stopContinuous();
  delay(300);
  // start interleaved continuous mode with period of 100 ms
  Cover_sensor.startInterleavedContinuous(100);
   
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
   
  }
  Serial.println("Measuring voltage and current with INA219 ...");
   
  SetAccelerometer();
  getSettings();
  CalculateCoverRange();
  CalculateCoverRange();
  CalculateCoverRange();
  readGaslevel();
  ReadAccelerometer();
  CheckDevices();
  AirPollution();

 
  xTaskCreatePinnedToCore(
      blinkGreen, /* Task function. */
      "Task1",    /* name of task. */
      10000,      /* Stack size of task */
      NULL,       /* parameter of the task */
      1,          /* priority of the task */
      &Task1,     /* Task handle to keep track of created task */
      0);         /* pin task to core 0 */
 
  Serial.println("NFC Reader Start reading...");

  if (Sen6==1) 
  while (millis() - start_time < 30000 && tagId == "" && Garbagecollection==0)
  {
   //if ( CoverStatus!=Locker_Closed)
   // CalculateCoverRange();
    readNFC();
   // ReadAccelerometer();
   //ProceessAccelerometer();
  }
  restore_Tags();
  if (!readTaglist())
  {
    store_Tag(MaintenanceTag, "F2516E1B");
    store_Tag(MaintenanceTag, "D292FF1B");
    store_Tag(MaintenanceTag, "911E3B1B");
    store_Tag(MaintenanceTag, "5240A149");
    store_Tag(Garbage_CollectorTag, "F9A8E904");
    store_Tag(Garbage_CollectorTag, "43FB8BA3");
    store_Tag(Garbage_CollectorTag, "33B186A3");
    store_Tag(Garbage_CollectorTag, "C274A11B");
    store_Tag(Garbage_CollectorTag, "914C541B");
    store_Tag(Garbage_CollectorTag, "CDA27F51");
    store_Tag(Garbage_CollectorTag, "D2EE1D1B");
  }

  //CheckNFC();
  checkFormat();

  //  File f = SPIFFS.open("/Serial", "r");          
  //  if (!f.available())
  //  initialize_Scale("bin_scale_test");
  //  f.close();
  
  getSerial();
  
  if (chgTimes != chgTimesNew)
  {
    Serial.println("chgTimes New ");
    mAh = 0;
    chgTimes = chgTimesNew;
    storeSettings();
  }
  getGr();
  //test**********************************
  checkFullBin();
 
  
  getSettings();
  

  // Serial.println("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
     Serial.println("**  DEDICATED TO THE MEMORY OF MY FATHER PRODROMO AND MY MOTHER NIKI**");
     Serial.println("********                     REST IN PEACE                      ******");
  // Serial.println("*******   ΜΗΤΕΡΑ ΣΕ ΕΥΧΑΡΙΣΤΩ ΠΟΛΥ ΠΟΥ ΜΕ ΜΕΓΑΛΩΣΕΣ ΓΕΡΟ ΚΑΙ ΔΥΝΑΤΟ  ********");
  // Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
     Serial.println("****************   STARTING   BINBOT SYSTEM  ver 6.0    ***************");
  
     
    
    
   if (mustUnlock == 0)
      CheckNFC();
  if (mustUnlock == 1  )  unlock();
      // Set-up modem  power pin
       Serial.println("\nStarting Up Modem...");
       pinMode(LED_PIN, OUTPUT);
       digitalWrite(LED_PIN, HIGH);
       Working_time_connecting = millis();
       modemPowerOn(); 
   
 }
boolean readTaglist()
{
  for (int i = 0; i < Taglist.size(); i++)
  {
    if (Taglist.get(i)->tagId == "F2516E1B")
      return true;
  }
  return false;
}
int counter;
void ScaleCal()
{
      if (wa >= 3 and wa<20) 
      {
        wa=20;
        
      //  Serial.println("weightA size" + String(WeightList.size()));
        for (int i = 0; i < WeightList.size(); i++)
        {
          weightA = weightA + WeightList.get(i);
        }
        {
          weightA = weightA / WeightList.size();
          WeightList.clear();
          Serial.println("Weight Offset --> " + String(weightA));
        }
      }


        
}

void loop()
{
   ScaleCal();
   
  if (stopWorking == 1)
    return;
  if (mustUnlock == 1)
    {
      unlock(); 
    }

  ProceessAccelerometer();

  if (Lockstatus == 1 && lc == 0 && System_Tag == 0)
    lc = 1;
  // FIRST READ NFC -
  if ( mustUnlock == 0 &&  (tagId1!=tagId))
  {
    readNFC();
    CheckNFC();
  }

  if (digitalRead(STATUSSCALE_PIN) == Locker_Opened)
      lock_time = 0;
  //δεν ανιχνευτηκε κάποιο Tag
  if (millis() - start_time > 20000 && tagId == "")
  {
    CloseScaleBin();
    Working_time = millis() - Working_time;
    Serial.println("SYSTEM POWER OFF DUE NO TAG DETECTION");
    if (Lockstatus == 1 )
      lockByServo();
    beep(2);
    done();
  }

  
  //δεν ανοιξε ο κάδος λόγω προβλήματος ή  άλλου θέματος ή autostart
  if (millis() - start_time > 15000 && System_Tag == 0)
    if (CoverStatus==Locker_Closed  && lc == 0 && tagId != "")
    {
      CloseScaleBin();
      Working_time = millis() - Working_time;
      Serial.println("SYSTEM POWER OFF BECAUSE COVER NOT OPENED");
      lockByServo();
       
      mustUnlock = 0;
      TagPresent=0;
      if (tagId != "")
      {
        String s = "";
        StaticJsonDocument<250> doc;
        binstatus = ProblemCoverNotOpening;
        doc["sst"] = binstatus;
        doc["scd"] = DSN;
        doc["dts"] = "timestamp";
        doc["btr"] = batterylevel;
        serializeJson(doc, s);
       // modemPowerOn();
        Serial.println(s);
        Datalist.add(s);
        gr++;
        storeGr();
        store_Datalist(gr);
        transmit_data();
        beep(1);
        done();
      }
    }

  if (Lockstatus == 0 && CoverStatus==Locker_Closed )
    AlarmOFF();

  //**************************************************************
  if (System_Tag == 1)
    return;
  if (Mainntanace == 1)
  {

    Serial.print("Maintenance is on. Wifi commection");
    WiFi.mode(WIFI_STA);
    //openwifi binbot; download firmware
    WiFi.begin("Safebot", "s@f3b0t1");
    int tries = 0;

    delay(5000);
    while (WiFi.status() != WL_CONNECTED && tries < 60)
    {
      Serial.print('.');
      tries++;
      delay(1000);
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Mainntanace = 0;
      checkVersion();
    }
    else
    {
      mAh = 0;
      storeSettings();
    }

    Mainntanace = 0;
    return;
  }

  //δεν έκλεισε ο Garbagecollector τον κάδο πάνω απο 120 δευτερόλεπτα
  if (millis() - start_time > 120000 && start_time>0)
    if (digitalRead(STATUSSCALE_PIN) == Locker_Opened && (Mainntanace == 1 || Garbagecollection == 1) && tagId != "") //&& lc == 1
    {
      Serial.println("Cover is opened too long");
      // if (millis() - start_time > 50000 && millis() - start_time < 60000)
      TagPresent=0;
      if (millis() - start_time > 40000)
      {
        //beepDelay(1000);
        // i = 0;
        // if (tagId != "")
        // while (we == 0 && i < 30)
        //   {

        //     readscale();
        //     if (we == 1)
        //       break; //change
        //     i++;
        //   }
        StaticJsonDocument<250> doc;
        binstatus = ProblemCoverIsOpentoLong;
        doc["sst"] = binstatus;
        doc["scd"] = DSN;
        doc["dts"] = "timestamp";
        doc["btr"] = batterylevel;
        serializeJson(doc, s);
        Serial.println(s);
        Datalist.add(s);
        
        if (Garbagecollection == 1)
        binstatus = Garbage_collection;
      if (Mainntanace == 1)
        binstatus = Maintenance;
        doc["sst"] = binstatus;
        doc["scd"] = DSN;
        doc["dts"] = "timestamp";
        doc["btr"] = batterylevel;
        serializeJson(doc, s);
        Serial.println(s);
        Datalist.add(s);
        gr++;
        storeGr();
        store_Datalist(gr);
        enablePost = 1;
        transmit_data();
        working_period = 0;
        //start_time = millis();
        if (CoverStatus==Locker_Closed  )
          lockByServo();
        Serial.println("SYSTEM POWER OFF");
        beep(2);

        done();
        done();
        ESP.restart();
      }
    }

  //*******************  O ΚΑΔΟΣ ΕΚΛΕΙΣΕ ΚΑΙ ΠΡΕΠΕΙ ΝΑ ΣΤΕΙΛΕΙ ΔΕΔΟΜΕΝΑ ********************
     if (CoverStatus==Locker_Closed  && lock_time==0)
      { 
        lock_time = millis();
        if (Garbagecollection == 1)
        lock_time = millis()-10000;
      }
 
  if ((CoverStatus==Locker_Closed  && lc == 1 && (millis() - lock_time > 2000 &&  lock_time>0)   )  
      || (digitalRead(STATUSSCALE_PIN) == Locker_Opened &&  (millis() - start_time)>15000  && Garbagecollection == 0 &&
       Mainntanace == 0))
   {
    
   // lock_time = millis();    
        
    TagPresent=0;
    if (lc == 1)
    {
      if (Garbagecollection == 1)
        Serial.println("H αποκομιδή τελείωσε και το καπάκι έκλεισε ");
      else
        Serial.println("Ο δημότης πεταξε τα σκουπίδια μέσα και το σύστημα πρέπει να τα ζυγίσει και να τα στείλει ");
      lc = 0;
    }
    if ( CoverStatus==Locker_Closed )
      lockByServo();

    //******************* Ο δημότης πεταξε τα σκουπιδια μέσα και το σύστημα πρέπει να τα ζυγίσει και να τα στείλει *************
    if (Garbagecollection == 0)
    // if (step == 1)
    {
      //  if (Lockstatus==1) lc=1;

      //********  ΖΥΓΙΣΗ ΣΚΟΥΠΙΔΙΩΝ  **********
      {
        i = 0;
        while (we == 0 && i < 30)
        {

          readscale();
          if (we == 1)
            break; //change
          i++;
        }
      }
    }
    
    CloseScaleBin();
    if ( CoverStatus==Locker_Closed )  lockByServo();
    if (Garbagecollection == 1 || Mainntanace == 1)
    {
      StaticJsonDocument<250> doc;
      if (Garbagecollection == 1)
        binstatus = Garbage_collection;
      if (Mainntanace == 1)
        binstatus = Maintenance;
      doc["sst"] = binstatus;
      doc["scd"] = DSN;
      doc["dts"] = "timestamp";
      doc["btr"] = batterylevel;
      serializeJson(doc, s);
      Serial.println(s);
      Datalist.add(s);
      gr++;
      storeGr();
      store_Datalist(gr);
      enablePost = 1;
      Garbagecollection = 0;
    }
    //*******************************************************************************************************************************'
    if ( CoverStatus==Locker_Closed )
    lockByServo();
 
    transmit_data();

    if (millis() - start_time > 45000 && start_time>0)
    if (digitalRead(STATUSSCALE_PIN) == Locker_Opened && Mainntanace == 0 && Garbagecollection == 0 && tagId != "") //&& lc == 1
    {
      Serial.println("Cover is opened too long");
      // if (millis() - start_time > 50000 && millis() - start_time < 60000)
      TagPresent=0;
      StaticJsonDocument<250> doc;
      binstatus = ProblemCoverIsOpentoLong;
      doc["sst"] = binstatus;
      doc["scd"] = DSN;
      doc["dts"] = "timestamp";
      doc["btr"] = batterylevel;
      serializeJson(doc, s);
     // modem_on(); 
      if (dt!="")
      s.replace("timestamp", dt);
      Serial.println(s);
      Datalist.add(s);
      gr++;
      storeGr();
      store_Datalist(gr);
      transmit_data();
    }
    working_period = 0;
    if ( CoverStatus==Locker_Closed )
    lockByServo();
    beep(1);
    done();
    done();
    ESP.restart();
  }

  //step=3;
}

