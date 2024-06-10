//#include <ADXL345.h>
//I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
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

AES aes;

// Our AES key. Same in NodeJS but in hex bytes
byte key[] = {0x53, 0x40, 0x66, 0x65, 0x62, 0x30, 0x74, 0x54, 0x68, 0x65, 0x33, 0x21, 0x21, 0x21, 0x21, 0x40};
//ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
TaskHandle_t Task1;
TaskHandle_t Task2;
//double ax,ay,az;
String str, CCID;
int i;
int we;
int lc = 0;
int gr = -1;
int System_Tag = 0;
byte stopWorking = 0;
long open_cover_time = millis();
long start_time = millis();
float Working_time = millis();
float Working_time_connecting = 0;
float Working_time_posting = 0;
float mAh = 0.0;
float lat=0.0,  lon=0.0;
int chgTimes = 0;
int chgTimesNew = 2;
int p, DAY1, MONTH_, YEAR_, YEAR1, Seconds_, Hours_, Minutes_, hourOfDay, DAY_OF_WEEK, hasModemSIM, contentLength;
boolean GPRS_on;
bool reply = false;
//int ADXL345 = 0x53;
float X_out, Y_out, Z_out;  // Outputs
enum _LockerStatus
{
  Locker_Opened,
  Locker_Closed

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
#define SENSOR1 32         //INPUT
#define SENSOR2 35         //INPUT
//INPUT I2C
#define SDA_PIN 21
#define SCL_PIN 22
#define ADC_PIN 36

#define BUZZER_CHANNEL 0
Servo myservo; // create servo object to control a servo

LinkedList<String> Datalist = LinkedList<String>();
LinkedList<String> LostDatalist = LinkedList<String>();
LinkedList<int> LostData = LinkedList<int>();
LinkedList<double> WeightList = LinkedList<double>();
LinkedList<Tag *> Taglist = LinkedList<Tag *>();
PN532_I2C pn532_i2c(Wire);
NfcAdapter nfc = NfcAdapter(pn532_i2c);
String tagId = "";
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
int a = 0;
int scaleIsConnected = 0;
int step, LockStatus = 0;
byte scale_status = 0;
long delayTime;
boolean hasServo = true;
double weight, weightF, weightA;
long working_period = millis();
long nfc_period = millis();
float batterylevel, voltage;
String APs="",APs_="";
int mVperAmp = 270;           // this the 5A version of the ACS712 -use 100 for 20A Module and 66 for 30A Module
double Watt = 0;
double TotalWatt = 0;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

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
  Serial.println("locking");
  delay(1000);
  if (digitalRead(STATUSSCALE_PIN) == Locker_Closed)
    myservo.write(100);

  // for (pos1 = 0; pos1 <= 180; pos1 += 1) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   myservo.write(pos1);              // tell servo to go to position in variable 'pos'
  //   delay(15);                       // waits 15ms for the servo to reach the position
  // }
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
  Serial.println("Unlocking");
  myservo.write(0);
  if (ultime == 0)
    beepDelay(500);
  ultime++;
  if (ultime == 100000) //test
  {
    String s = "";
    StaticJsonDocument<250> doc;
    binstatus = ProblemCoverNotOpening;
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
    //transmit_data();
    // working_period = 0;
    //done();
  }
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
void check_battery()
{
  Voltage = getVPP();
  VRMS = (Voltage/2.000) *0.707;   //root 2 is 0.707
  AmpsRMS = ((VRMS * 1000)/mVperAmp); //0.3 is the error I got for my sensor
 
  Serial.print(AmpsRMS);
  Serial.print(" Amps RMS  ---  ");
  Watt = (AmpsRMS*5.000/1.200);
  // note: 1.2 is my own empirically established calibration factor
// as the voltage measured at D34 depends on the length of the OUT-to-D34 wire
// 240 is the main AC power voltage – this parameter changes locally
  Serial.print(Watt);
  Serial.print(" Watts");
  TotalWatt=TotalWatt+Watt*1000/3600;
  Serial.print("---Total mWh:" + String(TotalWatt));
  double bat=(30*5*1000-TotalWatt)*100/(30*5*1000);
  Serial.println("---Battery% :" + String(bat));



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
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);
  digitalWrite(PWR_PIN, HIGH);
}

void modemPowerOff()
{

  modem.gprsDisconnect();
  SerialMon.println(F("GPRS disconnected"));
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1500); //Datasheet Ton mintues = 1.2S
  digitalWrite(PWR_PIN, HIGH);
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
void modem_on()
{
  // Set-up modem  power pin
  Serial.println("\nStarting Up Modem...");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
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

  for (int i = 0; i <= 4; i++)
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
    bool isConnected = false;
    int tryCount = 60;
    while (tryCount--)
    {
      int16_t signal = modem.getSignalQuality();
      Serial.print("Signal: ");
      Serial.print(signal);
      Serial.print(" ");
      Serial.print("isNetworkConnected: ");
      isConnected = modem.isNetworkConnected();
      Serial.println(isConnected ? "CONNECT" : "NO CONNECT");
      if (isConnected)
      {
        break;
      }
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
  Serial.println("Device is connected .");
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
    check_battery();
    if (digitalRead(STATUSSCALE_PIN) == Locker_Closed && mustUnlock == 0)
    {
      if (we == 1)
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
    }
  }
}

boolean postlog(String payload)

{
  boolean ret = false;
  //GPRS_close();

  // if (DTC == 1)
  // Checkinternet();

  //
  //   Serial.println("getFreeHeap " + String(ESP.getFreeHeap()));
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
  Serial.println(("OpenScale " + String(digitalRead(STATUSSCALE_PIN))));
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
  String url1="http:/3v.device.informula.gr/api/Garb/DeviceUpdateBinInfo";

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
        url = "http://3v.device.informula.gr/api/Garb/DeviceUpdateBinInfo";
      else
        url = "http://device.scalebot.gr/api/Garb/DeviceSendBinInfo";

      payload.replace("timestamp", dt);
      // payload.replace("5240A149","code_mix1");//test

      Serial.println(url);
      payload.replace("\"[","[");
      payload.replace("]\"","]");
      payload.replace("\\","");
      payload.replace("Bin_scale_V3","Bin_scale_Ver3");
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
  if (WeightList.size() >= 100)
  {
    Serial.println("weight size" + String(WeightList.size()));
    for (int i = 49; i < 100; i++)
    {
      // Serial.println("weight list" + String(i));
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

String readscale()
{
  String str;
  scaleIsConnected = 0;
  delay(100);
  while (Serial1.available())
  {
    scaleIsConnected = 1;
    str += Serial1.readStringUntil('\n');
    str.trim();
    // Serial.println("Weight:" + str); //Forward what Software Serial received to Serial Port
    // ParseSerialData(str,";");
    str.replace("kg", "");
    str.replace("+", "");
    str.replace("wn", "");
    weight = str.toDouble();
    WeightList.add(weight);
    if (WeightList.size() > 100)
      WeightList.remove(0);
    Serial.println("Weight:" + str);
  }
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
    if (weightA == weightA)
      weightF = abs(abs(weight) - abs(weightA));
    else
      weightF = abs(weight);
    if (we == 0 && weightF > 0.3 && weight > 0.3)
    {
      we = 1;
      PutDataBuff();
      //beep(1);
      beepDelay(200);
    }

    Serial.println("Weight true:" + String(weightF));
    open_cover_time = millis();
  }

  return str;
}

void readscale1()
{
  String str;
  scaleIsConnected = 0;
  delay(100);
  while (Serial1.available())
  {

    scaleIsConnected = 1;
    str += Serial1.readStringUntil('\n');
    str.trim();
    // Serial.println("Weight:" + str); //Forward what Software Serial received to Serial Port
    // ParseSerialData(str,";");
    str.replace("kg", "");
    str.replace("+", "");
    str.replace("wn", "");
    weight = str.toDouble();
    if (weight == weight)
    {
      WeightList.add(weight);
      a++;
    }
    if (WeightList.size() > 10)
      WeightList.remove(0);
    Serial.println("WeightA:" + str);
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
      mAh = doc["MAH"];
      Serial.println("mAh : " + String(mAh));
      chgTimes = doc["CHG"];
      Serial.println("chgTimes : " + String(chgTimes));
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
    js["MAH"] = mAh;
    js["CHG"] = chgTimes;

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

boolean checkFullBin()
{
  Serial.println("**** CheckFullBin ****");
  if (digitalRead(SENSOR1) == 0)
    Serial.println("SENSOR1 ΑΔΕΙΟΣ");
  else
    Serial.println("SENSOR1 ΓΕΜΑΤΟΣ");

  if (digitalRead(SENSOR2) == 0)
    Serial.println("SENSOR2 ΑΔΕΙΟΣ");
  else
    Serial.println("SENSOR2 ΓΕΜΑΤΟΣ");

  if (digitalRead(SENSOR1) == 1 && digitalRead(SENSOR2) == 1)
    return true;
  else
    return false;
}

void unlock()
{

  if (digitalRead(STATUSSCALE_PIN) == Locker_Closed) //&& LockStatus ==0
  {
    // for (int i = 0; i < 90; i++)
    // {
    //   delay(100);
    //   if ((millis() - start_time) > 7000)
    //   break;
    // }

    if (hasServo == false)
    {
      //trigger locker to unlock
      pinMode(LOCKER_PIN, OUTPUT);
      digitalWrite(LOCKER_PIN, HIGH);

      scaleIsConnected = 0;
      ultime++;
      if (ultime < 10)
        beep(1);
      else
      {
        delay(1000);
      }
      pinMode(LOCKER_PIN, INPUT);
      //delay(1000);

      //readscale();
      if (ultime == 30) //test
      {
        StaticJsonDocument<250> doc;
        binstatus = ProblemCoverNotOpening;
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
        done();
      }
    }
    else
      UnlockByServo();

    //if (eTaskGetState(&Task1)==2)
  }
  //bin is opened
  if (digitalRead(STATUSSCALE_PIN) == Locker_Opened)
  {

    pinMode(LOCKER_PIN, INPUT);
    if (ultime == 0)
      beepDelay(250);
    ultime = 0;
    Serial.println("Bin is opened by the citizen");
    //if (scaleIsConnected == 0 && Garbagecollection == 0) // open scale if citizen is here
    // openScaleBin();
    //  scale_status = 1;
    LockStatus = 1;
    mustUnlock = 0;
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
        Serial.println("Tag Αποκομιδής άδειασμα κάδου");
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
    if (digitalRead(STATUSSCALE_PIN) == Locker_Opened)
      beepDelay(250);
    if (LockStatus == 1) //checkTags()==false && //TEST
      step = 1;
    we = 0;
    //if (LockStatus == 0)
  }
  else
  {
  }
  //delay(50);
}

void CheckNFC()
{
  // Serial.println("Nfc read" );
  if (tagId != "")
  {

    {
      if (checkTags())
        if (mustUnlock == 0)
        {
          if ((Garbagecollection == 0 && Mainntanace == 0)) // !checkFullBin()  && // && digitalRead(STATUSSCALE_PIN) == Locker_Closed
            mustUnlock = 1;
          // beep(1);
          String covr;
          if (digitalRead(STATUSSCALE_PIN) == Locker_Opened)
            covr = " Opened";
          else
            covr = " Closed";
          Serial.println("BIN is:" + covr);
          step = 0;
          if (Garbagecollection == 1 || Mainntanace == 1) // unlock without open scale for gargage collection
          {
            //if (Garbagecollection == 1)
            mustUnlock = 1;
            CloseScaleBin();
            Green();
            unlock();
            start_time = millis();
            beep(2);
            return;
          }

          if (digitalRead(STATUSSCALE_PIN) == Locker_Closed && (Garbagecollection == 0 && Mainntanace == 0))
          {
            //Check if bin is full
            if (checkFullBin() && BinFull == 3) //disabled
            {
              BinFull = 1;
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

              // if (Garbagecollection != 1 && Mainntanace != 1)
              // {
              //   enablePost=1;
              //   transmit_data();
              //   done();
              // }
              return;
            }
            if (!checkFullBin())
            {
              BinFull = 0;
              storeSettings();
            }

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
  if (gr >= 0) // && enablePost==1
  {
    Serial.println("BinBot has DATA thus began the trasmitting procedure");
    //*****************************   Κλείσε την ζυγαρία  *************************************
    CloseScaleBin();

    //***************  Immediately open modem to establish internet connection  ****************
    // modemPowerOn();
    //*******************************************************************************************
    // SerialAT.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX); //Modem port

    modem_on();
    getRTC();
    int r;
    File f;
    Working_time_posting = millis();
    delay(5000);
    SPIFFS.begin();

    for (int i1 = 0; i1 < 50; i1++)
    {
      if (SPIFFS.exists("/Datalist_" + String(i1)))
      {
        f = SPIFFS.open("/Datalist_" + String(i1), "r");
        Serial.println("/Datalist_" + String(i1));
        if (f)
        {

           restore_Datalist(i1);;
           //restore_LostDatalist(i1);
          if (Datalist.size() > 0)
            secure_postlog_gprs();
          while (LostDatalist.size() != Datalist.size() && r < 5)
          {
            restore_Datalist(i1);
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
    gr = -1;
    storeGr();
    modemPowerOff();

    Working_time_posting = millis() - Working_time_posting;
    SPIFFS.end();
    delay(1000);
    done();
  }

  done();
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
      beepDelay(1000);
      if (digitalRead(STATUSSCALE_PIN) == Locker_Closed)
        lockByServo();
      Mainntanace = 0;
      ESP.restart();
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

}

void Read_GPS_LatLon()
{
  
    modem_on();
    getRTC();
    enableGPS(); 
    Serial.println("Start positioning . Make sure to locate outdoors.");
    Serial.println("The blue indicator light flashes to indicate positioning.");
    int tries=0;
    while (1 && tries<150) {
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
void ReadAccelerometer()
{
   // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
   float roll, pitch, yaw;
    accAngleX = (atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    accAngleY = (atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
    Serial.println("accAngleY:"   + String (accAngleY)); 
    Serial.println("accAngleX:" + String (accAngleX)); 

    //***************************** ΑΠΟΚΟΜΙΔΗ IS ON ******************************
    if ( (abs(accAngleX)>60.0 || abs(accAngleY)>60.0) && checkFullBin())   
      {
      // UnlockByServo();
        beep(2);
        Garbagecollection=1;
        tagId="F9A8E904";
        Serial.println("ΑΠΟΚΟΜΙΔΗ IS ON"); 
      }

  //****************************   VIOLATION DETECT  ****************************
   if ( (abs(accAngleX)>60.0 || abs(accAngleY)>60.0)  &&  !checkFullBin()  ) //
          {
            int tries=0;
            while ( tries<80) {
              // read raw accel/gyro measurements from device
                accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
                accAngleX = (atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
                accAngleY = (atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
                Serial.println("accAngleY:"   + String (accAngleY)); 
                Serial.println("accAngleX:" + String (accAngleX)); 
              if ( (abs(accAngleX)<20.0 && abs(accAngleY)<20.0))
               {
                 return;
               }
               delay(1000);
              tries++;
            }
            if ((abs(accAngleX)>60.0 || abs(accAngleY)>60.0))
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
  if (digitalRead(STATUSSCALE_PIN) == Locker_Opened)
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

  Serial.println(("SENSOR1 " + String(digitalRead(SENSOR1))));

  Serial.println(("SENSOR2 " + String(digitalRead(SENSOR2))));
  Serial.println(("OpenScale " + String(digitalRead(STATUSSCALE_PIN))));
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
  if (APs_!="")
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
void setup(void)
{
  //******* Open Scale to calibrate  **********
  Serial1.begin(9600, SERIAL_8N1, RXSCALE_PIN, TXSCALE_PIN, TXSCALE_PIN); //Scale port
  openScaleBin();
  openWifi();
   
  //********************************************
  Working_time = millis();
  pinMode(DONE_PIN, OUTPUT);
  digitalWrite(DONE_PIN, LOW);
  pinMode(STATUSSCALE_PIN, INPUT); //read BIN  cover status
  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  myservo.attach(SERVO_PIN); // attaches the servo on pin 18 to the servo object
                             // using default min/max of 1000us and 2000us
                             // different servos may require different min/max settings
                             // for an accurate 0 to 180 sweep
  Serial.begin(115200);
  SetAccelerometer();
  xTaskCreatePinnedToCore(
      blinkGreen, /* Task function. */
      "Task1",    /* name of task. */
      10000,      /* Stack size of task */
      NULL,       /* parameter of the task */
      1,          /* priority of the task */
      &Task1,     /* Task handle to keep track of created task */
      0);         /* pin task to core 0 */

  //calibrate Scale
 // if (digitalRead(STATUSSCALE_PIN) == Locker_Closed)
    {
      while (millis() - start_time > 3500 && a < 10)
      {
        readscale1();
      }
      {
        Serial.println("weightA size" + String(WeightList.size()));
        for (int i = 0; i < WeightList.size(); i++)
        {
          weightA = weightA + WeightList.get(i);
        }
        {
          weightA = weightA / WeightList.size();
          WeightList.clear();
          Serial.println("weight Initialized -->" + String(weightA));
        }
      }
    }
  
  //ΑΜΕΣΟ ΞΕΚΛΕΙΔΩΜΑ ΚΑΙ ΕΛΕΓΧΟΣ ΚΑΡΤΑΣ ΜΕΤΑ  
  unlock(); 
  nfc.begin();
  while (millis() - start_time < 20000 && tagId == "")
  {
    Serial.println("NFC Reader Start reading...");
    //if ( digitalRead(STATUSSCALE_PIN) == Locker_Closed)
    readNFC();
    ReadAccelerometer();
    if (!nfc.begin())
    {
      //nfc = NfcAdapter(pn532_i2c);

      Serial.println("Nfc restart");
      NFC_RST();
      delay(100);
      nfc.begin();
      nfc_period = millis() - 14000;
    }
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

  CheckNFC();
 
  checkFormat();
  // File f = SPIFFS.open("/Serial", "r");
  // if (!f.available())

  //initialize_Scale("bin_scale_test");
  // f.close();
  getSerial();
  getSettings();
  if (chgTimes != chgTimesNew)
  {
    Serial.println("chgTimes New ");
    mAh = 0;
    chgTimes = chgTimesNew;
    storeSettings();
  }
  getGr();
  //test**********************************
 
  //========================================
  if (!checkFullBin())
  {
    BinFull = 0;
    storeSettings();
  }
  else
  {
    // CloseScaleBin();
  }

  
  //*******************ΕΛΕΓΧΟΣ ΓΙΑ ΠΑΡΑΒΙΑΣΗ ********************
  // if (LockStatus ==0 && digitalRead(STATUSSCALE_PIN) == Locker_Opened )
  // for (int i = 0; i <1; i++)
  //   {
  //     AlarmOn();
  //     delay(1000);
  //     return;
  //   }

  if (digitalRead(STATUSSCALE_PIN) == Locker_Opened)
    Serial.println("Cover is opened");
  else
    Serial.println("Cover is closed");

  //UnlockByServo();
 
  a = 0;

  //start_time=millis();
  //test();
  //Serial.println(("SENSOR1 " + String(digitalRead(SENSOR1))));
  //Serial.println(("SENSOR2 " + String(digitalRead(SENSOR2))));

  getSettings();
  Serial.println("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  Serial.println("**  ΑΦΙΕΡΩΜΕΝΟ ΣΤΗΝ ΜΝΗΜΗ ΤΟΥ ΠΑΤΕΡΑ ΜΟΥ  ΠΡΟΔΡΟΜΟΥ & ΤΗΣ ΜΗΤΕΡΑ ΜΟΥ ΝΙΚΗΣ **");
  Serial.println("********  ΠΑΤΕΡΑ ΣΕ ΕΥΧΑΡΙΣΤΩ ΠΟΛΥ ΓΙΑ ΟΤΙ ΕΧΩ ΚΑΤΑΦΕΡΕΙ ΧΑΡΗ ΣΕ ΕΣΕΝΑ ******");
  Serial.println("*******   ΜΗΤΕΡΑ ΣΕ ΕΥΧΑΡΙΣΤΩ ΠΟΛΥ ΠΟΥ ΜΕ ΜΕΓΑΛΩΣΕΣ ΓΕΡΟ ΚΑΙ ΔΥΝΑΤΟ  ********");
  Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  Serial.println("****************     STARTING   BINBOT SYSTEM  ver 5.0     ***************");
  if (!checkSamePlace())
    {
       prepare_Wifi_Gps_Data(NewLocationDetected);

    }
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

void loop()
{
  if (stopWorking == 1)
    return;
  ReadAccelerometer();
  if (mustUnlock == 1 && System_Tag == 0)
  {
    unlock();
  }
  if (LockStatus == 1 && lc == 0 && System_Tag == 0)
    lc = 1;
  // FIRST READ NFC -
  if (LockStatus == 0)
  {
    readNFC();
    CheckNFC();
  }
  //δεν άνοιξε καθόλου ο κάδος
  if ( tagId == "")
  if (millis() - start_time > 30000 && digitalRead(STATUSSCALE_PIN) == Locker_Closed && lc == 0 )
  {
    Working_time = millis() - Working_time;
    Serial.println("SYSTEM POWER OFF DUE NO TAG DETECTION");
    if (digitalRead(STATUSSCALE_PIN) == Locker_Opened)
      lockByServo();
      transmit_data();   
      beep(2);
    done();
  }

  //δεν ανοιξε ο κάδος λόγω προβλήματος ή  άλλου θέματος  
  if (millis() - start_time > 35000 && System_Tag == 0)
  if (digitalRead(STATUSSCALE_PIN) == Locker_Closed && lc == 0 && tagId != "")
    {
      Working_time = millis() - Working_time;
      Serial.println("SYSTEM POWER OFF BECAUSE COVER NOT OPENED");
      lockByServo();
      mustUnlock = 0;
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

  if (LockStatus == 0 && digitalRead(STATUSSCALE_PIN) == Locker_Closed && System_Tag == 0)
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

  //δεν έκλεισε ο δημότης τον κάδο πάνω απο 40 δευτερόλεπτα
  if (millis() - start_time > 80000)
    if (digitalRead(STATUSSCALE_PIN) == Locker_Opened && Mainntanace == 0 && Garbagecollection == 0) //&& lc == 1
    {
      Serial.println("Cover is opened too long");
      // if (millis() - start_time > 50000 && millis() - start_time < 60000)

      if (millis() - start_time > 80000)
      {
        //beepDelay(1000);

        i = 0;
        if (tagId != "")
          while (we == 0 && i < 200)
          {

            readscale();
            if (we == 1)
              break; //change
            i++;
          }
        StaticJsonDocument<250> doc;
        binstatus = ProblemCoverIsOpentoLong;
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
        start_time = millis();
        if (digitalRead(STATUSSCALE_PIN) == Locker_Closed && lc == 1)
          lockByServo();
        Serial.println("SYSTEM POWER OFF");
        beep(2);

        done();
        done();
        ESP.restart();
      }
    }

  //*******************  O ΚΑΔΟΣ ΕΚΛΕΙΣΕ ΚΑΙ ΠΡΕΠΕΙ ΝΑ ΣΤΕΙΛΕΙ ΔΕΔΟΜΕΝΑ ********************

  //if (millis() - start_time > 4000)
  if (digitalRead(STATUSSCALE_PIN) == Locker_Closed && lc == 1)
  {

    if (lc == 1)
    {
      if (Garbagecollection == 1)
        Serial.println("H αποκομιδή τελείωσε και το καπάκι έκλεισε ");
      else
        Serial.println("Ο δημότης πεταξε τα σκουπίδια μέσα και το σύστημα πρέπει να τα ζυγίσει και να τα στείλει ");
      lc = 0;
    }
    if (hasServo == true)
      lockByServo();
    if (tagId == "") //ΑΧΑΡΑΚΤΙΡΙΣΤΑ
     tagId = "D2EE1D1B";

    //******************* Ο δημότης πεταξε τα σκουπιδια μέσα και το σύστημα πρέπει να τα ζυγίσει και να τα στείλει *************
    if (Garbagecollection == 0)
    // if (step == 1)
    {
      //  if (LockStatus==1) lc=1;

      //********  ΖΥΓΙΣΗ ΣΚΟΥΠΙΔΙΩΝ  **********
      {
        i = 0;
        while (we == 0 && i < 200)
        {

          readscale();
          if (we == 1)
            break; //change
          i++;
        }
      }
    }
    //*****************************   Κλείσε την ζυγαρία  ***********************************
    CloseScaleBin();

    //****************************    Έλεγχσς αν ο κάδος γέμισε  ****************************
    if (checkFullBin() && BinFull == 0)
    {

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

    transmit_data();
    working_period = 0;
    beep(1);
    done();
    ESP.restart();
  }

  //step=3;
}
