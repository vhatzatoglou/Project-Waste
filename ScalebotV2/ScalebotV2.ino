
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
#include "ESP32httpUpdate.h"
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
#include "DFPlayer_Mini_Mp3.h"
#include "timestamp32bits.h"

//#include "Zanshin_BME680.h" // Include the BME680 Sensor library
#include <LinkedList.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <Time.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "FS.h"
#include "SPIFFS.h"
#include <Wire.h>
#include "PN532_I2C.h"
#include "PN532.h"
#include "NfcAdapter.h"
#include <Tone32.h>
 
#include "AES.h"
#include "Base64.h"
#include <HTTPClient.h>
AES aes;

// Our AES key. Same in NodeJS but in hex bytes
byte key[] = {0x53, 0x40, 0x66, 0x65, 0x62, 0x30, 0x74, 0x54, 0x68, 0x65, 0x33, 0x21, 0x21, 0x21, 0x21, 0x40};

#define SerialAT Serial2
TaskHandle_t Task1;
TaskHandle_t Task2;
String str, CCID;
int i, lc=0,gr=-1,Maintanance=0;
long open_cover_time = millis();
long start_time = millis();
int p, DAY1, MONTH_, YEAR_, YEAR1, Seconds_, Hours_, Minutes_, hourOfDay, DAY_OF_WEEK, hasModemSIM, contentLength;
boolean GPRS_on;
bool reply = false;
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

//#define BUZZER_PIN 32      //OUTPUT
//#define DONE_PIN 18        //OUTPUT
//#define RED_PIN 14         //OUTPUT
//#define GREEN_PIN 23       //OUTPUT
//#define BLUE_PIN 19        //OUTPUT
//#define LOCKER_PIN 33      //OUTPUT
//#define LOCKER_PIN1 35    //OUTPUT
//#define SCALE_ONOFF_PIN 13  //OUTPUT
//#define RXSCALE_PIN 15
//#define TXSCALE_PIN 17
//#define PIN_DTR 25
//#define PIN_TX 27
//#define PIN_RX 26
//#define PWR_PIN 4
// #define RXMODEM_PIN 26
// #define TXMODEM_PIN 27
//#define STATUSSCALE_PIN 34 //INPUT
//#define COVER_PIN 35       //INPUT
//#define SDA_PIN 21
//#define SCL_PIN 22
//#define ADC_PIN 12

//OUTPUT
#define DONE_PIN 18        //OUTPUT
#define ALARM_PIN 13       //OUTPUT
#define GREEN_PIN 23       //OUTPUT
#define LOCKER_PIN 33      //OUTPUT TRIGGER LOCKER
#define SCALE_ONOFF_PIN 14 //OUTPUT TRIGGER TO OPEN INDICATOR
#define BUZZER_PIN 32      //OUTPUT
#define NFC_RST_PIN 19
//SCALE COMMUNICATION
#define RXSCALE_PIN 15
#define TXSCALE_PIN 17
#define PIN_DTR 25
// LilyGO T-SIM7000G Pinout
#define UART_BAUD           115200
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4
#define LED_PIN             12
//INPUT
#define STATUSSCALE_PIN 34 //INPUT LOCKER 
// #define SENSOR1 32         //INPUT
// #define SENSOR2 35         //INPUT

//INPUT I2C
#define SDA_PIN 21
#define SCL_PIN 22
#define ADC_PIN 36

//#define BUZZER_CHANNEL 0

#define BUZZER_CHANNEL 0
LinkedList<String> Datalist = LinkedList<String>();
LinkedList<String> LostDatalist = LinkedList<String>();
LinkedList<int> LostData = LinkedList<int>();
LinkedList<double> WeightList = LinkedList<double>();
LinkedList<String> Taglist = LinkedList<String>();
PN532_I2C pn532_i2c(Wire);
NfcAdapter nfc = NfcAdapter(pn532_i2c);
String tagId = "";
String userId = "";
byte nuidPICC[4];
String dt;
int hasSIM = 0;
int formatF;
const char *ntpServer = "pool.ntp.org";
const long gmtOfSPIFFSet_sec = 0;
const int daylightOfSPIFFSet_sec = 3600;

String FirstBoot;
String DSN;
String URL;
String f;
int scaleIsConnected;
int step, LockStatus;
byte scale_status = 0;
long delayTime;
double weight, weightF;
long working_period = millis();
long Transmit_working_period = millis();
long nfc_period = millis();
float batterylevel,voltage;
float Working_time = millis();
float Working_time_connecting = 0;
float Working_time_posting = 0;
float mAh=0;
int chgTimes=0;
int chgTimesNew=2;
void check_battery()
{
  int analogValue = analogRead(ADC_PIN);

  Serial.println("analogValue : " + String(analogValue));

  batterylevel = 100 * analogValue / 4095;

  Serial.println("batterylevel : " + String(batterylevel));
}
void calc_battery()
{
  batterylevel = (30000 -mAh)*100/ 30000;
  Serial.println("Calculated Batterylevel : " + String(batterylevel));
}


void check_battery1()
{
  int analogValue = analogRead(ADC_PIN);
   
  Serial.println("analogValue : " + String(analogValue));  
  
  batterylevel = 100 * analogValue /4095;
  
   Serial.println("batterylevel : " + String(batterylevel));
   
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
  Working_time_connecting=millis();
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
    delay(15000);
    Working_time_connecting=0;
    return;
  }

  Serial.print("GPRS status: ");
  if (modem.isGprsConnected())
  {
    Serial.println("connected");
     Working_time_connecting=millis()- Working_time_connecting;
  }
  else
  {
    Working_time_connecting=millis()- Working_time_connecting;
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
  if (hasModemSIM > 0) //&& WiFiMulti.run() != WL_CONNECTED
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
boolean GPRS_open()
{

  // SerialAT.println("AT+CEREG=? ");
  // delay(500);
  // String str=updateSerial();
  // SerialAT.println("AT+CEREG=? ");
  // delay(1000);

  int rep1;

  while (true)
  {
    SerialAT.println("AT+CEREG=?");
    delay(500);
    str = updateSerial();
    rep1++;
    Serial.println("Index " + (rep1));
    if (str.indexOf(",4") > 0 || rep1 > 180)
      break;
  }
  GPRS_on = false;
  // if (str.indexOf("0,5")>0)
  //     GPRS_on= true;
  {
    str = "";
    delay(100);

    SerialAT.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
    delay(50);
    updateSerial();
    SerialAT.println("AT+SAPBR=3,1,\"APN\",\"iot.1nce.net\"");
    delay(50);
    updateSerial();
    SerialAT.println("AT+SAPBR=1,1");
    delay(50);
    updateSerial();
    delay(50);

    int i = 0;
    int l;
    while (l == 0 && i < 100)
    {

      i++;
      //
      SerialAT.println("AT+SAPBR=2,1");
      String str = "";
      delay(500);
      str = updateSerial();
      int l = str.indexOf("\"");
      str = str.substring(l);
      str.replace("\"", "");
      str.replace(".", "");
      Serial.println("ip " + str);
      l = 0;
      l = str.toInt();
      // hasModemSIM = 2;
      //  return true;
      GPRS_on = true;
      str = "";
      if (l > 0)
      {

        Serial.println("OK VALID " + str);
        return GPRS_on;
      }
      str = "";
    }

    return GPRS_on;
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

void beep(int times)
{
  for (int i = 0; i < times; i++)
  {
    pinMode(GREEN_PIN, OUTPUT);
    digitalWrite(GREEN_PIN, LOW);
    tone(BUZZER_PIN, 600, 120, BUZZER_CHANNEL);
    pinMode(GREEN_PIN, INPUT);
    noTone(BUZZER_PIN, BUZZER_CHANNEL);
    delay(120);
  }
  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, LOW);
}

void PutDataBuff1()
{

  String s = "";
  Serial.println("getFreeHeap " + String(ESP.getFreeHeap()));
  //getRTC();
  StaticJsonDocument<250> doc;

  doc["uid"] = "1245ebae";  //: 1000000, ----- user_id όπου εσύ δεν έχεις αρα μπορεις και να μην το στείλεις καθόλου (long)
  doc["gtp"] = 2;           //: 2,----- garbage_type όπου εσύ δεν έχεις αρα μπορεις και να μην το στείλεις καθόλου(short)
  doc["gwg"] = 1.200;       //,----- garbage_weight (decimal)
  doc["tcd"] = "111";       //: "qwertyuiopasdfghjjkzcxzxvxcbcnvmbn",----- tag_code (string)
  doc["scd"] = "SCH_1";     //: "qwertyuiopasdfghjjkzcxzxvxcbcnvmbn",------ scale_code(string)
  doc["dts"] = "timestamp"; //: "8/24/2022 6:01:28 PM"",------ system datetime for encryption(string)
  doc["tmp"] = 30.1;        ///-----temperature (decimal)
  doc["hmt"] = 60;          //-----humity (decimal)
  doc["ver"] = "10.1";      // version number
  doc["btr"] = 5.0;         // battery
  doc["sst"] = 0;           // scale status -> opened=1;closed=2;

  serializeJson(doc, s);
  Serial.println(s);
  postlog(s);
  secure_postlog_gprs();
  doc.clear();
  s = "";
}
String msg; // "{\"uid\":\"de8f0hg7573174f6b9\", \"gtp\":2, \"gwg\":16.2, \"tcd\":\"qwertyihjjkzcxzx\",\"scd\":\"XRB-SGR-40B9\",\"dts\":\"2002/10/06 16:01:30\",\"tmp\":30.2,\"hmt\":60.2,\"ver\":11.2,\"btr\":5.0,\"sst\":1}";
String encrypt(String msg)
{
  char b64data[msg.length() * 2+50];
  byte cipher[msg.length() * 2+50];
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

    String url = "http://device.scalebot.gr/api/Garb/DeviceSendInfo";
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
  String url = "http://device.scalebot.gr/api/Garb/DeviceSendInfo";

  //delay(1000);
  for (int i = 0; i < Datalist.size(); i++)
  {
    if (checklist(i) == 1)
    {
      payload = Datalist.get(i);
      payload.replace("timestamp", dt);
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
        if (httpContent != "0" )
        {
          // LostDatalist.add(i);
          // store_LostDatalist();
         // beep(2);
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
  String url = "http://device.scalebot.gr/api/Garb/DeviceSendInfo";

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
  const size_t CAPACITY =   JSON_ARRAY_SIZE(250);
  StaticJsonDocument<CAPACITY> doc;
  Serial.println("Store Datalist_"+ String(g));
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
        //         Serial.println("Store Log data" + s);
        //         f.println(s);
        //         doc.clear();
        //         array = doc.to<JsonArray>();
        //         s = "";
        //         o++;
        // }
       
      }
         
         
      
      serializeJson(doc, s);
      if (s!="")
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
  const size_t CAPACITY =  JSON_ARRAY_SIZE(250);
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
          if  (!v.isNull())         
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

void restore_Taglist()
{

  s = "";
  const size_t CAPACITY = JSON_ARRAY_SIZE(100);
  Taglist.clear();
  Serial.println("Size CAPACITY " + String(CAPACITY));
  StaticJsonDocument<CAPACITY> doc;
  File f;

  f = SPIFFS.open("/Taglist", "r");
  if (f)
  {

    while (f.available())
    {

      s = f.readStringUntil('\n');
      s.trim();

      Serial.println("Restore Taglist " + s);

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

          Taglist.add(v.as<String>());
        }
        doc.clear();
      }
      doc.clear();
      s = "";
    }

    f.close();
  }
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
        
      s = "";
      Serial.println("getFreeHeap " + String(ESP.getFreeHeap()));
      //getRTC();
      StaticJsonDocument<200> doc;
      //doc["uid"]=userId;//: 1000000, ----- user_id όπου εσύ δεν έχεις αρα μπορεις και να μην το στείλεις καθόλου (long)
       
      doc["gwg"] = weightF;      //,----- garbage_weight (decimal)
      doc["tcd"] = tagId;        //: "qwertyuiopasdfghjjkzcxzxvxcbcnvmbn",----- tag_code (string)
      doc["scd"] = DSN;      //: "qwertyuiopasdfghjjkzcxzxvxcbcnvmbn",------ scale_code(string)
      doc["dts"] = "timestamp";  //: "8/24/2022 6:01:28 PM"",------ system datetime for encryption(string)
      doc["btr"] = batterylevel;  // battery
    //doc["sst"] = scale_status; // scale status -> opened=1;closed=2;

      serializeJson(doc, s);
      Serial.println(s);
      Datalist.add(s);
      if (gr<0)
       {
        gr=0;
        storeGr();
        }
      store_Datalist(gr);
      if (Datalist.size()>=10)
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
  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, LOW);
  tone(BUZZER_PIN, 600, delay, BUZZER_CHANNEL);
  noTone(BUZZER_PIN, BUZZER_CHANNEL);
  pinMode(GREEN_PIN, INPUT);
}

void blinkGreen(void *pvParameters)
{
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
 
  for (;;)
  {
    //pinMode(RED_PIN, INPUT);
    //pinMode(RED_PIN, INPUT);
    vTaskDelay(100);
     if (digitalRead(STATUSSCALE_PIN) == 0 )
    {
      if ((millis() - delayTime) < 500 && (millis() - delayTime) > 0)
      {
        pinMode(GREEN_PIN, OUTPUT);
        digitalWrite(GREEN_PIN, LOW);
      }
      if ((millis() - delayTime) > 500 && (millis() - delayTime) < 1000)
      {
        pinMode(GREEN_PIN, INPUT);
      }
      if ((millis() - delayTime) > 1000)
      {
        delayTime = millis();
        pinMode(GREEN_PIN, OUTPUT);
        digitalWrite(GREEN_PIN, LOW);
      }
    }
  }
}

void blinkBlue(void *pvParameters)
{
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    pinMode(GREEN_PIN, INPUT);
    //pinMode(RED_PIN, INPUT);
    vTaskDelay(100);
    {
      if ((millis() - delayTime) < 500 && (millis() - delayTime) > 0)
      {
       //pinMode(BLUE_PIN, OUTPUT);
       // digitalWrite(BLUE_PIN, LOW);
      }
      if ((millis() - delayTime) > 500 && (millis() - delayTime) < 1000)
      {
        //pinMode(RED_PIN, INPUT);
      }
      if ((millis() - delayTime) > 1000)
      {
        delayTime = millis();
       //pinMode(BLUE_PIN, OUTPUT);
        digitalWrite(GREEN_PIN, LOW);
      }
    }
  }
}

void blinkBlue1()
{
  //pinMode(GREEN_PIN, INPUT);
  {
    if ((millis() - delayTime) > 0)
    {
     //pinMode(BLUE_PIN, OUTPUT);
      digitalWrite(GREEN_PIN, LOW);
    }
    if ((millis() - delayTime) > 250)
      //pinMode(RED_PIN, INPUT);
    if ((millis() - delayTime) > 500)
      delayTime = millis();
  }
}
void Green()
{
  //pinMode(RED_PIN, INPUT);
  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, LOW);
}
void NFC_RST()
{
  
  pinMode(NFC_RST_PIN, OUTPUT);
  digitalWrite(NFC_RST_PIN, LOW);
  delay(1000);
  pinMode(NFC_RST_PIN, INPUT);
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
   
 // if (WeightList.size() == 30)
  {
    Serial.println("weight size" + String(WeightList.size()));
    for (int i = 0; i < WeightList.size() - 1; i++)
    {
      // Serial.println("weight list" + String(i));
      if (WeightList.get(WeightList.size() - 1) == WeightList.get(i))
        cn++;
      else
      {
       // cn--;
      }

      if (cn == 5)
        break;
    }
    if (cn >= 5 && WeightList.size() >= 20)
      {

      weight = WeightList.get(WeightList.size() - 1);
      WeightList.clear();
      Serial.println("weight -->" + String(weight));

      return true;
     }
  }

  return false;
}
int we;
 
String readscale()
{
  String str;
  scaleIsConnected=0;
  delay(100);
  while (Serial1.available())
  {
    scaleIsConnected=1;
    str += Serial1.readStringUntil('\n');
    str.trim();
    Serial.println("Weight:" + str); //Forward what Software Serial received to Serial Port
    // ParseSerialData(str,";");
    str.replace("kg", "");
    str.replace("+", "");
    str.replace(" ", "");
    weight = str.toDouble();
    WeightList.add(weight);
    if (WeightList.size() >20)
     WeightList.remove(0);
    // Serial.println("Weight:" + str);
  }
  if (checkWeight() == true && weight > 0.10 )
  {
     
    weightF = weight;
    if  (we==0) 
    {
      PutDataBuff();
      beep(2);}
      we=1;
      Serial.println("Weight true:" + String(weight));
      open_cover_time = millis();
  
  }

  return str;
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

    Serial.println("Spiffs formatted");
    delay(30000);
    File f = SPIFFS.open("/formatComplete.txt", "w");
    if (!f)
    {

      Serial.println("file open failed");
    }
    else
    {

      f.println("Format Complete");

      // sendsms("Hello from Nicos!");
      formatF = 1;
      store_frsboot();
      ////ESP.restart();
    }
    f.close();
  }
  else
  {

    Serial.println("SPIFFS is formatted. Moving along...");
  }
  //  while (!testsenors())
  //     delay(50);
}
void checkVersion()
{
    beep(5);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Serial.println();
    Serial.println();
    Serial.println();

    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] WAIT %d...\n", t);
        Serial.flush();
       // beepDelay(500);
    }

  // wait for WiFi connection
  if ((WiFi.status() == WL_CONNECTED))
  {
    
    Serial.println("Downloading new version ....");
    t_httpUpdate_return ret = ESPhttpUpdate.update("http://device.scalebot.gr/api/Garb/GetDeviceVersion/2/1.0");
    
    ESPhttpUpdate.rebootOnUpdate(false);
    Serial.println(F("\n---- ota update done ----\n"));
    switch (ret)
    {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      beep(1);
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      beep(5);
      ESP.restart();
      break;
    }
  }
}


void getSettings()
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
      Serial.println(F("Scalebot is Initialized "));

      const char *SER = doc["DSN"];
      DSN = reinterpret_cast<const char *>(SER);
    }

    f.close();
    doc.clear();
  }
}
void initialize_Scale(String DSN)
{
  SPIFFS.begin();
 // File f = SPIFFS.open("/Serial", "r");
 // if (!f.available())
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
  Taglist.add("3312E191");
  Taglist.add("731EC49A");
  Taglist.add("33FD940D");
  Taglist.add("B381DE91");
  Taglist.add("13DE820D");
  Taglist.add("C422F729");
  Taglist.add("527A9A49");
  Taglist.add("D491FC29");
  Taglist.add("0E4F3574");
  Taglist.add("D3BC960D");
  
  store_Taglist();
}

void storeGr()
{
   
  
  {
    SPIFFS.begin();
    StaticJsonDocument<300> js;
    js["GR"] = gr;
    js["MAH"] = mAh;
    js["CHG"] = chgTimes;
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
    StaticJsonDocument<300> js;
     
    File f = SPIFFS.open("/Gr", "r");
    DeserializationError error = deserializeJson(js, f);
    if (error)
    {

      Serial.println(F("Failed to read GR "));
    }
    else
    {
      gr=js["GR"];
      Serial.println("GR: "+ String(gr));
      mAh=js["MAH"];
      Serial.println("mAh : " + String(mAh));
      chgTimes=js["CHG"];
      Serial.println("chgTimes : " + String(chgTimes));
      calc_battery();
    }
    f.close();
  }
   
}



void unlock()
{

  if (digitalRead(STATUSSCALE_PIN) == 0 )//&& LockStatus ==0
  {
    Serial.println(("unlocking"));  
     pinMode(LOCKER_PIN, OUTPUT);
     digitalWrite(LOCKER_PIN, HIGH);
     delay(500);
     pinMode(LOCKER_PIN, INPUT);
    
    scaleIsConnected=0;
    openScale();
    readscale();
  
    if (digitalRead(STATUSSCALE_PIN) == 1)
    {
        // if (scaleIsConnected==0)
        //  openScale();
        // while (digitalRead(STATUSSCALE_PIN) ==0)
        // delay(100);
        pinMode(LOCKER_PIN, INPUT);
        scale_status=1;
      // PutDataBuff();
        LockStatus = 1;
        if (gr==-1) gr=0;
        gr++;
        storeGr();
  }
    
    //if (eTaskGetState(&Task1)==2)
   
  }
}
void lock()
{

  if (digitalRead(STATUSSCALE_PIN) == 1)
  {
    LockStatus = 0;
   // postlog("");

    // Datalist.clear();
    // store_Datalist();
    SPIFFS.end();
  }

}
boolean checkTags()
{
   for (int i = 0; i < Taglist.size(); i++)
      {
        if (Taglist.get(i)==tagId)
        //if (tagId == String("3312E191"))
        return true;
      }
      if (tagId=="F2516E1B" || tagId=="911E3B1B" || tagId=="D292FF1B" || tagId=="5240A149" )
      { Maintanance=1;
       return true;}
    return false;    
}
void readNFC()
{
  if (nfc.tagPresent())
  {
    Green();
    NfcTag tag = nfc.read();
    tag.print();
    nfc_period = millis();
    tagId = tag.getUidString();
    tagId.replace(" ", "");
    open_cover_time = millis();
    if ( LockStatus == 1)  //checkTags()==false &&
     step = 1;
    we = 0;
    working_period=millis();
    Transmit_working_period=working_period;
    if (LockStatus == 0)
      for (int i = 0; i < Taglist.size(); i++)
      {
        //if (Taglist.get(i)==tagId)
       if (checkTags())
        {
          Serial.println(("Input: " + String(digitalRead(STATUSSCALE_PIN))));
          userId=tagId;
          unlock();
         // vTaskDelete(Task1);
          step = 0;
          if (digitalRead(STATUSSCALE_PIN) == 1)
          {
              Serial.println("Unlock System");
               
            // beepD(2000);
              for (int i = 0; i < 8; i++)
              {
                
                //Green();
                delay(1000);
              }
                readscale();
              if (weight=!0)
              {
                 
                beepD(2000);
              }
              else
                Green();
              return;
              }
        }
      }
    beep(1);
  }
  else
  {
     
    // nfc.begin();
    // delay(1000);
    // Red();
  }
  
  delay(100);
}
 
 void done()
{   
     
     
    Serial.println("Working_time:" + String((float)((float)Working_time/10000.00)));
    Serial.println("Working_time_connecting:" + String((float)((float)Working_time_connecting/10000.00)));
    Serial.println("Working_time_posting:" + String((float)((float)Working_time_posting/10000.00)));
   if (Working_time>0)
    mAh=mAh + (float) ((Working_time/10000.00)*1.7); 
      Serial.println("Calculation Consumption:" + String(mAh));
    if (Working_time_connecting>0)
    mAh=mAh + (float)((Working_time_connecting/10000.00)*1.5); 
      Serial.println("Calculation Consumption:" + String(mAh));
    if (Working_time_posting>0)    
    mAh=mAh + (float) ((Working_time_posting/10000.00)*1.5);    
    Serial.println("Calculation Consumption:" + String(mAh));
    
    Working_time=0;
    Working_time_connecting=0;
    Working_time_posting=0;
    Serial.println("SYSTEM POWER OFF");
    storeGr();
    
    SPIFFS.end();
    pinMode(GREEN_PIN, OUTPUT);
    digitalWrite(GREEN_PIN, LOW);
    pinMode(DONE_PIN, OUTPUT);
    digitalWrite(DONE_PIN, HIGH);
   
}

void transmit_data()
{

 // if (Datalist.size() > 0 || LostDatalist.size() > 0)
 if (gr>=0)
  {
 
    xTaskCreatePinnedToCore(
        blinkGreen, /* Task function. */
        "Task2",   /* name of task. */
        10000,     /* Stack size of task */
        NULL,      /* parameter of the task */
        1,         /* priority of the task */
        &Task2,    /* Task handle to keep track of created task */
        0);
   
    SerialAT.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX); //Modem port
    modem_on();
    Working_time_posting=millis();
    //postlog(s);
    int r;
    File f;
    SPIFFS.begin();
    for (int i1 = 0; i1 < 500; i1++)
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
        while (LostDatalist.size() != Datalist.size() && r<10)
        {secure_postlog_gprs(); r++;}
         
        r=0;
        LostDatalist.clear();
        Datalist.clear();
        SPIFFS.remove("/Datalist_" + String(i1));
        SPIFFS.remove("/LostDatalist_" + String(i1));
        Serial.println("Remove /Datalist_" + String(i1));
      
      }
      f.close();
       }
       
    } 
    gr=-1;
    storeGr();
    Working_time_posting=millis()-Working_time_posting; 
    modemPowerOff();
    SPIFFS.end();
    delay(1000);
    done();
  }

  //beep(1000);
  done();
  SPIFFS.end();
}
 void playPolice()
 {
  Serial1.end();
  Serial1.begin (9600, SERIAL_8N1, 14, 15,15); 
   
  mp3_set_serial(Serial1);  
  mp3_set_volume(15);
  mp3_volume_decrease();
  // mp3_stop();
  mp3_play(1);
  mp3_play_physical(1);
  Serial1.end();
 }

void setup(void)
{
  
  Working_time = millis();
  Transmit_working_period=millis();
  pinMode(DONE_PIN, OUTPUT);
  digitalWrite(DONE_PIN, LOW);
  pinMode(STATUSSCALE_PIN, INPUT_PULLUP); //read scale cover status
  pinMode(STATUSSCALE_PIN, INPUT); //read BIN  cover status
        
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  //playPolice();
   
  Serial.println("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  Serial.println("**  ΑΦΙΕΡΩΜΕΝΟ ΣΤΗΝ ΜΝΗΜΗ ΤΟΥ ΠΑΤΕΡΑ ΜΟΥ  ΠΡΟΔΡΟΜΟΥ & ΤΗΣ ΜΗΤΕΡΑ ΜΟΥ ΝΙΚΗΣ **");
  Serial.println("********  ΠΑΤΕΡΑ ΣΕ ΕΥΧΑΡΙΣΤΩ ΠΟΛΥ ΓΙΑ ΟΤΙ ΕΧΩ ΚΑΤΑΦΕΡΕΙ ΧΑΡΗ ΣΕ ΕΣΕΝΑ ******");
  Serial.println("*******   ΜΗΤΕΡΑ ΣΕ ΕΥΧΑΡΙΣΤΩ ΠΟΛΥ ΠΟΥ ΜΕ ΜΕΓΑΛΩΣΕΣ ΓΕΡΟ ΚΑΙ ΔΥΝΑΤΟ  ********");
  Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

  Serial.println("****************     STARTING   SCALEBOT SYSTEM  ver 3.0     ***************");
  nfc.begin();
 
  Serial.println(xPortGetCoreID());
  xTaskCreatePinnedToCore(
      blinkGreen, /* Task function. */
      "Task1",    /* name of task. */
      10000,      /* Stack size of task */
      NULL,       /* parameter of the task */
      1,          /* priority of the task */
      &Task1,     /* Task handle to keep track of created task */
      0);         /* pin task to core 0 */

  //SerialPort.begin (BaudRate, SerialMode, RX_pin, TX_pin).
  //Serial2.begin (15200, SERIAL_8N1, 16, 17);  //9600, 14400, 19200, 38400, 57600, 115200
  Serial1.begin(9600, SERIAL_8N1, RXSCALE_PIN, TXSCALE_PIN, TXSCALE_PIN); //Scale port
  Serial.println("System initialized");
  checkFormat();

  if (!SPIFFS.begin(true))
  {
    Serial.println("Mount Failed");
    return;
  }
  // Taglist.add("3312E191");
  // store_Taglist();
   
  //initialize_Scale("SCH_1");
   
   getSettings();
  //restore_frsboot();
  restore_Taglist();
  working_period = millis();
  // restore_Datalist();
  // restore_LostDatalist();
  //SerialAT.end();
  //Serial1.begin(9600, SERIAL_8N1, RXSCALE_PIN, TXSCALE_PIN, TXSCALE_PIN); //Scale port
  beepD(1000);
  getGr();
  if (chgTimes<chgTimesNew)
  { 
    mAh=0;
    chgTimes=chgTimesNew;
    storeGr();
  }
  nfc.begin();
  //TEST
 // weightF=0.200;     //,----- garbage_weight (decimal)
 // tagId="33FD940D";      //: "qwertyuiopasdfghjjkzcxzxvxcbcnvmbn",----- tag_code (string)
 // DSN="SCH_1";
 // PutDataBuff();
 // transmit_data();
}
int counter;
void loop()
{
     
    //  readscale();
    //     delay(200);
    //    return;

  LockStatus = digitalRead(STATUSSCALE_PIN);
  if (LockStatus == 1)
     Transmit_working_period = 0;
   //ΕΛΕΓΧΟΣ ΓΙΑ ΑΠΟΣTΟΛΗ ΔΕΔΟΜΕΝΩΝ  ΣΕ 50 ΛΕΠΤΑ ΛΕΙΤΟΥΡΓΙΑΣ ΧΩΡΙΣ TAG DETECT
  if (millis()-working_period>40*60000 && working_period > 0)
  {
    Serial.println("Scalebot has woken up and will transmit data");
    delay(2000);
    vTaskDelete(Task1);
    Working_time=millis()-Working_time;
    transmit_data();
    Transmit_working_period = 0;
    done();
  }
  if (LockStatus == 1 && lc == 0)
  {
    lc = 1;
    Green();
  }
  if (Maintanance == 1)
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
      checkVersion();
    }
    else
    {
      mAh=0;
      storeGr();
     
    }
    
    Maintanance = 0;
    return;
  }
  
  //Serial.println((millis() - Transmit_working_period);
  //ΕΛΕΓΧΟΣ ΓΙΑ ΑΠΟΣTΟΛΗ ΔΕΔΟΜΕΝΩΝ  ΣΕ 2 ΛΕΠΤΑ
  if (millis() - Transmit_working_period > 120000 && Transmit_working_period > 0 && LockStatus == 0)
  {

    Serial.println("Scalebot has woken up and will transmit data");
    delay(2000);
    vTaskDelete(Task1);
    Working_time=millis()-Working_time;
    transmit_data();
    Transmit_working_period = 0;
    done();
  }
  // //ΕΛΕΓΧΟΣ ΓΙΑ ΑΝΟΙΚΤΟ ΚΑΠΑΚΙ ΠΑΝΩ ΑΠΟ 2 ΛΕΠΤΑ
  // if (millis() - open_cover_time > 60000 && open_cover_time > 0 && LockStatus == 1)
  // {
  //   Serial.println("The cover is opened too long");
  //   Red();
  //   beepD(3000);
  //   Green();
  //   open_cover_time = millis();
  //   i++;
  //   if (i > 2)
  //     done();
  // }
  
  if (millis() - nfc_period > 15000)
  {
    nfc_period = millis();
     
  if (!nfc.begin())
    {
      //nfc = NfcAdapter(pn532_i2c);
       
      Serial.println("Nfc restart" );
      NFC_RST();
      delay(100);
      nfc.begin();
      nfc_period = millis()-14000;
      
    }
  }
  
  // readscale();
 if (step == 0 ) //&& (millis() - nfc_period > 2000)
  {
    
    readNFC();
    //we = 0;
  }
  if (step == 1) //&& LockStatus==1
  {
    //  if (LockStatus==1) lc=1;
    Green();
    if ( (millis()-working_period)>35000)
    { step = 0;   we = 0;}
   // if (we < 2)
    {
      i=0;
      readscale();  
     // nfc_period=millis();
     
    }
    if ( we == 1 && weight<0.10)  
         we = 2;
    if (we == 2)
    {
      step = 0;
      open_cover_time = millis();
      we = 0;
      // if (!nfc.begin())
      // Red();
    }
  }
   if (millis() - start_time > 5*60*60*1000 && start_time>0)
   done();
  if (millis() - start_time > 3000)
  if (LockStatus == 0 && lc == 1 )
  {
      Serial.println("System system is locked and will be powered off");
      lc = 0;
      scale_status=2;
      tagId=userId;
     //storeGr();
    // PutDataBuff();
    // restore_Datalist();
    // transmit_data();
    //beep(1000);
   // pinMode(DONE_PIN, INPUT);
    Working_time=millis()-Working_time;
    done();
    delay(1000);
   //pinMode(DONE_PIN, INPUT);
    done();
    open_cover_time = millis();
  }
  //step=3;
}
