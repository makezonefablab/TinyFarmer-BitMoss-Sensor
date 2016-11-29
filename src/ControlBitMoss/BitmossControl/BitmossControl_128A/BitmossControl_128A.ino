/*
 * 
 *  Project : Tiny Farmer 
 *  SubProject : Bitmoss Controller
 *  
 *  Since : 2015.11.01
 *  Author : Jang Sun yeon (Mediaflow)
 *  URL : www.tinyfarmer.com  / www.mediaflow.kr
 *  e-mail : iot@mediaflow.kr
 * 
 *  - 4 relays
 *  - LCD 16 X 2
 *  - RTC for Schedule from Cloud
 *  - EEPROM for storage of Schedules
 *  - RS485 communication
 * 
 * 
 * 
 * 
 */


#include <EEPROM.h>
#include <ArduinoJson.h>
#include <avr/eeprom.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>


// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>
#include "RTClib.h"

#define USE_RTC     

#define DEVICE_ID    3

//led pin
#define LED_B       14
#define LED_G       13
#define LED_R       12
#define MODE_LED     3

// DIAL ID pin
#define DIAL1_8 35
#define DIAL1_4 34
#define DIAL1_2 33
#define DIAL1_1 32
#define DIAL2_8 39
#define DIAL2_4 38
#define DIAL2_2 37
#define DIAL2_1 36


//relay pin
#define NUMBER_RELAY 4
#define RELAY1 25
#define RELAY2 24
#define RELAY3 23
#define RELAY4 22


//relay feedback pin
#define RELAY1FEEDBACK A0
#define RELAY2FEEDBACK A1
#define RELAY3FEEDBACK A3
#define RELAY4FEEDBACK A6


// RS485 PIN
#define RS485DIR   7

//heartbeat select
#define CONNECTION 1
#define HEARTBEAT  2

//heartbeat setting
#define HEARTBEAT_TIMER 30000
uint32_t heartbeatTimer;

//operation setting
#define OPERAT_TIMER 60000
uint16_t operationTimer[NUMBER_RELAY];
uint16_t operSpenTime[NUMBER_RELAY];
unsigned long operStartTime[NUMBER_RELAY];

//Relay value
#define RELAY_ON_VALUE 850
#define RELAY_OFF_VALUE 10


// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(26,27,28,29,30,31);
byte schedICON[8] = {
  B01110,
  B10101,
  B10101,
  B10111,
  B10001,
  B01110,
  B00000,
};

byte relayICON[8] = {
  B01110,
  B00100,
  B00100,
  B11111,
  B11111,
  B11111,
  B11111,
};

byte relayON[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};

byte relayOFF[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
};
////////////////////////////////////////////////////////////////////////////////////////////
//스케쥴 저장을 위한 메모리 설정 
//https://projectgus.com/2010/07/eeprom-access-with-arduino/

#define MAX_SCHED  10

/*
 * __eeprom_data is the magic name that maps all of the data we are 
 * storing in our EEPROM
 */
struct __eeprom_data {
  int id;
  int date;
  int hour;
  int min;
  int value;
  uint16_t delayedTime;
} ;
 
typedef struct __eeprom_data _schedule;
int count_of_Schedule = 0;
_schedule mySchedule[MAX_SCHED];


int nodeID = DEVICE_ID;                                 /* 메시 네트워크 Node 아이디, 스위치로부터 해당 값을 새로 설정 함 */
int ch[10] = {97, 1, 11, 22, 33, 44, 55, 66, 77, 88};   /* 채널 값 설정 테이블 97번 채널이 Default */
bool isSendFail = false;                                /* 메시 네트워크 전송 실패 여부 확인 Flag */


//RTC
#ifdef USE_RTC
#define DS3231_I2C_ADDRESS 0x68
DateTime now;
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tues", "Wed", "Thurs", "Fri", "Sat"};
DateTime startTime, endTime;
#endif


//relay state check
bool relay[4] = {0, 0, 0, 0};
uint16_t relayDelayedTime[4] = {0, 0, 0, 0};
bool check = false;
int relayPin[4] = {RELAY1, RELAY2, RELAY3, RELAY4};
int relayCheckPin[4] = {A0, A1, A3, A6};
bool feedBack[4] = {0, 0, 0, 0};
int sequenceID = 0;

//JSON
String ptCd;
String relayHistoryId;
String id;
String val;
String resCd = "0000";
String operTime ="0";
String seq ="0";
String recivedCommands[10];

// 기타 설정
#define SERIAL_SPEED                  9600 //115200 /* 115200 /* 시리얼 속도 */
#define JSON_BUFFER_SIZE              300 /* JSON 버퍼 사이즈 */
#define NUM_OF_RETRY                  5 /* 주기 정보 전송 재시도 횟수 */
#define DELAY_BETWEEN_MESSAGE         100 /* 메세지 사이의 최소 시간 */

////////////////////count_mac////////
int mac_count = 0;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
String recivedData = "";
byte recivedBytes[18];
boolean receivedStart = false;
int index = 0;
String Data = "";

/////////////////////////////////////
byte xorByte = 0x00;
int receivedIndex = 0;


int getID()
{
   int upVal = 0;
   int lowVal = 0;
   int id = 0;

   upVal =  8 * digitalRead(DIAL1_8) + 4 * digitalRead(DIAL1_4) + 2 * digitalRead(DIAL1_2) + digitalRead(DIAL1_1) ;
   lowVal =  8 * digitalRead(DIAL2_8) + 4 * digitalRead(DIAL2_4) + 2 * digitalRead(DIAL2_2) + digitalRead(DIAL2_1) ;

   id = upVal * 10 + lowVal;

   return id;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // 다이얼 ID 핀 설정
  pinMode(DIAL1_8, INPUT);
  pinMode(DIAL1_4, INPUT);
  pinMode(DIAL1_2, INPUT);
  pinMode(DIAL1_1, INPUT);
  pinMode(DIAL2_8, INPUT);
  pinMode(DIAL2_4, INPUT);
  pinMode(DIAL2_2, INPUT);
  pinMode(DIAL2_1, INPUT);

  //RS485 핀 설정 
  pinMode(RS485DIR, OUTPUT);
  digitalWrite(RS485DIR, LOW); //Enable low, RS485 shield waiting to receive data
  
  // 릴레이 핀 설정
  for (int i = 0; i < NUMBER_RELAY; i++) {
    pinMode(relayPin[i], OUTPUT);
    digitalWrite(relayPin[i], LOW);
    relay[i] = LOW; 
    val[i] = false;
  }

  // 노드 아이디 세팅 (다이얼 스위치로부터 입력 받음)
  nodeID = getID();

  
  Serial1.begin(9600);
  Serial1.setTimeout(5000);
  
  initLED();
  
  Serial.begin(9600);  //can't be faster than 19200 for softserial
  Serial.print("node ID : ");
  Serial.println(nodeID);
  Serial.println("Bitmoss Debugging");

  ////////////////////////////////////////////////////////////////
  // RTC 설정
  ////////////////////////////////////////////////////////////////
  
#ifdef USE_RTC
  delay(3000); // wait for console opening
  
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  //if (rtc.lostPower()) 
  {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  
#endif

  ////////////////////////////////////////////////////////////////
  // LCD 설정
  ////////////////////////////////////////////////////////////////
  // set up the LCD's number of columns and rows:
  _addCharLCD();
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("bitMossC V.1.0");
  lcd.setCursor(0, 1);
  lcd.print("booting ......");


  EEPROM.put( 0, 0);
  _schedule aa,bb,cc,dd;
  aa.id = 1;
  aa.date = 8 ; //wed
  aa.hour = 19;
  aa.min  = 32;
  aa.value= 8;
  aa.delayedTime = 1;
  _put_Schedule( 0, aa);

  bb.id = 2;
  bb.date = 8 ; //wed
  bb.hour = 19;
  bb.min  = 34;
  bb.value= 4;
  bb.delayedTime = 1;
  _put_Schedule( 1, bb);

  
  cc.id = 3;
  cc.date = 8 ; //wed
  cc.hour = 19;
  cc.min  = 36;
  cc.value= 2;
  cc.delayedTime = 1;
  _put_Schedule( 2, cc);

  dd.id = 4;
  dd.date = 8 ; //wed
  dd.hour = 19;
  dd.min  = 38;
  dd.value= 1;
  dd.delayedTime = 1;
  _put_Schedule( 3, dd);
  
  
  ////////////////////////////////////////////////////////////////
  // 스케쥴 꺼내기
  ////////////////////////////////////////////////////////////////
  EEPROM.get( 0, count_of_Schedule);

  for (int i = 0 ; i < count_of_Schedule ; i++)
  {
    mySchedule[i] = _get_Schedule(i);
    Serial.println("============ Schedule ==================");
    Serial.println(mySchedule[i].id);
    Serial.println(mySchedule[i].date);
    Serial.println(mySchedule[i].hour);
    Serial.println(mySchedule[i].min);
    Serial.println(mySchedule[i].value);
    Serial.println(mySchedule[i].delayedTime);
  }
  Serial.print("count_of_Schedule = ");
  Serial.println(count_of_Schedule);

  delay(1000);
  
  statusSend();
  delay(1000);
  
  digitalWrite(MODE_LED, HIGH);

  _printoutLCD("SYSTEM OK");

}


void loop() {


  initLED();

  
  if (stringComplete == true)
  {
    // Manual Control Protocol
    if(recivedBytes[1] == 0xF0)
    {
      String _ptCd = String((int)recivedBytes[1]);
      long  __relayHistoryIdL = (long)(recivedBytes[2] << 56)
                              + (long)(recivedBytes[3] << 48)
                              + (long)(recivedBytes[4] << 40)
                              + (long)(recivedBytes[5] << 32)
                              + (long)(recivedBytes[6] << 24)
                              + (long)(recivedBytes[7] << 16)
                              + (long)(recivedBytes[8] << 8)
                              + (long)recivedBytes[9];
  
                              
      String _relayHistoryId = String(__relayHistoryIdL);
      String _id = String((int)recivedBytes[10]);
      String _operTime = String((int)recivedBytes[11]<<8 + (int)recivedBytes[12]) ;
      String _seq = String((int)recivedBytes[13]);
      String _val = String((int)recivedBytes[14]>>7 & 0x01) // 11111111 11111111
                   +String((int)recivedBytes[14]>>6 & 0x01)
                   +String((int)recivedBytes[14]>>5 & 0x01)
                   +String((int)recivedBytes[14]>>4 & 0x01)
                   +String((int)recivedBytes[14]>>3 & 0x01)
                   +String((int)recivedBytes[14]>>2 & 0x01)
                   +String((int)recivedBytes[14]>>1 & 0x01)
                   +String((int)recivedBytes[14] & 0x01)
                   +String((int)recivedBytes[15]>>7 & 0x01)
                   +String((int)recivedBytes[15]>>6 & 0x01)
                   +String((int)recivedBytes[15]>>5 & 0x01)
                   +String((int)recivedBytes[15]>>4 & 0x01)
                   +String((int)recivedBytes[15]>>3 & 0x01)
                   +String((int)recivedBytes[15]>>2 & 0x01)
                   +String((int)recivedBytes[15]>>1 & 0x01)
                   +String((int)recivedBytes[15] & 0x01);
      
  
      Serial.println("============ parsing completed ==================");
      Serial.println(_ptCd);
      Serial.println(_relayHistoryId);
      Serial.println(_id);
      Serial.println(_operTime);
      Serial.println(_seq);
      Serial.println(_val);
      
      
      ptCd = _ptCd;
      relayHistoryId = _relayHistoryId;
      id = _id;
      operTime = _operTime;
      val = _val;
      seq = _seq;
  
      /*
       * {
       *  "ptCd":"07",
       *  "relayHistoryId":"1",
       *  "id":"1",
       *  "operTime":"1",
       *  "val":"0010",
       *  "seq":"3"
       *  } 
       */
      if(_ptCd == "7" && _id == String(nodeID))
      {
        /*
         *  seq값은 제어할 릴레이 번호, 제어값은 val에 해당 index 값
         */
        sequenceID = _seq.toInt();
        //if(sequenceID > 0 && sequenceID <= 4)
        for(int i = 0 ; i < NUMBER_RELAY ; i++)
        {
          //sequenceID = sequenceID - 1;
          relay[i] = (val[i] == '1' ? true : false);
          check = true;
        }
        
      }
    }
    // Schedule List Protocol
    else if()
    {
      
    }

    stringComplete = false;
  }


  /////////////////////////////////////////////
  // 스케쥴 체크
  ////////////////////////////////////////////
  check_run_schedule();


  if (check == true) 
  {
    
#ifdef USE_RTC    
    startTime = rtc.now();  // 명령이 시작된 시간 저장
#endif


    for(int i = 0 ; i < NUMBER_RELAY ; i++)
    {
      digitalWrite(relayPin[i], relay[i]);
      Serial.println("digitalWrite OK");
    }
    
    feedback();
    tcpSend();
    check = false;
  }
  else
  {
    for(int i = 0 ; i < NUMBER_RELAY ; i ++)
    {
      digitalWrite(relayPin[i], relay[i]);
    }
  }

  // "0"은 수동으로 제어된 신호이므로, 긴 시간 제어되게 임시 변경
  if(operTime == "0")
  {
    operSpenTime[sequenceID] = 999;
  }
  else
  {
    operSpenTime[sequenceID] = operTime.toInt();
  }
  
  
  //  heartbeatTimer
  if ((unsigned long)(millis() - heartbeatTimer) > HEARTBEAT_TIMER)
  {
    //connection_msg(HEARTBEAT);
    heartbeatTimer = millis();
    statusSend();
    Serial.println("heartbeatTimer ===== ");

#ifdef USE_RTC
    now = rtc.now();
  
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);// "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    Serial.print(" since midnight 1/1/1970 = ");
    Serial.print(now.unixtime());
    Serial.print("s = ");
    Serial.print(now.unixtime() / 86400L);
    Serial.println("d");
#else

#endif
  }

  
#ifdef USE_RTC
  // 지속시간이 경과되면 명령 원위치
  now = rtc.now();
  for(int i = 0 ; i < count_of_Schedule; i ++)
  {
    if ((operStartTime[i] > 0) && ((now.unixtime()-operStartTime[i]) >= relayDelayedTime[i]*60))
    {
      relay[i] = false;
      Serial.println(String(i+1)+"st Sched stop : "+ String(now.hour())+":"+String(now.minute()));
      
      operStartTime[i] = 0;
    }
  }
#endif


  delay(1000);

  _printoutLCD("");
  _printoutRelayStatus(relay);

}


void feedback()
{
  bool flag = false;
  delay(1000);
  for (int i = 0; i < NUMBER_RELAY; i++) {
    if (analogRead(relayCheckPin[i]) > RELAY_ON_VALUE) {
      feedBack[i] = true;
    }
    else if (analogRead(relayCheckPin[i]) < RELAY_OFF_VALUE) {
      feedBack[i] = false;
    }
  }

  /*
   * 연결된 아날로그 센서와 비교하여 값이 릴레이 제어 값이랑 달라야 
   */
  for (int i = 0; i < NUMBER_RELAY; i++) {
    if (relay[i] != feedBack[i]) 
    {
      //digitalWrite(relayPin[i], LOW); //현재 피드백센서가 없으므로 일단 주석처리
      resCd[i] = '1';
      // Error message send
      Serial.print("error : ");
      Serial.println(i);

      flag = true;
    }
    else {
      resCd[i] = '0';
    }

  }

  if (flag == false) {
    resCd = "0000";
  }

  ////////////////////////////////////////////////////////////////////////
  // 피드백 센서가 적용되기전까지는 resCd = status ..
  String _resCd ="";
  for (int i = 0; i < NUMBER_RELAY; i++)
  {
    _resCd += String(relay[i]);
  }
  resCd = _resCd;
  ////////////////////////////////////////////////////////////////////////

}


void feedback_error(int error_code)
{
  resCd = String(error_code);
  tcpSend();
}


void tcpSend() {
  
  String msg = "";
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  root["ptCd"] = "08";
  root["relayHistoryId"] = relayHistoryId;
  root["id"] = String(nodeID);
  root["val"] = val;
  root["resCd"] = resCd;

  digitalWrite(RS485DIR, HIGH); //Enable low, RS485 shield sending data
  root.printTo(Serial1);
  Serial1.println("");
  digitalWrite(RS485DIR, LOW); //Enable low, RS485 shield waiting to receive data
  
  //DEBUG
  root.printTo(Serial);
  Serial.println("");

}

void statusSend() {
  
  String msg = "";
  StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  
  String status_Relay ="";
  
  for (int i = 0; i < NUMBER_RELAY; i++)
  {
    status_Relay += String(relay[i]);
  }

  root["ptCd"] = "02";
  root["dtCd"] = "03";
  root["id"] = String(nodeID);
  root["status"] = status_Relay;

  digitalWrite(RS485DIR, HIGH); //Enable low, RS485 shield sending data
  root.printTo(Serial1);
  Serial1.println("");
  digitalWrite(RS485DIR, LOW); //Enable low, RS485 shield waiting to receive data

  //DEBUG
  root.printTo(Serial);
  Serial.println("");

}


void setLED(char color, int on)
{
  if (color == 'R')
  {
    digitalWrite(LED_R, !on);
  }
  else if (color == 'G')
  {
    digitalWrite(LED_G, !on);
  }
  else if (color == 'B')
  {
    digitalWrite(LED_B, !on);
  }
}


void initLED()
{
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
}




void check_run_schedule()
{
#ifdef USE_RTC
  DateTime now = rtc.now();
  
  for (int i = 0 ; i < count_of_Schedule ; i++)
  {
    if(_isToday(now.dayOfTheWeek(), mySchedule[i].date) == true)
    {
      String _val = String((int)mySchedule[i].value>>3 & 0x01)
                   +String((int)mySchedule[i].value>>2 & 0x01)
                   +String((int)mySchedule[i].value>>1 & 0x01)
                   +String((int)mySchedule[i].value & 0x01);

      /*
       * operStartTime[i]가 0보다 크면 이미 스케쥴 중이라고 판단
       * 0인것만 스케쥴 한번 시작
       */
      if((mySchedule[i].hour == now.hour()) && (mySchedule[i].min == now.minute()) && operStartTime[i] == 0)
      {

        Serial.println(String(i+1)+"st SCHE OK :" + String(now.hour())+":"+String(now.minute()));

        // delay time 
        relayDelayedTime[i] = mySchedule[i].delayedTime;
        operStartTime[i] = now.unixtime();
        
        for(int j = 0 ; j < NUMBER_RELAY ; j++)
        {
          relay[j] = (_val[j] == '1' ? true : false);
          
          
        }
      }
    }
  }
#endif
}



void serialEvent1() {

  while(Serial1.available()) 
  {
    
    byte inChar = (byte)Serial1.read();

    // 수신된 패킷이 수동제어 헤더 바이트인지 검사
    if(inChar == 0xF0 && receivedStart == false)
    {
      memset(recivedBytes,0x00,11);
      recivedBytes[0] = inChar;

      xorByte = 0x00;
      stringComplete = false;
      receivedIndex = 0;
      receivedStart = true;
    }
    // 수신된 패킷이 스케쥴 헤더 바이트인지 검사 
    else if(inChar == 0xF1 && receivedStart == false)
    {
      
    }

    // HEADER 수신 성공 시
    if(receivedStart == true)
    {
      
      recivedBytes[receivedIndex] = inChar;

      // * Manual Control Protocol
      //
      // start(1Byte)  ptcd(1Byte)  relayHist(8Byte) id(1Byte)  opertime1(1Byte)  opertime2(1Byte)  seq(1Byte)  val1(1Byte)  val2(1Byte)  payload Len(1Byte)  Payload   end(1Byte)
      // 0xF0          0x07         [0x01~8ea]       0x08       0x00              0x00              0x01        0x0F         0x01         0x00                0xF1      END       = 19bytes
      //
      // End = ptcd ^ relayHist ^ id ^ opertime1 ^ opertime2 ^ seq ^ val ^ payload Len
      //
      if(recivedBytes[0] == 0xF0)
      {

          // Payload Len 까지만 Xor 처리
          if(receivedIndex >= 1 && receivedIndex <= 16)
          {
            xorByte = xorByte^inChar;
          }
    
          //////////////////////////////////////////////////////////
          // 정상적으로 프토토콜이 받아졌는지 검사
          //////////////////////////////////////////////////////////
          if(xorByte == recivedBytes[18] && receivedIndex == 18)
          {
            stringComplete = true;
            receivedStart = false;
            receivedIndex = 0;
          }
          else if(xorByte != recivedBytes[18] && receivedIndex == 18) // 깨진 프로토콜 또는 쓰레기 문자 수신 18byte 시
          {
            receivedStart = false;
            receivedIndex = 0;
            feedback_error(99); // 패킷수신 실패 
            Serial.println("XOR : "+String(xorByte,HEX));
          }
      }

      // * Schedule List Protocol
      //
      // OxF1 count id date hour min value1 value2 delay1 delay2 END   =  11bytes
      //
      // END = count ^ id ^ date ^ hour ^ min ^ value1 ^ value2 ^ delay1 ^ delay2 
      //
      else if(recivedBytes[0] == 0xF1)
      {
          // delay2 까지만 Xor 처리
          if(receivedIndex >= 1 && receivedIndex <= 9)
          {
            xorByte = xorByte^inChar;
          }

          //////////////////////////////////////////////////////////
          // 정상적으로 프토토콜이 받아졌는지 검사
          //////////////////////////////////////////////////////////
          if(xorByte == recivedBytes[10] && receivedIndex == 10)
          {
            stringComplete = true;
            receivedStart = false;
            receivedIndex = 0;
          }
          else if(xorByte != recivedBytes[10] && receivedIndex == 10) // 깨진 프로토콜 또는 쓰레기 문자 수신 18byte 시
          {
            receivedStart = false;
            receivedIndex = 0;
            feedback_error(98); // 패킷전송 다시 요구
            Serial.println("XOR : "+String(xorByte,HEX));
          }
      }

      receivedIndex ++;
      
    }

  }//while
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////


// Convert normal decimal numbers to binary coded decimal
byte _decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}


// Convert binary coded decimal to normal decimal numbers
byte _bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}


//////////////////////////
boolean _isToday(int _today, int _scheduleDay)
{
   int date[7] = {0x40,0x20,0x10,0x08,0x04,0x02,0x01}; // sun, mon, tues, wed, thur, fri, sat
   boolean ret = false;

   if((date[_today]&_scheduleDay) > 0)
   {
     ret = true;
   }

   return ret;
}


//////////////////////////
void _put_Schedule(int _eeAddress , _schedule sched)
{
   int count = 0;
   
   EEPROM.put(_eeAddress*sizeof(_schedule)+2, sched); // offset +2
   
   EEPROM.get(0, count);
   count ++;
   EEPROM.put(0, count);
   
}



//////////////////////////
_schedule _get_Schedule(int _eeAddress)
{
   _schedule sched;

   EEPROM.get( _eeAddress*sizeof(_schedule)+2, sched ); // offset +2

   return sched;
}


void _clear_Schedule()
{
   EEPROM.put(0, 0);
}


void _printoutLCD(String _arg)
{

   lcd.clear();
   lcd.setCursor(0, 0);
   
#ifdef USE_RTC
   now = rtc.now();
   
   String _date = String(daysOfTheWeek[now.dayOfTheWeek()])+" ";
   String _hour = String(now.hour());
   String _min = String(now.minute());
   String _sec = String(now.second());

   if(_hour.length() == 1) _hour="0"+_hour;
   if(_min.length() == 1) _min="0"+_min;
   if(_sec.length() == 1) _sec="0"+_sec;
   
   lcd.print(_date);
   lcd.print(_hour+":"+_min+":"+_sec);
#endif
   lcd.setCursor(15, 0);
   lcd.write(byte(0));
   lcd.setCursor(0, 1);
   lcd.print(_arg);
}

void _printoutRelayStatus(bool * _relay)
{
   lcd.setCursor(0, 1);
   lcd.print("                "); // clear

   lcd.setCursor(0, 1);
   lcd.write(byte(3));
   lcd.print(":"); 
   
   for(int i = 0 ; i < NUMBER_RELAY ; i++)
    {
      if(_relay[i] == true)
      {
        lcd.setCursor(i+3, 1);
        lcd.write(byte(1));
      }
      else
      {
        lcd.setCursor(i+3, 1);
        lcd.write(byte(2));
      }
    }
}


void _addCharLCD()
{
   lcd.createChar(0, schedICON);
   lcd.createChar(1, relayON);
   lcd.createChar(2, relayOFF);
   lcd.createChar(3, relayICON);
}

