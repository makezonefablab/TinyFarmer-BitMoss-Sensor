/*
 * 
 *  Project : Tiny Farmer 
 *  SubProject : Bitmoss Sensor
 *  
 *  Since : 2015.11.01
 *  Author : Jang Sun yeon (Mediaflow)
 *  URL : www.tinyfarmer.com  / www.mediaflow.kr
 *  e-mail : iot@mediaflow.kr
 * 
 *  - 8 kinds sensors (more than 8 sensors)
 *  - RF moddule via UART interface
 *  - Energy saving (not yet)
 *  
 * 
 * 
 */

//#include <JeeLib.h>             /* Sleep 함수를 위한 LIB */
#include <Wire.h>
#include <math.h>
#include "DHT.h"

#include <ArduinoJson.h>
#include <Arduino.h>
#include <stdint.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


#define USE_HC12                 /* HC12 모듈 사용 시 */
#define COMM_TEST_MODE     OFF

#define SLEEP_PERIOD       0     /* 최초 시작 이후 첫번째 데이터 취득 대기 시간 (분) */
#define DEFUALT_PERIOD     0     /* 최초 시작 이후 첫번째 데이터 취득 대기 시간 (분) */
#define MINUATE            60000  /* 1분의 기준 ms */
#define HEARTBEAT_TIMER    30000 /* HEARTBEAT 갱신 주기 */

#define HC12_SET           40

#define DHTPIN             4      /* 온습도 센서 PIN 번호 */
#define DHTTYPE            DHT22  /* 온습도 센서 종류 */
DHT dht(DHTPIN, DHTTYPE);

// 센서 SW 시리얼
#define RX                 6      /* RX 핀 번호 */
#define TX                 5      /* TX 핀 번호 */
//SoftwareSerial mySerial(RX, TX);

// 디버그 모니터링 시리얼 설정
#define DEBUG_TX           12
#define DEBUG_RX           13
//SoftwareSerial DEBUG(DEBUG_RX, DEBUG_TX);

#define ON                 1      /* 센서 연결 ON */
#define OFF                0      /* 센서 연결 OFF */

#define NO_SENSOR          0      /* NONE Connected  */
#define PH_SENSOR          1      /* PH Sensor */
#define WEATHER_STATION    2      /* Weather Station Sensor */
#define FRUIT_SENSOR       3      /* Fruit Sensor */
#define EC_SENSOR          4      /* EC Sensor */

#define SENSOR_SELECT      NO_SENSOR        /* 위에서 센서 종류 선택 or NO_SENSOR */
#define CO2_SENSOR         OFF              /* ON or OFF */

#define LED_BLUE 16                         /* 상황 LED */
#define LED_RED 8
#define LED_CENTER 3

#define POWER_PIN           12              /* 센서 전력 차단을 위한 핀 Select */

#define SERIAL_SPEED        9600
#define JSON_BUFFER_SIZE    300             /* JSON 버퍼 사이즈 */
bool isCenterLEDOn = false;                 /* 하트 비트 LED ON/OFF 여부 */

// 전역 변수
char databuffer[35];                        /* 시리얼 데이터 읽기 위한 버퍼 */

int BH1750address = 0x23;                   /* 조도 I2C 주소 */
byte buff[2];                               /* 조도 읽기 위한 버퍼 */

String sensorstring = "";                   /* 센서 데이터 저장을 위한 버퍼 */
boolean sensor_string_complete = false;     /* 센서로부터 데이터 전부 수신 했는지 여부 값 */
float pH;                                   /* PH 획득 값 변수 */
String HC12Data = "";

int nodeID = -1;                            /* 메시 네트워크 Node 아이디, 스위치로부터 해당 값을 새로 설정 함 */
int channel = 1;                            /* 기본 채널 1 */


// 프로토콜
String ptCd = "02";
String dtCd = "03";
String id = "1";
String Sleep = "";

unsigned long periodTimer = 0;              /* 주기 계산을 위한 타이머 변수 */
unsigned long ledTimer = 0;                 /* 하트비트 LED를 위한 타이머 변수 */
unsigned long timeCount = 0;

int period = DEFUALT_PERIOD;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
String recivedData = "";

byte sensor_data_packet[30];     // 데이터 패킷

struct sensor_data_t {           /* 센서 데이터 Struct: 메시 네트워크 내부에서만 사용 */
  int dhId;                      // 채널번호 , 1byte
  int id;                        // 1byte
  float temp;                    // 1byte.1byte
  float hum;                     // 1byte.1byte
  long ill;                      // 3byte
  int co2;                       // 2byte
  float ph;                      //
  float ec;                      //
  float soil_temp;               // 1byte.1byte
  float soil_hum;                // 1byte.1byte
  int wind_dir;                  // 2byte
  float wind_vol;                // 
  float rainfall;                //
  bool isBooted;
};

sensor_data_t sensorDataT;       /* 센서 데이터 전송을 위한 센서 데이터 Struct 선언 */

// S/W 리셋 함수
void(* resetFunc) (void) = 0;//declare reset function at address 0

ISR(TIMER0_COMP_vect)     
{
/* 이곳에 타이머 인터럽트에 사용할 내용을 넣어줌 */
}

ISR(TIMER1_COMPA_vect)
{
    timeCount ++;

    if(timeCount > 60)
    {
        sleep_disable();
        Timer1_stop();
        timeCount = 0;
        //onSleep = false;
        //cli();
    }

    wdt_reset();

//    digitalWrite(LED_RED,toggleLED);
//    if(toggleLED == HIGH) toggleLED = LOW;
//    else toggleLED = HIGH;
} 

/*===============================================================================*/
/*=         AVR Timer0을 이용하여 1ms 타이머 설정                                                                                           =*/
/*===============================================================================*/
void TIMER0_INIT()
{
/* Timer/Counter Control Register */
/* TCCR0 <= FOC0 | WGM00 | COM01 | COM00 | WGM01 | CS02 | CS01 | CS00 */
    TCCR0 = (0 << FOC0)
      | (1 << WGM01) | (0 << WGM00) /* CTC Mode of Operation */
      | (0 << COM01) | (0 << COM00)/* Normal port Operation,OC0 disconnected */
      | (1 << CS02) | (0 << CS01) | (0 << CS00);  /* 1/64 Prescaler */


/* Timer/Counter Interrupt Mask Register */
/* TIMSK <= OCIE2 | TOIE2 | TICIE1 | OCIE1A | OCIE1B | TOIE1 | OCIE0 | TOIE0 */
    TIMSK &= 0xfc;    /* 1111 1100 */
    TIMSK |= (0 << OCIE0)  /* Timer/Counter0 Output Compare Match                                                                Interrupt Disable */
      | (0 << TOIE0);       /* Timer/Counter0 Overflow Interrupt Disable */ 


/* Output Compare Register 0 */
    OCR0 = 0xfa;      /* 1ms / (64 / 16MHz) = 250 = 0xfa */

  /* Timer/Counter0 */
  /* Initialize Counter */
  TCNT0 = 0x00;
} 



/*===============================================================================*/
/*=         AVR Timer1을 이용하여 1초 만들기                                                                                           =*/
/*===============================================================================*/
void Timer1_initialize()
{
   /* Timer/Counter Control Register A */ 
   /* TCCR1A <= COM1A1 | COM1A0 | COM1B1 | COM1B0 | COM1C1 | COM1C0 | WGM11 | WGM10 */
   TCCR1A = (0 << COM1A1) | (0 << COM1A0)          /* Normal port operation, OC1A/OC1B/OC1C disconnected */
            | (0 << COM1B1) | (0 << COM1B0) 
            | (0 << COM1C1) | (0 << COM1C0)
            | (0 << WGM11 ) | (0 << WGM10 );       /* CTC in Timer/Counter Mode of Operation */
   
   /* Timer/Counter Control Register B */
   /* TCCR1B <= ICNC1 | ICES1 | - | WGM13 | WGM12 | CS12 | CS11 | CS10 */
   TCCR1B = (0 << ICNC1) | (0 << ICES1)            /* no input capture */
            | (0 << WGM13) | (1 << WGM12)          /* CTC in Timer/Counter Mode of Operation */
            | (1 << CS12) | (0 << CS11) | (1 << CS10);   /* 1/1024 Prescaler */


   /* Timer/Counter Control Register C */
   /* TCCR1C <= FOC1A | FOC1B | FOC1C | - | - | - | - */
   TCCR1C = (0 << FOC1A) | (0 << FOC1B) | (0 << FOC1C);

   /* Output Compare Register 1 A */ 
   OCR1AH = 0x3D; /* 1 / (1024 / 16*10^6) = 15625 = 0x3D09 */
   OCR1AL = 0x09;
   //OCR1AH = 0x3D; /* 4 / (1024 / 16*10^6) = 15625 = 0x3D09 */
   //OCR1AL = 0x09;
   

   /* Timer/Counter Interrupt Mask Register */
   /* TIMSK <= OCIE2 | TOIE2 | TICIE1 | OCIE1A | OCIE1B | TOIE1 | OCIE0 | TOIE0 */
   TIMSK |= (0 << TICIE1)                          /* No Input Capture Interrupt */
            | (1 << OCIE1A)                        /* Use Timer/Counter1 Output Compare A Match Interrupt */
            | (0 << OCIE1B)                        /* No Timer/Counter1 Overflow Interrupt */
            | (0 << TOIE1);                        /* No Timer/Counter1 Overflow Interrupt */

   /* Extended Timer/Counter Interrupt Mask Register */
   /* ETIMSK <= - | - | TICIE3 | OCIE3A | OCIE3B | TOIE3 | OCIE3C | OCIE1C */
   ETIMSK |= (0 << OCIE1C);                        /* No Timer/Counter1 Output Compare C Match Interrupt */ 

   /* Timer/Counter1 */
   /* Initialize Counter */
   TCNT1H = 0x00;
   TCNT1L = 0x00;
}

void Timer1_stop()
{
    TIMSK |= (0 << TICIE1)                         /* No Input Capture Interrupt */
            | (0 << OCIE1A)                        /* No Timer/Counter1 Output Compare A Match Interrupt */
            | (0 << OCIE1B)                        /* No Timer/Counter1 Overflow Interrupt */
            | (0 << TOIE1);                        /* No Timer/Counter1 Overflow Interrupt */
}

void sleepNow()         // here we put the arduino to sleep
{
  
    //set_sleep_mode(SLEEP_MODE_IDLE);   // sleep mode is set here SLEEP_MODE_PWR_DOWN / SLEEP_MODE_IDLE
    set_sleep_mode(SLEEP_MODE_STANDBY);
 
    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin
    //sleep_cpu();
    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
 
    //sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
}


// Sleep 모드로 들어가는 함수 (분 단위)
void GoSleep(int minuate)
{
  boolean onSleep = true;
  
  Serial.println("Going Sleep\r\n");
  digitalWrite(POWER_PIN, LOW);
  delay(1000);

  
 ///sei();

  while(onSleep)
  {
    if(timeCount > minuate * 60)
    {
        sleep_disable();
        Timer1_stop();
        timeCount = 0;
        onSleep = false;
        //cli();
    }
    else
    {
        sleepNow();
    }
  }

  Serial.println("Waking Up\r\n");
  Serial.print("timeCount : ");
  Serial.println(timeCount);
}


int getChan()
{
   int ch = 0;
 
   ch = 8 * digitalRead(31) + 4 * digitalRead(30) + 2 * digitalRead(29) + digitalRead(28) ;

   return ch;
}

int getID()
{
   int upVal = 0;
   int lowVal = 0;
   int id = 0;

   upVal =  8 * digitalRead(35) + 4 * digitalRead(34) + 2 * digitalRead(33) + digitalRead(32) ;
   lowVal =  8 * digitalRead(39) + 4 * digitalRead(38) + 2 * digitalRead(37) + digitalRead(36) ;

   id = upVal * 10 + lowVal;

   return id;
}


void setup()
{
  // 로터리 스위치
  for(int i = 0 ; i < 4 ; i++)
  {
    pinMode(28+i, INPUT);
    pinMode(32+i, INPUT);
    pinMode(36+i, INPUT);
  }
  
  // LED 제어 (OUTPUT으로 초기화)
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_CENTER, OUTPUT);

  // POWER 제어 (OUTPUT으로 초기화)
  pinMode(POWER_PIN, OUTPUT);

  // LED 점검 코드 (1초 켰다가 끔)
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);

  delay(1000);

  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

  // 시리얼 셋팅
  Serial.begin(9600);
  Serial1.begin(9600);

  // 노드 아이디 세팅 (다이얼 스위치로부터 입력 받음)
  nodeID = getID();
  channel = getChan();


#ifdef USE_HC12
  ///////////////////////////////////////////////////////////////
  // HC 12 셋팅
  ///////////////////////////////////////////////////////////////
  String chString = "AT+C00" + String(channel) +"\r\n";
  
  // 초기화 (HC-12)
  pinMode(HC12_SET, OUTPUT);
  digitalWrite(HC12_SET, LOW);  // AT 설정 모드 진입
  delay(200);

  Serial.print(F("AT+DEFAULT\r\n")); // 공장초기화 설정
  delay(200);

  Serial.print(chString); // 채널설정
  delay(200);

  digitalWrite(HC12_SET, HIGH); // AT 설정 모드 빠져나옴
#endif


  // 온습도 조도 센서 초기화
  dht.begin();
  Wire.begin();

  // PH 센서 초기화
  sensorstring.reserve(30);

  // 센서 데이터 초기화
  sensorDataT.dhId = channel;
  sensorDataT.id = nodeID;
  sensorDataT.ph = 0;
  sensorDataT.ec = 0;
  sensorDataT.soil_temp = 0;
  sensorDataT.soil_hum = 0;
  sensorDataT.ill = 0;
  sensorDataT.co2 = 0;
  sensorDataT.wind_dir = 0;
  sensorDataT.wind_vol = 0;
  sensorDataT.rainfall = 0;
  sensorDataT.isBooted = true;

  memset(sensor_data_packet,0x00,sizeof(sensor_data_packet));

  //Serial.println("BitMoss " + String(nodeID) + " Ready.. Contacting Master in " + String(period * MINUATE + 3000) + " ms" );
  delay(3000);
  timeCount = 0;
  Timer1_initialize();
}


void loop()
{
  // Power PIN Select에 신호를 ON하여 센서와 RF에 전원 공급
  digitalWrite(POWER_PIN, HIGH);
  delay(500);

  // HeartBeat for Bitmoss (3초에 한번씩 깜짝이도록)
  if ((unsigned long)(millis() - ledTimer) > HEARTBEAT_TIMER)
  {
    ledTimer = millis();

    if (isCenterLEDOn)
    {
      digitalWrite(LED_CENTER, HIGH);
      isCenterLEDOn = false;
    }
    else
    {
      digitalWrite(LED_CENTER, LOW);
      isCenterLEDOn = true;
    }
  }


  //////////////////////////////////////////////////////////////////////////////////////////
  //
  // 센서 값을 읽어 메시 네트워크로 전송
  //  : 마스터가 해당 ID로 요청
  //  : 요청된 ID 센서노드만 데이터 전송
  //
  //  {"ch":"[CH]","id":"[ID]","req":"1"}
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  if (stringComplete == true)
  {
      String ch = "";
      String id = "";
      String req = "";
      
      getProtocolFromMaster(recivedData, ch, id, req); // 수신된 명령어 추출

    
      // 수신된 ID와 센서 ID 비교
      if (id == String(nodeID) && req == "1")
      {
    
        //Serial.println("It's time! Let's run!");
        periodTimer = millis();
    
        // 센서 값 읽기
        if (COMM_TEST_MODE == OFF)
          readSensorValue();
    
        // HC12 네트워크 연결
        //Serial.println(F("Connecting to the HC-12..."));
        digitalWrite(LED_BLUE, LOW);

        /*
         * JSON 
         * 
         * {"dhid":"00", "id":"00", "temp":"00", "hum":"00", "ill":"00", "co2":"00", "ph":"00", "ec":"00", "soil_temp":"00", "soil_hum":"00", "wind_dir":"00", "id":"rainfall", "isBooted":"1", "sleep":"00"}
         *  
         */
        sendSensorDataToMaster(sensorDataT); // 센서 데이터 전송       
        sensorDataT.isBooted = false;

        digitalWrite(LED_BLUE, HIGH);
    
        // 대기 기간동안 Sleep 진행
        // GoSleep(SLEEP_PERIOD);
      }
      
      stringComplete = false;
  }
}

// 수신된 데이터 분석
int getProtocolFromMaster(String receivedData, String & _ch, String & _id, String & _req)
{
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(receivedData);
    int ret = -1;
    
    //Serial.println("(parsing Jason)");
    
    if (root.success())
    {
      String ch = root["ch"];
      String id = root["id"];
      String req = root["req"];
    
      _ch = ch;
      _id = id;
      _req = req;
    
      //Serial.println("(parsing completed)");

      ret = 1;
    }
    else
    {
      ret = -1;
    }

    return ret;
    
}

// 마스터로 전송할 센서 데이터
int sendSensorDataToMaster(sensor_data_t _data)
{
    StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    
    root["dhId"] = sensorDataT.dhId; //  id, temp, hum, ill, isBooted 까지만 HC 12모듈에서 전송가능
    root["id"] = _data.id;
    root["temp"] = _data.temp;
    root["hum"] = _data.hum;
    root["ill"] = _data.ill;
    root["co2"] = _data.co2;
    
    root["ph"] = _data.ph;
    root["ec"] = _data.ec;
    root["soil_temp"] = _data.soil_temp;
    root["soil_hum"] = _data.soil_hum;
    root["wind_dir"] = _data.wind_dir;
    root["wind_vol"] = _data.wind_vol;
    root["rainfall"] = _data.rainfall;
    
    root["isBooted"] = _data.isBooted;    
//    if(period == 0)
//        root["sleep"] = "";
//    else
//        root["sleep"] = String(period);

    root.printTo(Serial);
    Serial.println("");

    return 1;
}


// 센서 정보를 읽어 들임
void readSensorValue()
{
  // CO2 데이터 읽기
  if (CO2_SENSOR == ON)
  {
    int co2sensor = analogRead(A1);
    int co2valueMap = map(co2sensor, 160, 800, 400, 2000);
    int co2value = constrain(co2valueMap, 400, 2000);
    delay(100);
    sensorDataT.co2 = co2value;
  }

  // 센서 리딩
  ReadSensor(SENSOR_SELECT);


  // 조도 읽기
  uint16_t val = 0;
  BH1750_Init(BH1750address);
  delay(200);

  if (2 == BH1750_Read(BH1750address))
  {
    val = ((buff[0] << 8) | buff[1]) / 1.2;
    // Serial.print(val,DEC); 
    sensorDataT.ill = val;
  }

  delay(250);

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Temperature & Humidity sensor value
  if (isnan(h) || isnan(t)) {
    //Serial.println("Failed to read from DHT using previous data.");
  }
  else {
    sensorDataT.temp = t;
    sensorDataT.hum = h;
  }
}

/* 하위 센서 초기화 및 데이터 읽기 제공 API */

////////////////////////////////////////////
// Light - I2C
void BH1750_Init(int address)
{
  Wire.beginTransmission(address);
  Wire.write(0x10);//1lx reolution 120ms
  Wire.endTransmission();
}


int BH1750_Read(int address)
{
  int i = 0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while (Wire.available())
  {
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission();
  return i;
  // check if returns are valid, if they are NaN (not a number) then something went wrong!
}

////////////////////////////////////////////
// EC (SoftwareSerial)
float transCharToIntEC(char *_buffer, int _start, int _stop)
{
  int _index;
  float result = 0;
  int num = _stop - _start + 1;
  int _temp[num];
  for (_index = _start; _index <= _stop; _index ++)
  {
    _temp[_index - _start] = _buffer[_index] - '0';
    result = 10 * result + _temp[_index - _start];
  }
  return result;
}

float h()                                                                  //Wind Direction
{
  return transCharToIntEC(databuffer, 1, 3) / 10;
}

float ec()                                                                  //Wind Direction
{
  return transCharToIntEC(databuffer, 4, 6) / 100;
}

float t()                                                                  //Wind Direction
{
  return transCharToIntEC(databuffer, 7, 9) / 10;
}

////////////////////////////////////////////
// Weather (SoftwareSerial)
float transCharToIntWeather(char *_buffer, int _start, int _stop)
{
  int _index;
  float result = 0;
  int num = _stop - _start + 1;
  int _temp[num];
  for (_index = _start; _index <= _stop; _index ++)
  {
    _temp[_index - _start] = _buffer[_index] - '0';
    result = 10 * result + _temp[_index - _start];
  }
  return result;
}

int WindDirection()                                                                  //Wind Direction
{
  return transCharToIntWeather(databuffer, 1, 3);
}


float WindSpeedAverage()                                                             //air Speed (1 minute)
{
  return 0.44704 * transCharToIntWeather(databuffer, 5, 7);
}

float RainfallOneHour()                                                              //Rainfall (1 hour)
{
  return transCharToIntWeather(databuffer, 17, 19) * 25.40 * 0.01;
}

/* 현재 사용 센서에 따라 값을 읽기 */
void ReadSensor(int selectSensor)
{
  if (selectSensor == NO_SENSOR)
    return;

  if (selectSensor == EC_SENSOR)
  {
    // Read
    int index;
    for (index = 0; index < 18; index ++) {
      if (Serial1.available()) {
        databuffer[index] = Serial1.read();

        if (databuffer[0] != 'A') {
          index = -1;
        }

      }
      else
      {
        index --;
      }
    }

    // save
    sensorDataT.ec = ec();
    sensorDataT.soil_temp = t();
    sensorDataT.soil_hum = h();

  }
  else if (selectSensor == PH_SENSOR)
  {
    while (Serial1.available() > 0 && sensor_string_complete != true) {                     //if we see that the Atlas Scientific product has sent a character
      char inchar = (char)Serial1.read();              //get the char we just received

      sensorstring += inchar;                           //add the char to the var called sensorstring
      if (inchar == '\r') {                             //if the incoming character is a <CR>
        sensor_string_complete = true;                  //set the flag
      }
    }


    if (sensor_string_complete) {                        //if a string from the Atlas Scientific product has been received in its entirety
      // Serial.println(sensorstring);                     //send that string to the PC's serial monitor
      pH = sensorstring.toFloat();                      //convert the string to a floating point number so it can be evaluated by the Arduino

      /*
        if (pH >= 7.0) {                                  //if the pH is greater than or equal to 7.0
        Serial.println("high");                         //print "high" this is demonstrating that the Arduino is evaluating the pH as a number and not as a string
        }

        if (pH <= 6.999) {                                //if the pH is less than or equal to 6.999
        Serial.println("low");                          //print "low" this is demonstrating that the Arduino is evaluating the pH as a number and not as a string
        }
      */

      sensorstring = "";                                //clear the string:
      sensor_string_complete = false;                    //reset the flag used to tell if we have received a completed string from the Atlas Scientific product
    }

    //save
    sensorDataT.ph = pH;
  }
  else if (selectSensor == WEATHER_STATION)
  {

    // read
    int index;
    for (index = 0; index < 35; index ++)
    {
      if (Serial1.available())
      {
        databuffer[index] = Serial1.read();
        if (databuffer[0] != 'c')
        {
          index = -1;
        }
      }
      else
      {
        index --;
      }
    }

    //save
    sensorDataT.wind_dir = WindDirection();
    sensorDataT.wind_vol = WindSpeedAverage();
    sensorDataT.rainfall = RainfallOneHour();

    /* DEBUG
      Serial.println(sensorDataT.wind_dir);
      Serial.println(sensorDataT.wind_vol);
      Serial.println(sensorDataT.rainfall);
    */

  }
  else if (selectSensor == FRUIT_SENSOR)
  {
    // Not Supported yet,
  }

}

void serialEvent() {
  while (Serial.available()) {
    
    // get the new byte:
    char inChar = (char)Serial.read();
    
    // add it to the inputString:
    inputString += inChar;

    if (inChar == '}') {
      recivedData = inputString;
      inputString = "";
      stringComplete = true;
    }
  }
}

