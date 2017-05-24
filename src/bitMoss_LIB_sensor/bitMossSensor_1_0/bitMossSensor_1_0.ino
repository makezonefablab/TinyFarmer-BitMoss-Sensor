/*
 * 
 *  Project : Tiny Farmer 
 *  SubProject : temperature and humidity sensor with Bitmoss Sensor
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
 *  modification (2016.12.26)
 *  - 3 user-defined sensors added 
 * 
 */
#include <ArduinoJson.h>
#include "BTM_BitMoss.h"
#include <DHT.h>


#define COMM_TEST_MODE     OFF

#define HEARTBEAT_TIMER    3000  /* HEARTBEAT 갱신 주기 */




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



#define LED_BLUE           16               /* 상황 LED */
#define LED_RED            8
#define LED_CENTER         3
#define LED_PWR            13               /* 상황 LED */
#define LED_COMM           14
#define LED_ACTIVE         15

#define POWER_PIN          12               /* 센서 전력 차단을 위한 핀 Select */

#define setPIO             40               /* for shield : 7 , BitmossSensor : 40  */




#define DHTPIN 4        // DHT pin number, 온습도 센서 핀 번호
#define DHTTYPE DHT22   // DHT Sensor Type, 온습도 센서 타입

DHT dht(DHTPIN, DHTTYPE); 

#define NODE_ID  57
#define CHANNEL  7

// ****** 1. define varialbes
BTM_PROTOCOL _proto;
BTM_COMMUNICATION _comm(HC12M);

String  inputString = "";          // a string to hold incoming data
boolean stringComplete = false;    // whether the string is complete
String  receivedData = "";
unsigned long ledTimer = 0;                 /* 하트비트 LED를 위한 타이머 변수 */
unsigned long periodTimer = 0;              /* 주기 계산을 위한 타이머 변수 */

int nodeID = -1;                            /* 메시 네트워크 Node 아이디, 스위치로부터 해당 값을 새로 설정 함 */
int channel = 1;                            /* 기본 채널 1 */


String sensorstring = "";                   /* 센서 데이터 저장을 위한 버퍼 */
boolean sensor_string_complete = false;     /* 센서로부터 데이터 전부 수신 했는지 여부 값 */
float pH;                                   /* PH 획득 값 변수 */
String HC12Data = "";


bool isCenterLEDOn = false;                 /* 하트 비트 LED ON/OFF 여부 */



int BH1750address = 0x23;                   /* 조도 I2C 주소 */
byte buff[2];                               /* 조도 읽기 위한 버퍼 */
char databuffer[35];                        /* 시리얼 데이터 읽기 위한 버퍼 */




//
//struct sensor_data_t {           /* 센서 데이터 Struct: 메시 네트워크 내부에서만 사용 */
//  int dhId;                      // 채널번호 , 1byte
//  int id;                        // 1byte
//  float temp;                    // 1byte.1byte
//  float hum;                     // 1byte.1byte
//  long ill;                      // 3byte
//  int co2;                       // 2byte
//  float ph;                      // 2byte
//  float ec;                      // 2byte
//  float soil_temp;               // 1byte.1byte
//  float soil_hum;                // 1byte.1byte
//  int wind_dir;                  // 2byte
//  float wind_vol;                // 2byte
//  float rainfall;                // 2byte
//  String ct1;                    // 
//  String ct2;                    
//  String ct3;
//  bool isBooted;
//};
//
//sensor_data_t sensorDataT;       /* 센서 데이터 전송을 위한 센서 데이터 Struct 선언 */




// the setup function runs once when you press reset or power the board
void setup() {

  // initialize digital pin LED_BUILTIN as an output.
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
  pinMode(LED_PWR, OUTPUT);
  pinMode(LED_COMM, OUTPUT);
  pinMode(LED_ACTIVE, OUTPUT);
  

  // POWER 제어 (OUTPUT으로 초기화)
  pinMode(POWER_PIN, OUTPUT);

  
  // LED 점검 코드 (1초 켰다가 끔)
  digitalWrite(LED_ACTIVE, LOW);
  digitalWrite(LED_COMM, LOW);

  delay(1000);

  digitalWrite(LED_ACTIVE, HIGH);
  digitalWrite(LED_COMM, HIGH);
  digitalWrite(LED_PWR, HIGH);

  // 시리얼 셋팅
  Serial.begin(9600);
  Serial1.begin(9600);

  // 노드 아이디 세팅 (다이얼 스위치로부터 입력 받음)
  nodeID = getID();
  channel = getChan();


  // 온습도 조도 센서 초기화
  dht.begin();
  Wire.begin();

  // PH 센서 초기화
  sensorstring.reserve(30);




  // 1.1  Serial & HC12 setup, 시리얼 통신 및 HC12 모듈 설정
  _comm.Init(setPIO, 9600, CHANNEL,NODE_ID, &Serial);

  // 2 ****** sensor value clear 
  _proto.clearValues(); // all clear stored data in memory

  // 2.1 ****** choose sensors
  /*
   *  sTEMP      for temperature sensor
      sHUMI      for humidity sensor
      sCO2       for co2 sensor
      sILL       for illumination sensor
      sPH        for PH sensor 
      sEC        for EC sensor 
      sSOIL_TEMP for soil temperature sensor
      sSOIL_HUMI for soil humidity sensor
      sWIND_DIR  for wind direction sensor
      sWIND_VOL  for wind volume sensor
      sRAINFALL  for rainfall sensor
      sCUSTOM1   for custom sensor sensor
      sCUSTOM2   for custom sensor sensor
      sCUSTOM3   for custom sensor sensor
      sCUSTOM4   for custom sensor sensor
   * 
   */
  _proto.setSensorKind(sTEMP); // use Temperature sensor, 사용할 센서 종류 할당
  _proto.setSensorKind(sHUMI); // use Humidity sensor, 사용할 센서 종류 할당
  _proto.setSensorKind(sILL); // use Humidity sensor, 사용할 센서 종류 할당
  _proto.setSensorKind(sCO2); // use Humidity sensor, 사용할 센서 종류 할당


  digitalWrite(LED_PWR,LOW);
}

// the loop function runs over and over again forever
void loop() {


  // Power PIN Select에 신호를 ON하여 센서와 RF에 전원 공급
  digitalWrite(POWER_PIN, HIGH);
  delay(500);

  // HeartBeat for Bitmoss (3초에 한번씩 깜짝이도록)
  if ((unsigned long)(millis() - ledTimer) > HEARTBEAT_TIMER)
  {
    ledTimer = millis();

    if (isCenterLEDOn)
    {
      digitalWrite(LED_ACTIVE, HIGH);
      isCenterLEDOn = false;
    }
    else
    {
      digitalWrite(LED_ACTIVE, LOW);
      isCenterLEDOn = true;
    }
  }
  
  //////////////////////////////////////////////////////////////////////////////////////////
  //
  // 센서 값을 읽어 메시 네트워크로 전송
  //  : 마스터가 해당 ID로 요청
  //  : 요청된 ID 센서노드만 데이터 전송
  //
  //  {"ch":"7","id":"57","req":"1"}
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  
  if (stringComplete == true)
  { 
      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& root = jsonBuffer.parseObject(receivedData);

      String ch = "";
      String id = "";
      String req = "";


      // Received Json Format
      // {"ch":"5","id":"5","req":"1"}
      
      if (root.success())
      {
        String _ch = root["ch"];
        String _id = root["id"];
        String _req = root["req"];

        ch = _ch;
        id = _id;
        req = _req;

        digitalWrite(LED_COMM,LOW);
        delay(300);
        digitalWrite(LED_COMM,HIGH);

      }

      // compare the recived ID and the node's ID, 수신된 ID와 센서 ID 비교
      if (id == String(NODE_ID) && req == "1")
      {

        periodTimer = millis();



        // 센서 값 읽기
        if (COMM_TEST_MODE == OFF)
          readSensorValue();
    
  
        // ******  4. send the gathered data
        String _data;
        
        _data = _proto.getServerPacketJSON();//_data

        Serial.println(_data);

        // 데이터 전송 후 표시 
        digitalWrite(LED_COMM, LOW);
        delay(1000);
        digitalWrite(LED_COMM, HIGH);

      }

      stringComplete =  false;
  
  }
  

  delay(1000);                       // wait for a second
}



// 센서 정보를 읽어 들임
void readSensorValue()
{
  
  // ******  3. assign Sensor value
  _proto.setNodeID(NODE_ID);

        
  // CO2 데이터 읽기
  if (CO2_SENSOR == ON)
  {
    int co2sensor = analogRead(A1);
    int co2valueMap = map(co2sensor, 160, 800, 400, 2000);
    int co2value = constrain(co2valueMap, 400, 2000);
    delay(100);
    // ******  3. assign Sensor value
    _proto.setSensorKind(sCO2); // use Humidity sensor, 사용할 센서 종류 할당
    _proto.setCO2(co2value);
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
    
    // ******  3. assign Sensor value
    _proto.setSensorKind(sILL); // use Humidity sensor, 사용할 센서 종류 할당
    _proto.setIllum(val);
  }

  delay(250);

  
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Temperature & Humidity sensor value
  if (isnan(h) || isnan(t)) {
    //Serial.println("Failed to read from DHT using previous data.");
  }
  else {
    // ******  3. assign Sensor value
    _proto.setSensorKind(sTEMP); // use Temperature sensor, 사용할 센서 종류 할당
    _proto.setSensorKind(sHUMI); // use Humidity sensor, 사용할 센서 종류 할당
    _proto.setTemp(t);
    _proto.setHumidity(h);
  }
}



// *********** Don't edit below codes *********** 
void serialEvent() {
  while (Serial.available()) {
    
    // get the new byte:
    char inChar = (char)Serial.read();
    
    // add it to the inputString:
    inputString += inChar;

    if (inChar == '}') {
      receivedData = inputString;
      inputString = "";
      stringComplete = true;
    }
  }
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
    _proto.setSensorKind(sEC); // use Humidity sensor
    _proto.setCO2(ec());

    _proto.setSensorKind(sSOIL_TEMP); // use Humidity sensor
    _proto.setSoilTemp(t());

    _proto.setSensorKind(sSOIL_HUMI); // use Humidity sensor
    _proto.setSoilHumidity(h());

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
    _proto.setSensorKind(sPH); // use Humidity sensor
    _proto.setPH(pH);
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
    _proto.setSensorKind(sWIND_DIR); // use Humidity sensor
    _proto.setWindDir(WindDirection());

    _proto.setSensorKind(sWIND_VOL); // use Humidity sensor
    _proto.setWindVolume(WindSpeedAverage());

    _proto.setSensorKind(sRAINFALL); // use Humidity sensor
    _proto.setRainFall(RainfallOneHour());

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

