#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <JeeLib.h>             /* Sleep 함수를 위한 LIB */

#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "DHT.h"
#include <SoftwareSerial.h>

#define DEFUALT_PERIOD     0     /* 최초 시작 이후 첫번째 데이터 취득 대기 시간 (분) */
#define MINUATE            60000  /* 1분의 기준 ms */

#define NUM_RETRY          50     /* 주기 데이터 재시도 횟수 */
#define RETRY_DELAY        100    /* 재시도 사이의 대기 시간 */

#define DHTPIN             4      /* 온습도 센서 PIN 번호 */
#define DHTTYPE            DHT22  /* 온습도 센서 종류 */
DHT dht(DHTPIN, DHTTYPE);

#define RX                 6      /* RX 핀 번호 */
#define TX                 5      /* TX 핀 번호 */
SoftwareSerial mySerial(RX, TX);

#define ON                 1      /* 센서 연결 ON */
#define OFF                0      /* 센서 연결 OFF */

#define NO_SENSOR          0      /* NONE Connected  */
#define PH_SENSOR          1      /* PH Sensor */
#define WEATHER_STATION    2      /* Weather Station Sensor */
#define FRUIT_SENSOR       3      /* Fruit Sensor */
#define EC_SENSOR          4      /* EC Sensor */

#define SENSOR_SELECT      NO_SENSOR        /* 위에서 센서 종류 선택 or NO_SENSOR */
#define CO2_SENSOR         OFF              /* ON or OFF */

#define RESET_LIMIT 3                       /* 소프트 리셋하기전에 시도 횟수 */

#define LED_BLUE 16                         /* 상황 LED */
#define LED_RED 8
#define LED_CENTER 3

#define POWER_PIN 7                         /* 센서 전력 차단을 위한 핀 Select */

bool isCenterLEDOn = false;                 /* 하트 비트 LED ON/OFF 여부 */

// 전역 변수
char databuffer[35];                        /* 시리얼 데이터 읽기 위한 버퍼 */

int BH1750address = 0x23;                   /* 조도 I2C 주소 */
byte buff[2];                               /* 조도 읽기 위한 버퍼 */

String sensorstring = "";                   /* 센서 데이터 저장을 위한 버퍼 */
boolean sensor_string_complete = false;     /* 센서로부터 데이터 전부 수신 했는지 여부 값 */
float pH;                                   /* PH 획득 값 변수 */

// RF 관련 초기화
RF24 radio(9, 10);                                      /* 비트모스 Digital 9, 10 핀 사용 */
RF24Network network(radio);
RF24Mesh mesh(radio, network);

int nodeID = -1;                                        /* 메시 네트워크 Node 아이디, 스위치로부터 해당 값을 새로 설정 함 */
int ch[10] = {97, 1, 11, 22, 33, 44, 55, 66, 77, 88};   /* 채널 값 설정 테이블 97번 채널이 Default */
bool isSendFail = false;                                /* 메시 네트워크 전송 실패 여부 확인 Flag */

// 다이얼 스위치 매핑
int dial_table[10] = {0, 340, 511, 650, 613, 730, 767, 833, 682, 783};
int analog_pin[3] = {A6, A7, A3};

unsigned long periodTimer = 0;                          /* 주기 계산을 위한 타이머 변수 */
unsigned long ledTimer = 0;                             /* 하트비트 LED를 위한 타이머 변수 */

int period = DEFUALT_PERIOD;
int meshConnectFail = 1;

struct sensor_data_t {                                  /* 센서 데이터 Struct: 메시 네트워크 내부에서만 사용 */
  int id;
  float temp;
  float hum;
  long ill;
  int co2;
  float ph;
  float ec;
  float soil_temp;
  float soil_hum;
  int wind_dir;
  float wind_vol;
  float rainfall;
  bool isBooted;
  String _sleep;
};

sensor_data_t sensorDataT;                            /* 센서 데이터 전송을 위한 센서 데이터 Struct 선언 */

// S/W 리셋 함수
void(* resetFunc) (void) = 0;//declare reset function at address 0

ISR(WDT_vect) {
  Sleepy::watchdogEvent();  // Setup the watchdog
}

void setup()
{

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
  Serial.begin(115200);
  mySerial.begin(9600);

  // 노드 아이디 세팅 (다이얼 스위치로부터 입력 받음)
  nodeID = return_dgree(analogRead(analog_pin[0])) * 10 + return_dgree(analogRead(analog_pin[1]));

  // 온습도 조도 센서 초기화
  dht.begin();
  Wire.begin();

  // PH 센서 초기화
  sensorstring.reserve(30);

  // 센서 데이터 초기화
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
  if(period == 0)
        sensorDataT._sleep = "";
    else
        sensorDataT._sleep = String(period);
  

  Serial.println("BitMoss " + String(nodeID) + " Ready.. Contacting Master in " + String(period * MINUATE + 3000) + " ms" );
  delay(3000);
}

// Sleep 모드로 들어가는 함수 (분 단위)
void GoSleep(int minuate)
{
  Serial.println("Going Sleep\r\n");
  digitalWrite(POWER_PIN, LOW);
  delay(1000);

  for (byte i = 0; i < minuate; ++i)
    Sleepy::loseSomeTime(60000);

  Serial.println("Waking Up\r\n");
}

void loop()
{
  // Power PIN Select에 신호를 ON하여 센서와 RF에 전원 공급
  digitalWrite(POWER_PIN, HIGH);
  delay(1000);

  // HeartBeat for Bitmoss (3초에 한번씩 깜짝이도록)
  if ((unsigned long)(millis() - ledTimer) > 3000)
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

  // 센서 값을 읽어 메시 네트워크로 전송
  if ((unsigned long)(millis() - periodTimer) > period * MINUATE || isSendFail == true )
  {
    // 송신 실패 여부 초기화
    isSendFail = false;

    Serial.println("It's time! Let's run!");
    periodTimer = millis();

    // 센서 값 읽기
    readSensorValue();

    // 메시 네트워크 연결
    Serial.println(F("Connecting to the mesh..."));
    digitalWrite(LED_BLUE, LOW);

    mesh.setNodeID(nodeID);

    // 다이얼로부터 채널 정보를 가져와서 메시 네트워크 시작함
    if (!mesh.begin(ch[return_dgree(analogRead(analog_pin[2]))], RF24_250KBPS))
    {
      Serial.println(F("Mesh Begin Failed"));

      // 메쉬 네트워크로부터 연결 해제 및 주소 Release 요청
      int result = mesh.releaseAddress();

      if (result == 1)
        Serial.println("Release OK:\r\n");
      else
        Serial.println("Release FAILED:\r\n");

      isSendFail = true;

      if (meshConnectFail >= RESET_LIMIT)
      {
        delay(3000);
        resetFunc(); // call reset
      }
      else
      {
        digitalWrite(LED_RED, LOW);
        meshConnectFail++;
      }

      return;
    }

    // 실패 LED OFF
    digitalWrite(LED_RED, HIGH);

    mesh.update();

    int retry = NUM_RETRY;

    // 메시 네트워크 데이터 전송
    bool writeResult = mesh.write(&sensorDataT, 'M', sizeof(sensorDataT));

    while (!writeResult && retry >= 0)
    {
      digitalWrite(LED_RED, LOW);
      Serial.println("+");
      delay(RETRY_DELAY);
      mesh.update();
      writeResult = mesh.write(&sensorDataT, 'M', sizeof(sensorDataT));
      retry -= 1;

      if (retry == 0)
      {
        Serial.println("");
        Serial.println("Mesh.write() failed! Let's retry...");
        mesh.releaseAddress();
        isSendFail = true;
        return;
      }
    }
    digitalWrite(LED_RED, HIGH);


    // 데이터 전송 성공 했을 시에 전송 내용 시리얼 로깅
    if (writeResult)
    {
      Serial.println("");
      Serial.println("--- SENT Succeed ----------------------");
      Serial.println("ID : " + String(sensorDataT.id));
      Serial.println("Temp : " + String(sensorDataT.temp));
      Serial.println("Hum : " + String(sensorDataT.hum));
      Serial.println("Light : " + String(sensorDataT.ill));
      Serial.println("Co2 : " + String(sensorDataT.co2));
      Serial.println("PH : " + String(sensorDataT.ph ));
      Serial.println("EC : " + String(sensorDataT.ec ));
      Serial.println("Soil_Temp : " + String(sensorDataT.soil_temp));
      Serial.println("Soil_Hum : " + String(sensorDataT.soil_hum));
      Serial.println("Wind_Dir : " + String(sensorDataT.wind_dir));
      Serial.println("Wind_Vol : " + String(sensorDataT.wind_vol));
      Serial.println("RainFall : " + String(sensorDataT.rainfall));
    }

    Serial.println("--- Waiting Period Info From Master -------");

    sensorDataT.isBooted = false;
    meshConnectFail = 1;

    // Master 노드로 부터 Period 정보를 대기 함
    retry = NUM_RETRY;

    while (!network.available() && retry >= 0)
    {
      mesh.update();
      delay(RETRY_DELAY);
      retry -= 1;
      Serial.print(".");

      // 최종적으로 마스터로 부터 답이 없을 때
      if (retry == 0)
      {
        Serial.println("");
        Serial.println("No response from Master: Period not received");
      }
    }

    // 수신 받은 Period 값을 저장 하고 해당 내용을 시리얼 로깅
    while (network.available())
    {
      RF24NetworkHeader header;
      network.peek(header);
      switch (header.type) {
        case 'P':
          network.read(header, &period, sizeof(int));
          Serial.println("");
          Serial.print("Period info received:  ");
          Serial.println(period);
          break;

      }
    }

    // 메쉬 네트워크로부터 연결 해제 및 주소 Release 요청
    int result = mesh.releaseAddress();

    if (result == 1)
      Serial.println("Release OK:\r\n");
    else
      Serial.println("Release FAILED:\r\n");

    digitalWrite(LED_BLUE, HIGH);

    // 대기 기간동안 Sleep 진행
    GoSleep(period);
  }
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
    Serial.println("Failed to read from DHT using previous data.");
  }
  else {
    sensorDataT.temp = t;
    sensorDataT.hum = h;
  }
}

// 다이얼 스위치의 값 변환
int return_dgree(int value) {
  if ( dial_table[1] - 5 < value && dial_table[1] + 5 > value) {
    return 1;
  }
  else if ( dial_table[2] - 5 < value && dial_table[2] + 5 > value) {
    return 2;
  }
  else if ( dial_table[3] - 5 < value && dial_table[3] + 5 > value) {
    return 3;
  }
  else if ( dial_table[4] - 5 < value && dial_table[4] + 5 > value) {
    return 4;
  }
  else if ( dial_table[5] - 5 < value && dial_table[5] + 5 > value) {
    return 5;
  }
  else if ( dial_table[6] - 5 < value && dial_table[6] + 5 > value) {
    return 6;
  }
  else if ( dial_table[7] - 5 < value && dial_table[7] + 5 > value) {
    return 7;
  }
  else if ( dial_table[8] - 5 < value && dial_table[8] + 5 > value) {
    return 8;
  }
  else if ( dial_table[9] - 5 < value && dial_table[9] + 5 > value) {
    return 9;
  }
  else {
    return 0;
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
      if (mySerial.available()) {
        databuffer[index] = mySerial.read();

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
    while (mySerial.available() > 0 && sensor_string_complete != true) {                     //if we see that the Atlas Scientific product has sent a character
      char inchar = (char)mySerial.read();              //get the char we just received

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
      if (mySerial.available())
      {
        databuffer[index] = mySerial.read();
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




