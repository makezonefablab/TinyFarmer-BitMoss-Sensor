#include <SoftwareSerial.h>
#include <ArduinoJson.h>


#define HC12_SET          7

#define SERIAL_SPEED 19200
#define JSON_BUFFER_SIZE              300   /* JSON 버퍼 사이즈 */


int nodeID = 1;                            /* 메시 네트워크 Node 아이디, 스위치로부터 해당 값을 새로 설정 함 */
int channel = 9;                            /* 기본 채널 1 */


String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
String recivedData = "";

struct sensor_data_t {                      /* 센서 데이터 Struct: 메시 네트워크 내부에서만 사용 */
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
};

sensor_data_t sensorDataT;                            /* 센서 데이터 전송을 위한 센서 데이터 Struct 선언 */


void setup()
{

  // 시리얼 셋팅
  Serial.begin(19200);

  String chString = "AT+C00" + String(channel) + "\r\n";


  // 초기화 (HC-12)
  pinMode(HC12_SET, OUTPUT);
  digitalWrite(HC12_SET, LOW);  // AT 설정 모드 진입

  Serial.println(chString);    // channel setting


  delay(100);
  digitalWrite(HC12_SET, HIGH); // AT 설정 모드 빠져나옴
  Serial.println("HC12 Init Done" );


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

  delay(3000);
}


void loop()
{
  // Power PIN Select에 신호를 ON하여 센서와 RF에 전원 공급
  delay(1000);

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  // 센서 값을 읽어 메시 네트워크로 전송
  //  : 마스터가 해당 ID로 요청
  //  : 요청된 ID 센서노드만 데이터 전송
  //
  //  {"ch":"00","id":"00","req":"1"}
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  if (stringComplete == true)
  {
    //Serial.println("parsing recivedData : " + recivedData);

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(recivedData);

    //Serial.println("parsing Jason");

    String ch = "";
    String id = "";
    String req = "";

    // {"ch":"ddd","id":"ddd","req":"2222"}

    if (root.success())
    {
      String _ch = root["ch"];
      String _id = root["id"];
      String _req = root["req"];

      ch = _ch;
      id = _id;
      req = _req;

      //Serial.println("parsing completed");
    }

    // 수신된 ID와 센서 ID 비교
    if (id == String(nodeID) && req == "1")
    {
      readSensorValue();

      /*
         JSON

         {"id":"00", "temp":"00", "hum":"00", "ill":"00", "co2":"00", "ph":"00", "ec":"00", "soil_temp":"00", "soil_hum":"00", "wind_dir":"00", "id":"rainfall", "isBooted":"1", "sleep":"00"}

      */

      StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
      JsonObject& root = jsonBuffer.createObject();
      root["id"] = String(nodeID);
      //root["temp"] = sensorDataT.temp;
      //root["hum"] = sensorDataT.hum;
      //        root["ill"] = sensorDataT.ill;
      root["co2"] = sensorDataT.co2;
      //        root["ph"] = sensorDataT.ph;

      //        root["ec"] = sensorDataT.ec;
      //        root["soil_temp"] = sensorDataT.soil_temp;
      //        root["soil_hum"] = sensorDataT.soil_hum;
      //        root["wind_dir"] = sensorDataT.wind_dir;
      //        root["rainfall"] = sensorDataT.rainfall;
      //        root["isBooted"] = sensorDataT.isBooted;
      //        root["sleep"] = "";
      root.printTo(Serial);

    }
    stringComplete = false;
  }
}

//센서 데이터 읽기
void readSensorValue()
{
    int co2sensor = analogRead(A1);
    int co2valueMap = map(co2sensor, 160, 800, 400, 2000);
    int co2value = constrain(co2valueMap, 400, 2000);
    delay(100);
    sensorDataT.co2 = co2value;
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

