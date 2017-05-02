#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

#define DEVICE_ID    3

//led pin
#define LED_B       11
#define LED_G       12
#define LED_R       13
#define MODE_LED     3

//relay pin
#define NUMBER_RELAY 4
#define RELAY1 3
#define RELAY2 8
#define RELAY3 3
#define RELAY4 8

//relay feedback pin
#define RELAY1FEEDBACK A0
#define RELAY2FEEDBACK A1
#define RELAY3FEEDBACK A3
#define RELAY4FEEDBACK A6

//heartbeat select
#define CONNECTION 1
#define HEARTBEAT 2

//heartbeat setting
#define HEARTBEAT_TIMER 30000
uint32_t heartbeatTimer;

//operation setting
#define OPERAT_TIMER 60000
uint32_t operationTimer;

//Relay value
#define RELAY_ON_VALUE 850
#define RELAY_OFF_VALUE 10

int nodeID = DEVICE_ID;                                 /* 메시 네트워크 Node 아이디, 스위치로부터 해당 값을 새로 설정 함 */
int ch[10] = {97, 1, 11, 22, 33, 44, 55, 66, 77, 88};   /* 채널 값 설정 테이블 97번 채널이 Default */
bool isSendFail = false;                                /* 메시 네트워크 전송 실패 여부 확인 Flag */

// 다이얼 스위치 매핑
int dial_table[10] = {0, 340, 511, 650, 613, 730, 767, 833, 682, 783};
int analog_pin[3] = {A6, A7, A3};

//relay state check
bool relay[4] = {0, 0, 0, 0};
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


// 기타 설정
#define SERIAL_SPEED                  9600 //115200 /* 115200 /* 시리얼 속도 */
#define JSON_BUFFER_SIZE              300 /* JSON 버퍼 사이즈 */
#define NUM_OF_RETRY                  5 /* 주기 정보 전송 재시도 횟수 */
#define DELAY_BETWEEN_MESSAGE         100 /* 메세지 사이의 최소 시간 */

////////////////////count_mac////////
int mac_count = 0;


SoftwareSerial dbgSerial(7, 6); // RX, TX
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
String recivedData = "";

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  for (int i = 0; i < NUMBER_RELAY; i++) {
    pinMode(relayPin[i], OUTPUT);
    digitalWrite(relayPin[i], LOW);
    relay[i] = 0;
    val[i] = false;
  }

  // 노드 아이디 세팅 (다이얼 스위치로부터 입력 받음)
  nodeID = 77;//return_dgree(analogRead(analog_pin[0])) * 10 + return_dgree(analogRead(analog_pin[1]));

  Serial.begin(9600);
  Serial.setTimeout(5000);
  initLED();
  dbgSerial.begin(9600);  //can't be faster than 19200 for softserial
  dbgSerial.print("node ID : ");
  dbgSerial.println(nodeID);
  dbgSerial.println("Bitmoss Debugging");

  delay(1000);

  //digitalWrite(MODE_LED, HIGH);
  //delay(5000);

}


void loop() {

  initLED();

  if (stringComplete == true)
  {
    dbgSerial.println("parsing recivedData");
    
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(recivedData);
    
    dbgSerial.println("parsing Jason");
    
    if (root.success())
    {
      String _ptCd = root["ptCd"];
      String _relayHistoryId = root["relayHistoryId"];
      String _id = root["id"];
      String _operTime = root["operTime"];
      String _val = root["val"];
      String _seq = root["seq"];

      dbgSerial.println("parsing completed");
      
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
       *  
       *  } 
       */
      if(_ptCd == "07" && _id == String(nodeID))
      {
//        for (int i = 0; i < NUMBER_RELAY; i++)
//        {
//          //relay[i] = (val[i] == '1' ? true : false);
//        }

        sequenceID = _seq.toInt();
        if(sequenceID > 0 && sequenceID <= 4)
        {
          sequenceID = sequenceID - 1;
          relay[sequenceID] = (val[sequenceID] == '1' ? true : false);
          check = true;
        }
        
      }
      
    }
    else {
      dbgSerial.println("parsing error");
      dbgSerial.println(recivedData);
      feedback_error(99);
    }
    recivedData = "";
    stringComplete = false;
  }


  if (check == true) {
//    for (int i = 0; i < NUMBER_RELAY; i++) {
//      digitalWrite(relayPin[i], relay[i]);
//    }
    digitalWrite(relayPin[sequenceID], relay[sequenceID]);
    dbgSerial.println("digitalWrite OK");
    feedback();
    tcpSend();
    check = false;
    
  }



  //  operationTimer
  if (millis() - operationTimer > (float)(operTime.toInt()*60000))
  {
    //connection_msg(HEARTBEAT);
    relay[sequenceID] = false;
    digitalWrite(relayPin[sequenceID], relay[sequenceID]);
    operationTimer = millis();
  }




  //  heartbeatTimer
  if (millis() - heartbeatTimer > HEARTBEAT_TIMER)
  {
    //connection_msg(HEARTBEAT);
    heartbeatTimer = millis();
    StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    
    String _status ="";
    
    for (int i = 0; i < NUMBER_RELAY; i++)
    {
      _status += String(relay[i]);
    }

    root["ptCd"] = "02";
    root["dtCd"] = "03";
    root["id"] = String(nodeID);
    root["status"] = _status;
    root.printTo(Serial);
    Serial.println("");
    heartbeatTimer = millis();
  }


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

  for (int i = 0; i < NUMBER_RELAY; i++) {
    if (relay[i] != feedBack[i]) {
      digitalWrite(relayPin[i], LOW);
      resCd[i] = '1';
      // Error message send
      dbgSerial.print("error : ");
      dbgSerial.println(i);

      flag = true;
    }
    else {
      resCd[i] = '0';
    }
  }

  if (flag == false) {
    resCd = "0000";
  }

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

  root.printTo(msg);
  
  dbgSerial.println(msg);
  
  Serial.println(msg);

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

