#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

//connection information
#define SSID "mediaflow_00"
#define PASS "abcdef1234567"
#define DST_IP "192.168.0.77"
#define PORT  "7001"

//led pin
#define LED_B       11
#define LED_G       12
#define LED_R       13
#define MODE_LED     3

//relay pin
#define RELAY1 2
#define RELAY2 4
#define RELAY3 6
#define RELAY4 9

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

//Relay value
#define RELAY_ON_VALUE 850
#define RELAY_OFF_VALUE 10




//relay state check
bool relay[4] = {0, 0, 0, 0};
bool check = false;
int relayPin[4] = {2, 6, 4, 9};
int relayCheckPin[4] = {A0, A1, A3, A6};
bool feedBack[4] = {0, 0, 0, 0};


//JSON
int ptCd;
String controlID;
String dt;
String val;
String resCd = "0000";

//char WLAN_SSID[13];
//char WLAN_PASS[14];
//char ip_address[16];
//char ip_port[5];

//gcc error str -> char array convert
String str_mac = "";
char char_mac[12];
char char_mac_dot[20];

////////////////////count_mac////////
int mac_count = 0;


SoftwareSerial dbgSerial(7, 5); // RX, TX



void setup() {
  // put your setup code here, to run once:
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(relayPin[i], OUTPUT);
  }

  Serial.begin(115200);
  Serial.setTimeout(5000);
  initLED();
  dbgSerial.begin(9600);  //can't be faster than 19200 for softserial
  dbgSerial.println("Bitmoss Debugging");


  //reset
  Serial.println("AT+RST");
  delay(200);

  if (Serial.find("ready"))
  {
    dbgSerial.println("Module is ready");
  }
  else
  {
    dbgSerial.println("Module have no response.");
    setLED('R', LOW);
    while (1)
    {
      setLED('R', HIGH);
      delay(100);
      setLED('R', LOW);
      delay(100);
    }
  }
  delay(1000);

  String inChar = "";

  // AT Command + get mac address
  String cmd = "AT+CIPAPMAC?";
  Serial.println(cmd);
  delay(200);

  // read mac address
  if (Serial.available()) {
    inChar = Serial.readString();
  }

  //  delay(1000);

  // mac address write EEPROM
  for (int i = 28; i < 45; i++) {
    char_mac_dot[i - 28] = inChar[i];
  }

  for (int i = 0; i < 19; i++) {
    if (i % 3 == 2) {

    }
    else {
      char_mac[mac_count] = char_mac_dot[i];
      mac_count++;
    }
  }


  dbgSerial.println("=====");
  dbgSerial.println(char_mac);
  delay(1000);


  boolean connected = false;
  for (int i = 0; i < 5; i++)
  {
    if (connectWiFi())
    {
      connected = true;
      break;
    }
  }


  if (!connected) {
    while (1)
    {
      setLED('B', HIGH);
      digitalWrite(MODE_LED, LOW);
      delay(500);
      digitalWrite(MODE_LED, HIGH);
      delay(500);
    }
  }


  digitalWrite(MODE_LED, HIGH);
  delay(5000);


  //set the single connection mode
  Serial.println("AT+CIPMUX=0");
  delay(1000);



  Serial.println("AT+CIFSR");

  if (Serial.find("OK"))
  {
    dbgSerial.println("Got IP");
  }
  else
  {
    dbgSerial.println("fail to get ip");
    digitalWrite(LED_R, LOW);
    while (1)
    {
      digitalWrite(MODE_LED, LOW);
      delay(100);
      digitalWrite(MODE_LED, HIGH);
      delay(100);
    }
  }
  delay(500);

  digitalWrite(MODE_LED, HIGH);

  cmd = "AT+CIPSTART=\"TCP\",\"";
  cmd += DST_IP;
  //  cmd += "\",9001";
  cmd += "\",";
  cmd += PORT;


  Serial.println(cmd);
  dbgSerial.println(cmd);

  if (Serial.find("OK")) {
    dbgSerial.println("ok!!!");
  }
  setLED('B', HIGH);

  connection_msg(CONNECTION);

  setLED('B', LOW);

}




void loop() {
//  int ii = 0;
  initLED();

  if (Serial.available() > 0)
  {
    String recivedData = Serial.readString();
    recivedData.remove(0, 10);
    //    dbgSerial.println(recivedData);
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(recivedData);

    if (root.success())
    {
      int _ptCd = root["ptCd"];
      String _controlID = root["id"];
      String _dt = root["dt"];
      String _val = root["val"];

      ptCd = _ptCd;
      controlID = _controlID;
      dt = _dt;
      val = _val;

      for (int i = 0; i < 4; i++)
      {
        relay[i] = (val[i] == '1' ? true : false);
      }
      check = true;
    }
    else {
      dbgSerial.println("parsing error");
      dbgSerial.println(recivedData);
    }
  }


  if (check == true) {
    for (int i = 0; i < 4; i++) {
      digitalWrite(relayPin[i], relay[i]);
    }
    feedback();
    tcpSend();
    check = false;
  }


  //  heartbeatTimer
  if (millis() - heartbeatTimer > HEARTBEAT_TIMER)
  {
    connection_msg(HEARTBEAT);
    heartbeatTimer = millis();
  }


}


void feedback()
{
  bool flag = false;
  delay(3000);
  for (int i = 0; i < 4; i++) {
    if (analogRead(relayCheckPin[i]) > RELAY_ON_VALUE) {
      feedBack[i] = true;
    }
    else if (analogRead(relayCheckPin[i]) < RELAY_OFF_VALUE) {
      feedBack[i] = false;
    }
  }

  for (int i = 0; i < 4; i++) {
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


void tcpSend() {
  String msg = "";
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  root["ptCd"] = "08";
  root["id"] = char_mac;
  root["dt"] = dt;
  root["val"] = val;
  root["resCd"] = resCd;

  root.printTo(msg);
  
  dbgSerial.println(msg);
  sendWIFIData(msg + "\r\n", sizeof(msg));

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


boolean connectWiFi()
{
  Serial.println("AT+CWMODE=3");

  String cmd = "AT+CWJAP=\"";
  cmd += SSID;
  cmd += "\",\"";
  cmd += PASS;
  cmd += "\"";

  //cmd = "AT+CWJAP=\"mediaflow_00\",\"abcdef1234567\"";

  dbgSerial.println(cmd);
  Serial.println(cmd);
  delay(2000);

  if (Serial.find("OK"))
  {
    dbgSerial.println("OK, Connected to WiFi.");
    return true;
  } else {
    dbgSerial.println("Can not connect to the WiFi.");
    return false;
  }
}


boolean sendWIFIData(String data, int len)
{
  Serial.print("AT+CIPSEND=");
  Serial.println(data.length());
  if (Serial.find(">"))
  {
    dbgSerial.print(">");
  }


  Serial.print(data);
  delay(300);

  return true;
}


void connection_msg(int type) {

  String msg = "";
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  if (type == 1)   //connection_msg
  {
    root["ptCd"] = "01";
    root["dtCd"] = "03";
    root["id"] = char_mac;

  }
  else if (type == 2)   //heartbeat
  {
    root["ptCd"] = "02";
    root["dtCd"] = "03";
    root["id"] = char_mac;
  }

  root.printTo(msg);
  sendWIFIData(msg + "\r\n", sizeof(msg));
  dbgSerial.println(msg);
}
