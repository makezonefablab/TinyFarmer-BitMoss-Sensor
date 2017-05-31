# TinyFarmer-BitMoss-Sensor

센서값을 모아 전달하는 센서노드 (Sensor Node)
==========================

![TinyFarmer_Bitmoss_Sensor Intro](https://github.com/makezonefablab/TinyFarmer-BitMoss-Sensor/blob/master/img/bitmossSensor1.png)  

[*타이니파머 비트모스 센서 - 사이트 바로가기*](http://106.240.234.10/mediafarmHome/?page_id=14036)

[*타이니파머 개발관련 카페 - 사이트 바로가기*](http://cafe.naver.com/makezone#)



#### 수정내용
> 라이브러리 버젼관리 시작 1.5버전 

> Init(setIO, 9600, CHANNEL,NODE_ID, &Serial) 함수에 설정 핀 추가(setIO) (2017.03.06)

> 비트모스 전용 라이브러리 시작 "libraries/BTM_BitMoss" (2017.01.11)

> RFM69 통신 모듈 적용 - BTM_COMMUNICATION 클래스 (2017.01.17)

> DHT22 온습도 센서 적용 예제  (2017.01.24)

> 조도 센서 적용 예제  (2017.01.24)



 
 --------------
 소개
 --------------
농작물의 환경데이터를 수집하는 비트모스(BitMoss TM)는 다양한 센서와 연결하고 부가 기능의 액세서리 보드를 연결하여 확장 가능한 구성으로 이루어졌습니다.
전용 AP (TinyFarmer AP)와의 연결로 쉽게 데이터를 모아 중앙 센터로 전송합니다.

 
 
 
 --------------
 설치
 --------------
비트모스센서는 반드시 타이니파머 허브와 연동이 되어 사용되어집니다.


[타이니파머 허브와 연동되어 사용해야 합니다. - 사이트 바로가기]
(https://github.com/makezonefablab/TinyFarmer-HUB)


소스 업로드를 위해서는 비트모스 쉴드 라이브러리를 반드시 포함하여야 합니다. 해당 라이브러리는 아래의 주소에서 다운받아 설치합니다.



#### BitMoss shield 라이브러리

> https://github.com/makezonefablab/TinyFarmer-BitMoss-Sensor/


소스를 다운받아 다운로더 장치로 다운받는다. 다운로더는 USB2SERIAL장치 중 대부분과 호환이 되며, 특히, DFRobot의 USB Serial Light(https://www.dfrobot.com/product-581.html) 와 100% 호환된다. 
다운로드 방법은 Arduino IDE 환경에서 가능하다.




준비물
--------------



