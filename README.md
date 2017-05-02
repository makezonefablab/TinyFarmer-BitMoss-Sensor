# TinyFarmer-BitMoss-Controller
  
  
 ![TinyFarmer-Bitmoss-Controller Intro](https://github.com/makezonefablab/TinyFarmer-BitMoss-Controller/blob/master/img/bitmossController.png)      
 
 [*비트모스 컨트롤러 - 사이트 바로가기*](http://106.240.234.10/mediafarmHome/?page_id=13816)
 
 [*타이니파머 개발관련 카페 - 사이트 바로가기*](http://cafe.naver.com/makezone#)
 
 소개
 --------------
농장도 직접 제어가 가능합니다.
비트모스에 4개의 접점을 통해 기존에 수동으로 관리되던 전기 제어 장치를 자동 또는 원거리 제어가 가능하도록 확장합니다.
부가 기능의 액세서리 보드를 연결하여 확장 가능하므로 어떠한 제어 대상의 장치도 비트모스 하나로 똑똑해질 수 있습니다.

### Latest version 
2017.05.02
```
- 통신보안(security)기능 적용
- 비트모스콘트롤러 프로토콜 변경
- my.tinyfarmer.com과 호환되지 않음 (클라우드 업데이트 후 사용가능)
- LCD현재 상태 표시
- RTC시간 수정 버튼 기능
- 타이니파머 허브 "라즈베리 3 unstable (2017.04.12)" 이상 버전부터 호환

```



설치 
--------------
소스를 다운받아 다운로더 장치로 다운받는다. 다운로더는 USB2SERIAL장치 중 대부분과 호환이 되며, 특히, DFRobot의 [*USB Serial Light*] (https://www.dfrobot.com/product-581.html)와 100% 호환된다. 
다운로드 방법은 Arduino IDE 환경에서 가능하다.

> 1. 소스 다운로드 
```
~ $ cd /home/mediaflow
~ $ wget https://github.com/makezonefablab/TinyFarmer-HUB/tree/master/src/TinyfarmerHubWeb.zip
~ $ wget https://github.com/makezonefablab/TinyFarmer-HUB/tree/master/src/TinyfarmerHub.zip
~ $ tar xvf TinyfarmerHub.zip
~ $ tar xvf TinyfarmerHubWeb.zip
```
> 2.자바 메인모듈 
```
~ $ cd TinyfarmerHub/bin
~ $ sudo chown root:root TinyfarmerHub.sh
~ $ sudo chmod 744 TinyfarmerHub.sh
```
> 3.Web Application GUI 
```
~ $ sudo vi /usr/local/tomcat-8.0.36/conf/Catalina/localhost/ROOT.xml    (아래 XML 내용 추가)
~ $ sudo service tomcat restart
```


~~~ xml
<?xml version='1.0' encoding='utf-8'?>
<Context crossContext="true" path="" docBase="/home/mediaflow/TinyfarmerHubWeb" >
</Context >
~~~




준비물
--------------

![TinyFarmer-HUB App](https://github.com/makezonefablab/TinyFarmer-HUB/blob/master/img/rasp.jpg) 

