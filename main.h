#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <RTOS.h>
#include <HTTPClient.h>
#include <U8g2lib.h>
#include <TimeLib.h>
#include <DHT.h>
#include <DHT_U.h>
#include <string.h>
#include "../lib/picture/picture.h"
#include "../lib/connectNet/connectWifi.h"

#ifndef MAIN_H_
#define MAIN_H_

//Time
void ntpInit();
void showTime(void);
time_t getNtpTime(void);
void sendNTPpacket(IPAddress& address);
//DHT11
void dhtInit(void);
void dhtData(void);
//waterSensor
void sensorInit(void);
void sensorData();
//sr501
void sr501Init(void);
void sr501Data(void);
//GMdiode
void gmInit(void);
void gmData(void);
//U8g2
void u8g2Init(void);
void showWait(void);
void showConnect(void);
void oledClockDisplay(void);
//systeam
void taskInit();
void task1(void * parameter);
void task2(void * parameter);
//*******************************getTime*********************************************//
static WiFiUDP Udp;
static const char ntpServerName[] = "ntp1.aliyun.com"; //NTP server,aliyun
static const int timeZone = 8;   
static const int NTP_PACKET_SIZE = 48;
static byte packetBuffer[NTP_PACKET_SIZE]; // Buffers for input and output packets
time_t prevDisplay = 0;   //当时钟已经显示

void ntpInit(void){
    setSyncProvider(getNtpTime);
    setSyncInterval(30); //每300秒同步一次时间
}

void showTime(void){
    if (timeStatus() != timeNotSet){
        if (now() != prevDisplay){ //时间改变时更新显示
            prevDisplay = now();
            oledClockDisplay();
        }
    }
}

time_t getNtpTime(void)
{
  IPAddress ntpServerIP;          // NTP服务器的地址

  while (Udp.parsePacket() > 0);  // 丢弃以前接收的任何数据包
  Serial.println("Transmit NTP Request");
  // 从池中获取随机服务器
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 3000)
  {
    int size = Udp.parsePacket();
    if(size != 0){
          Serial.println("Parse packet has ready");
    }
    if (size >= NTP_PACKET_SIZE){
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // 将数据包读取到缓冲区
      unsigned long secsSince1900;
      // 将从位置40开始的四个字节转换为长整型，只取前32位整数部分
      secsSince1900 = (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * 3600UL;
    }
  }
  Serial.println("No NTP Response :-("); //无NTP响应
  getNtpTime();

  return 0; //如果未得到时间则返回0
}

void sendNTPpacket(IPAddress& address)
{
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;          // Stratum, or type of clock
  packetBuffer[2] = 6;          // Polling Interval
  packetBuffer[3] = 0xEC;       // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  Udp.beginPacket(address, 123); //NTP需要使用的UDP端口号为123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

//*******************************DHT11(温湿度传感器)*********************************************//
#define dhtPin  33     // Digital pin connected to the DHT sensor 
#define dhtType DHT11     // DHT 11
#define humLevel 85 
int rbaOpen = 0;
DHT_Unified dht(dhtPin, dhtType);
int tem = 0;
int hum = 0;
boolean rbaNature = true;

void dhtInit(void){
    dht.begin();
    Serial.println("DHT11 has ready");
}

void dhtData(void){ // Get temperature event and print its value.
    delay(200);
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        Serial.println(F("Error reading temperature!"));
        tem = 0;}
    else {
        Serial.print(F("Temperature: "));
        tem = event.temperature;
        Serial.print(tem);
        Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        Serial.println(F("Error reading humidity!"));
        hum = 0;}
    else {
        Serial.print(F("Humidity: "));
        hum = event.relative_humidity;
        Serial.print(hum);
        Serial.println(F("%"));
        if(hum < humLevel && rbaNature == true){rbaOpen = true;}
    }
}
//*******************************waterSensor(水位传感器)*********************************************//
#define sensorPin  32       //Set depthSensor input pin to Analog 32.
#define waterLevel 2500
int waterValue = 0;

void sensorInit(){
    pinMode(sensorPin,INPUT);
}

void sensorData(){
    waterValue = analogRead(sensorPin); // Read the sensor values.
    Serial.printf("The water value is ");
    Serial.println(waterValue);
    delay(1000);  // Read current sensor value every three seconds.
}
//*******************************SR501(人体感应器*)*********************************************//
#define sr501Pin 19
#define elePin 18 
uint32_t leaveTime = 0;
boolean someOne = true;

void sr501Init(void){
    pinMode(sr501Pin,INPUT);
    pinMode(elePin,OUTPUT);
}

void sr501Data(void){
    if(digitalRead(sr501Pin) == 1){
        Serial.printf("someone coming\n");
        digitalWrite(elePin,HIGH);
        leaveTime = 0;
        someOne = true;
    }
    else{
        Serial.printf("none coming\n");
        if(someOne == true){
            leaveTime = millis();
            someOne = false;
            Serial.println("Time starts!");
        }
        if(millis() - leaveTime >= 2 * 60 * 1000){
            digitalWrite(elePin,HIGH);
        }
    }
}

//*******************************GMdiode(光敏传感器)*********************************************//
#define gmPin   35
#define lightOFF   0
#define lightHIGH  3
int lightValue  = lightOFF;

void gmInit(void){
    pinMode(gmPin,INPUT);
}

void gmData(){
    if(digitalRead(gmPin) == 0){
        lightValue = 1;
        Serial.println("the light value had set LOW");
    }
}
//*******************************OLED*********************************************//
#define SCL 22
#define SDA 21

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   

void u8g2Init(){
    u8g2.begin();  
    u8g2.enableUTF8Print(); 
    u8g2.setFont(u8g2_font_ncenB08_tr);//设定字体 
    Serial.println("The u8g2 has ready!");
    delay(200);
}

void showWait(){
    u8g2.firstPage();
    do {
        u8g2.drawStr(0,12,"Waitting to connect...");
    } while (u8g2.nextPage());
}

void showConnect(){
    u8g2.firstPage();
    do {
        u8g2.drawStr(0,12,"Connect sussess");
        u8g2.drawStr(0,28,"Local IP:");
        u8g2.drawStr(55,28,WiFi.localIP().toString().c_str());
        u8g2.drawStr(0,44,"Gate IP:");
        u8g2.drawStr(55,44,WiFi.gatewayIP().toString().c_str());
    }while (u8g2.nextPage());
}

void oledClockDisplay()
{
    int years, months, days, hours, minutes, seconds, weekdays;
    years = year();
    months = month();
    days = day();
    hours = hour();
    minutes = minute();
    seconds = second();
    weekdays = weekday();
    Serial.printf("%d/%d/%d %d:%d:%d Weekday:%d\n", years, months, days, hours, minutes, seconds, weekdays);
    
    String currentTime = "";
    if (hours < 10)
        currentTime += 0;
    currentTime += hours;
    
    currentTime += ":";
    if (minutes < 10)
        currentTime += 0;
    currentTime += minutes;

    String currentDay = "";
    currentDay += years;
    currentDay += "/";
    if (months < 10)
        currentDay += 0;
    currentDay += months;
    currentDay += "/";
    if (days < 10)
        currentDay += 0;
    currentDay += days;

    String T = String(tem);
    // String strT = "the temperature is " + T;
    String H = String(hum);
    // String strH = "the humidity is " + H;
    // Serial.println(strT+";"+strH);    

    u8g2.firstPage();
    do {
        u8g2.setFont(u8g2_font_unifont_t_chinese2);
        u8g2.drawXBM(0  ,0  ,16 ,16 ,Wen);
        u8g2.drawXBM(17 ,0  ,8  ,16 ,MAOHAO);
        u8g2.drawXBM(40 ,0  ,8  ,16 ,BaiFenHao);
        u8g2.drawStr(25,14,T.c_str());
        u8g2.drawXBM(64  ,0 ,16 ,16 ,Shi);
        u8g2.drawXBM(81 ,0  ,8  ,16 ,MAOHAO);
        u8g2.drawXBM(104 ,0  ,8  ,16 ,BaiFenHao);
        u8g2.drawStr(89,14,H.c_str());

        u8g2.setFont(u8g2_font_logisoso24_tr);
        u8g2.drawStr(25, 44,currentTime.c_str());

        if(waterValue < waterLevel){
            u8g2.drawXBM(0,48,16,16,SHUI);
            u8g2.drawXBM(16,48,16,16,WEI);
            u8g2.drawXBM(32,48,16,16,GUO);
            u8g2.drawXBM(48,48,16,16,DI);
            u8g2.drawXBM(64,48,16,16,DOUHAO);
            u8g2.drawXBM(80,48,16,16,QING);
            u8g2.drawXBM(96,48,16,16,BU);
            u8g2.drawXBM(112,48,16,16,CHONG);
        }

    } while (u8g2.nextPage());
}
//*******************************System*********************************************//
#define lightCtrlPin 27         //灯触摸引脚
#define lightPin 5              //灯激活引脚
boolean lightTouch = false;     //没触发灯
const int lightFreq = 1000;     //设置频率
const int lightChannel = 9;     //通道号，取值0 ~ 15
const int lightResolution = 8;  //计数位数，取值0 ~ 20

#define rbaCtrlPin   14         //雾化器触摸引脚
#define rbaPin 17               //雾化器激活引脚
boolean rbaTouch = false;       //没触发雾化器
boolean mode = true;

// 任务1
#define TASK1_TASK_PRIO  1      // 任务优先级
#define TASK1_STK_SIZE   1024   // 任务堆栈大小
TaskHandle_t Tasks1_TaskHandle; // 任务句柄
// 任务2
#define TASK2_TASK_PRIO  1      // 任务优先级
#define TASK2_STK_SIZE   1024   // 任务堆栈大小
TaskHandle_t Tasks2_TaskHandle; // 任务句柄

void taskInit(){
    pinMode(rbaPin,OUTPUT);
    xTaskCreate(task1, "task1_task",TASK1_STK_SIZE,NULL,TASK1_TASK_PRIO,NULL);
    xTaskCreate(task2, "task2_task",TASK2_STK_SIZE,NULL,TASK2_TASK_PRIO,NULL);
    ledcSetup(lightChannel, lightFreq, lightResolution);
    ledcAttachPin(lightPin, lightChannel);

    Serial.println("FreeRTOS had ready");
}

void task1( void * parameter ){//台灯按钮检测·
    while(true){
        if(lightTouch == false && touchRead(lightCtrlPin) <= 40){lightTouch = true;
        lightValue += 1;
        if(lightValue == lightHIGH+1){lightValue = lightOFF;}
        }
        else if(lightTouch == true && touchRead(lightCtrlPin) > 40) {lightTouch = false;}

        if(lightValue == 0){
            ledcWrite(lightChannel,0);//指定通道输出一定占空比波形
        }
        else if(lightValue == 1){
            ledcWrite(lightChannel,127);//指定通道输出一定占空比波形
        }
        else if(lightValue == 2){
            ledcWrite(lightChannel,191);//指定通道输出一定占空比波形
        }
        else if(lightValue == 3){
            ledcWrite(lightChannel,255);//指定通道输出一定占空比波形
        }
        vTaskDelay(20);
    }
}

void task2( void * parameter ){//雾化器按钮检测
    while(true){
        if(rbaTouch == false && touchRead(rbaCtrlPin) <= 40){rbaTouch = true;
            rbaOpen += 1;
            if(rbaOpen == 2){rbaOpen = 0;}
            if(rbaOpen == 1 && rbaNature == true){
                rbaNature = false;
                Serial.println("the RBA has been set USER");}
            }
        else if(rbaTouch == true && touchRead(rbaCtrlPin) > 40){rbaTouch = false;}
        if(rbaOpen == 1){
            digitalWrite(rbaPin,HIGH);
        }
        else{
            digitalWrite(rbaPin,LOW);
        }
        vTaskDelay(20); 
        }
}

#endif
