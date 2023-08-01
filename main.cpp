#include "main.h"

void setup() {
    Serial.begin(115200);
	dhtInit();		//温湿度传感器初始化
	sensorInit();	//水位传感器初始化
	sr501Init();	//人体感应器初始化
	gmInit();		//光敏二极管初始化
	u8g2Init();		//显示屏
	showWait();		//展示等待界面
	netInit();		//连接网络
	showConnect();	//展示连接的IP地址
	ntpInit();		//连接ntp网络获取时间
    taskInit();		//rtos任务初始化,包含触摸引脚、雾化器、灯
}

void loop() {
	Serial.println("--------------------");
	sr501Data();									//人体感应器
	dhtData();										//温湿度传感器
	sensorData();									//水位传感器
	gmData();										//光照传感器
	Serial.printf("The rba open is %d\n",rbaOpen);	//雾化器 
	showTime();										//时间
	Serial.println();

}