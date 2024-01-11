/*
This example was tested on a ESP32

Connect the YDLidarX4 with a - ideally separate power source - 5V@2A
and connect to the ESP32 with the following pins:

  ESP32   |  YDLidarX4
  --------+-----------
  GND     | GND
  GPIO 13 | M_SCTR
  TX2     | Rx
  RX2     | Tx

After flashing the code the lidar should scanning WHILE the button on den defined pin is pressed
and display every second the received packet count with some buffer detail information.
*/

#include <YDLidarX4.h>
using sensorYDLidarX4::YDLidarX4;
using sensorYDLidarX4::OnLidarPacketHandler;

#define LIDAR_SERIAL Serial2
#define LIDAR_ENABLE GPIO_NUM_13
#define BUTTON_PIN GPIO_NUM_0

YDLidarX4* lidar;
int pkgs = 0;
int lastPkgs=0;

OnLidarPacketHandler countReceivedPackets = [&](float minAngleInDegree, float maxAngleInDegree, float anglesInDegree[], float ranges[], size_t rangesLenght){
  pkgs++;
};

void setup(){
  Serial.begin(9600);
  Serial.println("\n\n");
  Serial.println("YDLidarX4");
  Serial.println("---------------------------\n");
  Serial.printf("--> press and hold the button on pin %d to scan <--\n\n", BUTTON_PIN);
  
  lidar = YDLidarX4::builder()
    .setSerialDevice(LIDAR_SERIAL)
    .setMotorEnablePin(LIDAR_ENABLE)
    .setPacketHandler(countReceivedPackets)
    .buildPtr();

  LIDAR_SERIAL.begin(128000);
  LIDAR_SERIAL.onReceive([&](){
    lidar->doReceive();
  });

  lidar->stop();
}

void printLidarStatus(){
  Serial.printf(
    "pkg:%lu[+%d]\tserial:%d Byte\tlidar:%lu/%lu/%lu Byte\n",
    pkgs, pkgs-lastPkgs,
    LIDAR_SERIAL.available(),
    lidar->getQueueCapacity(), lidar->getMaxUsedQueueSize(), lidar->getQueueSize()
  );
  lastPkgs = pkgs;
}

void checkButtonStateAndToggleLidar(){
  bool buttonPressed = !digitalRead(BUTTON_PIN);
  if(buttonPressed){
    if(!lidar->isScanning()){
      Serial.println("=> starting lidar ...");
      Serial.println("pkg:<count>[+<diff>]\tserial:<buffersize> Byte\tlidar:<buffermax>/<maxused>/<current> Byte\n");
      lidar->start();
    }
  }else{
    if(lidar->isScanning()){
      Serial.println("=> stoping lidar ...");
      lidar->stop();
    }
  }
}

unsigned long last = millis();

void loop(){
  lidar->run();

  unsigned long now = millis();
  if((now-last)>1000){
    last=now;

    checkButtonStateAndToggleLidar();

    if(lidar->isScanning()){
      printLidarStatus();
    }
  }
}
