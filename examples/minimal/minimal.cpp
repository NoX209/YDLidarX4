/*
Connect the YDLidarX4 with a - ideally separate power source - 5V@2A
and connect to the ESP32 with the following pins:

  ESP32   |  YDLidarX4
  --------+-----------
  GND     | GND
  GPIO 13 | M_SCTR
  TX2     | Rx
  RX2     | Tx
*/

#include <YDLidarX4.h>
using sensorYDLidarX4::YDLidarX4;
using sensorYDLidarX4::OnLidarPacketHandler;

#define LIDAR_SERIAL Serial2
#define LIDAR_ENABLE GPIO_NUM_13

YDLidarX4* lidar;

OnLidarPacketHandler handlePackets = [&](float minAngleInDegree, float maxAngleInDegree, float anglesInDegree[], float ranges[], size_t rangesLenght){
  // Handle your angle and distance data here !
};

void setup(){
  lidar = YDLidarX4::builder()
    .setSerialDevice(LIDAR_SERIAL)
    .setMotorEnablePin(LIDAR_ENABLE)
    .setPacketHandler(handlePackets)
    .buildPtr();

  LIDAR_SERIAL.begin(128000);
  LIDAR_SERIAL.onReceive([&](){
    lidar->doReceive();
  });

  lidar->start();
}

void loop(){
  lidar->run();
}
