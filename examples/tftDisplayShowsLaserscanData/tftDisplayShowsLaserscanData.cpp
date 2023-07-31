/*
This example was tested on a ESP32 and a ILI9341 TFT display

Connect the YDLidarX4 with a - ideally separate power source - 5V@2A
and connect to the ESP32 with the following pins:

  ESP32   |  YDLidarX4
  --------+-----------
  GND     | GND
  GPIO 13 | M_SCTR
  TX2     | Rx
  RX2     | Tx

Connect the TFT to the ESP32 with the following pins:

  ESP32   |  TFT
  --------+-----------
  3V3/5V  | VCC (according to your display/-setting)
  GND     | GND
  GPIO 15 | CS
  GPIO 2  | RESET
  GPIO 0  | DC/RS
  GPIO 23 | SDI/MOSI
  GPIO 18 | SCK
  3V3/5V  | LED (logic, no power, can be driven from a output pin with high level)
  GPIO 19 | SDO/MISO (optional, not available on my display)

After flashing the code, the button on den defined pin toggles the scaning state.
If the lidar is active it shows the lidarscan data on the TFT display.

To compile this example, you must add this to your platformio.ini:
lib_deps =
  nox/YDLidarX4
  bodmer/TFT_eSPI@^2.4.11
build_flags =
  -DUSER_SETUP_LOADED=1
  -DILI9341_DRIVER=1
  -DTFT_WIDTH=240
  -DTFT_HEIGHT=320
  -DTFT_MISO=-1  ; or 19
  -DTFT_MOSI=23
  -DTFT_SCLK=18
  -DTFT_CS=15
  -DTFT_DC=0
  -DTFT_RST=2
  -DSPI_FREQUENCY=27000000
*/
#include <queue>
#include <YDLidarX4.h>
#include <TFT_eSPI.h>

using sensorYDLidarX4::YDLidarX4;
using sensorYDLidarX4::OnLidarPacketHandler;

#define LIDAR_SERIAL Serial2
#define LIDAR_ENABLE GPIO_NUM_13
#define BUTTON_PIN GPIO_NUM_0

#define TFT_ROTATION 3
#define TFT_FOREGROUND_COLOR TFT_WHITE
#define TFT_BACKGROUND_COLOR TFT_BLUE
#define TFT_LIDAR_DRAW_SCALE 32
#define TFT_LIDAR_DRAW_POINT_COUNT 720
#define TFT_CENTER_X (TFT_ROTATION==1|TFT_ROTATION==3?TFT_HEIGHT/2:TFT_WIDTH/2);
#define TFT_CENTER_Y (TFT_ROTATION==1|TFT_ROTATION==3?TFT_WIDTH/2:TFT_HEIGHT/2)

struct Pixel{
  int32_t x;
  int32_t y;
};

YDLidarX4* lidar;
TFT_eSPI tft = TFT_eSPI();
std::queue<Pixel> pixelQueue;

void clearPointIfQueueIsFull(){
  if(pixelQueue.size() < TFT_LIDAR_DRAW_POINT_COUNT){
    return;
  }

  Pixel pixel = pixelQueue.front();
  pixelQueue.pop();
  tft.drawPixel(pixel.x,pixel.y,TFT_BACKGROUND_COLOR);
}

void drawPolarPoint(float angleInDegree, float rangeInMillimeter){
  // do not draw short ranges (== 0.00)
  if(rangeInMillimeter<0.01){
    return;
  }

  // adjust display rotation
  angleInDegree += 90*TFT_ROTATION;

  // limit range to 0 <= angleInDegree <= 360
  angleInDegree+=360;
  while (angleInDegree>=360){
    angleInDegree-=360;
  }

  // convert to cartesian coordinate, scale and center it on screen
  float angleInRad = angleInDegree*DEG_TO_RAD;
  int32_t x = rangeInMillimeter * cos(angleInRad) / TFT_LIDAR_DRAW_SCALE + TFT_CENTER_X;
  int32_t y = rangeInMillimeter * sin(angleInRad) / TFT_LIDAR_DRAW_SCALE + TFT_CENTER_Y;

  // store the pixel in the queue, to clear them later and draw it
  pixelQueue.push({x,y});
  tft.drawPixel(x,y,TFT_FOREGROUND_COLOR);
}

void drawLidarscan(float minAngleInDegree, float maxAngleInDegree, float rangesInMillimeter[], size_t rangesLenght){
  if(minAngleInDegree>maxAngleInDegree){
    maxAngleInDegree+=360;
  }
  float angleIncrementInDegree = (maxAngleInDegree-minAngleInDegree)/rangesLenght;
  for(int i=0; i<rangesLenght; i++) {
    clearPointIfQueueIsFull();
    float angleInDegree = minAngleInDegree + angleIncrementInDegree * i;
    drawPolarPoint(angleInDegree, rangesInMillimeter[i]);
  }
}

OnLidarPacketHandler countReceivedPackets = [&](float minAngleInDegree, float maxAngleInDegree, float anglesInDegree[], float rangesInMillimeter[], size_t rangesLenght){
  drawLidarscan(minAngleInDegree, maxAngleInDegree, rangesInMillimeter, rangesLenght);
};

void checkButtonStateAndToggleLidar(){
  static unsigned long last = millis();
  if(millis()-last < 1000){
    return;
  }

  bool buttonPressed = !digitalRead(BUTTON_PIN);
  if(buttonPressed){
    if(lidar->isScanning()){
      lidar->stop();
    }else{
      lidar->start();
    }
    last=millis();
  }
}

void setup(){
  // TFT
  tft.init();
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(TFT_BACKGROUND_COLOR);
  tft.setCursor(0, 0, 4);
  tft.setTextColor(TFT_FOREGROUND_COLOR, TFT_BACKGROUND_COLOR);
  tft.setTextColor(TFT_FOREGROUND_COLOR, TFT_BACKGROUND_COLOR);
  tft.println(String()+"Press the button on pin "+BUTTON_PIN);
  tft.println("to toggle the lidar state.");

  // YDLidarX4
  lidar = YDLidarX4::builder()
    .setSerialDevice(LIDAR_SERIAL)
    .setMotorEnablePin(LIDAR_ENABLE)
    .setPacketHandler(countReceivedPackets)
    .setMaxQueueElements(5000)
    .setTimeoutInMilliseconds(1000)
    .setAutoRestartOnTimeout(false)
    .buildPtr();
  LIDAR_SERIAL.begin(128000);
  LIDAR_SERIAL.onReceive([&](){
    lidar->doReceive();
  });
  lidar->stop();
}

void loop(){
  lidar->run();
  checkButtonStateAndToggleLidar();
}
