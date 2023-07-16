#ifndef __YD_LIDAR_X4__
#define __YD_LIDAR_X4__

#include <YDLidarX4StateMachine.h>
#include <SynchronizedQueue.h>
#include <string>
#include <DummyPrint.h>
#include <LogLevel.h>

using std::string;

// https://www.ydlidar.com/dowfile.html?cid=5&type=3
// https://www.ydlidar.com/Public/upload/files/2022-06-28/YDLIDAR%20X4%20Development%20Manual%20V1.6(211230).pdf

namespace sensorYDLidarX4{

class YDLidarX4{
private:
  // Lidar commands
  uint8_t LIDAR_START_SCAN_CMD[2]    = {0xA5, 0x60};
  uint8_t LIDAR_STOP_SCAN_CMD[2]     = {0xA5, 0x65};
  uint8_t LIDAR_DEVICE_INFO_CMD[2]   = {0xA5, 0x90};
  uint8_t LIDAR_HEALTH_STATUS_CMD[2] = {0xA5, 0x91};
  uint8_t LIDAR_SOFT_REBOOT_CMD[2]   = {0xA5, 0x80};

  SynchronizedQueue<uint8_t> queue;

  // internal variables
  YDLidarX4StateMachine stateMachine;
  Stream *serial;
  uint16_t maxQueueElements;
  Print *debug;
  Print *trace;
  LogLevel logLevel;
  int motorEnablePin;
  unsigned long timeoutInMilliseconds;
  bool autoRestart; 

  bool trySetPinMode();
  bool tryDisableMotor();
  bool tryEnableMotor();
  bool tryReceiveAllBytes();

  // stats
  volatile uint16_t statMaxLidarQueueSize;
  volatile unsigned long statLastTimeReceivedData;
  void initializeLidarStats();
  void handleLidarStats();

  template<typename TYPENAME, std::size_t SIZE>
  size_t sendCmd(TYPENAME (&cmd)[SIZE]);

  YDLidarX4(Stream *lidarSerial, OnLidarPacketHandler *packetHandler, OnLidarIndexPacketHandler *indexPacketHandler, unsigned long timeoutInMilliseconds, bool autoRestart, int motorEnablePin, uint16_t maxQueueElements, Print *debug, Print *trace, LogLevel logLevel);
  class YDLidarX4Builder;

public:
  ~YDLidarX4();
  static YDLidarX4Builder builder();

  bool start();
  bool stop();
  bool requestDeviceInfo();
  bool requestHealtStatus();
  bool requestSoftReboot();
  bool restart();
  bool doReceive();
  void run();
  bool runOnce();
  void handleError();
  void handleTimeout();
  void checkReceiveTimeout();

  bool isScanning();

  size_t getQueueSize();
  size_t getMaxUsedQueueSize();
  size_t getQueueCapacity();
  string getState();

  void debugPrintQueue();
  void debugPrintRawQueue();
  void setLogLevel(LogLevel logLevel);
};

class YDLidarX4::YDLidarX4Builder{
  private:
    Stream *serial;
    OnLidarPacketHandler *packetHandler;
    OnLidarIndexPacketHandler *indexPacketHandler;
    uint16_t maxQueueElements;
    unsigned long timeoutInMilliseconds;
    bool autoRestart;
    int motorEnablePin;
    Print *debug;
    Print *trace;
    LogLevel logLevel;

  public:
    YDLidarX4Builder();
    YDLidarX4 build();
    YDLidarX4* buildPtr();

    YDLidarX4Builder& setSerialDevice(Stream &serial);
    YDLidarX4Builder& setPacketHandler(OnLidarPacketHandler &packetHandler);
    YDLidarX4Builder& setIndexPacketHandler(OnLidarIndexPacketHandler &indexPacketHandler);
    YDLidarX4Builder& setMaxQueueElements(uint16_t maxQueueElements);
    YDLidarX4Builder& setTimeoutInMilliseconds(unsigned long timeoutInMilliseconds);
    YDLidarX4Builder& setAutoRestartOnTimeout(bool autoRestart);
    YDLidarX4Builder& setMotorEnablePin(int motorEnablePin);
    YDLidarX4Builder& setDebug(Print &debug);
    YDLidarX4Builder& setTrace(Print &trace);
    YDLidarX4Builder& setLogLevel(LogLevel logLevel);
};


} // end namespace

#endif
