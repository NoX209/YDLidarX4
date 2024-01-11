#ifndef __YD_Lidar_X4_STATE_MACHINE__
#define __YD_Lidar_X4_STATE_MACHINE__

#include <Arduino.h>
#include <SynchronizedQueue.h>
#include <string>
#include <LogLevel.h>

using std::string;

// https://www.ydlidar.com/dowfile.html?cid=5&type=3
// https://www.ydlidar.com/Public/upload/files/2022-06-28/YDLIDAR%20X4%20Development%20Manual%20V1.6(211230).pdf

namespace sensorYDLidarX4 {

// define packet handler lambda interfaces
typedef std::function<void(float minAngleInDegree, float maxAngleInDegree, float correctedAnglesInDegree[], float rangesInMillimeter[], size_t lenght)> OnLidarPacketHandler;
typedef std::function<void(float angleInDegree, float correctedAngleInDegree, float rangeInMillimeter)> OnLidarIndexPacketHandler;

class YDLidarX4StateMachine{
private:
  enum YDLidarX4Packet{
    packetHeaderLsb=0, packetHeaderMsb=1,
    packetType=2,
    packetSampleQuantity=3,
    startAngleLsb=4, startAngleMsb=5,
    lastAngleLsb=6, lastAngleMsb=7,
    checkSumLsb=8, checkSumMsb=9,
    sampleBeginPos=10
  };
  enum YDLidarX4PacketType{
    scanPacket=0x00,
    indexPacket=0x01
  };
  enum State {
    IDLE,
    READY,
    START,
    START_NEED_MORE_DATA,
    START_CHECK_PAKET,
    START_REMOVE_PAKET,
    SCAN_NEED_HEADER,
    SCAN_NEED_SIZE,
    SCAN_NEED_DATA,
    SCAN_CHECK_CRC,
    SCAN_SEND_MESSAGE,
    STOP,
    TIMEOUT,
    END,
    ERROR
  };

  union Paket{
    uint8_t rawData[sampleBeginPos+sizeof(uint16_t)*256]; // sampleBeginPos + 2Bytes * sizeof(packetSampleQuantity) = 10+2*256
    struct {
      uint16_t header;
      uint8_t type;
      uint8_t quantity;
      uint16_t startAngle;
      uint16_t lastAngle;
      uint16_t checkSum;
      uint16_t samples[256];
    }__attribute__((packed));
  }__attribute__((packed));
  

  State state;
  SynchronizedQueue<uint8_t> &queue;
  OnLidarPacketHandler *packetHandler;
  OnLidarIndexPacketHandler *indexPacketHandler;
  Print *debug;
  Print *trace;
  LogLevel logLevel;
  Paket paket;
  size_t expectedPaketSize;

  // start stream expectations
  const static size_t startHeaderSize = 7;
  uint8_t expecedLidarStartResponse[startHeaderSize] = {
    0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81,
  };

  // state handler
  State handleState();
  State handleStateIdle();
  State handleStateReady();
  State handleStateStart();
  State handleStateStartNeedMoreData();
  State handleStateStartCheckPaket();
  bool isCorrectStartPaket();
  State handleStateStartRemovePaket();
  bool removeBytesFromQueue(uint16_t byteCountToRemove);
  State handleStateScanNeedHeader();
  State handleStateScanNeedSize();
  State handleStateScanNeedData();
  uint16_t getScanPaketExpectedSize();
  State handleStateScanCheckCrc();
  bool isCorrectCrc();
  State handleStateScanSendMessages();
  void handleValidPacket();
  State handleStateStop();
  State handleStateTimeout();

  void calculateRangesAndAnglesFromPaket(Paket &paket, uint8_t sampleQuantity, float startAngleInDegree, float stopAngleInDegree, float* ranges, float* anglesInDegree);
  void printRangesAndAnglesFromPaket(bool isIndexPaket, size_t lenght, float minAngleInDegree, float maxAngleInDegree, float anglesInDegree[], float ranges[]);
  // notify the handlers
  void notifyHandlers(bool isIndexPaket, float minAngleInDegree, float maxAngleInDegree, float anglesInDegree[], float ranges[], size_t lenght);
  void notifyIndexPacketHandler(float minAngleInDegree, float maxAngleInDegree, float anglesInDegree[], float ranges[]);
  void notifyPacketHandler(float minAngleInDegree, float maxAngleInDegree, float anglesInDegree[], float ranges[], size_t lenght);

  // internal helper funktions
  string getState(State lidarState);
  void setState(State lidarState);
  string hex(uint8_t value);
  string hex(uint16_t value);

  // lidar calc helper funktions
  float calculateAngleInDegree(float distanceValue_i, float startAngleInDegree, float stopAngleInDegree, uint16_t sampleNum, uint8_t sampleQuantity);
  float angleInDegree(uint16_t value);
  float distanceInMillimeter(uint16_t value);
  float calculateCorrectingAngleInDegree(float distance);

public:
  YDLidarX4StateMachine(SynchronizedQueue<uint8_t> &queue, OnLidarPacketHandler *callback, OnLidarIndexPacketHandler *indexPacketHandler, Print *debug, Print *trace, LogLevel logLevel);
  bool runOne();
  
  void setStateIdle();
  void setStateStop();
  void setStateTimeout();
  void setStateError();

  bool hasError();
  bool hasTimeout();
  bool isScanning();
  string getState();

  void debugPrintQueue(const char *message="");
  void debugPrintQueue(Print* out, const char *message="");
  void debugPrintFullQueue(const char *message="");
  void debugPrintFullQueue(Print* out, const char *message="");
  void debugPrintRawQueue(const char *message="");
  void debugPrintRawQueue(Print* out, const char *message="");
  void setLogLevel(LogLevel logLevel);
};

} // end namespace

#endif
