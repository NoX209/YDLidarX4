#include <YDLidarX4.h>

#define IF_DEBUG if(debug != &DummyPrint)
#define IF_TRACE if(trace != &DummyPrint && logLevel == LOG_TRACE)

// https://www.ydlidar.com/dowfile.html?cid=5&type=3
// https://www.ydlidar.com/Public/upload/files/2022-06-28/YDLIDAR%20X4%20Development%20Manual%20V1.6(211230).pdf

namespace sensorYDLidarX4 {

YDLidarX4::YDLidarX4(Stream *serial, OnLidarPacketHandler *packetHandler, OnLidarIndexPacketHandler *indexPacketHandler, unsigned long _timeoutInMilliseconds, bool autoRestart, int _motorEnablePin, uint16_t maxQueueElements, Print *debug, Print *trace, LogLevel logLevel)
  : serial(serial),
  debug(debug),
  trace(trace),
  logLevel(logLevel),
  queue(SynchronizedQueue<uint8_t>(maxQueueElements)),
  stateMachine(queue, packetHandler, indexPacketHandler, debug, trace, logLevel),
  timeoutInMilliseconds(_timeoutInMilliseconds),
  autoRestart(autoRestart),
  motorEnablePin(_motorEnablePin)
{
  // REMINDER do not use debug->print in construtor
  trySetPinMode();
  tryDisableMotor();
}

YDLidarX4::~YDLidarX4(){
  stop();
}

YDLidarX4::YDLidarX4Builder YDLidarX4::builder(){
  return YDLidarX4Builder();
}

// --- motor handling -----------------------------------------------------------------------------
bool YDLidarX4::start(){
  stateMachine.setStateIdle();
  // queue.clear();
  
  int availableBytes = serial->available();
  for(int byteCount=0; byteCount<availableBytes; byteCount++){
    serial->read();
  }

  initializeLidarStats();
  tryEnableMotor();
  sendCmd(LIDAR_START_SCAN_CMD);
  return true;
}

bool YDLidarX4::stop(){
  sendCmd(LIDAR_STOP_SCAN_CMD);
  stateMachine.setStateStop();
  tryDisableMotor();
  return true;
}

bool YDLidarX4::requestDeviceInfo(){
  sendCmd(LIDAR_DEVICE_INFO_CMD);
  return true;
}

bool YDLidarX4::requestHealtStatus(){
  sendCmd(LIDAR_HEALTH_STATUS_CMD);
  return true;
}

bool YDLidarX4::requestSoftReboot(){
  sendCmd(LIDAR_SOFT_REBOOT_CMD);
  return true;
}

bool YDLidarX4::restart(){
  IF_DEBUG debug->printf("   *** RESTARTING LIDAR ***\n");
  bool isStopped = stop();
  bool isStarted = start();
  return isStopped & isStarted;
}

template<typename TYPENAME, std::size_t SIZE>
size_t YDLidarX4::sendCmd(TYPENAME (&cmd)[SIZE]){
  if(serial == nullptr){
    return 0;
  }

  return serial->write(cmd, SIZE);
}

bool YDLidarX4::trySetPinMode(){
    if(this->motorEnablePin == -1){
    return false;
  }

  pinMode(this->motorEnablePin, OUTPUT);
  return true;
}

bool YDLidarX4::tryDisableMotor(){
  if(this->motorEnablePin == -1){
    return false;
  }

  digitalWrite(this->motorEnablePin, LOW);
  return true;
}

bool YDLidarX4::tryEnableMotor(){
  if(this->motorEnablePin == -1){
    return false;
  }

  digitalWrite(this->motorEnablePin, HIGH);
  return true;
}

// --- handling of lidar data reading -------------------------------------------------------------
bool YDLidarX4::doReceive(){
  if(!stateMachine.isScanning()){
    return false;
  }

  if(serial == nullptr){
    return false;
  }

  if(!tryReceiveAllBytes()){
    return false;
  }

  handleLidarStats();
  return true;
}

bool YDLidarX4::tryReceiveAllBytes(){
  int availableBytes = serial->available();
  for(int byteCount=0; byteCount<availableBytes; byteCount++){

    int data = serial->read();
    if(data == -1){
      return false;
    }

    uint8_t byteFromLidar = (uint8_t)data;
    bool hasDataAdded = queue.add(byteFromLidar);
    if(!hasDataAdded){
      IF_DEBUG debug->printf("\n--> could not add data to lidar queue\n\n");
      stateMachine.setStateError();
      return false;
    }
  }
  return true;
}

// --- statistic functions | used for timeout -----------------------------------------------------
void YDLidarX4::initializeLidarStats(){
  statLastTimeReceivedData = millis();
  statMaxLidarQueueSize = 0;
}

void YDLidarX4::handleLidarStats(){
  statLastTimeReceivedData = millis();
  uint16_t queueSize = queue.size();
  if(queueSize > statMaxLidarQueueSize){
    statMaxLidarQueueSize = queueSize;
  }
}

// --- run/runOnce function -----------------------------------------------------------------------
void YDLidarX4::run(){
  while(runOnce());
}

bool YDLidarX4::runOnce(){
  bool isStateChanged = stateMachine.runOne();
  handleError();
  handleTimeout();
  checkReceiveTimeout();
  return isStateChanged;
}

void YDLidarX4::handleError(){
  if(!stateMachine.hasError()){
    return;
  }

  IF_DEBUG debug->printf("lidar has an error - stopping with queue size of %lu Bytes\n", queue.size());
  stop();

  const char* horizontalRow = string(90, '#').c_str();
  IF_DEBUG debug->println(horizontalRow);
  IF_DEBUG stateMachine.debugPrintQueue(debug);
  IF_DEBUG debug->println(horizontalRow);
  IF_DEBUG stateMachine.debugPrintFullQueue(debug);
  IF_DEBUG debug->println(horizontalRow);
  IF_TRACE stateMachine.debugPrintRawQueue(trace);
  IF_TRACE trace->println(horizontalRow);
}

void YDLidarX4::handleTimeout(){
  if(!stateMachine.hasTimeout()){
    return;
  }

  IF_DEBUG debug->printf("handle lidar timeout\n");
  if(autoRestart){
    restart();
  }else{
    stop();
  }
}

void YDLidarX4::checkReceiveTimeout(){
  // fetch data
  unsigned long statMilliseconds = statLastTimeReceivedData;
  unsigned long currentMilliseconds = millis();

  // calculate difference
  unsigned long receiveDeltaMilliseconds = (currentMilliseconds - statMilliseconds);
  bool isLastReceivedDataOverTimeout = timeoutInMilliseconds < receiveDeltaMilliseconds;

  if(stateMachine.isScanning() && isLastReceivedDataOverTimeout ){
    IF_DEBUG debug->printf("\n/--------------------------\n");
    IF_DEBUG debug->printf("| ********* TIMEOUT detected\n");
    IF_DEBUG debug->printf("| state:%s isScanning:%d\n", stateMachine.getState().c_str(), stateMachine.isScanning());
    IF_DEBUG debug->printf("|\n");
    IF_DEBUG debug->printf("| lidar has an timeout - QueueSize(capacity/maxUsed/currentUsed):%lu/%lu/%lu\n", queue.getCapacity(), statMaxLidarQueueSize, queue.size());
    IF_DEBUG debug->printf("\\--------------------------\n");

    stateMachine.setStateTimeout();
  }
}

// --- some getters -------------------------------------------------------------------------------
bool YDLidarX4::isScanning(){
  return stateMachine.isScanning();
}

size_t YDLidarX4::getQueueSize(){
  return queue.size();
}

size_t YDLidarX4::getMaxUsedQueueSize(){
  return statMaxLidarQueueSize;
}

size_t YDLidarX4::getQueueCapacity(){
  return queue.getCapacity();
}

string YDLidarX4::getState(){
  return stateMachine.getState();
}

// --- debug output of queue ----------------------------------------------------------------------
void YDLidarX4::debugPrintQueue(){
  stateMachine.debugPrintQueue();
}

void YDLidarX4::debugPrintRawQueue(){
  stateMachine.debugPrintRawQueue();
}

void YDLidarX4::setLogLevel(LogLevel logLevel){
  this->logLevel = logLevel;
  stateMachine.setLogLevel(logLevel);
}

// --- Builder ------------------------------------------------------------------------------------

// set some default values
YDLidarX4::YDLidarX4Builder::YDLidarX4Builder()
  : timeoutInMilliseconds(1000),
  autoRestart(true),
  serial(nullptr),
  packetHandler(nullptr),
  indexPacketHandler(nullptr),
  maxQueueElements(360),  // should handle at least almost 3 complete serial data blocks = 3*120bytes
  debug(&DummyPrint),
  trace(&DummyPrint),
  logLevel(LOG_NONE),
  motorEnablePin(-1)
{
}

YDLidarX4 YDLidarX4::YDLidarX4Builder::build(){
  return YDLidarX4(serial, packetHandler, indexPacketHandler, timeoutInMilliseconds, autoRestart, motorEnablePin, maxQueueElements, debug, trace, logLevel);
}

YDLidarX4* YDLidarX4::YDLidarX4Builder::buildPtr(){
  return new YDLidarX4(serial, packetHandler, indexPacketHandler, timeoutInMilliseconds, autoRestart, motorEnablePin, maxQueueElements, debug, trace, logLevel);
}

YDLidarX4::YDLidarX4Builder& YDLidarX4::YDLidarX4Builder::setSerialDevice(Stream &serial){
  this->serial = &serial;
  return *this;
}

YDLidarX4::YDLidarX4Builder& YDLidarX4::YDLidarX4Builder::setPacketHandler(OnLidarPacketHandler &packetHandler){
  this->packetHandler = &packetHandler; 
  return *this;
}

YDLidarX4::YDLidarX4Builder& YDLidarX4::YDLidarX4Builder::setIndexPacketHandler(OnLidarIndexPacketHandler &indexPacketHandler){
  this->indexPacketHandler = &indexPacketHandler; 
  return *this;
}

YDLidarX4::YDLidarX4Builder& YDLidarX4::YDLidarX4Builder::setMaxQueueElements(uint16_t maxQueueElements){
  this->maxQueueElements = maxQueueElements; 
  return *this;
}

YDLidarX4::YDLidarX4Builder& YDLidarX4::YDLidarX4Builder::setTimeoutInMilliseconds(unsigned long timeoutInMilliseconds){
  this->timeoutInMilliseconds = timeoutInMilliseconds; 
  return *this;
}

YDLidarX4::YDLidarX4Builder& YDLidarX4::YDLidarX4Builder::setAutoRestartOnTimeout(bool autoRestart){
  this->autoRestart = autoRestart; 
  return *this;
}

YDLidarX4::YDLidarX4Builder& YDLidarX4::YDLidarX4Builder::setMotorEnablePin(int motorEnablePin){
  this->motorEnablePin = motorEnablePin; 
  return *this;
}

YDLidarX4::YDLidarX4Builder& YDLidarX4::YDLidarX4Builder::setDebug(Print &debug){
  this->debug = &debug; 
  return *this;
}

YDLidarX4::YDLidarX4Builder& YDLidarX4::YDLidarX4Builder::setTrace(Print &trace){
  this->trace = &trace; 
  return *this;
}

YDLidarX4::YDLidarX4Builder& YDLidarX4::YDLidarX4Builder::setLogLevel(LogLevel logLevel){
  this->logLevel = logLevel; 
  return *this;
}

} // end namespace
