#include <YDLidarX4StateMachine.h>
#include <DummyPrint.h>

using std::to_string;

#define IF_DEBUG if(debug != &DummyPrint)
#define IF_TRACE if(trace != &DummyPrint && logLevel == LOG_TRACE)

// https://www.ydlidar.com/dowfile.html?cid=5&type=3
// https://www.ydlidar.com/Public/upload/files/2022-06-28/YDLIDAR%20X4%20Development%20Manual%20V1.6(211230).pdf

namespace sensorYDLidarX4 {

YDLidarX4StateMachine::YDLidarX4StateMachine(SynchronizedQueue<uint8_t> &queue, OnLidarPacketHandler *packetHandler, OnLidarIndexPacketHandler *indexPacketHandler, Print *debug, Print *trace, LogLevel logLevel)
  : queue(queue),
  packetHandler(packetHandler),
  indexPacketHandler(indexPacketHandler),
  debug(debug),
  trace(trace),
  logLevel(logLevel),
  state(IDLE)
{
}

bool YDLidarX4StateMachine::runOne(){
  State oldState = state;

  unsigned long start = micros();
  state = handleState();
  static unsigned long stop = micros();

  unsigned long timeWaitedInMicroseconds = start-stop;
  stop = micros();
  unsigned long timeDiffInMicroseconds = stop-start;

  // bool showStateChange = true;
  // if(showStateChange){
  //   static uint16_t sizeOld = queue.size();
  //   uint16_t sizeAct = queue.size();
  //   if(state!=oldState || sizeOld != sizeAct){
  //     // IF_TRACE trace->printf("Statechange:%d->%d (%luus needed) (%luus waited) size:%lu\n", oldState, state, timeDiffInMicroseconds, timeWaitedInMicroseconds, sizeAct);
  //   }
  //   sizeOld = sizeAct;
  // }
  // FEATURE REQUEST: add statistics for state changes https://github.com/RobTillaart/Statistic/blob/master/examples/TimingTest/TimingTest.ino
  // if(timeDiffInMicroseconds > 3120){
  //   static long old = millis();
  //   long now = millis();

  //   if(now>old+1000){
  //     old = now;
  //     IF_DEBUG debug->printf("   PERFORMANCE: Statechange %s->%s needed %luus waited:%luus\n", getState(oldState).c_str(), getState(state).c_str(), timeDiffInMicroseconds, timeWaitedInMicroseconds);
  //   }
  // }

  return state != oldState;
}

void YDLidarX4StateMachine::setStateIdle(){
  setState(IDLE);
}

void YDLidarX4StateMachine::setStateStop(){
  setState(STOP);
}

void YDLidarX4StateMachine::setStateTimeout(){
  setState(TIMEOUT);
}

void YDLidarX4StateMachine::setStateError(){
  setState(ERROR);
}

void YDLidarX4StateMachine::setState(State newState){
  IF_DEBUG debug->printf("\n--> set state | %s->%s\n\n", getState(state).c_str(), getState(newState).c_str());
  state = newState;
}

bool YDLidarX4StateMachine::hasError(){
  return state == ERROR;
}

bool YDLidarX4StateMachine::hasTimeout(){
  return state == TIMEOUT;
}

bool YDLidarX4StateMachine::isScanning(){
  return 
    state == READY ||
    state == START ||
    state == START_NEED_MORE_DATA ||
    state == START_CHECK_PAKET ||
    state == START_REMOVE_PAKET ||
    state == SCAN_NEED_HEADER ||
    state == SCAN_NEED_SIZE ||
    state == SCAN_NEED_DATA ||
    state == SCAN_CHECK_CRC ||
    state == SCAN_SEND_MESSAGE;
}

string YDLidarX4StateMachine::getState(){
  return getState(state);
}

string YDLidarX4StateMachine::getState(State lidarState){
  switch (lidarState){
    case IDLE:                  return "Idle";
    case READY:                 return "Ready";
    case START:                 return "Start";
    case START_NEED_MORE_DATA:  return "StartNeedMoreData";
    case START_CHECK_PAKET:     return "StartCheckPaket";
    case START_REMOVE_PAKET:    return "StartRemovePaket";
    case SCAN_NEED_HEADER:      return "ScanNeedHeader";
    case SCAN_NEED_SIZE:        return "ScanNeedSize";
    case SCAN_NEED_DATA:        return "ScanNeedData";
    case SCAN_CHECK_CRC:        return "ScanCheckCrc";
    case SCAN_SEND_MESSAGE:     return "ScanSendMessages";
    case STOP:                  return "Stop";
    case TIMEOUT:               return "Timeout";
    case END:                   return "End";
    default:                    return "Error";
  }
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleState(){
  switch (state){
    case IDLE:                  return handleStateIdle();
    case READY:                 return handleStateReady();
    case START:                 return handleStateStart();
    case START_NEED_MORE_DATA:  return handleStateStartNeedMoreData();
    case START_CHECK_PAKET:     return handleStateStartCheckPaket();
    case START_REMOVE_PAKET:    return handleStateStartRemovePaket();
    case SCAN_NEED_HEADER:      return handleStateScanNeedHeader();
    case SCAN_NEED_SIZE:        return handleStateScanNeedSize();
    case SCAN_NEED_DATA:        return handleStateScanNeedData();
    case SCAN_CHECK_CRC:        return handleStateScanCheckCrc();
    case SCAN_SEND_MESSAGE:     return handleStateScanSendMessages();
    case STOP:                  return handleStateStop();
    case TIMEOUT:               return handleStateTimeout();
    case END:                   return END;
    default:                    return ERROR;
  }
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateIdle(){
  queue.clear();
  return READY;
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateReady(){
  if(queue.size() == 0){
    return READY;
  }
  return START;
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateStart(){
  if(queue.get(0) != 0xA5){
    IF_DEBUG debug->printf("   ### handleStateStart: first byte did not match\n");

    if(queue.get(0) == 0xAA){
      if(queue.size()>1 && queue.get(1) == 0x55){
        // sometimes the header is missing !!! :O -> short cut | TODO or only with testdata?
        IF_DEBUG debug->printf("   ### handleStateStart: found packet without header -> goto START_NEED_MORE_DATA\n");
        return SCAN_NEED_SIZE;
      }
      return START;
    }
    return ERROR;
  }
  return START_NEED_MORE_DATA;
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateStartNeedMoreData(){
  if(queue.size() < startHeaderSize){
    return START_NEED_MORE_DATA;
  }
  return START_CHECK_PAKET;
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateStartCheckPaket(){
  if(!isCorrectStartPaket()){
    IF_DEBUG debug->printf("   ### incorrect pkg start\n");
    return ERROR;
  };
  return START_REMOVE_PAKET;
}

bool YDLidarX4StateMachine::isCorrectStartPaket(){
  for(int bytePosition=0; bytePosition<startHeaderSize; bytePosition++){
    uint8_t actual = queue.get(bytePosition);
    uint8_t expected = expecedLidarStartResponse[bytePosition];
    if (actual != expected){
      return false;
    }
  }
  return true;
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateStartRemovePaket(){
  if(!removeBytesFromQueue(startHeaderSize)){
    IF_DEBUG debug->printf("   ### could not remove start pkg from queue\n");
    return ERROR;
  }
  return SCAN_NEED_SIZE;
}

bool YDLidarX4StateMachine::removeBytesFromQueue(uint16_t byteCountToRemove){
  if(queue.size() < byteCountToRemove){
    return false;
  }
  queue.remove(byteCountToRemove);
  return true;
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateScanNeedHeader(){
  if(queue.size() <= packetHeaderMsb){
    return SCAN_NEED_HEADER;
  }
  if(queue.get(0) != 0xAA){
    IF_DEBUG debug->printf("   ### paket start not 0xAA\n");
    // debugPrintQueue("(something wrong)");

    if(queue.get(0) == 0xA5){
      IF_DEBUG debug->printf("   ### paket start with 0xA5 -> todo, impl this\n");
      if(queue.get(1) == 0x5A){
        IF_DEBUG debug->printf("   ### paket start with 0xA5 0x5A -> todo, impl this\n");
        if(queue.get(6)==0x06){
          debugPrintQueue("(possible HEALF STATUS)");
        }else if(queue.get(6)==0x04){
          debugPrintQueue("(possible DEVICE INFORMATION)");
        }else{
          debugPrintQueue("(unknown)");
        }
        int lenToRemove = queue.get(2)+7;
        debug->printf(" --- remove %d bytes from buffer\n",lenToRemove);
        // todo do we have enought bytes in the buffer to delete them?
        queue.remove(lenToRemove); // TODO this is a workaround - extract it to its own state
        return SCAN_NEED_HEADER;
      }
    }else if(queue.get(0) == 0x00){
      debugPrintQueue("(zeros ?)");
      int i;
      for(i=0;queue.get(0)== 0x00;i++){
        queue.remove(1);
      }
      IF_DEBUG debug->printf("   ### removed %d 0x00 byte\n", i);
      return SCAN_NEED_HEADER;
    }

    return ERROR;
  }
  if(queue.get(1) != 0x55){
    IF_DEBUG debug->printf("   ### paket start not 0xAA 0x55\n");
    return ERROR;
  }
  return SCAN_NEED_SIZE;
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateScanNeedSize(){
  if(queue.size() <= packetSampleQuantity){
    return SCAN_NEED_SIZE;
  }
  expectedPaketSize = getScanPaketExpectedSize();
  return SCAN_NEED_DATA;
}

uint16_t YDLidarX4StateMachine::getScanPaketExpectedSize(){
  uint8_t paketSize = queue.get(packetSampleQuantity);
  uint16_t expectedPaketSize = sampleBeginPos + 2 * paketSize;
  return expectedPaketSize;
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateScanNeedData(){
  if(queue.size() < expectedPaketSize){
    return SCAN_NEED_DATA;
  }
  return SCAN_CHECK_CRC;
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateScanCheckCrc(){
  queue.extract(paket.rawData, expectedPaketSize);
  if(!isCorrectCrc()){
    IF_DEBUG debug->printf("   ### wrong crc on pkg\n");
    return ERROR;
  }
  return SCAN_SEND_MESSAGE;
}

bool YDLidarX4StateMachine::isCorrectCrc(){
  uint16_t ph = paket.header;
  uint8_t type =  paket.type;
  uint8_t sampleQuantity = paket.quantity;
  uint16_t startAngle = paket.startAngle;
  uint16_t lastAngle = paket.lastAngle;
  uint16_t cs =  paket.checkSum;

  uint16_t ct_lsn = 256 * sampleQuantity + type;

  uint16_t crc = 0;
  crc ^= ph;
  crc ^= ct_lsn;
  crc ^= startAngle;
  crc ^= lastAngle;
  for(uint16_t sampleId=0; sampleId<(uint16_t)sampleQuantity; sampleId++){
    uint16_t distance = paket.samples[sampleId];
    crc ^= distance;
  }

  if(cs != crc){
    uint16_t diff_crc = cs^crc;
    IF_DEBUG debug->printf("crc_error --> crc_calc(%s) crc_pkg(%s) diff(%s)\n", hex(crc).c_str(), hex(cs).c_str(), hex(diff_crc).c_str());
    for(int i=0; i<(sampleBeginPos+sizeof(uint16_t)*256); i++){
      IF_DEBUG debug->printf("0x%s ",hex(paket.rawData[i]).c_str());
    }
    IF_DEBUG debug->println();
  }
  return cs == crc;
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateScanSendMessages(){
  handleValidPacket();
  return SCAN_NEED_HEADER;
}

void YDLidarX4StateMachine::handleValidPacket(){
  uint8_t type =  paket.type;
  uint8_t sampleQuantity = paket.quantity;
  uint16_t startAngle = paket.startAngle;
  uint16_t lastAngle = paket.lastAngle;

  float startAngleInDegree = angleInDegree(startAngle);
  float stopAngleInDegree = angleInDegree(lastAngle);
  bool isIndexPaket = type==indexPacket;

  float ranges[sampleQuantity];
  float anglesInDegree[sampleQuantity];
  calculateRangesAndAnglesFromPaket(paket, sampleQuantity, startAngleInDegree, stopAngleInDegree, ranges, anglesInDegree);
  printRangesAndAnglesFromPaket(isIndexPaket, sampleQuantity, startAngleInDegree, stopAngleInDegree, anglesInDegree, ranges);
  notifyHandlers(isIndexPaket, startAngleInDegree, stopAngleInDegree, anglesInDegree, ranges, sampleQuantity);
}

// tested: 0x6fe5 | fsa 223.78=223.781 | cfsa -6.7622=-6.76219 | 217.0178 degree = 217.019
// tested: 0x79bd | lsa 243.47=243.469 | lfsa -7.8374=-7.83743 | 235.6326 degree = 235.631
void YDLidarX4StateMachine::calculateRangesAndAnglesFromPaket(Paket &paket, uint8_t sampleQuantity, float startAngleInDegree, float stopAngleInDegree, float* ranges, float* anglesInDegree){
  for(uint16_t sampleNum=0; sampleNum<(uint16_t)sampleQuantity; sampleNum++){
    uint16_t distanceBytes_i = paket.samples[sampleNum];

    float distanceValue_i = distanceInMillimeter(distanceBytes_i);
    ranges[sampleNum] = distanceValue_i;
    anglesInDegree[sampleNum] = calculateAngleInDegree(distanceValue_i, startAngleInDegree, stopAngleInDegree, sampleNum, sampleQuantity);
  }
}

void YDLidarX4StateMachine::printRangesAndAnglesFromPaket(bool isIndexPaket, size_t lenght, float minAngleInDegree, float maxAngleInDegree, float anglesInDegree[], float ranges[]){
  if(trace == &DummyPrint){
    return;
  }
  if(logLevel != LOG_TRACE){
    return;
  }

  trace->printf("   /--------------------\n");
  trace->printf("   | paket type: %s\n", isIndexPaket?"INDEX":"SCAN");
  trace->printf("   | angles %f° - %f°\n", minAngleInDegree, maxAngleInDegree);
  for(uint16_t sampleNum=0; sampleNum<(uint16_t)lenght; sampleNum++){
    trace->printf("   | distance at %f°: %fmm\n", anglesInDegree[sampleNum], ranges[sampleNum]);
  }
  trace->printf("   \\--------------------\n");
  // trace->printf("PKG: angles %f° - %f° => %d messures\n", minAngleInDegree, maxAngleInDegree, lenght);
}

float YDLidarX4StateMachine::calculateAngleInDegree(float distanceValue_i, float startAngleInDegree, float stopAngleInDegree, uint16_t sampleNum, uint8_t sampleQuantity){
  float angleInDegree_i = startAngleInDegree + sampleNum * (stopAngleInDegree - startAngleInDegree)/(sampleQuantity - 1);
  float angleInDegree_correced_i = angleInDegree_i + calculateCorrectingAngleInDegree(distanceValue_i);
  return angleInDegree_correced_i;
}

void YDLidarX4StateMachine::notifyHandlers(bool isIndexPaket, float minAngleInDegree, float maxAngleInDegree, float anglesInDegree[], float ranges[], size_t lenght){
  if(isIndexPaket){
    notifyIndexPacketHandler(minAngleInDegree, maxAngleInDegree, anglesInDegree, ranges);
  }else{
    notifyPacketHandler(minAngleInDegree, maxAngleInDegree, anglesInDegree, ranges, lenght);
  }
}


void YDLidarX4StateMachine::notifyIndexPacketHandler(float minAngleInDegree, float maxAngleInDegree, float anglesInDegree[], float ranges[]){
  if(indexPacketHandler == nullptr){
    if(packetHandler != nullptr){
      packetHandler->operator()(minAngleInDegree, maxAngleInDegree, anglesInDegree, ranges, 1);
    }
  }else{
    indexPacketHandler->operator()(anglesInDegree[0], ranges[0]);
  }
}

void YDLidarX4StateMachine::notifyPacketHandler(float minAngleInDegree, float maxAngleInDegree, float anglesInDegree[], float ranges[], size_t lenght){
  if(packetHandler != nullptr){
    packetHandler->operator()(minAngleInDegree, maxAngleInDegree, anglesInDegree, ranges, lenght);
  }
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateStop(){
  queue.clear();
  return END;
}

YDLidarX4StateMachine::State YDLidarX4StateMachine::handleStateTimeout(){
  return TIMEOUT;
}

void YDLidarX4StateMachine::debugPrintQueue(const char *message){
  IF_DEBUG debugPrintQueue(debug, message);
}

void YDLidarX4StateMachine::debugPrintQueue(Print* out, const char *message){
  if(out == nullptr){
    return;
  }
  size_t count = queue.size();
  size_t head = queue.getHead();
  size_t tail = queue.getTail();
  out->printf("queue%s[%lu]{head:%lu,tail:%lu,count:%lu}: ", message, count, head, tail, count);
  for(size_t i = 0; i < count; i++){
    if(i==packetSampleQuantity) out->print("(");
    if(i>=sampleBeginPos && i%2==0 && i<(sampleBeginPos+2*queue.get(packetSampleQuantity))) out->print("[");
    out->printf("0x%s", hex(queue.get(i)).c_str());

    if(i==packetSampleQuantity) out->printf("=%u)", queue.get(i));

    if(i>=sampleBeginPos && i%2==1 && i<(sampleBeginPos+2*queue.get(packetSampleQuantity))) out->printf("]%d", ((i-sampleBeginPos)/2)+1);
    if(i>packetSampleQuantity && i==(sampleBeginPos+2*queue.get(packetSampleQuantity)-1)) out->print(" |");
    out->print(" ");
  }
  out->println();
}

void YDLidarX4StateMachine::debugPrintFullQueue(const char *message){
  IF_DEBUG debugPrintFullQueue(debug, message);
}

void YDLidarX4StateMachine::debugPrintFullQueue(Print* out, const char *message){
  if(out == nullptr){
    return;
  }
  size_t count = queue.size();
  size_t capacity = queue.getCapacity();
  size_t head = queue.getHead();
  size_t tail = queue.getTail();
 
  // print prev: tail->head
  size_t i = 0;
  out->print("processed:");
  for(; i < capacity-count; i++){
    size_t index = i + tail;
    out->print(hex(queue.getRaw(index)).c_str());
    out->print(" ");
  }
  out->println("");

  // print curr: head->tail
  out->print("unprocessed:");
  for(; i < capacity; i++){
    size_t index = i + tail;
    out->print(hex(queue.getRaw(index)).c_str());
    out->print(" ");
  }
  out->println("");
}

void YDLidarX4StateMachine::debugPrintRawQueue(const char *message){
  IF_DEBUG debugPrintRawQueue(debug, message);
}

void YDLidarX4StateMachine::debugPrintRawQueue(Print* out, const char *message){
  if(out == nullptr){
    return;
  }
  size_t count = queue.size();
  size_t head = queue.getHead();
  size_t tail = queue.getTail();
  string msg = string("queue") + message + "[" + to_string(count) + "]: ";
  string msgSpaces = string(msg.length(), ' ');
 
  // draw head position
  out->print(msgSpaces.c_str());
  string headString = string(2*head, '_').append("_↓ head"); // todo does not work
  out->println(headString.c_str());

  // output data
  out->print(msg.c_str());
  for(size_t i = 0; i < queue.getCapacity(); i++){
    out->print(hex(queue.getRaw(i)).c_str());
    out->print(" ");
  }
  out->println("");

  // draw tail position
  out->print(msgSpaces.c_str());
  string tailString = string(2*tail, '_').append("_↑ tail");
  out->println(tailString.c_str());
}

void YDLidarX4StateMachine::setLogLevel(LogLevel logLevel){
  this->logLevel = logLevel;
}


string YDLidarX4StateMachine::hex(uint8_t value){
  char output[3];
  sprintf(output, "%02X", value);
  return string(output);
}

string YDLidarX4StateMachine::hex(uint16_t value){
  char output[5];
  sprintf(output, "%02X%02X", (value&0xff), (value>>8));
  return string(output);
}

// --- helper funktions ---------------------------------------------------------------------
float YDLidarX4StateMachine::angleInDegree(uint16_t value){
  uint16_t value_shifted = (value>>1);
  double angle = value_shifted/64.0;
  return angle;
}

// tested : 0x6fe5 should be 7161.25mm = 7161.25
float YDLidarX4StateMachine::distanceInMillimeter(uint16_t value){
  return value/4.0;
}
float YDLidarX4StateMachine::calculateCorrectingAngleInDegree(float distance){
  if(distance == 0){
    return 0;
  }
  static float inDegree = 180/3.14159265359;
  return atan(21.8*((155.3-distance)/(155.3*distance)))*inDegree;
}

} // end namespace
