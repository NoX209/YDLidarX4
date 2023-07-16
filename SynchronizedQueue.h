#ifndef __SYNCHRONIZED_QUEUE__
#define __SYNCHRONIZED_QUEUE__

#include <cstring>
using std::memcpy;

namespace sensorYDLidarX4 {

/*
this class writes new elements to the end
and provide a copy of selected size from the head
*/

template <typename Type>
class SynchronizedQueue{
  private:
    size_t maxElements;
    Type* buffer;
    size_t head;
    size_t tail;
    size_t count;
    SemaphoreHandle_t  lock;

    size_t wrap(size_t);

  public:
    SynchronizedQueue(size_t maxElements);
    ~SynchronizedQueue();
    bool add(Type element);
    bool add(Type *elements, size_t size);
    Type get(size_t index);
    Type getRaw(size_t index);
    Type dequeue();
    bool extract(Type *targetBuffer, size_t size);
    void remove(size_t sizeToRemove);
    void clear();
    bool isFull();
    bool isEmpty();
    size_t size();
    size_t getCapacity();
    size_t getHead();
    size_t getTail();
};

template <typename Type>
SynchronizedQueue<Type>::SynchronizedQueue(size_t elementCount)
  : lock(xSemaphoreCreateMutex()),
  head(0),
  tail(0),
  count(0),
  maxElements(elementCount)
{
  buffer = new Type[maxElements];
}

template <typename Type>
SynchronizedQueue<Type>::~SynchronizedQueue(){
  delete[] buffer;
}

template <typename Type>
inline size_t SynchronizedQueue<Type>::wrap(size_t size){
  return size % maxElements;
}

template <typename Type>
bool SynchronizedQueue<Type>::add(Type element){
  bool dataWasAdded;
  xSemaphoreTake(lock, portMAX_DELAY); 
  if(count < maxElements){
    buffer[tail] = element;
    tail = wrap(++tail);
    ++count;
    dataWasAdded = true;
  }else{
    dataWasAdded = false;
  }
  xSemaphoreGive(lock); 
  return dataWasAdded;
}

template <typename Type>
bool SynchronizedQueue<Type>::add(Type *elements, size_t size){
  bool dataWasAdded;
  xSemaphoreTake(lock, portMAX_DELAY); 
  if(count + size < maxElements){
    for(size_t i=0; i<size; i++){
      buffer[tail] = elements[i];
      tail = wrap(++tail);
    }
    count += size;
    dataWasAdded = true;
  }else{
    dataWasAdded = false;
  }
  xSemaphoreGive(lock);
  return dataWasAdded;
}

template <typename Type>
Type SynchronizedQueue<Type>::get(size_t index){
  uint8_t element;
  xSemaphoreTake(lock, portMAX_DELAY); 
  if(index < count){
    element = buffer[wrap(head+index)];
  }
  xSemaphoreGive(lock); 
  return element;
}

template <typename Type>
Type SynchronizedQueue<Type>::getRaw(size_t index){
  uint8_t element;
  xSemaphoreTake(lock, portMAX_DELAY); 
  if(index < maxElements){
    element = buffer[wrap(index)];
  }
  xSemaphoreGive(lock); 
  return element;
}

template <typename Type>
Type SynchronizedQueue<Type>::dequeue(){
  uint8_t element;
  xSemaphoreTake(lock, portMAX_DELAY); 
  if(0 < count){
    element = buffer[head];
    head = wrap(++head);
    count -= 1;
  }
  xSemaphoreGive(lock); 
  return element;
}

template <typename Type>
bool SynchronizedQueue<Type>::extract(Type *targetBuffer, size_t size){
  bool isExtracted = false;
  xSemaphoreTake(lock, portMAX_DELAY); 
  if(size <= count){
    if(head<tail || size<=maxElements-head){
      // normal - one block
      memcpy(&targetBuffer[0], &buffer[head], size);
    }else{
      // with wrap around - two blocks
      size_t sizeOfFirstPart = maxElements-head;
      // 1. head --> maxElements
      memcpy(&targetBuffer[0], &buffer[head], sizeOfFirstPart);
      // 2. 0 --> size-sizeOfFirstPart
      memcpy(&targetBuffer[sizeOfFirstPart], &buffer[0], size-sizeOfFirstPart);
    }
    head = wrap(head + size);
    count -= size;
    isExtracted = true;
  }
  xSemaphoreGive(lock); 
  return isExtracted;
}

template <typename Type>
void SynchronizedQueue<Type>::remove(size_t sizeToRemove){
  xSemaphoreTake(lock, portMAX_DELAY);
  if(sizeToRemove <= count){
    head = wrap(head + sizeToRemove);
    count -= sizeToRemove;
  }
  xSemaphoreGive(lock); 
}

template <typename Type>
void SynchronizedQueue<Type>::clear(){
  xSemaphoreTake(lock, portMAX_DELAY);
  head = tail;
  count = 0;
  xSemaphoreGive(lock); 
}

template <typename Type>
bool SynchronizedQueue<Type>::isFull(){
  bool isFull;
  xSemaphoreTake(lock, portMAX_DELAY); 
  isFull = count == maxElements;
  xSemaphoreGive(lock); 
  return isFull;
}

template <typename Type>
bool SynchronizedQueue<Type>::isEmpty(){
  bool isEmpty;
  xSemaphoreTake(lock, portMAX_DELAY); 
  isEmpty = count == 0;
  xSemaphoreGive(lock); 
  return isEmpty;
}

template <typename Type>
size_t SynchronizedQueue<Type>::size(){
  size_t result;
  xSemaphoreTake(lock, portMAX_DELAY); 
  result = count;
  xSemaphoreGive(lock); 
  return result;
}

template <typename Type>
size_t SynchronizedQueue<Type>::getCapacity(){
  return maxElements;
}

template <typename Type>
size_t SynchronizedQueue<Type>::getHead(){
  return head;
}

template <typename Type>
size_t SynchronizedQueue<Type>::getTail(){
  return tail;
}

} // end namespace

#endif
