#include <DummyPrint.h>

size_t _DummyPrint::printf(const char * format, ...){
  return 0;
};
size_t _DummyPrint::print(const __FlashStringHelper *){
  return 0;
};
size_t _DummyPrint::print(const String &){
  return 0;
};
size_t _DummyPrint::print(const char[]){
  return 0;
};
size_t _DummyPrint::print(char){
  return 0;
};
size_t _DummyPrint::print(unsigned char, int){
  return 0;
};
size_t _DummyPrint::print(int, int){
  return 0;
};
size_t _DummyPrint::print(unsigned int, int){
  return 0;
};
size_t _DummyPrint::print(long, int){
  return 0;
};
size_t _DummyPrint::print(unsigned long, int){
  return 0;
};
size_t _DummyPrint::print(long long, int){
  return 0;
};
size_t _DummyPrint::print(unsigned long long, int){
  return 0;
};
size_t _DummyPrint::print(double, int){
  return 0;
};
size_t _DummyPrint::print(const Printable&){
  return 0;
};
size_t _DummyPrint::print(struct tm * timeinfo, const char * format){
  return 0;
};

size_t _DummyPrint::println(const __FlashStringHelper *){
  return 0;
};
size_t _DummyPrint::println(const String &s){
  return 0;
};
size_t _DummyPrint::println(const char[]){
  return 0;
};
size_t _DummyPrint::println(char){
  return 0;
};
size_t _DummyPrint::println(unsigned char, int){
  return 0;
};
size_t _DummyPrint::println(int, int){
  return 0;
};
size_t _DummyPrint::println(unsigned int, int){
  return 0;
};
size_t _DummyPrint::println(long, int){
  return 0;
};
size_t _DummyPrint::println(unsigned long, int){
  return 0;
};
size_t _DummyPrint::println(long long, int){
  return 0;
};
size_t _DummyPrint::println(unsigned long long, int){
  return 0;
};
size_t _DummyPrint::println(double, int){
  return 0;
};
size_t _DummyPrint::println(const Printable&){
  return 0;
};
size_t _DummyPrint::println(struct tm * timeinfo, const char * format){
  return 0;
};
size_t _DummyPrint::println(void){
  return 0;
};

_DummyPrint DummyPrint;
