#include "Arduino.h"
#include "SPI.h"
#include "RingBuffer.h"
#include "Printable.h"

#ifndef _ESP_ARDUINO_CLASS_H
#define _ESP_ARDUINO_CLASS_H

#ifndef NINA_GPIO0
#define NINA_GPIO0      (28)
#endif

#ifndef NINA_RESETN
#define NINA_RESETN     (29)
#endif


static bool _readflag = false;
void isrRead();

class ESPC3SpiClass : public Stream {
public:
  ESPC3SpiClass();
  ~ESPC3SpiClass();
  void begin();
  uint8_t readFromSlave(uint8_t * rxBuffer);
  uint8_t readFromSlave();
  void writeToSlave(uint8_t *data, uint16_t size = 0x01U);
  virtual size_t write(uint8_t);
  virtual size_t write(uint8_t *, size_t);
  size_t write(const char *str);
  virtual int available(void);
  virtual int read(void);
  virtual int peek(void);
  virtual void flush(void);
  size_t print(const char[]);
  size_t print(const String &s);
  size_t print(char);
  size_t println(const String &s) override;
  size_t println(const char[]) override;
  size_t println(char);
  size_t print(const __FlashStringHelper *);
  size_t println(const __FlashStringHelper *);

private:
  void sendRxSpiRequest(uint8_t *rxBuffer, uint8_t raedableByte);
  void txSpiRequest(uint8_t * data, uint16_t size);
  uint8_t queryRxStatus();
  void queryTxStatus();
  void readSpi(uint8_t *data, int size, uint8_t *readBuffer);
  uint32_t readSpi();
  void writeSpi(void *data, int size);
  void rxDone();
  void txDone();
  void notifyWrite();


private:
  SPIClass& _spi = SPI1;
  SPISettings _spiSettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
  //uint8_t * _rxBuffer;
  RingBufferN<4096> _rxBuffer;
  int _cs = 31;
  byte _hs = 28;

};
#endif //_ESP_ARDUINO_CLASS_H
