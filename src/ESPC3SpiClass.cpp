#include "ESPC3SpiClass.h"

volatile bool _readflag = false;

ESPC3SpiClass::ESPC3SpiClass() {

}

ESPC3SpiClass::~ESPC3SpiClass() {

}

void isrRead() {
  _readflag = true;
}

void ESPC3SpiClass::begin(uint8_t baud) {
  pinMode(NINA_GPIO0, OUTPUT);
  pinMode(NINA_RESETN, OUTPUT);

  digitalWrite(NINA_GPIO0, HIGH);
  delay(100);
  digitalWrite(NINA_RESETN, HIGH);
  digitalWrite(NINA_RESETN, LOW);
  digitalWrite(NINA_RESETN, HIGH);

  pinMode(_cs, OUTPUT);
  pinMode(_hs, INPUT);
  digitalWrite(_cs, HIGH);
  _spi.begin();
  attachInterrupt(_hs, isrRead, RISING);
}

uint8_t ESPC3SpiClass::readFromSlave(uint8_t * rxBuffer) {
  if (!_readflag) {
    return 0;
  }
  _readflag = false;


  uint16_t raedableByte = queryRxStatus();
  uint8_t bufferc[4096];
  sendRxSpiRequest(rxBuffer, raedableByte);
  for (int i = 0; i < raedableByte; i++) {
    rxBuffer[i] = rxBuffer[3 + i];
    _rxBuffer.store_char(rxBuffer[i]);
  }
  rxDone();

  return raedableByte;
}


uint8_t ESPC3SpiClass::readFromSlave() {
  uint8_t rxBuffer[4096];
  if (!_readflag) {
    return 0;
  }

  _readflag = false;

  uint16_t raedableByte = queryRxStatus();
  uint8_t bufferc[4096];
  sendRxSpiRequest(rxBuffer, raedableByte);
  for (int i = 0; i < raedableByte; i++) {
    rxBuffer[i] = rxBuffer[3 + i];
    _rxBuffer.store_char(char(rxBuffer[i]));
      //Serial.print((char)rxBuffer[i]);
  }
  //Serial.println();
  rxDone();

  return raedableByte;
}

void ESPC3SpiClass::writeToSlave(uint8_t *data, uint16_t size) {
  notifyWrite();
  delay(10);
  if (!_readflag) {
    return;
  }
  _readflag = false;

  // query status
  queryTxStatus();
  txSpiRequest(data, size);
  txDone();
}

size_t ESPC3SpiClass::write(uint8_t data){
  char tmp = (char) data;
  _txBuffer[_head] = tmp;
  _head++;
  Serial.print((char)data);
  if(_head >= 2 && _txBuffer[_head-2] == '\r' && _txBuffer[_head-1] == '\n' ) {
    writeToSlave((uint8_t*)_txBuffer, _head);
    memset(_txBuffer, 0, 255);
    _head = 0;
    Serial.println();
  }
}

size_t ESPC3SpiClass::write(uint8_t * data, size_t size){

  writeToSlave(data, (uint16_t)size);
}

int ESPC3SpiClass::available(void){
  //Serial.println("m_puart->available()");
   if(!(_rxBuffer.available())){
     readFromSlave();
   } 
    //Serial.println(_rxBuffer.available());
  //delay(10);
  return _readflag || _rxBuffer.available();//_rxBuffer.available();
}

int ESPC3SpiClass::read(void) {
  //if(!_rxBuffer.available()){
    // readFromSlave();
    // if(!_rxBuffer.available()) {
    //   return NULL;
    // }
  //}
  return _rxBuffer.read_char();

}

int ESPC3SpiClass::peek(void) {
  return _rxBuffer.peek();
}

void ESPC3SpiClass::flush(void) {
  //_rxBuffer.flush();
}

size_t ESPC3SpiClass::print(const char data[]) {
  writeToSlave((uint8_t*)data, (uint16_t)sizeof(data));
}

size_t ESPC3SpiClass::print(char data) {
//  Serial.println("print(char data)");
}

size_t ESPC3SpiClass::println(char data) {
 // Serial.println("println(char data)");
}

size_t ESPC3SpiClass::print(const __FlashStringHelper * ifsh){
  const char * data = reinterpret_cast<const char *>(ifsh);
  uint8_t txbuffer[4096];
  uint16_t size = (uint16_t) strlen(data);
  for(int i =0; i<size;i++){
    txbuffer[i] = data[i];
  }
  txbuffer[size++] = 0x0D;
  txbuffer[size++] = 0x0A;
  writeToSlave(txbuffer, size);
}

size_t ESPC3SpiClass::println(const __FlashStringHelper * ifsh){
  const char * data = reinterpret_cast<const char *>(ifsh);
  uint8_t txbuffer[4096];
  uint16_t size = (uint16_t) strlen(data);
  for(int i =0; i<size;i++){
    txbuffer[i] = data[i];
  }
  txbuffer[size++] = 0x0D;
  txbuffer[size++] = 0x0A;
  writeToSlave(txbuffer, size);
}

size_t ESPC3SpiClass::println(const String &s){
  char buffer[s.length() + 1];
  s.toCharArray(buffer, s.length() + 1);
  uint16_t size = s.length();
  buffer[size++] = 0x0D;
  buffer[size++] = 0x0A;

  writeToSlave((uint8_t*)buffer, size);
}

size_t ESPC3SpiClass::print(const String &s) {
  char buffer[s.length() + 1];
  s.toCharArray(buffer, s.length() + 1);
  uint16_t size = s.length();
  buffer[size++] = 0x0D;
  buffer[size++] = 0x0A;

  writeToSlave((uint8_t*)buffer, size);
}

size_t ESPC3SpiClass::println(const char data[]) {
  uint8_t * txBuffer;
  txBuffer = (uint8_t *) data;
  uint16_t size = (uint16_t) sizeof(data);
  txBuffer[size++] = 0x0D;
  txBuffer[size++] = 0x0A;

  writeToSlave(txBuffer, size);
}

size_t ESPC3SpiClass::write(const char *str){
  writeToSlave((uint8_t*)str, (uint16_t)sizeof(str));
}

void ESPC3SpiClass::sendRxSpiRequest(uint8_t *rxBuffer, uint16_t raedableByte) {
  // read data
  uint8_t bufferc[4096];
  bufferc[0] = 0x04;
  bufferc[1] = 0x00;
  bufferc[2] = 0x00;
  for (int i = 0; i < raedableByte; i++) {
    bufferc[3 + i] = 0xFF;
  }

  readSpi(bufferc, 3 + raedableByte, rxBuffer);
}

void ESPC3SpiClass::txSpiRequest(uint8_t * data, uint16_t size) {
  // write data
  uint8_t bufferc[4096];
  bufferc[0] = 0x03;
  bufferc[1] = 0x00;
  bufferc[2] = 0x00;
  for (int i = 0; i < (size) ; i++) {
    bufferc[i + 3] = (uint8_t) data[i];
   // Serial.print((char)data[i]);
  }
  Serial.print("size ");
  Serial.print(size);
  Serial.println();
  writeSpi((void *)bufferc, size + 3);
}

uint16_t ESPC3SpiClass::queryRxStatus() {
  uint8_t buffers[7];
  uint16_t raedableByte = 0x00;
  buffers[0] = 0x02;
  buffers[1] = 0x04;
  buffers[2] = 0x00;
  buffers[3] = 0xFF;
  buffers[4] = 0xFF;
  buffers[5] = 0xFF;
  buffers[6] = 0xFF;

  uint8_t readBuffer[7];
  readSpi(buffers, sizeof(buffers), readBuffer);
  for(int i = 0; i<sizeof(readBuffer); i++){
    Serial.print(readBuffer[i],HEX);
    Serial.print(" ");
  }
  Serial.println();
  raedableByte |= readBuffer[6];
  raedableByte = raedableByte << 8;
  raedableByte |= readBuffer[5];
  Serial.print("size from rx response: ");
  Serial.println(raedableByte);
   
  return raedableByte;
}

void ESPC3SpiClass::queryTxStatus() {
  uint8_t buffers[7];
  buffers[0] = 0x02;
  buffers[1] = 0x04;
  buffers[2] = 0x00;
  buffers[3] = 0xFF;
  buffers[4] = 0xFF;
  buffers[5] = 0xFF;
  buffers[6] = 0xFF;
  writeSpi((void *)buffers, sizeof(buffers));
}

void ESPC3SpiClass::readSpi(uint8_t *data, int size, uint8_t *readBuffer) {
  uint8_t read[4096];

  digitalWrite(_cs, LOW);

  delayMicroseconds(1);

  _spi.beginTransaction(_spiSettings);

  for (int i = 0; i < size; i++) {
    readBuffer[i] = _spi.transfer(data[i]);
  }
  _spi.endTransaction();

  digitalWrite(_cs, HIGH);
}

uint32_t ESPC3SpiClass::readSpi() {
  uint32_t read = 0x00;

  digitalWrite(_cs, LOW);

  delayMicroseconds(1);

  _spi.beginTransaction(_spiSettings);


  for (int i = 0; i < 4; i++) {
    read <<= 8;
    read |= _spi.transfer(0);
    read = _spi.transfer(0);
  }

  _spi.endTransaction();

  digitalWrite(_cs, HIGH);
  return read;
}

void ESPC3SpiClass::writeSpi(void *data, int size) {

  digitalWrite(_cs, LOW);

  delayMicroseconds(1);
  _spi.beginTransaction(_spiSettings);

  _spi.transfer(data, size);

  _spi.endTransaction();
  digitalWrite(_cs, HIGH);
}

void ESPC3SpiClass::rxDone() {
  // read done
  uint8_t bufferd[3];
  bufferd[0] = 0x08;
  bufferd[1] = 0x00;
  bufferd[2] = 0x00;
  writeSpi((void *)bufferd, sizeof(bufferd));
}

void ESPC3SpiClass::txDone() {
  // write done
  uint8_t bufferd[3];
  bufferd[0] = 0x07;
  bufferd[1] = 0x00;
  bufferd[2] = 0x00;
  writeSpi((void *)bufferd, sizeof(bufferd));
}

void ESPC3SpiClass::notifyWrite() {
  // Notify write
  uint8_t bufferw[7];
  bufferw[0] = 0x01;
  bufferw[1] = 0x00;
  bufferw[2] = 0x00;
  bufferw[3] = 0XFE;
  bufferw[4] = 0X10;
  bufferw[5] = 0x04;
  bufferw[6] = 0;
  writeSpi((void *)bufferw, sizeof(bufferw));
}
