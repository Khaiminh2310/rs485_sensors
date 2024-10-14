/*!
 *  @file OiltankModbus.cpp
 *
 *  This library is for the "Oil Tank RS485 Modbus RTU" series of sensors.
 *  These sensors communicate via RS485 standard.
 *  So you will need an "RS485 to UART (TTL) communication converter circuit" for use with Microcontrollers.
 *  Like, UNO board, MEGA board, ...
 *
 *  Wiring diagram of sensor with "Communication Converter Circuit":
 *  V+      : BROWN         - Power supply 5~28VDC
 *  V-      : BLACK         - Power supply 0VDC (Mass)
 *  RS485-A : YELLOW/GREEN  - Signal wire A
 *  RS485-B : BLUE          - Signal wire B
 *
 *  @author khaiminh2310
 */

#include "OiltankModbus.h"

/* ------------------------------ Constructor ------------------------------ */

OILTANK_Sensor::OILTANK_Sensor(const char *id)
{
  port = &Serial;
  typeSerial = HARD_SERIAL;

  _ctrlPin = 0xFF;
  convertID(id, _addr, 3);
}

OILTANK_Sensor::OILTANK_Sensor(uint8_t rxPin, uint8_t txPin, const char *id)
{
  SoftwareSerial *ss = new SoftwareSerial(rxPin, txPin);
  port = ss;
  typeSerial = SOFT_SERIAL;

  _ctrlPin = 0xFF;
  convertID(id, _addr, 3);
}

OILTANK_Sensor::OILTANK_Sensor(uint8_t rxPin, uint8_t txPin, uint8_t ctrlPin, const char *id)
{
  SoftwareSerial *ss = new SoftwareSerial(rxPin, txPin);
  port = ss;
  typeSerial = SOFT_SERIAL;

  _ctrlPin = ctrlPin;
  convertID(id, _addr, 3);
}

/* ----------------------------- Initialization ---------------------------- */

void OILTANK_Sensor::begin()
{
  DEBUG_PRINTF("Starting oiltank sensor with baudrate = %u...", BAUDRATE_DEFAULT);
  if (typeSerial == HARD_SERIAL)
  {
    HardwareSerial *hs = (HardwareSerial *)port;
    hs->begin(BAUDRATE_DEFAULT);
    hs->setTimeout(1000);
  }
  else if (typeSerial == SOFT_SERIAL)
  {
    SoftwareSerial *ss = (SoftwareSerial *)port;
    ss->begin(BAUDRATE_DEFAULT, SWSERIAL_8N1);
    ss->setTimeout(1000);
  }
  DEBUG_PRINTLN("Done");
}

void OILTANK_Sensor::setTimeout(uint16_t timeOut)
{
  if (timeOut < MIN_OILTANK_TIMEOUT)
  {
    _timeOut = MIN_OILTANK_TIMEOUT;
  }
  else if (timeOut > MAX_OILTANK_TIMEOUT)
  {
    _timeOut = MAX_OILTANK_TIMEOUT;
  }
  else
    _timeOut = timeOut;
}

/* --------------------------------- Struct -------------------------------- */

dataOiltank OILTANK_Sensor::getData()
{
  dataOiltank value;

  float *_level = readLevel();
  delay(1000);
  float *_all_temp = readFiveTemperature();
  delay(1000);
  float _avg_temp = readAverageTemperature();

  value.averageTemperature = _avg_temp;
  value.productLevel = _level[0];
  value.waterLevel   = _level[1];
  value.temperature1 = _all_temp[0];
  value.temperature2 = _all_temp[1];
  value.temperature3 = _all_temp[2];
  value.temperature4 = _all_temp[3];
  value.temperature5 = _all_temp[4];

  return value;
}

/* ------------------------------- Read Data ------------------------------- */

float *OILTANK_Sensor::readLevel()
{
  DEBUG_PRINTLN(">>> Start read Level...");

  float *level = new float[2]{0.0, 0.0};
  bool complete = false;
  uint8_t myBuf[LEVEL_FRAME_SIZE];

  /* Remove all previous junk data (if have) */
  while (port->available())
  {
    DEBUG_PRINTLN("Remove all previous junk data");
    port->read();
  }

  /* Send level reading command
   * Command frame format structure:
   *
   * Frame head     : 1 Byte (0x68)
   * Frame length   : 1 Byte (0x05)
   * ID             : 3 Byte
   * Data type      : 1 Byte (0x50)
   * Data           : 1 Byte (0x00)
   * H_CRC          : 1 Byte
   * L_CRC          : 1 Byte
   * Frame tail     : 1 Byte (0x16)
   */
  if (_ctrlPin != 0xFF)
  {
    digitalWrite(_ctrlPin, RS485Transmit);
    delay(20);
  }

  getLevel[2] = _addr[0];
  getLevel[3] = _addr[1];
  getLevel[4] = _addr[2];

  uint16_t crc = crcCheck(&getLevel[2], CRC_LENGTH);
  getLevel[7] = (crc >> 8) & 0xFF;  // High byte
  getLevel[8] = crc & 0xFF;         // Low byte

#if defined(ENABLE_OILTANK_DEBUG)
  DEBUG_PRINT("SEND: ");
  for (int i = 0; i < ASK_FRAME_SIZE; i++)
  {
    DEBUG_PRINTF("%02X ", getLevel[i]);
  }
  DEBUG_PRINTLN("");
#endif

  port->write(getLevel, ASK_FRAME_SIZE);

  unsigned long now = millis();

  /* Number of bytes to respond
  *
  * Frame head     : 1 Byte (0x68)
  * Frame length   : 1 Byte
  * ID             : 3 Byte
  * Data type      : 1 Byte (0x50)
  * Data           : 6 Byte
  * H_CRC          : 1 Byte
  * L_CRC          : 1 Byte
  * Frame tail     : 1 Byte (0x16)
  */
  if (_ctrlPin != 0xFF)
  {
    digitalWrite(_ctrlPin, RS485Receive);
  }

  while ((millis() - now) < _timeOut)
  {
    if (port->available())
    {
      port->readBytes(myBuf, LEVEL_FRAME_SIZE);
      complete = true;
      break;
    }
  }

  if (complete)
  {
#if defined(ENABLE_OILTANK_DEBUG)
    DEBUG_PRINT("RECEIVED: ");
    for (int i = 0; i < LEVEL_FRAME_SIZE; i++)
    {
      DEBUG_PRINTF("%02X ", myBuf[i]);
    }
    DEBUG_PRINTLN("");
#endif
    /* Check CRC */
    if (crcCheck(&myBuf[2], myBuf[1]) == static_cast<uint16_t>(myBuf[LEVEL_FRAME_SIZE-2] | static_cast<uint16_t>(myBuf[LEVEL_FRAME_SIZE-3]) << 8))
    {
      level[0] = static_cast<float>(myBuf[6] << 16 | myBuf[7] << 8 | myBuf[8]) * 0.01 / 1000;
      level[1] = static_cast<float>(myBuf[9] << 16 | myBuf[10] << 8 | myBuf[11]) * 0.01 / 1000;
      DEBUG_PRINTF("Product level: %.3f m\r\n", level[0]);
      DEBUG_PRINTF("Water level: %.3f m\r\n", level[1]);
      DEBUG_PRINTLN("");
    }
    else
    {
      DEBUG_PRINTLN("Error CRC in read Level");
    }
  }
  else
  {
    DEBUG_PRINTLN("Get timeout in read Level");
  }

  return level;
}

float *OILTANK_Sensor::readFiveTemperature()
{
  DEBUG_PRINTLN(">>> Start read Five Temperature...");

  float *all_temp = new float[5]{0.0, 0.0, 0.0, 0.0, 0.0};
  bool complete = false;
  uint8_t myBuf[FIVE_TEMP_FRAME_SIZE];

  /* Remove all previous junk data (if have) */
  while (port->available())
  {
    DEBUG_PRINTLN("Remove all previous junk data");
    port->read();
  }

  /* Send level reading command
   * Command frame format structure:
   *
   * Frame head     : 1 Byte (0x68)
   * Frame length   : 1 Byte (0x05)
   * ID             : 3 Byte
   * Data type      : 1 Byte (0x52)
   * Data           : 1 Byte (0x00)
   * H_CRC          : 1 Byte
   * L_CRC          : 1 Byte
   * Frame tail     : 1 Byte (0x16)
   */
  if (_ctrlPin != 0xFF)
  {
    digitalWrite(_ctrlPin, RS485Transmit);
    delay(20);
  }

  getFiveTemperature[2] = _addr[0];
  getFiveTemperature[3] = _addr[1];
  getFiveTemperature[4] = _addr[2];

  uint16_t crc = crcCheck(&getFiveTemperature[2], CRC_LENGTH);
  getFiveTemperature[7] = (crc >> 8) & 0xFF;  // High byte
  getFiveTemperature[8] = crc & 0xFF;         // Low byte

#if defined(ENABLE_OILTANK_DEBUG)
  DEBUG_PRINT("SEND: ");
  for (int i = 0; i < ASK_FRAME_SIZE; i++)
  {
    DEBUG_PRINTF("%02X ", getFiveTemperature[i]);
  }
  DEBUG_PRINTLN("");
#endif

  port->write(getFiveTemperature, ASK_FRAME_SIZE);

  unsigned long now = millis();

  /* Number of bytes to respond
  *
  * Frame head     : 1 Byte (0x68)
  * Frame length   : 1 Byte
  * ID             : 3 Byte
  * Data type      : 1 Byte (0x52)
  * Data           : 10 Byte
  * H_CRC          : 1 Byte
  * L_CRC          : 1 Byte
  * Frame tail     : 1 Byte (0x16)
  */
  if (_ctrlPin != 0xFF)
  {
    digitalWrite(_ctrlPin, RS485Receive);
  }

  while ((millis() - now) < _timeOut)
  {
    if (port->available())
    {
      port->readBytes(myBuf, FIVE_TEMP_FRAME_SIZE);
      complete = true;
      break;
    }
  }

  if (complete)
  {
#if defined(ENABLE_OILTANK_DEBUG)
    DEBUG_PRINT("RECEIVED: ");
    for (int i = 0; i < FIVE_TEMP_FRAME_SIZE; i++)
    {
      DEBUG_PRINTF("%02X ", myBuf[i]);
    }
    DEBUG_PRINTLN("");
#endif
    /* Check CRC */
    if (crcCheck(&myBuf[2], myBuf[1]) == static_cast<uint16_t>(myBuf[FIVE_TEMP_FRAME_SIZE-2] | static_cast<uint16_t>(myBuf[FIVE_TEMP_FRAME_SIZE-3]) << 8))
    {
      all_temp[0] = static_cast<float>(myBuf[6] << 8 | myBuf[7]) * 0.01;
      all_temp[1] = static_cast<float>(myBuf[8] << 8 | myBuf[9]) * 0.01;
      all_temp[2] = static_cast<float>(myBuf[10] << 8 | myBuf[11]) * 0.01;
      all_temp[3] = static_cast<float>(myBuf[12] << 8 | myBuf[13]) * 0.01;
      all_temp[4] = static_cast<float>(myBuf[14] << 8 | myBuf[15]) * 0.01;
      DEBUG_PRINTF("Temperature 1: %.2f degree C\r\n", all_temp[0]);
      DEBUG_PRINTF("Temperature 2: %.2f degree C\r\n", all_temp[1]);
      DEBUG_PRINTF("Temperature 3: %.2f degree C\r\n", all_temp[2]);
      DEBUG_PRINTF("Temperature 4: %.2f degree C\r\n", all_temp[3]);
      DEBUG_PRINTF("Temperature 5: %.2f degree C\r\n", all_temp[4]);
      DEBUG_PRINTLN(""); 
    }
    else
    {
      DEBUG_PRINTLN("Error CRC in read Five Temperature");
    }
  }
  else
  {
    DEBUG_PRINTLN("Get timeout in read Five Temperature");
  }

  return all_temp;
}

float OILTANK_Sensor::readAverageTemperature()
{
  DEBUG_PRINTLN(">>> Start read Average Temperature...");

  float avg_temp = 0.0f;
  bool complete = false;
  uint8_t myBuf[AVG_TEMP_FRAME_SIZE];

  /* Remove all previous junk data (if have) */
  while (port->available())
  {
    DEBUG_PRINTLN("Remove all previous junk data");
    port->read();
  }

  /* Send level reading command
   * Command frame format structure:
   *
   * Frame head     : 1 Byte (0x68)
   * Frame length   : 1 Byte (0x05)
   * ID             : 3 Byte
   * Data type      : 1 Byte (0x51)
   * Data           : 1 Byte (0x00)
   * H_CRC          : 1 Byte
   * L_CRC          : 1 Byte
   * Frame tail     : 1 Byte (0x16)
   */
  if (_ctrlPin != 0xFF)
  {
    digitalWrite(_ctrlPin, RS485Transmit);
    delay(20);
  }

  getAverageTemperature[2] = _addr[0];
  getAverageTemperature[3] = _addr[1];
  getAverageTemperature[4] = _addr[2];

  uint16_t crc = crcCheck(&getAverageTemperature[2], CRC_LENGTH);
  getAverageTemperature[7] = (crc >> 8) & 0xFF;  // High byte
  getAverageTemperature[8] = crc & 0xFF;         // Low byte

#if defined(ENABLE_OILTANK_DEBUG)
  DEBUG_PRINT("SEND: ");
  for (int i = 0; i < ASK_FRAME_SIZE; i++)
  {
    DEBUG_PRINTF("%02X ", getAverageTemperature[i]);
  }
  DEBUG_PRINTLN("");
#endif

  port->write(getAverageTemperature, ASK_FRAME_SIZE);

  unsigned long now = millis();

  /* Number of bytes to respond
  *
  * Frame head     : 1 Byte (0x68)
  * Frame length   : 1 Byte
  * ID             : 3 Byte
  * Data type      : 1 Byte (0x51)
  * Data           : 2 Byte
  * H_CRC          : 1 Byte
  * L_CRC          : 1 Byte
  * Frame tail     : 1 Byte (0x16)
  */
  if (_ctrlPin != 0xFF)
  {
    digitalWrite(_ctrlPin, RS485Receive);
  }

  while ((millis() - now) < _timeOut)
  {
    if (port->available())
    {
      port->readBytes(myBuf, AVG_TEMP_FRAME_SIZE);
      complete = true;
      break;
    }
  }

  if (complete)
  {
#if defined(ENABLE_OILTANK_DEBUG)
    DEBUG_PRINT("RECEIVED: ");
    for (int i = 0; i < AVG_TEMP_FRAME_SIZE; i++)
    {
      DEBUG_PRINTF("%02X ", myBuf[i]);
    }
    DEBUG_PRINTLN("");
#endif
    /* Check CRC */
    if (crcCheck(&myBuf[2], myBuf[1]) == static_cast<uint16_t>(myBuf[AVG_TEMP_FRAME_SIZE-2] | static_cast<uint16_t>(myBuf[AVG_TEMP_FRAME_SIZE-3]) << 8))
    {
      avg_temp = static_cast<float>(myBuf[6] << 8 | myBuf[7]) * 0.01;
      DEBUG_PRINTF("Average Temperature: %.2f degree C\r\n", avg_temp);
      DEBUG_PRINTLN("");
    }
    else
    {
      DEBUG_PRINTLN("Error CRC in read Average Temperature");
    }
  }
  else
  {
    DEBUG_PRINTLN("Get timeout in read Average Temperature");
  }

  return avg_temp;
}

uint16_t OILTANK_Sensor::crcCheck(uint8_t *ptr, uint8_t length)
{
  uint16_t crc12out = 0;
  uint8_t i, j;

  for (j = 0; j < length; j++)
  {
    for (i = 0; i < 8; i++)
    {
      if (*(ptr + j) & (0x80 >> i))
      {
        crc12out |= 0x1;
      }
      if (crc12out >= 0x1000)
      {
        crc12out ^= 0x180d;
      }
      crc12out <<= 1;
    }
  }
  for (i = 0; i < 12; i++)
  {
    if (crc12out >= 0x1000)
    {
      crc12out ^= 0x180d;
    }
    crc12out <<= 1;
  }
  crc12out >>= 1;

  return (crc12out);
}

int OILTANK_Sensor::charToInt(char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  } else if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  } else if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return 0;
}

void OILTANK_Sensor::convertID(const char *id, uint8_t *result, uint8_t length)
{
  for (uint8_t i = 0; i < length; i++) {
    result[i] = (charToInt(id[i * 2]) << 4) | charToInt(id[i * 2 + 1]);
  }
}