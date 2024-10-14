/*!
 *  @file SHT40Modbus.h
 *
 *  This library is for the "SHT40 RS485 Modbus RTU" series of sensors.
 *  These sensors communicate via RS485 standard.
 *  So you will need an "RS485 to UART (TTL) communication converter circuit" for use with Microcontrollers.
 *  Like, UNO board, MEGA board, ...
 *
 *  Wiring diagram of SHT3C with "Communication Converter Circuit":
 *  V+      : RED           - Power supply 5~28VDC
 *  V-      : BLACK         - Power supply 0VDC (Mass)
 *  RS485-A : YELLOW/GREEN  - Signal wire A
 *  RS485-B : WHITE         - Signal wire B
 *
 *  @author khaiminh2310
 */

#ifndef __SHT40_MODBUS_H__
#define __SHT40_MODBUS_H__

#include <Arduino.h>
#include <SoftwareSerial.h>

/* -------------- DEBUG (uncomment to open the Debug function) ------------- */

#if defined(ENABLE_SHT40_DEBUG)
#define Debug Serial
#define DEBUG_PRINT(...) Debug.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Debug.println(__VA_ARGS__)
#define DEBUG_PRINTF(...) Debug.printf(__VA_ARGS__, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINTF(...)
#endif

/* ------------------------------------------------------------------------- */

#define RX_PIN_DEFAULT    26
#define TX_PIN_DEFAULT    27
#define CTRL_PIN_DEFAULT  25

#define RS485Transmit     HIGH
#define RS485Receive      LOW

#define MAX_SHT40_TIMEOUT (20000) // Unit (ms)
#define MIN_SHT40_TIMEOUT (100)   // Unit (ms)

/* ------------------------------------------------------------------------- */

enum
{
  HARD_SERIAL,
  SOFT_SERIAL
};

enum baudRate
{
  BAUD_9600 = 0x00,
  BAUD_14400,
  BAUD_19200,
  BAUD_38400,
  BAUD_56000,
  BAUD_115200,
  INVALID_BAUD
};

typedef struct
{
  float temperatureC;
  float temperatureF;
  float humidity;
} dataSHT40;

/* ------------------------------------------------------------------------- */

class SHT40
{
private:
  /**
   * Command frame format structure:
   *
   * Address  : 1 Byte
   * Command  : 1 Byte
   * Register : 2 Byte
   * Data     : 2 Byte
   * L_CRC    : 1 Byte
   * H_CRC    : 1 Byte
   */
  uint8_t getValue[8] = {0x00, 0x04, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00};  // 0x xx 04 00 01 00 02 xx xx *
  uint8_t setBaud[8]  = {0x00, 0x06, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00};  // 0x xx 06 01 02 xx xx xx xx *
  uint8_t getBaud[8]  = {0x00, 0x03, 0x01, 0x02, 0x00, 0x02, 0x00, 0x00};  // 0x xx 03 01 02 00 01 xx xx *
  uint8_t setAddr[8]  = {0x00, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};  // 0x xx 06 01 01 xx xx xx xx *
  uint8_t getAddr[8]  = {0xFF, 0x03, 0x01, 0x01, 0x00, 0x01, 0xD4, 0x36};  // 0x FF 03 01 01 00 01 D4 36 *

  Stream *port;
  uint8_t typeSerial;

  uint8_t _ctrlPin;
  uint8_t _addr;
  uint16_t _timeOut = MIN_SHT40_TIMEOUT;

public:
  SHT40(uint8_t addr = 0x01);
  SHT40(uint8_t rxPin = RX_PIN_DEFAULT, uint8_t txPin = TX_PIN_DEFAULT, uint8_t addr = 0x01);
  SHT40(uint8_t rxPin = RX_PIN_DEFAULT, uint8_t txPin = TX_PIN_DEFAULT, uint8_t ctrlPin = CTRL_PIN_DEFAULT, uint8_t addr = 0x01);
  virtual ~SHT40() { delete port; }

  /* Initialization */
  void begin(uint32_t baud);
  void setTimeout(uint16_t timeOut);

  /* Struct */
  dataSHT40 getData();

  /* Read Data */
  float readTemperature(bool isDegreeCelsius = true);
  float readHumidity();

  /* Read Configuration */
  uint32_t readBaudrate();
  uint8_t readAddress();

  /* Configuration Settings (return "TRUE" when success) */
  bool setBaudrate(baudRate baud);
  bool setAddress(uint8_t addr);
};

/* ------------------------------------------------------------------------- */

#endif
