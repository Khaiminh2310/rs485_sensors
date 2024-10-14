/*!
 *  @file OiltankModbus.h
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

#ifndef __OILTANK_MODBUS_H__
#define __OILTANK_MODBUS_H__

#include <Arduino.h>
#include <SoftwareSerial.h>

/* -------------- DEBUG (uncomment to open the Debug function) ------------- */

#if defined(ENABLE_OILTANK_DEBUG)
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

#define BAUDRATE_DEFAULT  2400

#define GENERAL_ID        "0000ff"

#define RS485Transmit     HIGH
#define RS485Receive      LOW

#define MAX_OILTANK_TIMEOUT (20000) // Unit (ms)
#define MIN_OILTANK_TIMEOUT (1000)   // Unit (ms)

#define ASK_LEVEL           0x50
#define ASK_AVG_TEMP        0x51
#define ASK_FIVE_TEMP       0x52
#define ASK_PROBE_ID        0x70
#define ASK_PROBE_LENGTH    0x71

#define LEVEL_FRAME_SIZE      15
#define FIVE_TEMP_FRAME_SIZE  19
#define AVG_TEMP_FRAME_SIZE   11

#define ASK_FRAME_SIZE        0x0A
#define FRAME_LENGTH          0x05
#define CRC_LENGTH            0x05

/* ------------------------------------------------------------------------- */

enum
{
  HARD_SERIAL,
  SOFT_SERIAL
};

typedef struct
{
  uint32_t productLevel;
  uint32_t waterLevel;
  float temperature1;
  float temperature2;
  float temperature3;
  float temperature4;
  float temperature5;
  float averageTemperature;
} dataOiltank;

/* ------------------------------------------------------------------------- */

class OILTANK_Sensor
{
private:
  /**
   * Command frame format structure:
   *
   * Frame head     : 1 Byte (0x68)
   * Frame length   : 1 Byte (0x05)
   * ID             : 3 Byte
   * Data type      : 1 Byte
   * Data           : 1 Byte (0x00)
   * H_CRC          : 1 Byte
   * L_CRC          : 1 Byte
   * Frame tail     : 1 Byte (0x16)
   */
  uint8_t getLevel[ASK_FRAME_SIZE]                = {0x68, FRAME_LENGTH, 0x00, 0x00, 0x00, ASK_LEVEL, 0x00, 0x00, 0x00, 0x16};        // 0x 68 05 xx xx xx 50 00 xx xx 16
  uint8_t getAverageTemperature[ASK_FRAME_SIZE]   = {0x68, FRAME_LENGTH, 0x00, 0x00, 0x00, ASK_AVG_TEMP, 0x00, 0x00, 0x00, 0x16};     // 0x 68 05 xx xx xx 51 00 xx xx 16
  uint8_t getFiveTemperature[ASK_FRAME_SIZE]      = {0x68, FRAME_LENGTH, 0x00, 0x00, 0x00, ASK_FIVE_TEMP, 0x00, 0x00, 0x00, 0x16};    // 0x 68 05 xx xx ff 52 00 xx xx 16
  uint8_t getProbeID[ASK_FRAME_SIZE]              = {0x68, FRAME_LENGTH, 0x00, 0x00, 0x00, ASK_PROBE_ID, 0x00, 0x00, 0x00, 0x16};     // 0x 68 05 xx xx xx 70 00 xx xx 16
  uint8_t getProbeLength[ASK_FRAME_SIZE]          = {0x68, FRAME_LENGTH, 0x00, 0x00, 0x00, ASK_PROBE_LENGTH, 0x00, 0x00, 0x00, 0x16}; // 0x 68 05 xx xx ff 71 00 xx xx 16

  uint8_t askFrame[ASK_FRAME_SIZE]                = {0x68, FRAME_LENGTH, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16};  // 0x 68 05 xx xx xx xx 00 xx xx 16

  Stream *port;
  uint8_t typeSerial;

  uint8_t _ctrlPin;
  uint8_t _addr[3];
  uint16_t _timeOut = MIN_OILTANK_TIMEOUT;

  uint16_t crcCheck(uint8_t *ptr, uint8_t length);
  int charToInt(char c);
  void convertID(const char *input, uint8_t *result, uint8_t length);

public:
  OILTANK_Sensor(const char *id = GENERAL_ID);
  OILTANK_Sensor(uint8_t rxPin = RX_PIN_DEFAULT, uint8_t txPin = TX_PIN_DEFAULT, const char *id = GENERAL_ID);
  OILTANK_Sensor(uint8_t rxPin = RX_PIN_DEFAULT, uint8_t txPin = TX_PIN_DEFAULT, uint8_t ctrlPin = CTRL_PIN_DEFAULT, const char *id = GENERAL_ID);
  virtual ~OILTANK_Sensor() { delete port; }

  /* Initialization */
  void begin();
  void setTimeout(uint16_t timeOut);

  /* Struct */
  dataOiltank getData();

  /* Read Data */
  float *readLevel();
  float *readFiveTemperature();
  float readAverageTemperature();
};

/* ------------------------------------------------------------------------- */

#endif