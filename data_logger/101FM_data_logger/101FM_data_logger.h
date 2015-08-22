// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _101FM_data_logger_H_
#define _101FM_data_logger_H_

#include "Arduino.h"
//add your includes for the project 101FM_data_logger here

#include <inttypes.h>
#include "Adafruit-MCP23017-Arduino-Library/Adafruit_MCP23017.h"
#include "ds3231/ds3231.h"
#include "ethercard/EtherCard.h"
#include "TimerOne/TimerOne.h"

#define I2C_EEPROM_PAGESIZE 128
#include "I2C_eeprom/I2C_eeprom.h"

/*
 * Chip select pin of EN28J60 is connected to which Arduino digital pin? Make sure this is properly configured before uploading code, otherwise ethernet won't work
 *
 * ***** The data logger installed at transmission is having Arduino pin 9 configured as CS pin *****
 * ***** All other data loggers uses pin 10 as CS for EN28J60
 */
#define EN28J60_CS 10

#define TCP_BUFF_MAX 160 // TCP buffer size reduced to save AVR SRAM for other uses

#define TCP_FLAGS_FIN_V 1 //as declared in net.h
#define TCP_FLAGS_ACK_V 0x10 //as declared in net.h

#define EEPROM_DEV_HEADER 0x50  // eeprom device where data header for the log is stored. This is where channel names are also stored
// channel names are stored from 0x0000 to 0x0f80. Each character is stored at adjacent bytes in the available 128 bytes in a page.
// data header is stored from 0xffff to 0x1000. This is also written into a full page (128 bytes) and the rest is kept blank.
#define EEPROM_DEV_DATA 0x51    // eeprom with this I²C address stores log data. Each log entry has 64 bytes of storage. Each page will contain two log entries.

/**
 * Not used since we are using polling
 */
//#define INTPIN0 (1 << PD2) // interrupt pin connected to MCP23017 at 0x20
//#define INTPIN1 (1 << PD3) // interrupt pin connected to MCP23017 at 0x21

#define EEPLED (1 << PD4)  // this is an on-board LED. It will also show any activity on EEPROM
#define SYSLED (1 << PD5) // this LED is for showing the device health. It should blink ~50ms in each ~1600ms on normal operation. it is mounted at the front of the unit
#define NETLED (1 << PD6) // this LED serves as an indicator for network activity, mounted at front.

#define INTTOBINARYPATTERN "%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d"
#define INTTOBINARY(_int)  \
  (_int & 0x8000 ? 1 : 0), \
  (_int & 0x4000 ? 1 : 0), \
  (_int & 0x2000 ? 1 : 0), \
  (_int & 0x1000 ? 1 : 0), \
  (_int & 0x0800 ? 1 : 0), \
  (_int & 0x0400 ? 1 : 0), \
  (_int & 0x0200 ? 1 : 0), \
  (_int & 0x0100 ? 1 : 0), \
  (_int & 0x0080 ? 1 : 0), \
  (_int & 0x0040 ? 1 : 0), \
  (_int & 0x0020 ? 1 : 0), \
  (_int & 0x0010 ? 1 : 0), \
  (_int & 0x0008 ? 1 : 0), \
  (_int & 0x0004 ? 1 : 0), \
  (_int & 0x0002 ? 1 : 0), \
  (_int & 0x0001 ? 1 : 0)

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project 101FM_data_logger here

struct DataHeader {
    uint16_t a; // address location of latest block of log
    uint16_t b; // address location of earliest block of log
    uint32_t t; // inverse of unix time when the latest block of log is written. there is a good reasone
                // that we use the inverse of unixtime. 24LC512 (and may be many other chips) comes all their
                // data bytes written as 0xff. Therefore we should pick a value that is being decremented over time.
};
void responseLog(char *data);
//void responseChannels();
void toggleSYS();
void toggleNET();
void beatSYS();
//uint8_t EEXReadByte(uint16_t address); // convenience method for reading from dual I²C EEPROMS
//uint8_t EEXWriteByte(uint32_t address, uint8_t val); // convenience method for writing to dual I²C EEPROMS
void read_data_header();
void write_data_header();
//void record_data(String str);
uint8_t record_data_page_write_mode(char* data);
void detect_pin_changes(Adafruit_MCP23017 *mcp);

//Do not add code below this line
#endif /* _101FM_data_logger_H_ */
