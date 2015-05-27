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
void responseChannels();
void toggleSYS();
void toggleNET();
void beatSYS();
//uint8_t EEXReadByte(uint16_t address); // convenience method for reading from dual I²C EEPROMS
//uint8_t EEXWriteByte(uint32_t address, uint8_t val); // convenience method for writing to dual I²C EEPROMS
void read_data_header();
void write_data_header();
//void record_data(String str);
uint8_t record_data_page_write_mode(unsigned char* data);
//void intCallBack0();
//void intCallBack1();

void handleInterrupt(Adafruit_MCP23017 *mcp,
		volatile boolean *awakenByInterrupt);
//void cleanInterrupts(volatile boolean  *awakenByInterrupt);

//Do not add code below this line
#endif /* _101FM_data_logger_H_ */
