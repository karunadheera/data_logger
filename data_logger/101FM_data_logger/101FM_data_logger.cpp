/**
 * Requirements:
 * 1. Log multiple events.
 * 	We are using two external eeproms (24LC512 x 2) since the internal EEPROM at ATMega328p is having around 100k erase/write cycles.
 * 	24LC512 on the other hand has 1000k (1M) erase/write cycles. 24LC512s are connected via I²C bus.
 * 	Under the implemented circuit configuration, they can be addressed via 0xA0 and 0xA1 addresses.
 * 2. Monitor up to 32 channels.
 * 	Using ATMega328p chip alone we are unable to achieve this. Since Mega already has SPI - http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
 * 	and I²C - http://en.wikipedia.org/wiki/I%C2%B2C in-built hardware, there is no issue in expanding it's capabilities. I have chosen MCP23017 since each
 * 	chip supports up to 16 channels and  8 chips can be attached to the same I²C bus.
 * 3. All events should be logged with real-time data.
 * 	For that purpose we are using a DS3231 temperature compensated RTC module with in-built 3V Lithum battery, which would run at leat 5 years since we
 * 	are using I²C bus only when the whole device is powered up externally. We are using Petre Rodan's ds3231 library to communicate easily.
 * 4. Logged data should be able to be monitored via ethernet.
 * 	We are using an ethernet module based on  ENC28J60 chip by Microchip. It is easily accessible via SPI and there are numerous open-source libraries exist.
 * 	I used EtherCard library by Guido Socher and Pascal Stang since their library can be used to construct large responses using multiple small packets. Small
 * 	packets are extremely important in embedded hardware/firmware solutions since the platform limited in resources.
 */

// Do not remove the include below
#include "101FM_data_logger.h"

#define TCP_BUFF_MAX 160 // TCP buffer size reduced to save AVR SRAM for other uses

#define TCP_FLAGS_FIN_V 1 //as declared in net.h
#define TCP_FLAGS_ACK_V 0x10 //as declared in net.h

#define EEPROM_DEV_HEADER 0x50
#define EEPROM_DEV_DATA 0x51

// Interrupts from the MCPs will be handled by these pins.
//byte INTPIN0 = 2, INTPIN1 = 3, HBLED = 4, HBLED2 = 5, NETLED = 6;

#define INTPIN0 2
#define INTPIN1 3
#define HBLED 4
#define HBLED2 5
#define NETLED 6

volatile uint32_t heartBeatStatus = 1; // counter for heartbeat LED

// These are the interrupt vectors for above pins. These correspond exactly to above Arduino pins and cannot be changed.
//byte int0 = 0, int1 = 1;

#define INT0 0
#define INT1 1

I2C_eeprom ee_h(EEPROM_DEV_HEADER);
I2C_eeprom ee_d(EEPROM_DEV_DATA);

static byte myip[] = { 192, 168, 2, 2 };
static byte gwip[] = { 192, 168, 2, 1 };
static byte mymac[] = { 0x74, 0x69, 0x69, 0xD1, 0x2F, 0x38 };

volatile uint8_t ee_busy = 0;

byte Ethernet::buffer[TCP_BUFF_MAX]; // tcp ip send and receive buffer

const char txt_header_404[] PROGMEM
		= "HTTP/1.0 404 NOT FOUND\r\nPowered-By: avr-gcc\r\nContent-Type: text/plain\r\n\r\n"; // TCP header for 404 status stored at Flash memory.
const char txt_header_400[] PROGMEM
		= "HTTP/1.0 404 BAD REQUEST\r\nPowered-By: avr-gcc\r\nContent-Type: text/plain\r\n\r\n"; // TCP header for 400 status stored at Flash memory.
const char txt_header_200[] PROGMEM
= "HTTP/1.0 200 OK\r\nPowered-By: avr-gcc\r\nContent-Type: text/plain\r\n\r\n"; // TCP  header for 200 status stored at Flash memory.

const char txt_body_404[] PROGMEM= "page not found"; // TCP body for 404 status stored at Flash memory.
const char txt_body_400[] PROGMEM= "bad request"; // TCP body for 400 status stored at Flash memory.
const char txt_body_busy[] PROGMEM = "busy"; // TCP body to be used as the response when the system is busy doing other tasks.
const char txt_body_time_updated[] PROGMEM = "time updated\n";

// References to MCP23017 IOExpander chips. There are two of them configured with addresses 0x00 and 0x01 via their hardware address pins.
Adafruit_MCP23017 mcp0, mcp1;

volatile boolean awakenByInterrupt0 = false, awakenByInterrupt1 = false; // Flags those get set when there is an interrupt on corresponding MCP23017 chips.

char buf_prog[41]; // Temporary buffer to be used to store words read from Flash (PROGMEM).
char buf[65];

volatile struct DataHeader *dh = (struct DataHeader*) malloc(
		sizeof(struct DataHeader)); // DataHeader global variable

/**
 * Arduino setup function.
 *
 * Sets up all hardware and finally attaches interrupts for ATMega328p
 */
void setup() {

	pinMode(HBLED, OUTPUT); // set up white heartbeat LED
	pinMode(HBLED2, OUTPUT); // set up white heartbeat LED2
	pinMode(NETLED, OUTPUT); // set up white activity LED

	Timer1.initialize(50000); // heartBeat() function to run every 50ms
	Timer1.attachInterrupt(initBeat);

	Serial.begin(9600);

	Serial.println("Setting up");

	_delay_ms(1000);

	ether.begin(sizeof Ethernet::buffer, mymac, 9); // 53 for the mega ethernet shield and 10 for normal ethernet shield
	ether.staticSetup(myip, gwip);

	Serial.println("Ethernet started");

	pinMode(INTPIN0, INPUT_PULLUP); // interrupt pin should be set up as INPUT
	pinMode(INTPIN1, INPUT_PULLUP); // interrupt pin should be set up as INPUT

	mcp0.begin(0);      // use default address 0
	mcp1.begin(1);

	// We mirror INTA and INTB, so that only one line is required between MCP23017 and AVR for int reporting
	// The INTA/B will not be Floating

	for (int a = 0; a < 16; a++) { // To address all 16 pins (GPIOA + GPIOB)
		mcp0.pinMode(a, INPUT);
		mcp0.pullUp(a, HIGH);
		mcp0.setupInterruptPin(a, CHANGE); // We instruct MCP23017 to trigger interrupts on pin CHANGE
		mcp1.pinMode(a, INPUT);
		mcp1.pullUp(a, HIGH);
		mcp1.setupInterruptPin(a, CHANGE); // We instruct MCP23017 to trigger interrupts on pin CHANGE
	}

	mcp0.readGPIOAB(); // Read IO lines from Bus 0 to clean any previous interrupts on MCP23017
	mcp1.readGPIOAB(); // Read IO lines from Bus 1 to clean any previous interrupts on MCP23017

	_delay_ms(10);

	// INTs will be signaled with a LOW
	mcp0.setupInterrupts(true, false, LOW);
	mcp1.setupInterrupts(true, false, LOW);

	_delay_ms(200);

	struct ts t;

	DS3231_get(&t); // Read the time from DS3231 into struct t.

	char buf[128];
	sprintf(buf, "Testing RTC : %04d-%02d-%02d %02d:%02d:%02d", t.year, t.mon,
			t.mday, t.hour, t.min, t.sec);

	Serial.println(buf);

	attachInterrupt(INT0, intCallBack0, LOW);
	attachInterrupt(INT1, intCallBack1, LOW);

	Timer1.detachInterrupt();
	Timer1.initialize(150000); // heartBeat() function to run every 150ms
	Timer1.attachInterrupt(heartBeat);
}

/**
 * Arduino loop function.
 *
 * This gets executed over and over again until the chip is powered up and no hangs occur within the program.
 */
void loop() {

	// Check each of the interrupt flags and act accordingly.
	// If a flag is set we should first unregister interrupts so that we can work on the interrupt without problem.
	// This also helps us get rid of bouncing issues on switches.
	// Finally re-attach interrupt to corresponding pin and function.
	if (awakenByInterrupt0) {
		detachInterrupt(INT0);
		handleInterrupt(&mcp0, &awakenByInterrupt0);
		attachInterrupt(INT0, intCallBack0, LOW);
	}

	// Check each of the interrupt flags and act accordingly.
	// If a flag is set we should first unregister interrupts so that we can work on the interrupt without problem.
	// This also helps us get rid of bouncing issues on switches.
	// Finally re-attach interrupt to corresponding pin and function.
	if (awakenByInterrupt1) {
		detachInterrupt(INT1);
		handleInterrupt(&mcp1, &awakenByInterrupt1);
		attachInterrupt(INT1, intCallBack1, LOW);
	}

	// recieve data from Ethernet card
	word pos = ether.packetLoop(ether.packetReceive());
	// check if valid tcp data is received
	if (pos) {
		// WOW! we have got some data.. Let's go ahead and check them out...
		char* data = (char *) Ethernet::buffer + pos;
		if (strncmp("GET / ", data, 6) == 0) {
			responseLog(data);
		} else if (strncmp("GET /log ", data, 9) == 0) { // this is the real deal. Someone has requested to check the log.
			responseLog(data);
		} else if (strncmp("GET /dump ", data, 10) == 0) { // Well... this is going to be slow sometimes. Because reading the whole log is not a good idea.
			ether.httpServerReplyAck();
			memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
			ether.httpServerReply_with_flags(sizeof txt_header_200 - 1,
			TCP_FLAGS_ACK_V);
			read_data_header(dh);
			char tmpbuff[66];
			uint16_t addra = dh->a;
			uint16_t addrb = dh->b;
			tmpbuff[64] = 0x0a; // new line character at end - 1. end should be NULL (\0)
			if (addra != addrb) {
				/*while (addra != addrb) {
				 for (uint16_t j = 0x00; j < 0x20; j++) {
				 tmpbuff[j] = EEXReadByte(
				 ((uint32_t) (addra - 0x0020 + j)) | 0x10000);

				 }
				 tmpbuff[32] = 0x0a;
				 addra -= 0x20;
				 memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
				 ether.httpServerReply_with_flags(sizeof tmpbuff - 1,
				 (addra == addrb) ?
				 TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V :
				 TCP_FLAGS_ACK_V); // Send final packet with FIN which ends the TCP transmission only if this is the last packet.
				 }*/
				while (addra != addrb) {

					ee_d.readBlock(addra - 0x0040, (uint8_t*) tmpbuff, 0x40);
					addra -= 0x40;
					memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
					ether.httpServerReply_with_flags(sizeof tmpbuff - 1,
							(addra == addrb) ?
							TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V :
												TCP_FLAGS_ACK_V); // Send final packet with FIN which ends the TCP transmission only if this is the last packet.
				}
			} else {
				sprintf(tmpbuff, "no data");
				tmpbuff[7] = 0x0a; // new line character at end - 1. end should be NULL (\0)
				memcpy(ether.tcpOffset(), tmpbuff, 8);
				ether.httpServerReply_with_flags(7,
				TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
			}
		} else if (strncmp("GET /addr ", data, 10) == 0) { // For debugging purposes. This results in the DataHeader printed out via HTTP.
			ether.httpServerReplyAck();
			memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
			ether.httpServerReply_with_flags(sizeof txt_header_200 - 1,
			TCP_FLAGS_ACK_V);
			read_data_header(dh);
			char tmpbuff[11];
			sprintf(tmpbuff, "%04x %04x", dh->a, dh->b);
			tmpbuff[9] = 0x0a;
			memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
			ether.httpServerReply_with_flags(sizeof tmpbuff - 1,
			TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V);
		} else if (strncmp("GET /clr ", data, 9) == 0) {
			clear_logs();
			ether.httpServerReplyAck();
			memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
			ether.httpServerReply_with_flags(sizeof txt_header_200 - 1,
			TCP_FLAGS_ACK_V);
			char tmpbuff[6];
			sprintf(tmpbuff, "done");
			tmpbuff[4] = 0x0a;
			memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
			ether.httpServerReply_with_flags(sizeof tmpbuff - 1,
			TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
		} else if (strncmp("GET /time?", data, 10) == 0) {
			struct ts t;

			char year[5];
			memcpy(year, &data[10], 4);
			t.year = atol(year);

			char mon[3];
			memcpy(mon, &data[14], 2);
			t.mon = atoi(mon);

			char mday[3];
			memcpy(mday, &data[16], 2);
			t.mday = atoi(mday);

			char hour[3];
			memcpy(hour, &data[18], 2);
			t.hour = atoi(hour);

			char min[3];
			memcpy(min, &data[20], 2);
			t.min = atoi(min);

			char sec[3];
			memcpy(sec, &data[22], 2);
			t.sec = atoi(sec);

			DS3231_set(t);

			DS3231_get(&t); // receive time from RTC
			ether.httpServerReplyAck();
			memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
			ether.httpServerReply_with_flags(sizeof txt_header_200 - 1,
			TCP_FLAGS_ACK_V);
			memcpy_P(ether.tcpOffset(), txt_body_time_updated,
					sizeof txt_body_time_updated);
			ether.httpServerReply_with_flags(sizeof txt_body_time_updated - 1,
			TCP_FLAGS_ACK_V);
			char tmpbuff[20];
			tmpbuff[19] = 0x0a;
			sprintf(tmpbuff, "%04d-%02d-%02d %02d:%02d:%02d", t.year, t.mon,
					t.mday, t.hour, t.min, t.sec);
			memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
			ether.httpServerReply_with_flags(sizeof tmpbuff - 1,
			TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
		} else if (strncmp("GET /time ", data, 10) == 0) {
			struct ts t;
			DS3231_get(&t); // receive time from RTC
			ether.httpServerReplyAck();
			memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
			ether.httpServerReply_with_flags(sizeof txt_header_200 - 1,
			TCP_FLAGS_ACK_V);
			char tmpbuff[20];
			tmpbuff[19] = 0x0a;
			sprintf(tmpbuff, "%04d-%02d-%02d %02d:%02d:%02d", t.year, t.mon,
					t.mday, t.hour, t.min, t.sec);
			memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
			ether.httpServerReply_with_flags(sizeof tmpbuff - 1,
			TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
		} else if (strncmp("GET /cnl?b", data, 10) == 0) {
			String s = data;
			uint8_t end = s.indexOf(" HTTP/1.");

			// start is 13
			if (end > 14 && end - 13 <= 40) {
				uint16_t addr;
				char tmp[2];
				tmp[1] = '\0';
				// we are going to receive something like below to set BANK 1 CHANNEL 16
				// GET /cnl?B1CFPROGRAM LINK FAILURE
				memcpy(tmp, &data[10], 1); // get index of the bank from GET parameters
				addr = strtoul(tmp, NULL, 16) * 0x0400; // calculate offset for the address using bank number
				memcpy(tmp, &data[12], 1); // get index of the channel from GET parameters
				addr += strtoul(tmp, NULL, 16) * 0x0040; // add pin address to the above offset to get actual address to store the name

				ether.httpServerReplyAck();
				memcpy_P(ether.tcpOffset(), txt_header_200,
						sizeof txt_header_200);
				ether.httpServerReply_with_flags(sizeof txt_header_200 - 1,
				TCP_FLAGS_ACK_V);

				char c_name[end - 13 + 1];

				s.substring(13).toCharArray(c_name, end - 12);

				char writebuff[41];
				sprintf(writebuff, "%40s", c_name);
				ee_h.writeBlock(addr, (uint8_t*) writebuff, 40);

				char tmpbuff[47];
				for (uint8_t x = 0; x < 0x20; x++) {
					ee_h.readBlock(0x0040 * (uint16_t) x, (uint8_t*) writebuff,
							40);
					sprintf(tmpbuff, "b%xc%x %40s", x / 0x10, x % 0x10,
							writebuff);
					tmpbuff[45] = 0x0a;
					memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
					ether.httpServerReply_with_flags(sizeof tmpbuff - 1,
							x == 0x1f ?
							TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V :
										TCP_FLAGS_ACK_V); // Send final packet with FIN which ends the TCP transmission.
				}
			} else {
				ether.httpServerReplyAck(); // send ack to the request
				memcpy_P(ether.tcpOffset(), txt_header_400,
						sizeof txt_header_400);
				ether.httpServerReply_with_flags(sizeof txt_header_400 - 1,
				TCP_FLAGS_ACK_V);
				memcpy_P(ether.tcpOffset(), txt_body_400, sizeof txt_body_400);
				ether.httpServerReply_with_flags(sizeof txt_body_400 - 1,
				TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
			}

		} else if (strncmp("GET /cnl?reset ", data, 15) == 0) {
			char writebuff[41];

			for (uint8_t x = 0; x < 0x20; x++) {
				sprintf(writebuff, "%36sb%xc%x", "", x / 0x10, x % 0x10);
				ee_h.writeBlock(0x0040 * (uint16_t) x, (uint8_t*) writebuff,
						40);
			}

			responseChannels();
		} else if (strncmp("GET /cnl ", data, 9) == 0) {
			responseChannels();
		} else { // Page not found. Yes 404.
			ether.httpServerReplyAck(); // send ack to the request
			memcpy_P(ether.tcpOffset(), txt_header_404, sizeof txt_header_404);
			ether.httpServerReply_with_flags(sizeof txt_header_404 - 1,
			TCP_FLAGS_ACK_V);
			memcpy_P(ether.tcpOffset(), txt_body_404, sizeof txt_body_404);
			ether.httpServerReply_with_flags(sizeof txt_body_404 - 1,
			TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
		}
	}
}
void responseLog(char *data) {
	ether.httpServerReplyAck();
	memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
	ether.httpServerReply_with_flags(sizeof txt_header_200 - 1,
	TCP_FLAGS_ACK_V);
	read_data_header(dh);
	char tmpbuff[66];
	uint16_t addra = dh->a;
	uint16_t addrb = dh->b;
	tmpbuff[64] = 0x0a;
	if (addra != addrb) {
		uint16_t i = 0x00;
		while (addra != addrb) {
			ee_d.readBlock(addra - 0x0040, (uint8_t*) tmpbuff, 0x40);
			addra -= 0x40;
			memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
			ether.httpServerReply_with_flags(sizeof tmpbuff - 1,
					(i == 0x1f || addra == addrb) ?
					TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V :
													TCP_FLAGS_ACK_V); // Send final packet with FIN which ends the TCP transmission only if this is the last packet.
			if (i == 0x1f || addra == addrb) {
				break;
			}
			i++;
		}
	} else {
		sprintf(tmpbuff, "no data");
		tmpbuff[7] = 0x0a;
		memcpy(ether.tcpOffset(), tmpbuff, 8);
		ether.httpServerReply_with_flags(7,
		TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
	}
}
void responseChannels() {
	ether.httpServerReplyAck();
	memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
	ether.httpServerReply_with_flags(sizeof txt_header_200 - 1,
	TCP_FLAGS_ACK_V);
	char writebuff[41];
	char tmpbuff[47];
	for (uint8_t x = 0; x < 0x20; x++) {
		ee_h.readBlock(0x0040 * (uint16_t) x, (uint8_t*) writebuff, 40);
		sprintf(tmpbuff, "b%xc%x %40s", x / 0x10, x % 0x10, writebuff);
		tmpbuff[45] = 0x0a;
		memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
		ether.httpServerReply_with_flags(sizeof tmpbuff - 1,
				x == 0x1f ?
				TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V :
							TCP_FLAGS_ACK_V); // Send final packet with FIN which ends the TCP transmission.
	}
}
void initBeat() {
	digitalWrite(HBLED, !digitalRead(HBLED));
	digitalWrite(HBLED2, !digitalRead(HBLED2));
}

void heartBeat() {
	if (!heartBeatStatus) {
		digitalWrite(HBLED, 1);
		digitalWrite(HBLED2, 1);
		heartBeatStatus = 1;
	} else {
		digitalWrite(HBLED, 0);
		digitalWrite(HBLED2, 0);
		heartBeatStatus <<= 1;
	}
}

/*
 * Convenience function to read a byte from specified address at data eeprom (ee_d, 24LC512). Any byte that is out of range (in our app) is
 * considered a "space".
 */
//uint8_t EEXReadByte(uint16_t address) {
//	uint8_t r;
//	r = ee_d.readByte(address);
//	r = r < 0x20 || r > 0x7e ? 0x20 : r;
//	return r;
//}
/**
 * To read data header from first EEPROM
 */
void read_data_header(volatile struct DataHeader *d) {
	d->a = (uint32_t) ee_h.readByte(0x8000);
	d->a |= (uint32_t) (ee_h.readByte(0x8001) << 8);
	d->b = (uint32_t) ee_h.readByte(0x8002);
	d->b |= (uint32_t) (ee_h.readByte(0x8003) << 8);
}

/**
 * To write data header into first EEPROM
 */
void write_data_header(volatile struct DataHeader *b) {
	ee_h.writeByte(0x8000, (uint8_t) (b->a & 0xff));
	ee_h.writeByte(0x8001, (uint8_t) ((b->a & 0xff00) >> 8));
	ee_h.writeByte(0x8002, (uint8_t) (b->b & 0xff));
	ee_h.writeByte(0x8003, (uint8_t) ((b->b & 0xff00) >> 8));
}

/**
 * To clear the logs. This doesn't clear the actual logs at all, rather acts like a deletion of a file from a hard disk
 * where only table header (in the first EEPROM chip) is set to 0x00000, 0x00000.
 */
void clear_logs(void) {
	if (ee_busy)
		return;
	ee_busy = 1;
	dh->a = 0x00000;
	dh->b = 0x00000;
	write_data_header(dh);
	ee_busy = 0;
}

/**
 * We have designed our logger to record 32 bytes for each incident. As of now we are calling EEXWriteByte 32 times
 * in order to log one incident where each byte written will take ~5ms. This time can be reduced by using page write
 * function of EEPROM. This is one of the places we can optimize.

 void record_data(char *str) {
 if (ee_busy)
 return;
 ee_busy = 1;
 read_data_header(dh);

 for (uint32_t j = 0; j < 0x20; j++) {
 EEXWriteByte(((uint32_t) (dh->a + j)) | 0x10000, str[j]);
 }

 dh->a += 0x0020;
 if (dh->a == dh->b) {
 dh->b += 0x00020;
 }
 write_data_header(dh);
 ee_busy = 0;
 }
 */

/**
 * Improved page write mode.
 * Only writes up to 32 chars. Using this function to write more than 32 bytes is currently not supported.
 * If we are at a page boundary and try to write more bytes wrapping around will occur which is a data corruption
 * at user end. Uses page_write function  of EEPROM to make it way faster than byte write.
 * Since Arduino Wire library has max 32 bytes as its buffer let's go for 16 bytes write each cycle. That is because
 * 2 bytes are dedicated for addresses.
 */

uint8_t record_data_page_write_mode(char* data) {

	if (ee_busy)
		return 0;
	ee_busy = 1;
	read_data_header(dh);

	/*Wire.beginTransmission(EEPROM_DEV_DATA); // we write to the second 24LC512.
	 Wire.write((int) ((dh->a) >> 8));   // MSB
	 Wire.write((int) ((dh->a) & 0xFF)); // LSB

	 for (uint32_t j = 0; j < 0x10; j++) {
	 Wire.write((byte) data[j]);
	 }

	 Wire.endTransmission();

	 delay(5);

	 Wire.beginTransmission(EEPROM_DEV_DATA); // we write to the second 24LC512.
	 Wire.write((int) ((dh->a + 0x10) >> 8));   // MSB
	 Wire.write((int) ((dh->a + 0x10) & 0xFF)); // LSB

	 for (uint32_t j = 0x10; j < 0x20; j++) {
	 Wire.write((byte) data[j]);
	 }

	 Wire.endTransmission();
	 delay(5);
	 */

	ee_d.writeBlock(dh->a, (uint8_t*) data, 0x40);

	dh->a += 0x0040;
	if (dh->a == dh->b) {
		dh->b += 0x00040;
	}
	write_data_header(dh);
	ee_busy = 0;

	return 1;
}

// The int handler will just signal that the int has happen
// we will do the work from the main loop.
/**
 * This is the function call which gets automatically triggered when MCP23017 for Bank 1 triggers an interrupt on ATMega328p's PIN4
 */
void intCallBack0() {
	awakenByInterrupt0 = true;
}

// The int handler will just signal that the int has happen
// we will do the work from the main loop.
/**
 * This is the function call which gets automatically triggered when MCP23017 for Bank 1 triggers an interrupt on ATMega328p's PIN4
 */
void intCallBack1() {
	awakenByInterrupt1 = true;
}

/**
 * Handle the registered interrupts. Below is the generic procedure. The implementation below is an extension upon below basic steps.
 * Look for the comments in code.
 * 1. Check which pin has been changed.
 * 2. Check the value of the pin.
 * 3. Get the time from RTC.
 * 4. Get user friendly name stored at the FLASH storage.
 * 5. Print the incident into serial (for debugging purposes).
 * 6. Record the data onto EEPROM.
 * 7. Clean the interrupt flags so that another interrupt can take place.
 */
void handleInterrupt(Adafruit_MCP23017 *mcp,
		volatile boolean *awakenByInterrupt) {
	struct ts t;

	DS3231_get(&t); // receive time from RTC

// Get more information from the MCP from the INT
	uint8_t pin = mcp->getLastInterruptPin();
	uint8_t val = mcp->getLastInterruptPinValue();

//	strcpy_P(buf_prog,
//			(char*) pgm_read_word(
//					&(CHANNELS[mcp->getAddr() ? pin + 0x10 : pin]))); // Necessary casts and dereferencing
	uint16_t addr = (0x0040 * ((uint16_t) pin)) + (mcp->getAddr() ? 0x0400 : 0);
	ee_h.readBlock(addr, (uint8_t*) buf_prog, 40);

//	Serial.println(addr, 16);

	sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d %40s %3s", t.year, t.mon,
			t.mday, t.hour, t.min, t.sec, buf_prog, val ? "ON" : "OFF");

	record_data_page_write_mode(buf); // Write to eeprom

	Serial.println(buf);

	uint8_t val0 = mcp->digitalRead(pin); // clear the interrupt condition on MCP. To speed up logging you can move this right after the line that reads uint8_t val = mcp->getLastInterruptPinValue();

	if (val0 != 0xff && val0 != val) { // lets check whether the pin has changed.
		DS3231_get(&t); // update the time again. it might have just taken few milliseconds yet there can be a time change in seconds.

		sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d %40s %3s", t.year, t.mon,
				t.mday, t.hour, t.min, t.sec, buf_prog, val0 ? "ON" : "OFF");

		record_data_page_write_mode(buf); // Write to eeprom

		Serial.println(buf);

		mcp->digitalRead(pin); // clear the interrupt condition on MCP. To speed up logging you can move this right after the line that reads val = mcp->getLastInterruptPinValue();
	}

// check again if there are any interrupts pending for the same MCP chip.
	while ((pin = mcp->getLastInterruptPin()) != 0xff) {

		DS3231_get(&t); //update the time

		// there seems a valid incoming interrupt pending
		val = mcp->getLastInterruptPinValue();

//		strcpy_P(buf_prog,
//				(char*) pgm_read_word(
//						&(CHANNELS[mcp->getAddr() ? pin + 0x10 : pin]))); // Necessary casts and dereferencing
		addr = (0x0040 * ((uint16_t) pin)) + (mcp->getAddr() ? 0x0400 : 0);
		ee_h.readBlock(addr, (uint8_t*) buf_prog, 40);

		sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d %40s %3s", t.year, t.mon,
				t.mday, t.hour, t.min, t.sec, buf_prog, val ? "ON" : "OFF");

		record_data_page_write_mode(buf); // Write to eeprom

		Serial.println(buf);

		mcp->digitalRead(pin); // clear the interrupt condition on MCP. To speed up logging you can move this right after the line that reads val = mcp->getLastInterruptPinValue();
	}

// and clean queued INT signal
	cleanInterrupts(awakenByInterrupt);
}

// handy for interrupts triggered by buttons
// normally signal a few due to bouncing issues
/**
 * This clears the interrupt register at μc and prepares itself for another interrupt.
 */
void cleanInterrupts(volatile boolean *awakenByInterrupt) {
	EIFR = 0x01;
	*awakenByInterrupt = false;
}

