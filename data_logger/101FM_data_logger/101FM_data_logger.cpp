/**
 * author prageeth
 *
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

//volatile uint32_t beatsysint = 1; // counter for heartbeat LED
volatile uint16_t togglesysint = 1;
volatile uint8_t pin_change_detect_flag = 0;

I2C_eeprom ee_h(EEPROM_DEV_HEADER);
I2C_eeprom ee_d(EEPROM_DEV_DATA);

static byte myip[] = { 192, 168, 2, 2 };
static byte gwip[] = { 192, 168, 2, 1 };
static byte mymac[] = { 0x74, 0x69, 0x69, 0xD1, 0x2F, 0x38 };

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
const char txt_body_interrupted[] PROGMEM = "\ninterrupted!\n";

// References to MCP23017 IOExpander chips. There are two of them configured with addresses 0x20 and 0x21 via their hardware address pins.
Adafruit_MCP23017 mcp0, mcp1;

/**
 * We will make sure that any pin is absolutely settled down before deciding on the pin value.
 * Above is achieved using two sets of history bits per each MCP23017 chip.
 * mcp00_bits and mcp10_bits are the former sets of history bits
 */
volatile uint16_t mcp00_bits = 0b1111111111111111; // all the pins are pulled high
volatile uint16_t mcp10_bits = 0b1111111111111111; // ''

/**
 * We use below bits to hold settled pin values so that we do not record the same ON/OFF condition when there is no absolute change
 */
volatile uint16_t mcp0_settled_bits = 0b1111111111111111; // confirmed settled pin values

/**
 * below are the most recent history bits
 */
volatile uint16_t mcp01_bits = 0b1111111111111111; // all the pins are pulled high
volatile uint16_t mcp11_bits = 0b1111111111111111; // ''

/**
 * We use below bits to hold settled pin values so that we do not record the same ON/OFF condition when there is no absolute change
 */
volatile uint16_t mcp1_settled_bits = 0b1111111111111111; // confirmed settled pin values

/**
 * Interrupts are no longer used.
 */
//volatile boolean awakenByInterrupt0 = false, awakenByInterrupt1 = false; // Flags those get set when there is an interrupt on corresponding MCP23017 chips.
char buf_prog[41]; // Temporary buffer to be used to store words read from Flash (PROGMEM).
char buf[65];

volatile struct DataHeader *dh = (struct DataHeader*) malloc(sizeof(struct DataHeader)); // DataHeader global variable

volatile uint16_t dh_addr = 0xff80;
struct ts t;
uint8_t dh_block[9];

/**
 * Arduino setup function.
 *
 * Sets up all hardware and finally attaches interrupts for ATMega328p
 */
void setup() {

    DDRD |= EEPLED | SYSLED | NETLED; // configure  EEPLED, SYSLED and NETLED as outputs
//    DDRD &= ~INTPIN0; // configure INTPIN0 as input
//    DDRD &= ~INTPIN0 & ~INTPIN1; // configure INTPIN0 and INTPIN1 as inputs
//    PORTD |= INTPIN0; // turn on the pull ups on INTPIN0
//    PORTD |= INTPIN0 | INTPIN1; // turn on the pull ups on INTPIN0 and INTPIN1

    toggleNET();	// toggles NETLED (will turn on)
    toggleSYS();	// toggles SYSLED and SYSLED (will turn on)
    _delay_ms(1000);	// enough time so that we can test if LEDs are fine
    toggleNET();	// toggles NETLED (will turn off)
    toggleSYS();	// toggles SYSLED and SYSLED (will turn off)

    Timer1.initialize(50000);	// timer1 runs every 50ms - value is in μS
    Timer1.attachInterrupt(toggleSYS);	// attaches the timer1 to beatSys function. This causes the beatSys function to be called every 50ms

    Serial.begin(230400);	// Setup serial line baudrate (3V3 TTL)

//    Serial.println("Setting up");	// for debugging

    // START DATA HEADER SEARCH
    // We have implemented simple wear leveling on write_data_header() so that whenever a new log is written to DATA eeprom
    // the new header data is written to HEADER eeprom at an incremental address location. When power reset happens we have
    // no clue where the latest data header is written to. So we are navigating through all available space for HEADER data
    // and find out what the most recent address is by using the inv(unixtime) written at the fist 4 bytes (uint32_t).
    uint32_t unix_tm_inv = 0xffffffff;
    uint32_t val;
    uint16_t addr = 0xff80;
    do {
        ee_h.readBlock(addr, (uint8_t*) dh_block, 8);	// read first 8 bytes in which we store header data
        val = (uint32_t) dh_block[0];
        val |= (uint32_t) ((uint32_t) dh_block[1] << 8);
        val |= (uint32_t) ((uint32_t) dh_block[2] << 16);
        val |= (uint32_t) ((uint32_t) dh_block[3] << 24);	// construct inv(unixtime) value
        if (val <= unix_tm_inv) {	// compare it to find min(inv(unixtime)) value
            unix_tm_inv = val;
            dh_addr = addr;
        }
        addr -= 0x80;	// go to next available page
    } while (addr > 0x0f80);	// check within these bounds

    // END DATA HEADER SEARCH

    read_data_header();	// load the data header to the RAM

//    Serial.print("HDER: 0x");
//    Serial.println(dh_addr, 16);
//    Serial.print("DH_T: ");
//    Serial.println(dh->t);

    Timer1.detachInterrupt();	// detach timer1 from previous function
    PORTD &= ~SYSLED; // turn off SYSLED and SYSLEDEXT as they may be ON by now
    Timer1.initialize(50000); 	// initialize timer1 to 50ms
    Timer1.attachInterrupt(toggleNET); 	// attach timer1 to toggle NETLED since below we are going to initialize network adapter

    if (!ether.begin(sizeof Ethernet::buffer, mymac, EN28J60_CS)) { // this will result in zero upon failure of network adaptor based on EN28J60
//        Serial.println("Ethernet failed!"); // send error through terminal and keep beating the NETLED
    } else {
        ether.staticSetup(myip, gwip);

        Timer1.detachInterrupt(); // We are done setting up the network. This stops blinking NETLED.
        PORTD &= ~NETLED;	// turns off NETLED
        Timer1.initialize(50000);
        Timer1.attachInterrupt(toggleSYS);

//        Serial.println("Ethernet started");

        mcp0.begin(0);	// initializes mcp0 object to refer to the MCP23017 at address 0x20. This chip handles BANK0
        mcp1.begin(1);	// 0x21. This handles BANK1

        // We mirror INTA and INTB, so that only one line is required between MCP23017 and AVR for int reporting
        // The INTA/B will not be Floating

        for (int a = 0; a < 16; a++) {		// To address all 16 pins (GPIOA + GPIOB)
            mcp0.pinMode(a, INPUT);
            mcp0.pullUp(a, HIGH);		// turns on internal pullups at 100kΩ. We have to add external pullups so that we do not pickup transient voltages generated by RF plus some caps at ~0.1μF
//            mcp0.setupInterruptPin(a, CHANGE); // We instruct MCP23017 to trigger interrupts on pin CHANGE
            mcp1.pinMode(a, INPUT);
            mcp1.pullUp(a, HIGH);
//            mcp1.setupInterruptPin(a, CHANGE); // We instruct MCP23017 to trigger interrupts on pin CHANGE
        }

//        mcp0.readGPIOAB(); // Read IO lines from Bus 0 to clean any interrupts on MCP23017 for BANK0
//        mcp1.readGPIOAB(); // Read IO lines from Bus 1 to clean any interrupts on MCP23017 for BANK1

        // INTs will be signaled with a LOW
//        mcp0.setupInterrupts(true, false, LOW);
//        mcp1.setupInterrupts(true, false, LOW);

        DS3231_get(&t); // Read the time from DS3231 into struct t. This is to test if the RTC is fine.

//        char buf[128];
//        sprintf(buf, "Testing RTC : %04d-%02d-%02d %02d:%02d:%02d", t.year, t.mon, t.mday, t.hour, t.min, t.sec);

//        Serial.println(buf);

//        PCICR |= (1 << PCIE2); // enable PCIE2 in Pin Change Interrupt Control Register (PCICR).
//        PCMSK2 |= (1 << PCINT18) | (1 << PCINT19); // PCINT18 and PCINT19 are related to Pin Change Mask Regsiter 2 (PCMSK2). So lets enable those two.

        Timer1.detachInterrupt();
        Timer1.initialize(100000);
        Timer1.attachInterrupt(beatSYS);

        sei();

        if ((0xffffffff - unix_tm_inv) > t.unixtime) { // check if we are getting an invalid time from RTC. A malfunctioning EEPROM also can be a reason
            Timer1.detachInterrupt();
            Timer1.initialize(80000);
            Timer1.attachInterrupt(toggleSYS);
        }
    }
}

/**
 * Arduino loop function.
 *
 * This gets executed over and over again until the chip is powered up and no hangs occur within the program.
 */
void loop() {

    if (pin_change_detect_flag) {
        detect_pin_changes(&mcp0);
        detect_pin_changes(&mcp1);
        pin_change_detect_flag = 0;
    }

    // recieve data from Ethernet card
    word pos = ether.packetLoop(ether.packetReceive());
    // check if valid tcp data is received
    if (pos) {
        // WOW! we have got some data.. Let's go ahead and check them out...
//		digitalWrite(NETLED, HIGH);
        PORTD |= NETLED;
        char* data = (char *) Ethernet::buffer + pos;
        if (strncmp("GET / ", data, 6) == 0) {
            responseLog(data);
        } else if (strncmp("GET /log ", data, 9) == 0) { // this is the real deal. Someone has requested to check the log.
            responseLog(data);
        } else if (strncmp("GET /dump ", data, 10) == 0) { // Well... this is going to be slow sometimes. Because reading the whole log is not a good idea.
            ether.httpServerReplyAck();
            memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
            ether.httpServerReply_with_flags(sizeof txt_header_200 - 1, TCP_FLAGS_ACK_V);
            read_data_header();
            char tmpbuff[66];
            uint16_t addra = dh->a;
            uint16_t addrb = dh->b;
            tmpbuff[64] = 0x0a; // new line character at end - 1. end should be NULL (\0)
            if (addra != addrb) {
                while (addra != addrb) {
                    ee_d.readBlock(addra - 0x0040, (uint8_t*) tmpbuff, 0x40);
                    addra -= 0x40;
                    memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
                    ether.httpServerReply_with_flags(sizeof tmpbuff - 1, (addra == addrb) ? TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V : TCP_FLAGS_ACK_V); // Send final packet with FIN which ends the TCP transmission only if this is the last packet.
                }
            } else {
                sprintf(tmpbuff, "no data");
                tmpbuff[7] = 0x0a; // new line character at end - 1. end should be NULL (\0)
                memcpy(ether.tcpOffset(), tmpbuff, 8);
                ether.httpServerReply_with_flags(7, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
            }
        } else if (strncmp("GET /addr ", data, 10) == 0) { // For debugging purposes. This results in the DataHeader printed out via HTTP.
            ether.httpServerReplyAck();
            memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
            ether.httpServerReply_with_flags(sizeof txt_header_200 - 1, TCP_FLAGS_ACK_V);
            read_data_header();
            char tmpbuff[11];
            sprintf(tmpbuff, "HDER %04x", dh_addr);
            tmpbuff[9] = 0x0a;
            memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
            ether.httpServerReply_with_flags(sizeof tmpbuff - 1, TCP_FLAGS_ACK_V);
            sprintf(tmpbuff, "%04x %04x", dh->a, dh->b);
            tmpbuff[9] = 0x0a;
            memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
            ether.httpServerReply_with_flags(sizeof tmpbuff - 1, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V);
        } else if (strncmp("GET /clr ", data, 9) == 0) { // clear data logs
            dh->b = dh->a; // To clear the logs. This doesn't clear the actual logs at all, rather acts like a deletion of a file from a hard disk  where only table header (in the first EEPROM chip) is modified.
            write_data_header();
            ether.httpServerReplyAck();
            memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
            ether.httpServerReply_with_flags(sizeof txt_header_200 - 1, TCP_FLAGS_ACK_V);
            char tmpbuff[6];
            sprintf(tmpbuff, "done");
            tmpbuff[4] = 0x0a;
            memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
            ether.httpServerReply_with_flags(sizeof tmpbuff - 1, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
        } else if (strncmp("GET /time?", data, 10) == 0) { // to set the time
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
            ether.httpServerReply_with_flags(sizeof txt_header_200 - 1, TCP_FLAGS_ACK_V);
            memcpy_P(ether.tcpOffset(), txt_body_time_updated, sizeof txt_body_time_updated);
            ether.httpServerReply_with_flags(sizeof txt_body_time_updated - 1, TCP_FLAGS_ACK_V);
            char tmpbuff[20];
            tmpbuff[19] = 0x0a;
            sprintf(tmpbuff, "%04d-%02d-%02d %02d:%02d:%02d", t.year, t.mon, t.mday, t.hour, t.min, t.sec);
            memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
            ether.httpServerReply_with_flags(sizeof tmpbuff - 1, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
        } else if (strncmp("GET /time ", data, 10) == 0) { // to get the time
            DS3231_get(&t); // receive time from RTC
            ether.httpServerReplyAck();
            memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
            ether.httpServerReply_with_flags(sizeof txt_header_200 - 1, TCP_FLAGS_ACK_V);
            char tmpbuff[20];
            tmpbuff[19] = 0x0a;
            sprintf(tmpbuff, "%04d-%02d-%02d %02d:%02d:%02d", t.year, t.mon, t.mday, t.hour, t.min, t.sec);
            memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
            ether.httpServerReply_with_flags(sizeof tmpbuff - 1, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
        } else if (strncmp("GET /cnl?b", data, 10) == 0) { // to set a name for a specified channel
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
                addr = strtoul(tmp, NULL, 16) * 0x0800; // calculate offset for the address using bank number
                memcpy(tmp, &data[12], 1); // get index of the channel from GET parameters
                addr += strtoul(tmp, NULL, 16) * 0x0080; // add pin address to the above offset to get actual address to store the name

                ether.httpServerReplyAck();
                memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
                ether.httpServerReply_with_flags(sizeof txt_header_200 - 1, TCP_FLAGS_ACK_V);

                char c_name[end - 13 + 1];

                s.substring(13).toCharArray(c_name, end - 12);

                char writebuff[41];
                sprintf(writebuff, "%40s", c_name);
                ee_h.writeBlock(addr, (uint8_t*) writebuff, 40);

                char tmpbuff[47];
                for (uint8_t x = 0; x < 0x20; x++) {
                    ee_h.readBlock(0x0080 * (uint16_t) x, (uint8_t*) writebuff, 40);
                    sprintf(tmpbuff, "b%xc%x %40s", x / 0x10, x % 0x10, writebuff);
                    tmpbuff[45] = 0x0a;
                    memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
                    ether.httpServerReply_with_flags(sizeof tmpbuff - 1, x == 0x1f ? TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V : TCP_FLAGS_ACK_V); // Send final packet with FIN which ends the TCP transmission.
                }
            } else { // invalid / incomplete format on the request for setting channel name
                ether.httpServerReplyAck(); // send ack to the request
                memcpy_P(ether.tcpOffset(), txt_header_400, sizeof txt_header_400);
                ether.httpServerReply_with_flags(sizeof txt_header_400 - 1, TCP_FLAGS_ACK_V);
                memcpy_P(ether.tcpOffset(), txt_body_400, sizeof txt_body_400);
                ether.httpServerReply_with_flags(sizeof txt_body_400 - 1, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
            }

        } else if (strncmp("GET /cnl?reset ", data, 15) == 0) { // resets channel names to defaults
            char writebuff[41];

            for (uint8_t x = 0; x < 0x20; x++) {
                sprintf(writebuff, "%36sb%xc%x", "", x / 0x10, x % 0x10);
                ee_h.writeBlock(0x0080 * (uint16_t) x, (uint8_t*) writebuff, 40);
            }

            ether.httpServerReplyAck();
            memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
            ether.httpServerReply_with_flags(sizeof txt_header_200 - 1, TCP_FLAGS_ACK_V);
            char tmpbuff[47];
            for (uint8_t x = 0; x < 0x20; x++) {
                ee_h.readBlock(0x0080 * (uint16_t) x, (uint8_t*) writebuff, 40);
                sprintf(tmpbuff, "b%xc%x %40s", x / 0x10, x % 0x10, writebuff);
                tmpbuff[45] = 0x0a;
                memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
                ether.httpServerReply_with_flags(sizeof tmpbuff - 1, x == 0x1f ? TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V : TCP_FLAGS_ACK_V); // Send final packet with FIN which ends the TCP transmission.
            }
        } else if (strncmp("GET /cnl ", data, 9) == 0) { // response names of all channels
            ether.httpServerReplyAck();
            memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
            ether.httpServerReply_with_flags(sizeof txt_header_200 - 1, TCP_FLAGS_ACK_V);
            char writebuff[41];
            char tmpbuff[47];
            for (uint8_t x = 0; x < 0x20; x++) {
                ee_h.readBlock(0x0080 * (uint16_t) x, (uint8_t*) writebuff, 40);
                sprintf(tmpbuff, "b%xc%x %40s", x / 0x10, x % 0x10, writebuff);
                tmpbuff[45] = 0x0a;
                memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
                ether.httpServerReply_with_flags(sizeof tmpbuff - 1, x == 0x1f ? TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V : TCP_FLAGS_ACK_V); // Send final packet with FIN which ends the TCP transmission.
            }
        } else if (strncmp("GET /read ", data, 10) == 0) { // response names of all channels
            ether.httpServerReplyAck();
            memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
            ether.httpServerReply_with_flags(sizeof txt_header_200 - 1, TCP_FLAGS_ACK_V);
            char tmpbuff[18];
            sprintf(tmpbuff, INTTOBINARYPATTERN, INTTOBINARY(mcp0.readGPIOAB()));
            tmpbuff[16] = 0x0a;
            memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
            ether.httpServerReply_with_flags(sizeof tmpbuff - 1, TCP_FLAGS_ACK_V);
            sprintf(tmpbuff, INTTOBINARYPATTERN, INTTOBINARY(mcp1.readGPIOAB()));
            tmpbuff[16] = 0x0a;
            memcpy(ether.tcpOffset(), tmpbuff, sizeof tmpbuff);
            ether.httpServerReply_with_flags(sizeof tmpbuff - 1, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V);
        } else { // Page not found. Yes 404.
            ether.httpServerReplyAck(); // send ack to the request
            memcpy_P(ether.tcpOffset(), txt_header_404, sizeof txt_header_404);
            ether.httpServerReply_with_flags(sizeof txt_header_404 - 1, TCP_FLAGS_ACK_V);
            memcpy_P(ether.tcpOffset(), txt_body_404, sizeof txt_body_404);
            ether.httpServerReply_with_flags(sizeof txt_body_404 - 1, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
        }
        PORTD &= ~NETLED;
    }
}
void responseLog(char *data) {
    ether.httpServerReplyAck();
    memcpy_P(ether.tcpOffset(), txt_header_200, sizeof txt_header_200);
    ether.httpServerReply_with_flags(sizeof txt_header_200 - 1, TCP_FLAGS_ACK_V);
    read_data_header();
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
            ether.httpServerReply_with_flags(sizeof tmpbuff - 1, (i == 0x1f || addra == addrb) ? TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V : TCP_FLAGS_ACK_V); // Send final packet with FIN which ends the TCP transmission only if this is the last packet.
            if (i == 0x1f || addra == addrb) {
                break;
            }
            i++;
        }
    } else {
        sprintf(tmpbuff, "no data");
        tmpbuff[7] = 0x0a;
        memcpy(ether.tcpOffset(), tmpbuff, 8);
        ether.httpServerReply_with_flags(7, TCP_FLAGS_ACK_V | TCP_FLAGS_FIN_V); // Send final packet with FIN which ends the TCP transmission.
    }
}

/**
 * Toggles SYSLED
 */
void toggleSYS() {
    PORTD ^= SYSLED;
}

/**
 * Toggles NETLED.
 */
void toggleNET() {
    PORTD ^= NETLED;
}

/**
 * When system functions normally the SYSLED fires up
 * Detects pin changes twice as much as SYSLED firing
 */
void beatSYS() {
    // detect changes in pins of MCP23017 at address 0x20
    pin_change_detect_flag = 1;
    if (!togglesysint) {
        PORTD |= SYSLED;
        togglesysint = 1;
    } else if (togglesysint == 0b00000010) {
        PORTD &= ~SYSLED;
    } else if (togglesysint == 0b00000100) {
        PORTD |= SYSLED;
    } else if (togglesysint == 0b00001000) {
        PORTD &= ~SYSLED;
    }
    togglesysint <<= 1;
}

/**
 * To read data header from first EEPROM
 */
void read_data_header() {
    ee_h.readBlock(dh_addr, (uint8_t*) dh_block, 8);
    dh->t = (uint32_t) dh_block[0];
    dh->t |= (uint32_t) ((uint32_t) dh_block[1] << 8);
    dh->t |= (uint32_t) ((uint32_t) dh_block[2] << 16);
    dh->t |= (uint32_t) ((uint32_t) dh_block[3] << 24);
    dh->a = (uint16_t) dh_block[4];
    dh->a |= (uint16_t) ((uint16_t) dh_block[5] << 8);
    dh->b = (uint16_t) dh_block[6];
    dh->b |= (uint16_t) ((uint16_t) dh_block[7] << 8);
}

/**
 * To write data header into first EEPROM
 */
void write_data_header() {
    DS3231_get(&t);
    dh->t = 0xffffffff - t.unixtime;
    dh_block[4] = (uint8_t) (dh->a & 0xff);
    dh_block[5] = (uint8_t) ((dh->a & 0xff00) >> 8);
    dh_block[6] = (uint8_t) (dh->b & 0xff);
    dh_block[7] = (uint8_t) ((dh->b & 0xff00) >> 8);
    dh_block[0] = (uint8_t) (dh->t & 0xff);
    dh_block[1] = (uint8_t) ((dh->t & 0xff00) >> 8);
    dh_block[2] = (uint8_t) ((dh->t & 0xff0000) >> 16);
    dh_block[3] = (uint8_t) ((dh->t & 0xff000000) >> 24);
    dh_addr -= 0x0080;
    if (dh_addr == 0x0f80) {
        dh_addr = 0xff80;
    }
    if (ee_h.writeBlock(dh_addr, (uint8_t*) dh_block, 8)) {
//        Serial.println("error writing data to ee_h!");
    }
}

/**
 * Improved page write mode.
 * Only writes up to 32 chars. Using this function to write more than 32 bytes is currently not supported.
 * If we are at a page boundary and try to write more bytes wrapping around will occur which is a data corruption
 * at user end. Uses page_write function  of EEPROM to make it way faster than byte write.
 * Since Arduino Wire library has max 32 bytes as its buffer let's go for 16 bytes write each cycle. That is because
 * 2 bytes are dedicated for addresses.
 */

uint8_t record_data_page_write_mode(char* data) {

    PORTD |= EEPLED;	// turn on EEPLED to show eeprom usage
    read_data_header();
    if (ee_d.writeBlock(dh->a, (uint8_t*) data, 0x40)) {
//        Serial.println("error writing data to ee_d!");
    } else {
        dh->a += 0x0040;
        if (dh->a == dh->b) {
            dh->b += 0x00040;
        }
        write_data_header();
    }
    PORTD &= ~EEPLED;	// turn off EEPLED
    return 1;
}

/**
 * Let's try to detect pin changes by comparing the current GPIOAB values with the ones in the history using XOR
 * When there is a pin change those bits should appear as 1
 */
void detect_pin_changes(Adafruit_MCP23017 *mcp) {
    // detect changes in pins of MCP23017
    uint16_t tmpbits = mcp->readGPIOAB(); // read gpio only once
    uint16_t changedBits = (mcp->getAddr() ? mcp10_bits : mcp00_bits) ^ tmpbits; // mcp->getAddr() tells us which MCP23017 is being used here
    (mcp->getAddr() ? mcp10_bits : mcp00_bits) = (mcp->getAddr() ? mcp11_bits : mcp01_bits); // move recent pin history to former history
    (mcp->getAddr() ? mcp11_bits : mcp01_bits) = tmpbits; // assign current pin values to recent history
    if (changedBits) { // we only have to worry if there is some ping change there
//        Serial.print("a pin-change detected at MCP23017 at addr:");
//        Serial.println(mcp->getAddr(), DEC);
//        Serial.print("changed bits are:");
//        Serial.println(changedBits, BIN);
        for (uint8_t pin = 0; pin < 16; pin++) { // go thorugh all 16 pins
            uint8_t bit = (changedBits >> pin) & 0b1; // grab the required bit out of the GPIOAB values
            if (bit) { // check if this bit is not zero
                uint8_t val0 = ((mcp->getAddr() ? mcp10_bits : mcp00_bits) >> pin) & 0b1;
                if (val0 == (((mcp->getAddr() ? mcp11_bits : mcp01_bits) >> pin) & 0b1)) {
                    if ((((mcp->getAddr() ? mcp1_settled_bits : mcp0_settled_bits) >> pin) & 0b1) != val0) { // make sure there is an absolute change in pin value

                        // if there is a change then set the value in settled bits.
                        if (val0) {
                            (mcp->getAddr() ? mcp1_settled_bits : mcp0_settled_bits) |= (1 << pin); // set the pin value
                        } else {
                            (mcp->getAddr() ? mcp1_settled_bits : mcp0_settled_bits) &= ~(1 << pin); // clear the pin value
                        }
                        uint16_t addr = (0x0080 * ((uint16_t) pin)) + (mcp->getAddr() ? 0x0800 : 0);
                        ee_h.readBlock(addr, (uint8_t*) buf_prog, 40);
                        DS3231_get(&t); // receive time from RTC
                        sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d %40s %3s", t.year, t.mon, t.mday, t.hour, t.min, t.sec, buf_prog, val0 ? "ON" : "OFF");
                        record_data_page_write_mode(buf); // Write to eeprom
                        Serial.println(buf);
                    } else {
                        Serial.println("no change in pin value");
                    }
                } else {
                    Serial.println("pins not settled yet");
                }
            }
        }
    }
}

