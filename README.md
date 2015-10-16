# data_logger
Data logging for electronic alarms.

Many radio transmission related equipment has alarm outputs for their status changes. There had been a requirement for logging these incidents in real-time. This project was built in order to fulfill that requirement.

The project had been a success and has been tested to support 32 ports in polling mode.

It uses ATMEGA328P as the brain. Uses DS3231 as real-time clock. It has ethernet support (10Mbps) via Microchip's ENC28J60 chip. Port expansion is done via MCP23017 io-expander. It has external EEPROMS which has much higher endurance than the eeprom inside the ATMega chip. A simple but handy wear-levelling algorithm is already in place (please look at 32_port_polling branch) which just works.

This project was delivered as a volunteer task to Radio Logan (101FM) QLD Australia. It is currently running as a technical equipment at the transmission hut.


 Requirements:
 1. Log multiple events.
 	We are using two external eeproms (24LC512 x 2) since the internal EEPROM at ATMega328p is having around 100k erase/write cycles.
 	24LC512 on the other hand has 1000k (1M) erase/write cycles. 24LC512s are connected via I²C bus.
 	Under the implemented circuit configuration, they can be addressed via 0xA0 and 0xA1 addresses.
 2. Monitor up to 32 channels.
 	Using ATMega328p chip alone we are unable to achieve this. Since Mega already has SPI - http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
 	and I²C - http://en.wikipedia.org/wiki/I%C2%B2C in-built hardware, there is no issue in expanding it's capabilities. I have chosen MCP23017 since each
 	chip supports up to 16 channels and  8 chips can be attached to the same I²C bus.
 3. All events should be logged with real-time data.
 	For that purpose we are using a DS3231 temperature compensated RTC module with in-built 3V Lithum battery, which would run at leat 5 years since we
 	are using I²C bus only when the whole device is powered up externally. We are using Petre Rodan's ds3231 library to communicate easily.
 4. Logged data should be able to be monitored via ethernet.
 	We are using an ethernet module based on  ENC28J60 chip by Microchip. It is easily accessible via SPI and there are numerous open-source libraries exist.
 	I used EtherCard library by Guido Socher and Pascal Stang since their library can be used to construct large responses using multiple small packets. Small
 	packets are extremely important in embedded hardware/firmware solutions since the platform limited in resources.
