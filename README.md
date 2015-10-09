# data_logger
Data logging for electronic alarms.

Many radio transmission related equipment has alarm outputs for their status changes. There had been a requirement for logging these incidents in real-time. This project was built in order to fulfill that requirement.

The project had been a success and has been tested to support 32 ports in polling mode.

It uses ATMEGA328P as the brain. Uses DS3231 as real-time clock. It has ethernet support (10Mbps) via Microchip's ENC28J60 chip. Port expansion is done via MCP23017 io-expander. It has external EEPROMS which has much higher endurance than the eeprom inside the ATMega chip. A simple but handy wear-levelling algorithm is already in place (please look at 32_port_polling branch) which just works.

This project was delivered as a volunteer task to Radio Logan (101FM) QLD Australia. It is currently running as a technical equipment at the transmission hut.

