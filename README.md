# SolisModbus-MQTT
ESP8266 bridge between Solis/ Ginlong Hybrid inverter and MQTT
jimmyburnworld, October 2022
SolisModbus-MQTT Bridge using ESP8266 based controller
Uses ModbusMaster and SoftwareSerial libraries for the RS485/ Modbus side of things
ESP8266WiFi and ArduinoMqttCLient for the MQTT side
https://github.com/jimmyburnworld/SolisModbus-MQTT

Uses Solis inverter 'COM' port. Set inverter 'address' to 2 - check communications to BMS and Meter after doing this before proceeding
to connect Arduino.

COM port: 1 = +5Vcc, 2 = 0V com, 3 = 'A' 4 = 'B' - This might vary by model, your responsibility to check before connecting
MAX485 GND needed to be connected to 'G' next to 3.3V pin to work.
Used inverter +5V and 0V to power ESP8266 using 'Vin' and 'G' connections & jumpered to MAX485 Vcc & Gnd

KNOWN ISSUES: Signed 16bit values and 32bit values do not work correctly.
The signed 16bit values are unsigned because the ModbusMaster library only returns uint16_t type.
32bit values are due to word/ byte swapping Endian-ness etc.
Only included the variables for this project, feel free to add more or remove
Search for RS485_MODBUS-Hybrid-BACoghlan-201811228-1854.pdf for protocol description and registers