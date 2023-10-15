# NIP-91 POC

https://github.com/nostr-protocol/nips/blob/iot/91.md

Get sensor data and push to relays

## Notes

Connections:
For this to work, make sure you connect the BME280 to the ESP32 as follows:

VCC -> 3.3V (of ESP32)
GND -> GND
SDA -> GPIO21 (Default SDA for ESP32)
SCL -> GPIO22 (Default SCL for ESP32)
If you're using a different I2C address for your BME280 (some modules may use 0x77), you'll need to modify the bme.begin(0x76) line accordingly.

After uploading the sketch to your ESP32, open the Arduino Serial Monitor to see the temperature, humidity, and pressure values displayed every 2 seconds.