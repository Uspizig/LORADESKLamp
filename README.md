# LORADESKLamp
A LORA DeskLamp that measures your Air Quality

Please see https://hackaday.io/project/180792-loralamp-airquality for further details


This files are additional Material LoRaLamp AIRQuality project published on Hackaday.io
(https://hackaday.io/project/180792-loralamp-airquality)
 
 
 
 WARNINGS
 1. Before you compile please ensure you modify all your credentials and used sensors in "credentials.h" which have been marked REPLACEMEUSER
 2. USE a Proper POWER SOURCE with at exactly 5 Volt(NOT 6V, not 4V)DC Voltage / The Power Supply should be capable of handling at least/minimum of 2 Ampere output
 3. To give You some more safety I set the default Amount of LEDS in the credentials.h to 6. 
 4. If you want to use more LEDS and have enough Power change ANZAHL_LEDS to 36
 This Source Code is provided "AS it is" . 
 If you use this code you agree that any harm, damage, burn or injuries are YOUR RESPONSIBILITY!
 The Author may not be held liable for any damage that might happen if you try this code out.
 
 
 Legal validity of this disclaimer
	This disclaimer is to be regarded as part of the internet publication which you were
	referred from. If sections or individual terms of this statement are not legal or correct,
	the content or validity of the other parts remain uninfluenced by this fact.

Referrals and links
	The author is not responsible for any contents linked or referred to from his pages -
	unless he has full knowledge of illegal contents and would be able to prevent the
	visitors of his site from viewing those pages. If any damage occurs by the use of
	information presented there, the author might not be liable.
	Furthermore the author is not liable for
	any postings or messages published by users of discussion boards, guest books or
	mailing lists provided.

 
 Also ensure you have installed the following libs:
 

Code Sources that influenced this Source Code and which libaries you may need:
- https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_auth/mqtt_auth.ino
- https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
- RGBW for SK6812: 
  - Original code by Jim Bumgardner (http://krazydad.com).
  - Modified by David Madison (http://partsnotincluded.com).
  - Extended by Christoph Wempe
  - https://gist.github.com/CWempe?direction=desc&sort=created

Adafruit BME/BMP280/SGP30 sketches/libs
- Please consider buying their products due to their great work in Arduino libs
- https://github.com/adafruit/Adafruit_SGP30
- https://github.com/adafruit/Adafruit_BMP280_Library

Bosch BME680 BSEC Lib
- https://github.com/BoschSensortec/BSEC-Arduino-library

Andreas Spiess
- OTA Sketch for OTA Update of the ESP32 over Wifie
- https://github.com/SensorsIot/ESP32-OTA

Hideakitai MPU9250 Sketch
- https://github.com/hideakitai/MPU9250

knolleary pubsubclient
- /pubsubclient/blob/master/examples/mqtt_auth/mqtt_auth
- https://github.com/knolleary/pubsubclient

Fastled
- https://github.com/FastLED/FastLED

/*
Goal:
Project Goal Build a Desktop Lamp that can measure the Air Quality and warn the user if certain levels are reached.


What is working:
 - Update with OTA over Wifi
 - Switch on the upper LED under the rotary encoder
 - Switch on the lower LED
 - Detect a turn of the LED with the MPU9250 Gyro
 - Measure Temperature with the BMP280
 - Measure Brghtness with the LDR 
 - Wifi connectivity
 - Publish and recieve Data over MQTT
 - Measure eCo2, TVOC Data from BME680
 - Gyro Calibration on startup
 - Code is working with SK6812 Diodes and WS2812

Where to improve:
- Action performed on Gyro Measurements
- FailSafe methods if Wifi or MQTT Server becomes unavailible

Things that needs to be implemented:
 - AdvANCED Methods that are performed whenever the lamp is turned
 - Current measurement to protect about over current and detect shortcuts to GND
 - Change or enable Light when AirQuality becomes over a specific value
 - Update all warning limits of the sensors(Temp, CO2, Humidity,Light ..)  over MQTT
 - Update Wifi Credentials, MQTT Server over Wifi
 - Integrate Demo Lora Sketch to this source
 - NTP:Current Time
 - Quiet Time: Times when the Dekslamp shall go to sleep
 - Adaptaions vor PVB V2 to disable quiscent current of each LED during off Phase
 - Low Power Mode(Sleep Mode of all sensors)
 - Ultra Low Power Mode: Let the Processor sleep


Basic Operation:
- Switch On the Desklamp with a Button PRESS [OK]
- Change Brightness or collor by rotating the knob [OK]
- Nice LED by LED Dimming for nice shutdown of the light[OK}

Advanced:
- Save the most used Brightness level by Longpress the button (5 Sec) [not yet implemented]
- Set the color temperature[Partly OK]
- Measure the Temperature/Humidity/CO2/Brightness[OK]
- Aktivate a Timer Function . Lamp Blinks once the timer has been reached: Aktivate by Turn the lamp head to 90° [Not yet implemented]
- Set the Color via MQTT[OK]

Advanced Phase 2:
- Set the Timerlength via MQTT [Not done yet]
- Detect Bluetooth Devices[needs to be merged to this sketch]
- Transmit Data over LORA [OK but needs to be merged to this sketch]

  LORA:
  - Trasnmit Data if LED is switched ON/OFF= PORT1
  - Transmit detected Bluetooth Devices, Transmit detected Users with Corona Warn APP = PORT2
  - Transmit Air Quality Sensors = PORT3
