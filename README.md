# FF_WeatherStation interface to MQTT and Domoticz
Integrates Lidl Weather station (SM-041/EM3390A) into MQTT and Domoticz.

## What's for?

This code reads 433 MHz messages sent by LIDL Weather station and sends them to MQTT and optionally to Domoticz. 433 MHz data is read using a CC1101 radio board.

## Prerequisites

You must have either Arduino IDE or VSCodium (free version of Visual Studio) installed in order to compile the code.

## Installation

Clone repository somewhere on your disk.
```
cd [where_you_want_to_install_it]
git clone https://github.com/FlyingDomotic/FF_WheaterStation.git FF_WheaterStation
```

## Update

Go to code folder and pull new version:
```
cd [where_you_installed_FF_WheaterStation]
git pull
```

Note: if you did any changes to files and `git pull` command doesn't work for you anymore, you could stash all local changes using:
```
git stash
```
or
```
git checkout [modified file]
```

## Connections between CC1101 and ESP8266

CC1101	Usage	ESP8266
- 	1	GND		GND
- 	2	VCC		3.3V
- 	3	MOSI	D7
- 	4	SCK		D5
 -	5	MISO	D6
 -	6	GDO2	D2
 -	7	GDO0	D1
 -	8	CSN		D8
