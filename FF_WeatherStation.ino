/*

	CC1101	Usage	ESP8266
		1	GND		GND
		2	VCC		3.3V
		3	MOSI	D7
		4	SCK		D5
		5	MISO	D6
		8	CSN		D8

*/

#define VERSION "1.0.17"									// Version of this code
#include <FF_WebServer.h>									// Defines associated to FF_WebServer class
#include <TimeLib.h>										// Date/time definition
#include "ELECHOUSE_CC1101_SRC_DRV.h"						// Modified version of https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
//#define PRINT_RECEIVED_FRAME
//#define PRINT_REJECT_REASON
//#define PRINT_EXTRACTED_MESSAGE
#define PRINT_EXTRACTED_BAD_MESSAGE
//#define PRINT_MESSAGE_DETAILS
#define PRINT_DECODED_MESSAGE
#define CHECK_PATTERN_INSTEAD_OF_CRC

//	User internal data
//		Declare here user's data needed by user's code

unsigned long badCrc = 0;					// Count of bad CRC
unsigned long goodCrc = 0;					// Count of good CRC
unsigned long lightLux = 0;					// Light level in lux
unsigned long lastRainSaved = 0;			// Time last rain was saved
unsigned long lastRadioScan = 0;			// Time last radio scan occured
float tempF = 0;							// Temperature in Fahrenheit
float tempC = 0;							// Temperature in degrees
float windKmh = 0;							// Wind in km/h
float windMs = 0;							// Wind in m/s
float rainMm = 0;							// Total rain in mm
float rainMn = 0;							// Rain mm per minute
float previousRainMm = 0;					// Total rain in mm
int rawTemp = 0;							// Raw temperature as extracted
int humidity = 0;							// Humidity in percent
int windRaw = 0;							// Wind speed as extracted
int windDir = 0;							// Wind direction in degrees
int rawRain = 0;							// Rain as extracted
int uvIndex = 0;							// UV index
int lux14 = 0;								// Raw lux offset 14
int lux15 = 0;								// Raw lux offset 15
int luxMulti = 0;							// Lux by 10 multiplier flag
int rssi = 0;								// RX RSSI
uint8_t radioMsg[32];						// Extracted radio buffer
char hexMsg[65];							// Hex message buffer
byte radioBuffer[257];						// Read radio buffer
char *windAbbreviation[] = {(char*)("N"), (char*)("NE"), (char*)("E"), (char*)("SE"), (char*)("S"), (char*)("SW"), (char*)("W"), (char*)("NW")};
char *windAbbr;								// Wind direction abbreviation
String domoticzRainIdx = "";				// Domoticz rain sensor IDX
String domoticzWindIdx = "";				// Domoticz wind sensor IDX
String domoticzTempIdx = "";				// Domoticz temperature sensor IDX

//	User configuration data (should be in line with userconfigui.json)
//		Declare here user's configuration data needed by user's code

// Declare here used callbacks
static CONFIG_CHANGED_CALLBACK(onConfigChangedCallback);
static HELP_MESSAGE_CALLBACK(onHelpMessageCallback);
static DEBUG_COMMAND_CALLBACK(onDebugCommandCallback);
static REST_COMMAND_CALLBACK(onRestCommandCallback);
static JSON_COMMAND_CALLBACK(onJsonCommandCallback);
static POST_COMMAND_CALLBACK(onPostCommandCallback);
static ERROR404_CALLBACK(onError404Callback);
static WIFI_CONNECT_CALLBACK(onWifiConnectCallback);
static WIFI_DISCONNECT_CALLBACK(onWifiDisconnectCallback);
static WIFI_GOT_IP_CALLBACK(onWifiGotIpCallback);
static MQTT_MESSAGE_CALLBACK(onMqttMessageCallback);

// Load hexMsg with a dump of a buffer
void loadHexMsg(uint8_t msg[], int len){
	for (int i = 0; i < len; i++){
		sprintf(&hexMsg[i << 1], "%02x", msg[i]);
	}
}

// Get wind abbreviation giving a wind direction in degrees
char *getWindAbbreviation(int windDir) {
	int windIndex = ((windDir + 22) * 10) / 450;
	return windAbbreviation[windIndex % 8];
}

#ifdef PRINT_MESSAGE_DETAILS
	// Extract up to 7 four bits sequences from message
	uint16_t extractBytes(uint8_t const msg[], uint16_t const pos, uint16_t const halfByteCount){
		uint16_t result = 0;
		uint8_t content;
		uint16_t startPos = pos;
		// For the given (<=8) count of 4 bits
		for (uint16_t i = 0; i < halfByteCount; i++) {
			// Extract content from message
			content = msg[startPos >> 1];
			// Shift content if even position
			if (!(startPos & 1)) {
				content = content >> 4;
			}
			// Add the four bits to result
			result = (result << 4) | (content & 0x0f);
			startPos++;
		}
		return result;
	}
#endif

// Add bytes value from message on a given number of bytes (used to compute CRC)
int addBytes(uint8_t const message[], uint16_t const num_bytes) {
	int result = 0;
	for (uint16_t i = 0; i < num_bytes; ++i) {
			result += message[i];
	}
	return result;
}

// Decode one 32 bytes message
void decodeMessage(uint8_t const msg[], const uint16_t len) {
	uint8_t msgCrc = addBytes(msg, 31) & 0xff;
	// Check CRC
	#ifdef CHECK_PATTERN_INSTEAD_OF_CRC
	if (msg[14] == 0x00 && msg[15] == 0x00 && msg[16] == 0x04 && msg[17] == 0x05) {
	#else
	if (msgCrc == msg[31]) {
	#endif
		goodCrc++;
		#ifdef PRINT_EXTRACTED_MESSAGE
			Serial.print("Frame: ");
			for (uint16_t i = 0; i < len; i++) {
				Serial.printf("%02x", msg[i]);
			}
			Serial.print(" - CRC Ok!\n");
		#endif
	} else {
		badCrc++;
		#if defined(PRINT_EXTRACTED_MESSAGE) || defined(PRINT_EXTRACTED_BAD_MESSAGE)
			Serial.print("Frame: ");
			for (uint16_t i = 0; i < len; i++) {
				Serial.printf("%02x", msg[i]);
			}
			Serial.printf(" - !!! BAD CRC !!! computed %02x, received %02x\n", msgCrc, msg[31]);
		#endif
		return;
	}
	#ifdef PRINT_MESSAGE_DETAILS
		for (uint16_t i = 0; i < (len + len); i++) {
			Serial.printf("%d: %2x %d %d %d %d\n", i, extractBytes(msg, i, 1), extractBytes(msg, i, 1), extractBytes(msg, i, 2), extractBytes(msg, i, 3), extractBytes(msg, i, 4));
		}
	#endif
	/*
	Frame with UV Lux without Wind Gust
	AA 04 II IB 0T TT HH 0W WW 0D DD RR RR UU LL LL 04 05 06 07 08 09 10 11 12 13 14 15 16 17 xx SS yy
	00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32
	00 00 00 00 00 11 11 11 11 11 22 22 22 22 22 33 33 33 33 33 44 44 44 44 44 55 55 55 55 55 66 66 66 
	01 23 45 67 89 01 23 45 67 89 01 23 45 67 89 01 23 45 67 89 01 23 45 67 89 01 23 45 67 89 01 23 45
		- K: (4 bit) Kind of device, = A if Temp/Hum Sensor or = 0 if Weather Rain/Wind station
		- C: (4 bit) channel ( = 4 for Weather Rain/wind station)
		- I: (12 bit) ID
		- B: (4 bit) BP01: battery low, pairing button, 0, 1
		- T: (12 bit) temperature in F, offset 900, scale 10
		- H: (8 bit) humidity %
		- R: (16) Rain
		- W: (12) Wind speed
		- D: (9 bit) Wind Direction
		- U: (5 bit) UV index
		- L: (1 + 15 bit) Lux value, if first bit = 1 , then x 10 the rest.
		- A: (4 bit) fixed values of 0xA
		- 0: (4 bit) fixed values of 0x0
		- xx: incremental value each tx
		- yy: incremental value each tx yy = xx + 1
		- S: (8 bit) checksum
	*/
	rawTemp = ((msg[4] & 0x0f) << 8) | (msg[5]); // weird format
	tempF = (rawTemp - 900) * 0.1f;
	tempC = (tempF - 32) * (5.0 / 9.0);
	humidity = msg[6];
	windRaw = (((msg[7] - 1) & 0xff) << 8) | ((msg[8] - 1) & 0xff);
	windKmh = windRaw * 0.2f;
	windMs = windKmh * 0.2777f;
	windDir = (((msg[9] - 1) & 0x0f) << 8) | ((msg[10] - 1) & 0xff);
	windAbbr = getWindAbbreviation(windDir);
	rawRain = (((msg[11] - 1) & 0xff) << 8) | ((msg[12] - 1) & 0xff);
	rainMm = rawRain * 0.2f;
	rainMn = 0;
	unsigned long now = millis();
	if (previousRainMm != 0 && rainMm > previousRainMm) {
		rainMn = (rainMm - previousRainMm) * (now - lastRainSaved) / 60000; 
	} 
	previousRainMm = rainMm;
	lastRainSaved = now;
	uvIndex = (msg[13] - 1) & 0x1f;
	lux14 = (msg[14] - 1) & 0xff;
	lux15 = (msg[15] - 1) & 0xff;
	luxMulti = ((lux14 & 0x80) >> 7);
	lightLux = ((lux14 & 0x7f) << 8) | (lux15);
	if (luxMulti == 1) {
		lightLux = lightLux * 10;
	}
	#ifdef PRINT_DECODED_MESSAGE
		Serial.printf("temp:%.1f, hum:%d, windSpeed:%.1f, windDir:%d/%s, rain:%.1f, rainMn:%.2f, uv:%d, lux:%lu\n",
			tempC, humidity, windKmh, windDir, windAbbr, rainMm, rainMn, uvIndex, lightLux);
	#endif

	loadHexMsg(radioMsg, sizeof(radioMsg));
	char tempBuffer[280];
	snprintf_P(tempBuffer, sizeof(tempBuffer)-1, 
		PSTR("{\"date\":\"%04d/%02d/%02d %02d:%02d:%02d\",\"temperature\":%.01f,\"humidity\":%d,\"windSpeed\":%.1f,\"windDirection\":%d,\"direction\":\"%s\",\"rain\":%.01f,\"rainMn\":%.02f,\"uv\":%d,\"lux\":%lu,\"frame\":\"%s\",\"rssi\":%d,\"goodCrc\":%lu,\"badCrc\":%lu}"),
		year(), month(), day(), hour(), minute(), second(),
		tempC, humidity,
        windKmh, windDir, windAbbr, rainMm, rainMn,
        uvIndex, lightLux, hexMsg, rssi, goodCrc, badCrc);
	FF_WebServer.mqttPublish("data", tempBuffer, true);
	// Update Domoticz wind
	if (domoticzWindIdx !="") {
		int domoticzMs = windMs * 10.0;
		snprintf_P(tempBuffer, sizeof(tempBuffer), "%d;%s;%d;0;0;0", windDir, windAbbr, domoticzMs);
		FF_WebServer.sendDomoticzValues(domoticzWindIdx.toInt(), tempBuffer, 0);
	}
	// Update Domoticz rain
	if (domoticzRainIdx !="") {
		snprintf_P(tempBuffer, sizeof(tempBuffer), "0;%.1f", rainMm);
		FF_WebServer.sendDomoticzValues(domoticzRainIdx.toInt(), tempBuffer, 0);
	}
	// Update Domoticz temperature
	if (domoticzTempIdx !="") {
		snprintf_P(tempBuffer, sizeof(tempBuffer), "%.1f;%d;1", tempC, humidity);
		FF_WebServer.sendDomoticzValues(domoticzTempIdx.toInt(), tempBuffer, 0);
	}
}

// Here are the callbacks code

/*!

	This routine is called when permanent configuration data has been changed.
		User should call FF_WebServer.load_user_config to get values defined in userconfigui.json.
		Values in config.json may also be get here.

	\param	none
	\return	none

*/

CONFIG_CHANGED_CALLBACK(onConfigChangedCallback) {
	trace_info_P("Entering %s", __func__);
	FF_WebServer.load_user_config("domoticzRainIdx", domoticzRainIdx);
	FF_WebServer.load_user_config("domoticzWindIdx", domoticzWindIdx);
	FF_WebServer.load_user_config("domoticzTempIdx", domoticzTempIdx);
}

/*!

	This routine is called when help message is to be printed

	\param	None
	\return	help message to be displayed

*/

HELP_MESSAGE_CALLBACK(onHelpMessageCallback) {
	return "";
}

/*!

	This routine is called when a user's debug command is received.

	User should analyze here debug command and execute them properly.

	\note	Note that standard commands are already taken in account by server and never passed here.

	\param[in]	lastCmd: last debug command entered by user
	\return	none

*/

DEBUG_COMMAND_CALLBACK(onDebugCommandCallback) {
	trace_info_P("Entering %s", __func__);
	// "user" command is a standard one used to print user variables
	if (debugCommand == "user") {
		trace_info_P("traceFlag=%d", FF_WebServer.traceFlag);
		trace_info_P("debugFlag=%d", FF_WebServer.debugFlag);
		// -- Add here your own user variables to print
		trace_info_P("domoticzRainIdx=%s", domoticzRainIdx.c_str());
		trace_info_P("domoticzWindIdx=%s", domoticzWindIdx.c_str());
		trace_info_P("domoticzTempIdx=%s", domoticzTempIdx.c_str());
		trace_info_P("badCrc=%lu", badCrc);
		trace_info_P("goodCrc=%lu", goodCrc);
		trace_info_P("tempF=%.1f", tempF);
		trace_info_P("tempC=%.1f", tempC);
		trace_info_P("windKmh=%.1f", windKmh);
		trace_info_P("windMs=%.1f", windMs);
		trace_info_P("rainMm=%.1f", rainMm);
		trace_info_P("rawTemp=%d", rawTemp);
		trace_info_P("humidity=%d", humidity);
		trace_info_P("windRaw=%d", windRaw);
		trace_info_P("windDir=%d", windDir);
		trace_info_P("rawRain=%d", rawRain);
		trace_info_P("uvIndex=%d", uvIndex);
		trace_info_P("lux14=%d", lux14);
		trace_info_P("lux15=%d", lux15);
		trace_info_P("luxMulti=%d", luxMulti);
		trace_info_P("lightLux=%lu", lightLux);
		loadHexMsg(radioMsg, sizeof(radioMsg));
		trace_info_P("radioMsg=%s", hexMsg);
		// -----------
		return true;
	// Put here your own debug commands
	} else if (debugCommand == "mycmd") {
		trace_info_P("I'm inside mycmd...", 0);
		return true;
	// -----------
	}
	return false;
}

/*!

	This routine analyze and execute REST commands sent through /rest GET command
	It should answer valid requests using a request->send(<error code>, <content type>, <content>) and returning true.

	If no valid command can be found, should return false, to let server returning an error message.

	\note	Note that minimal implementation should support at least /rest/values, which is requested by index.html
		to get list of values to display on root main page. This should at least contain "header" topic,
		displayed at top of page. Standard header contains device name, versions of user code, FF_WebServer template
		followed by device uptime. You may send something different.
		It should then contain user's values to be displayed by index_user.html file.

	\param[in]	request: AsyncWebServerRequest structure describing user's request
	\return	true for valid answered by request->send command, false else

*/
REST_COMMAND_CALLBACK(onRestCommandCallback) {
	if (request->url() == "/rest/values") {
		char tempBuffer[500];
		int updHours = 99;
		int updMin = 99;
		int updSec = 99;
        if (lastRainSaved) {
		unsigned long updateDelta = (millis() - lastRainSaved) / 1000;
            updHours = updateDelta / 3600;
		int secsRemaining = updateDelta % 3600;
            updMin = secsRemaining / 60;
            updSec = secsRemaining % 60;
        }

		tempBuffer[0] = 0;
		float pctGoodCrc = 0;
		if (goodCrc != 0) {
			pctGoodCrc = goodCrc * 100.0 / (goodCrc + badCrc);
		}

		snprintf_P(tempBuffer, sizeof(tempBuffer),
			PSTR(
				// -- Put header composition
				"header|%s V%s/%s, up since %s|div\n"
				// -- Put here user variables to be available in index_user.html
				"temp|%.1f|div\n"
				"hum|%d|div\n"
				"wSpd|%.1f|div\n"
				"wMs|%.1f|div\n"
				"wDir|%d|div\n"
				"wAbbr|%s|div\n"
				"rain|%.1f|div\n"
				"rainMn|%.2f|div\n"
				"uv|%d|div\n"
				"lux|%lu|div\n"
				"crcPct|%.1f|div\n"
				"upd|%02d:%02d:%02d|div\n"
				// -----------------
				)
			// -- Put here values of header line
			,FF_WebServer.getDeviceName().c_str()
			,VERSION, FF_WebServer.getWebServerVersion()
			,NTP.getUptimeString().c_str()
			// -- Put here values in index_user.html
			,tempC
			,humidity
			,windKmh
			,windMs
			,windDir
			,windAbbr
			,rainMm
			,rainMn
			,uvIndex
			,lightLux
			,pctGoodCrc
			,updHours, updMin, updSec
			// -----------------
			);
		request->send(200, "text/plain", tempBuffer);
		return true;
	}

	trace_info_P("Entering %s", __func__);					// Put trace here as /rest/value is called every second0
	if (request->url() == "/rest/err400") {
		request->send(400, "text/plain", "Error 400 requested by user");
		return true;
	}
	return false;
}

/*!

	This routine analyze and execute JSON commands sent through /json GET command
		It should answer valid requests using a request->send(<error code>, <content type>, <JSON content>)
		and returning true.

		If no valid command can be found, should return false, to let template code returning an error message.

	\param[in]	request: AsyncWebServerRequest structure describing user's request
	\return	true for valid answered by request->send command, false else

*/

JSON_COMMAND_CALLBACK(onJsonCommandCallback) {
	trace_info_P("Entering %s", __func__);
	return false;
}

/*!

	This routine analyze and execute commands sent through POST command
		It should answer valid requests using a request->send(<error code>, <content type>, <content>) and returning true.

	If no valid command can be found, should return false, to let template code returning an error message.

	\param[in]	request: AsyncWebServerRequest structure describing user's request
	\return	true for valid answered by request->send command, false else

*/

POST_COMMAND_CALLBACK(onPostCommandCallback) {
	trace_info_P("Entering %s", __func__);
	return false;
}

/*!

	This routine is called when a 404 error code is to be returned by server
		User can analyze request here, and add its own. In this case, it should answer using a request->send(<error code>, <content type>, <content>) and returning true.

	If no valid answer can be found, should return false, to let template code returning an error message.

	\param[in]	request: AsyncWebServerRequest structure describing user's request
	\return	true for valid answered by request->send command, false else

*/

ERROR404_CALLBACK(onError404Callback) {
	trace_info_P("Entering %s", __func__);
	return false;
}

/*!

	This routine is called each time WiFi station is connected to an AP

	\param[in]	data: WiFiEventStationModeConnected event data
	\return	none

*/
WIFI_CONNECT_CALLBACK(onWifiConnectCallback) {
	trace_info_P("Entering %s", __func__);
}

/*!

	This routine is called each time WiFi station is disconnected from an AP

	\param[in]	data: WiFiEventStationModeDisconnected event data
	\return	none

*/
WIFI_DISCONNECT_CALLBACK(onWifiDisconnectCallback) {
	trace_info_P("Entering %s", __func__);
}

/*!

	This routine is called each time WiFi station gets an IP

	\param[in]	data: WiFiEventStationModeGotIP event data
	\return	none

*/
WIFI_GOT_IP_CALLBACK(onWifiGotIpCallback) {
	trace_info_P("Entering %s", __func__);
}

/*!

	This routine is called each time MQTT is (re)connected

	\param	none
	\return	none

*/
MQTT_CONNECT_CALLBACK(onMqttConnectCallback) {
	trace_info_P("Entering %s", __func__);
}

/*!

	This routine is called each time MQTT receives a subscribed topic

	\note	** Take care of long payload that will arrive in multiple packets **

	\param[in]	topic: received message topic
	\param[in]	payload: (part of) payload
	\param[in]	len: length of (this part of) payload
	\param[in]	index: index of (this part of) payload
	\param[in]	total: total length of all payload parts
	\return	none

*/
MQTT_MESSAGE_CALLBACK(onMqttMessageCallback) {
	trace_info_P("Entering %s", __func__);
}

//	This is the setup routine.
//		Initialize Serial, LittleFS and FF_WebServer.
//		You also have to set callbacks you need,
//		and define additional debug commands, if required.
void setup() {
	// Open serial connection
	Serial.begin(74880);
	Serial.printf("\nStarting...\n");
	// Start Little File System
	LittleFS.begin();
	// Start FF_WebServer
	FF_WebServer.begin(&LittleFS, VERSION);
	Serial.setDebugOutput(false);
	// Register user's trace callback, if needed
	#ifdef FF_DISABLE_DEFAULT_TRACE
		trace_register(myTraceCallback);
	#endif
	// Set user's callbacks
	FF_WebServer.setConfigChangedCallback(&onConfigChangedCallback);
	FF_WebServer.setDebugCommandCallback(&onDebugCommandCallback);
	FF_WebServer.setRestCommandCallback(&onRestCommandCallback);
	FF_WebServer.setHelpMessageCallback(&onHelpMessageCallback);
	FF_WebServer.setJsonCommandCallback(&onJsonCommandCallback);
	FF_WebServer.setPostCommandCallback(&onPostCommandCallback);
	FF_WebServer.setError404Callback(&onError404Callback);
	FF_WebServer.setWifiConnectCallback(&onWifiConnectCallback);
	FF_WebServer.setWifiDisconnectCallback(&onWifiDisconnectCallback);
	FF_WebServer.setWifiGotIpCallback(&onWifiGotIpCallback);
	FF_WebServer.setMqttConnectCallback(&onMqttConnectCallback);
	FF_WebServer.setMqttMessageCallback(&onMqttMessageCallback);

	if (ELECHOUSE_cc1101.getCC1101()){		// Check the CC1101 Spi connection.
		trace_info_P("CC1101 connection OK", 0);
	} else {
		trace_error_P("CC1101 connection Error", 0);
		while(1);
	}
	ELECHOUSE_cc1101.setSres();				// Set reset
	ELECHOUSE_cc1101.Init();				// Must be set to initialize the cc1101!

	ELECHOUSE_cc1101.setCCMode(1);			// Set config for internal transmission mode.
	ELECHOUSE_cc1101.setChannel(0);			// Set the Channelnumber from 0 to 255. Default is channel 0.
	ELECHOUSE_cc1101.setChsp(199.95);		// The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz.
	ELECHOUSE_cc1101.setPA(10);				// Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
	ELECHOUSE_cc1101.setAdrChk(0);			// Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
	ELECHOUSE_cc1101.setAddr(0);			// Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
	ELECHOUSE_cc1101.setWhiteData(0);		// Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
	ELECHOUSE_cc1101.setCRC_AF(0);			// Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.
	ELECHOUSE_cc1101.setDcFilterOff(0);		// Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).
	ELECHOUSE_cc1101.setManchester(0);		// Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.
	ELECHOUSE_cc1101.setFEC(0);				// Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.
	ELECHOUSE_cc1101.setPRE(0);				// Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24
	ELECHOUSE_cc1101.setPQT(0);				// Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
	ELECHOUSE_cc1101.setAppendStatus(0);	// When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.

	ELECHOUSE_cc1101.setSidle();			// Set idle
	ELECHOUSE_cc1101.setMHZ(433.89);		// Base frequency
	ELECHOUSE_cc1101.setModulation(0);		// Set modulation mode.
	ELECHOUSE_cc1101.setDRate(11.22); 		// Set the Data Rate in kBaud
	ELECHOUSE_cc1101.setSyncMode(2);		// Combined sync-word qualifier mode (2 = 16/16 sync word bits detected)
	ELECHOUSE_cc1101.setSyncWord(0xCA, 0x54);	// Set sync word
	ELECHOUSE_cc1101.setPktFormat(0);		// Format of RX and TX data (0 = Normal mode, use FIFOs for RX and TX)
	ELECHOUSE_cc1101.setLengthConfig(1);	// 1 = Variable packet length mode
	ELECHOUSE_cc1101.setPacketLength(0);	// Indicates the packet length
	ELECHOUSE_cc1101.setCrc(0);				// 0 = CRC disabled for TX and RX.
	ELECHOUSE_cc1101.setRxBW(160);			// Set the Receive Bandwidth in kHz
	ELECHOUSE_cc1101.setDeviation(35);	// Set frequency deviation
    
	uint8_t foccfg = ELECHOUSE_cc1101.SpiReadReg(CC1101_FOCCFG);	// Read current FOCCFG
	ELECHOUSE_cc1101.SpiWriteReg(CC1101_FOCCFG, foccfg | (1 << 5));	// Force bit 5 (FOC_BS_CS_GATE)
	ELECHOUSE_cc1101.SpiWriteReg(CC1101_BSCFG, 0x6C);	// Bit Synchronization Configuration
	ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL0, 0x91);
	ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL1, 0x40);
	ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0x43);

    windAbbr = getWindAbbreviation(0);		// To avoid crash before first message received
}

//	This is the main loop.
//	Do what ever you want and call FF_WebServer.handle()
void loop() {
	// User part of loop

	//Checks whether something has been received every ms
    unsigned long now = millis();
    if ((now - lastRadioScan) >= 1) {
        lastRadioScan = now;
        if (ELECHOUSE_cc1101.CheckRxFifo(24)){              // It tooks approx 16 ms to read 250 chars of 10 bits @ 160 kHz
            memset(radioBuffer, 0, sizeof(radioBuffer));
            //Get received Data and calculate length
            int len = ELECHOUSE_cc1101.ReceiveData(radioBuffer);
            // Get RSSI
            rssi = ELECHOUSE_cc1101.getRssi();
            // Reject noisy frames
            if ((rssi >= -90) && len) {
                #ifdef PRINT_RECEIVED_FRAME
                    Serial.printf("Rssi: %d, len: %d, radioBuffer:", rssi, len);
                    for (int i = 0; i < len; i++) {
                        Serial.printf(" %02x", radioBuffer[i]);
                    }
                    Serial.print("\n");
                #endif
                // Check preamble
                if (radioBuffer[0] == 0xaa) {
                // Copy message after pattern
                memcpy(radioMsg, &radioBuffer[0], sizeof(radioMsg)); 
                decodeMessage(radioMsg, sizeof(radioMsg));
                #ifdef PRINT_REJECT_REASON
                } else {
                    Serial.printf("Frame too noisy or too short\n");
                #endif
                }
            }
        }
	}

	// Manage Web Server
	FF_WebServer.handle();
}

/*!
	This routine is called each time a trace is requested by FF_TRACE.

		It has to be declared in setup() by a "static trace_register(myTraceCallback);"

		\param[in]	_level: severity level of message (can be any FF_TRACE_LEVEL_xxxx value)
		\param[in]	_file: calling source file name with extension
		\param[in]	_line: calling source file line
		\param[in]	_function: calling calling source function name
		\param[in]	_message: text message to send
		\return	None
		\note	Note that defining FF_DISABLE_DEFAULT_TRACE will suppress default trace, if required.
				If you keep the default trace, you may add some other output media(s), like MQTT, file...
				This example code reproduce the default trace routine.
				Don't hesitate to change it, if it doesn't fit your needs.

*/
#ifdef FF_DISABLE_DEFAULT_TRACE
	trace_callback(myTraceCallback) {
		#if defined(FF_TRACE_USE_SYSLOG) || defined(FF_TRACE_USE_SERIAL) || defined(REMOTE_DEBUG) || defined(SERIAL_DEBUG)
			// Compose header with file, function, line and severity
			const char levels[] = "NEWIDV";
			char head[80];

			snprintf_P(head, sizeof(head), PSTR("%s-%s-%d-%c"), _file, _function, _line, levels[_level]);
			// Send trace to Serial if needed and not already done
			#if !defined(SERIAL_DEBUG) && defined(FF_TRACE_USE_SERIAL)
				Serial.print(head);
				Serial.print("-");
				Serial.println(_message);
			#endif
			// Send trace to syslog if needed
			#ifdef FF_TRACE_USE_SYSLOG
				syslog.deviceHostname(head);
				syslog.log(_message);
			#endif
			// Send trace to debug if needed
			#if defined(REMOTE_DEBUG) || defined(SERIAL_DEBUG)
				switch(_level) {
				case FF_TRACE_LEVEL_ERROR:
					debugE("%s-%s", head, _message);
					break;
				case FF_TRACE_LEVEL_WARN:
					debugW("%s-%s", head, _message);
					break;
				case FF_TRACE_LEVEL_INFO:
					debugI("%s-%s", head, _message);
					break;
				default:
					debugD("%s-%s", head, _message);
					break;
				}
			#endif
			#ifdef FF_TRACE_KEEP_ALIVE
				FF_WebServer.resetTraceKeepAlive();
			#endif
		#endif
	}
#endif
