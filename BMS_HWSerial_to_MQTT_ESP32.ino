//
//	BMS_Serial_to_MQTT
//
//	Read data from serial port on JBD BMS and send to Telegraf via MQTT.
//	Data is also broadcast on network for anything else to pick it up (eg:phone app) (removed)
//
//
//	developed for ESP32 platform
//	compile with "DOIT ESP32 DEVKIT V1" board setting
//
//	Stephen Cramp
//	github: PlayersZ28
//

#include "WiFi.h"
#include <WiFiUdp.h>
#include "EspMQTTClient.h"
#include "String.h"
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
	
#define		NETWORK_NAME		"your_network_name"
#define		NETWORK_PASSWORD	"your_network_password"
//#define		BROKER_IP			"127.0.0.1"
#define		MQTT_IP				"10.0.0.100"	// IP of the MQTT receiver

const char* ssid = NETWORK_NAME;
const char* password = NETWORK_PASSWORD;

#define		USE_JSON
//#define		USE_INLINE
#define		BATTERY_VALUE		1			// used for blinking the battery number
#define		BATTERY_NUM			"battery01"
#define		METER_NAME			"bms01"
#define		OTA_MESSAGE			"You are connected to " BATTERY_NUM "  To upload via OTA go to /update on this IP address"

#define 	TOPIC		"trailer/batteries/" BATTERY_NUM
#define 	GAUGES		TOPIC "/gauges"
#define 	CELLS		TOPIC "/cells"
#define 	GENERAL		TOPIC "/general"
#define 	BMSINFO		TOPIC "/bmsinfo"
//#define		BROKER		BROKER_IP
#define		PORT		1883

#define RXD2 16
#define TXD2 17

char		msg[50];
boolean		bMQTTConnected = false;
long		lSerialFailCount = 0;
int			iCurrentRequest = 0;	// set to 3, 4 or 5

EspMQTTClient client(
  NETWORK_NAME,
  NETWORK_PASSWORD,
  MQTT_IP,  		// MQTT Broker server ip
  "",   			// Can be omitted if not needed
  "",   			// Can be omitted if not needed
  BATTERY_NUM,      // Client name that uniquely identify your device
  1883
);

#define	NUM_GAUGES	6		// voltage, current, rmaining capacity, nominal capacity, temperature 1, temperature 2
#define MAX_CELLS	32		// add one for quantity of cells
#define NUM_GENERAL	10		// Cycle Times, Date Manufactured, Balanced State Low, Balanced State High, Protection State, Software Version, RSOC, MOSFET Control Status, Battery Serial Number
#define	NUM_BMSINFO	1		// bms information string

// for OTA updates
AsyncWebServer server(80);

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
  bMQTTConnected = true;
}

void blinky( int iTimes )
{
	while( iTimes-- > 0 )
	{
		digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
		delay(250);                       // wait for a second
		digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
		delay(250);                       // wait for a second
	}

}


void setup() {

	Serial.begin(115200);

	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);

	blinky( BATTERY_VALUE );		// blink LED to indicate which battery will be reported
	delay( 1000 );
	blinky( BATTERY_VALUE );

	// setup for OTA
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	Serial.println("");
	
	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
	delay(500);
	Serial.print(".");
	}
	Serial.println("");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
	
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
	request->send(200, "text/plain", OTA_MESSAGE);
	});
	
	AsyncElegantOTA.begin(&server);    // Start ElegantOTA
	server.begin();
	Serial.println("HTTP server started");
	
	bMQTTConnected = false;
	
	delay(2000);
	Serial.println("\nJBD BMS Serial to MQTT");
	
	Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
	
	//serialBMS.begin(9600, SWSERIAL_8N1, 12, 14, false, 256);
	//serialBMS.enableIntTx(true);

	// Optional functionnalities of EspMQTTClient : 
	//client.enableDebuggingMessages(); 					// Enable debugging messages sent to serial output
	client.enableHTTPWebUpdater(); 						// Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overrited with enableHTTPWebUpdater("user", "password").
	client.enableLastWillMessage("trailer/batteries/lastwill", "Offline-failed");  // You can activate the retain flag by setting the third parameter to true
}

void loop() {
	
	byte 	inBuff[128];
	int		bytesIn;
	float	flValue;
	int		iCounter;
	int		iDataPointer;
	int		iDataLength;
	char	string_buffer[256];

	String	strFormatted;

	const int capacity_gauges = JSON_OBJECT_SIZE(NUM_GAUGES + 1);
  	StaticJsonDocument<capacity_gauges> gauges;
  	
	const int capacity_cells = JSON_OBJECT_SIZE(MAX_CELLS + 2);
  	StaticJsonDocument<capacity_cells> cells;
  	
	const int capacity_general = JSON_OBJECT_SIZE(NUM_GENERAL + 1);
  	StaticJsonDocument<capacity_general> general;
  	
	const int capacity_bmsinfo = JSON_OBJECT_SIZE(NUM_BMSINFO + 1);
  	StaticJsonDocument<256> bmsinfo;
  	
  	char json_output[256];

	// 0x04 return data, data_length/2 = number of cells
	float	flCellVolts[32];		// individual cell voltages
	int		iNumberOfCells;			// valid cells in array

	// 0x03 return data
	float			flTotalVoltage;
	float			flCurrent;
	float 			flResidualCapacity;
	float			flNominalCapacity;
	int				iCycleTimes;
	unsigned int	iDateManufactured;
	unsigned int	iBalancedStateLow;		// bit pattern, 1st 16 cells, 0 = off, 1 = on
	unsigned int	iBalancedStateHigh;		// bit pattern, 2nd 16 cells, 0 = off, 1 = on
	unsigned int	iProtectionState;		// bit pattern, 0 = unprotected, 1 = protected
	byte			iSoftwareVersion;		// 0x10 -> v1.0, 1 byte field
	byte			iRSOC;					// percentage of remaining capacity, 1 byte field
	byte			iMOSFETControlStatus;	// MOSFET status indicatore, bit0 indicates charging, bit 1 indicates discharging, 0 = closing, 1 = opening
	byte			iBatterySerialNumber;	// 4S?
	byte			iNTCQuantity;			// number of temp sensors
	int				iNTCContent;			// temp values
	float			flTempValues[4];		// space for 4 temperature values

	//Serial.println((short)-1.6);
	//Serial.println(MakeFloatFromBytes(0x00,0xff));
	//Serial.println(MakeFloatFromBytes(0x80,0xff));
	//Serial.println(MakeFloatFromBytes(0x80,0x01));
	
	// check for OTA
	AsyncElegantOTA.loop();

	Serial.println("Requesting 03 data");
    Request03();
 	bytesIn = checkSerial(&inBuff[0]);
 	//Serial.print("Bytes Rcvd = "); Serial.println(bytesIn);

 	if( inBuff[0] == 0xdd and inBuff[1] == 0x03 ){
 		// basic data
 		//
 		//	0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33
 		// DD 03 00 1B 05 33 00 00 26 FC 27 10 00 00 2A AE 00 00 00 00 00 00 21 64 03 04 02 0B 8C 0B 8F FB BD 77



 		iDataLength = (inBuff[2] * 256 + inBuff[3])/2;

 		flTotalVoltage = MakeFloatFromBytes(inBuff[4], inBuff[5])/100.;
 		flCurrent = MakeFloatFromBytes(inBuff[6], inBuff[7])/100.;
 		flResidualCapacity = MakeFloatFromBytes(inBuff[8], inBuff[9])/100.;
 		flNominalCapacity = MakeFloatFromBytes(inBuff[10], inBuff[11])/100.;

 		iCycleTimes = (inBuff[12] * 256 + inBuff[13]);
 		iDateManufactured = (inBuff[14] * 256 + inBuff[15]);

 		iBalancedStateLow = (inBuff[16] * 256 + inBuff[17]);
 		iBalancedStateHigh = (inBuff[18] * 256 + inBuff[19]);


 		iProtectionState = (inBuff[20] * 256 + inBuff[21]);

 		iSoftwareVersion = inBuff[22];
 		
 		iRSOC = inBuff[23];
 		
 		iMOSFETControlStatus = inBuff[24];
 		
 		iBatterySerialNumber = inBuff[25];
 		
 		iNTCQuantity = inBuff[26];

 		if(iNTCQuantity > 0)
 			flTempValues[0] = (MakeFloatFromBytes(inBuff[27], inBuff[28])-2731)/10.;
 		if(iNTCQuantity > 1)
 			flTempValues[1] = (MakeFloatFromBytes(inBuff[29], inBuff[30])-2731)/10.;

 		/*Serial.print("Total Voltage = "); Serial.println(flTotalVoltage, DEC);
 		Serial.print("Current = "); Serial.println(flCurrent, DEC);
 		Serial.print("Residual Capacity = "); Serial.println(flResidualCapacity, DEC);
 		Serial.print("Nominal Capacity = "); Serial.println(flNominalCapacity, DEC);
 		Serial.print("Cycle Times = "); Serial.println(iCycleTimes, DEC);
 		Serial.print("Date Manufactured = "); Serial.println(iDateManufactured, HEX);
 		Serial.print("Balanced State Low = "); Serial.println(iBalancedStateLow, BIN);
 		Serial.print("Balanced State High = "); Serial.println(iBalancedStateHigh, BIN);
 		Serial.print("Protection State = "); Serial.println(iProtectionState, BIN);
 		Serial.print("Software Version = "); Serial.println(iSoftwareVersion, HEX);
 		Serial.print("RSOC = "); Serial.println(iRSOC, HEX);
 		Serial.print("MOSFET Control Status = "); Serial.println(iMOSFETControlStatus, BIN);
 		Serial.print("Battery Serial Number = "); Serial.println(iBatterySerialNumber, HEX);
 		Serial.print("NTC Quantity = "); Serial.println(iNTCQuantity, DEC);
 		if(iNTCQuantity > 0)
 			Serial.print("Temperature 1 = "); Serial.println(flTempValues[0], DEC);
 		if(iNTCQuantity > 1)
 			Serial.print("Temperature 2 = "); Serial.println(flTempValues[1], DEC);*/

		/*JSONencoder["device"] = "trailer";
		JSONencoder["sensorType"] = "Temperature";
		JsonArray& values = JSONencoder.createNestedArray("values");
		
		values.add(20);
		values.add(21);
		values.add(23);
		
		char JSONmessageBuffer[100];
		JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
		Serial.println("Sending message to MQTT topic..");
		Serial.println(JSONmessageBuffer);
		
		if (client.publish("esp/test", JSONmessageBuffer) == true) {
				Serial.println("Success sending message");
			} else {
				Serial.println("Error sending message");
			}
		
		client.loop();*/

		if( client.isMqttConnected() ) {
#ifndef USE_JSON
	 		snprintf (msg, 50, "%.4f", flTotalVoltage);	client.publish("trailer/batteries/battery1/gauges/voltage", msg);
	 		snprintf (msg, 50, "%.4f", flCurrent);	client.publish("trailer/batteries/battery1/gauges/current", msg);
	 		snprintf (msg, 50, "%.4f", flResidualCapacity);	client.publish("trailer/batteries/battery1/gauges/residual_capacity", msg);
	 		snprintf (msg, 50, "%.4f", flNominalCapacity);	client.publish("trailer/batteries/battery1/gauges/nominal_capacity", msg);
#endif

			gauges["meter"] = METER_NAME;
			gauges["voltage"] = flTotalVoltage;
			gauges["current"] = flCurrent;
			gauges["residual_capacity"] = flResidualCapacity;
			gauges["nominal_capacity"] = flNominalCapacity;
			gauges["percent_capacity"] = flResidualCapacity / flNominalCapacity;
			
	 		if(iNTCQuantity > 0) {
#ifndef USE_JSON
		 		snprintf (msg, 50, "%.4f", flTempValues[0]);	client.publish("trailer/batteries/battery1/gauges/temperature01", msg);
#endif
				gauges["temperature01"] = flTempValues[0];
	 		}
	 		if(iNTCQuantity > 1) {
#ifndef USE_JSON
		 		snprintf (msg, 50, "%.4f", flTempValues[1]);	client.publish("trailer/batteries/battery1/gauges/temperature02", msg);
#endif
	 			gauges["temperature02"] = flTempValues[1];
	 		}
	
			serializeJson(gauges,json_output);

			//Serial.println(json_output);
			
			if (client.publish(GAUGES, json_output) == true) {
					//Serial.println("Success sending message");
				} else {
					//Serial.println("Error sending message");
				}

/* 			temperatures["celcius"] = flTempValues[0];
			temperatures["meter"] = "bms_01";
			serializeJson(temperatures,json_output);
			Serial.println(json_output);
			client.publish(TEMPERATURES, json_output);*/

#ifndef USE_JSON
	 		snprintf (msg, 50, "%f", iDateManufactured);	client.publish("trailer/batteries/battery1/general/date_manufactured", msg);
	 		snprintf (msg, 50, "%f", iCycleTimes);	client.publish("trailer/batteries/battery1/general/cycle_times", msg);
	 		snprintf (msg, 50, "%f", iBalancedStateLow);	client.publish("trailer/batteries/battery1/general/balanced_state_low", msg);
	 		snprintf (msg, 50, "%f", iBalancedStateHigh);	client.publish("trailer/batteries/battery1/general/balanced_state_high", msg);
	 		snprintf (msg, 50, "%f", iProtectionState);	client.publish("trailer/batteries/battery1/general/protection_state", msg);
	 		snprintf (msg, 50, "%f", iSoftwareVersion);	client.publish("trailer/batteries/battery1/general/software_version", msg);
	 		snprintf (msg, 50, "%f", iRSOC);	client.publish("trailer/batteries/battery1/general/rsoc", msg);
	 		snprintf (msg, 50, "%f", iMOSFETControlStatus);	client.publish("trailer/batteries/battery1/general/mosfet_control_status", msg);
	 		snprintf (msg, 50, "%f", iBatterySerialNumber);	client.publish("trailer/batteries/battery1/general/battery_serial_number", msg);
#endif

			general["meter"] = METER_NAME;
	 		general["date_manufactured"] = iDateManufactured;
	 		general["cycle_times"] = iCycleTimes;
	 		general["balanced_state_low"] = iBalancedStateLow;
	 		general["balanced_state_high"] = iBalancedStateHigh;
	 		general["protection_state"] = iProtectionState;
	 		general["software_version"] = iSoftwareVersion;
	 		general["rsoc"] = iRSOC;
	 		general["mosfet_control_status"] = iMOSFETControlStatus;
	 		general["battery_serial_number"] = iBatterySerialNumber;

			serializeJson(general,json_output);

			//Serial.println(json_output);
			
			if (client.publish(GENERAL, json_output) == true) {
					//Serial.println("Success sending message");
				} else {
					//Serial.println("Error sending message");
				}

		}
 	}
 	 	
	client.loop();
	
	Serial.println("Requesting 04 data");
    Request04();
 	bytesIn = checkSerial( &inBuff[0]);
 	//Serial.print("Bytes Rcvd = "); Serial.println(bytesIn);

 	if( inBuff[0] == 0xdd and inBuff[1] == 0x04 ){
 		// battery voltages
		//
 		//	0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33
		// DD 04 00 08 0C FA 0D 05 0D 03 0D 02 FE C1 77
		//
 		iNumberOfCells = (inBuff[2] * 256 + inBuff[3])/2;
 		//Serial.print( "Battery has "); Serial.print(iNumberOfCells,DEC); Serial.println(" cells");

		if( iNumberOfCells > 0 ){
			iDataPointer = 4;	// first cell data location in buffer
 			for( iCounter = 0; iCounter < iNumberOfCells; iCounter++ ){
 				flCellVolts[iCounter] = MakeFloatFromBytes(inBuff[iDataPointer], inBuff[iDataPointer+1])/1000.;
 				iDataPointer += 2;
 			}

			// if connected to MQTT broker then publish the info
			if( client.isMqttConnected() ) {
#ifndef USE_JSON
				iDataPointer = 4;	// first cell data location in buffer
		 		snprintf (msg, 50, "%f", iNumberOfCells);	client.publish("trailer/batteries/battery1/cells/quantity", msg);
	 			for( iCounter = 1; iCounter <= iNumberOfCells; iCounter++ ){
	 				sprintf(string_buffer, "trailer/batteries/battery1/cells/cell%02d", iCounter);
	 				snprintf (msg, 50, "%.4f", (MakeFloatFromBytes(inBuff[iDataPointer], inBuff[iDataPointer+1])/1000)); client.publish(string_buffer, msg);
	 				iDataPointer += 2;
	 			}
#endif

				cells["meter"] = METER_NAME;
				iDataPointer = 4;	// first cell data location in buffer
				cells["quantity"] = iNumberOfCells;
	 			for( iCounter = 1; iCounter <= iNumberOfCells; iCounter++ ){
	 				sprintf(string_buffer, "cell%02d", iCounter);
	 				cells[string_buffer] = MakeFloatFromBytes(inBuff[iDataPointer], inBuff[iDataPointer+1])/1000.;
	 				iDataPointer += 2;
	 			}
				serializeJson(cells,json_output);
	
				if (client.publish(CELLS, json_output) == true) {
						//Serial.println("Success sending message");
					} else {
						//Serial.println("Error sending message");
					}
			}
			
			
		}

 		/*for( iCounter = 0; iCounter < iNumberOfCells; iCounter++ ) {
 			Serial.print("Cell "); Serial.print(iCounter); Serial.print(" = "); Serial.println(flCellVolts[iCounter],DEC);
 		}*/
 	}

	client.loop();
	
	Serial.println("Requesting 05 data");
    Request05();
 	bytesIn = checkSerial( &inBuff[0]);
 	//Serial.print("Bytes Rcvd = "); Serial.println(bytesIn);

 	if( inBuff[0] == 0xdd and inBuff[1] == 0x05 ){
 		// hardware information
		//
 		//	0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33
		// DD 05 00 19 4A 42 44 2D 53 50 30 34 53 30 32 38 2D 4C 34 53 2D 31 35 30 41 2D 42 2D 55 FA 01 77
		//
		char	myBuffer[256];
		
		//String myString = (char*)inBuff[4];
		//Serial.println(myString.c_str());
		
		// if connected to MQTT broker then publish the info
		if( client.isMqttConnected() && 0 ) {
			bmsinfo["meter"] = METER_NAME;
			iDataPointer = 4;
			iCounter = inBuff[3];
			//Serial.print("Buffer contains "); Serial.println(iCounter);
			for( ; iCounter > 0; iCounter-- ) {
				myBuffer[iDataPointer - 4] = inBuff[iDataPointer];
				iDataPointer++;
			}
			myBuffer[ iDataPointer - 4 ] = 0;
			
#ifndef USE_JSON
	 		snprintf (msg, 50, "%s", myBuffer);	client.publish("trailer/batteries/battery1/bmsinfo/name", msg);
#endif
	 		
			bmsinfo["name"] = myBuffer;
			serializeJson(bmsinfo,json_output);

			if (client.publish(BMSINFO, json_output) == true) {
					//Serial.println("Success sending message");
				} else {
					//Serial.println("Error sending message");
				}
		}
		
		/*for( ; iCounter > 0; iCounter-- ) {
			Serial.print( char(inBuff[iDataPointer]) );
			iDataPointer++;
		}
		Serial.println();*/
	}

	client.loop();
	
	delay(5000);

}


//	convert two byte values to a float
//
//	b1 is hi byte, b2 is lo byte in 2 byte value field
//
float MakeFloatFromBytes(byte b1, byte b2) {
	//float flVal;

	if(b1 & 0x80)
	{
		//Serial.println("Negative Number");
		return (float)((~((short)b1 * 256 + (short)b2)) & 0x7fff) * -1.0;
	}
	else
		return (float)((short)b1 * 256 + (short)b2);
/*
	Serial.print("Input = "); Serial.print(b1, HEX);Serial.print(" - "); Serial.print(b2, HEX); Serial.print("  Mid = "); Serial.print( (b1 & 0x7f) << 8, HEX ); Serial.print( " - " ); Serial.println( b2, HEX );
	
	if( b1 & 0x80 ) {
		flVal = ((float)((byte)~b1 << 8 + (byte)b2)) * -1.0;
		Serial.println("Negative value");
	}
	else
		flVal = ((float)((byte)b1 << 8 + (byte)b2));

	Serial.print(" Result = "); Serial.println( flVal, DEC);

	return flVal;
	
	Serial.print("Input = "); Serial.print(b1, HEX);Serial.print(" - "); Serial.print(b2, HEX); Serial.print("  Mid = "); Serial.print( (b1 & 0x7f) << 8, HEX ); Serial.print( " - " ); Serial.print( b2, HEX );
	
	flVal = float( (short)(byte)(b1 & 0x7f) << 8 | (byte)b2 );

	if( b1 & 0x80 ) {
		//flVal -= 32768.0;
		flVal = flVal * -1.0;
	}
	
	Serial.print(" Result = "); Serial.println( flVal, DEC);
	
	
	return flVal;*/
}

//	send request for 0x03 data block
//
void Request03() {
	iCurrentRequest = 3;
    Serial2.write(0xdd);
    Serial2.write(0xa5);
    Serial2.write(0x03);
    Serial2.write(0x00);
    Serial2.write(0xff);
    Serial2.write(0xfd);
    Serial2.write(0x77);
}

//	send request for 0x04 data block
//
void Request04() {
	iCurrentRequest = 4;
    Serial2.write(0xdd);
    Serial2.write(0xa5);
    Serial2.write(0x04);
    Serial2.write(0x00);
    Serial2.write(0xff);
    Serial2.write(0xfc);
    Serial2.write(0x77);
}

//	send request for 0x05 data block
//
void Request05() {
	iCurrentRequest = 5;
    Serial2.write(0xdd);
    Serial2.write(0xa5);
    Serial2.write(0x05);
    Serial2.write(0x00);
    Serial2.write(0xff);
    Serial2.write(0xfb);
    Serial2.write(0x77);
}

int checkSerial( byte* buff ) {
	byte ch;
	int	count = 0;
	
	//while (!Serial2.available());
	//ss->enableTx(true);
	//while (Serial.available()) {
	//	ch = Serial.read();
	//	ss->write(ch);
	//}
	//ss->enableTx(false);
	// wait 1 second for the reply from SOftwareSerial if any
	delay(1000);
	if (Serial2.available()) {
		//Serial.print("\nResult:");
		while (Serial2.available()) {
			ch = (byte)Serial2.read();
			*buff++ = ch;
			count++;
			//Serial.print(ch < 0x01 ? " 0" : " ");
			//Serial.print(ch, HEX);
		}
		//Serial.println();
	}
	else {
		lSerialFailCount++;
		Serial.print("No response from request "); Serial.print( iCurrentRequest ); Serial.print(", Fail number "); Serial.println( lSerialFailCount );
	}

	return count;
}
