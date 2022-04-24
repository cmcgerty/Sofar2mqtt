// Update these to match your inverter/network.
#define INVERTER_ME3000				// Uncomment for ME3000
//#define INVERTER_HYBRID			// Uncomment for Hybrid

// The device name is used as the MQTT base topic. If you need more than one Sofar2mqtt on your network, give them unique names.
const char* deviceName = "Sofar2mqtt";
const char* version = "v2.0b1";

#define WIFI_SSID	"xxxxx"
#define WIFI_PASSWORD	"xxxxx"
#define MQTT_SERVER	"mqtt"
#define MQTT_PORT	1883
#define MQTT_USERNAME	"auser"			// Empty string for none.
#define MQTT_PASSWORD	"apassword"

/*****
Sofar2mqtt is a remote control interface for Sofar solar and battery inverters.
It allows remote control of the inverter and reports the invertor status, power usage, battery state etc for integration with smart home systems such as Home Assistant and Node-Red vi MQTT.  
For read only mode, it will send status messages without the inverter needing to be in passive mode.  
It's designed to run on an ESP8266 microcontroller with a TTL to RS485 module such as MAX485 or MAX3485.  
Designed to work with TTL modules with or without the DR and RE flow control pins. If your TTL module does not have these pins then just ignore the wire from D5. 

Subscribe your MQTT client to:

Sofar2mqtt/state

Which provides:

running_state  
grid_voltage  
grid_current  
grid_freq  
battery_power  
battery_voltage  
battery_current  
batterySOC  
battery_temp  
battery_cycles  
grid_power  
consumption  
solarPV  
today_generation  
today_exported  
today_purchase  
today_consumption  
inverter_temp  
inverterHS_temp  
solarPVAmps  

With the inverter in Passive Mode, send MQTT messages to:

Sofar2mqtt/set/standby   - send value "true"
Sofar2mqtt/set/auto   - send value "true" or "battery_save"
Sofar2mqtt/set/charge   - send values in the range 0-3000 (watts)
Sofar2mqtt/set/discharge   - send values in the range 0-3000 (watts)

Each of the above will return a response on:
Sofar2mqtt/response/<function>, the message containing the response from
the inverter, which has a result code in the lower byte and status in the upper byte.

The result code will be 0 for success, 1 means "Invalid Work Mode" ( which possibly means
the inverter isn't in passive mode, ) and 3 means "Inverter busy." 2 and 4 are data errors
which shouldn't happen unless there's a cable issue or some such.

The status bits in the upper byte indicate the following:
Bit 0 - Charge enabled
Bit 1 - Discharge enabled
Bit 2 - Battery full, charge prohibited
Bit 3 - Battery flat, discharge prohibited

For example, a publish to Sofar2mqtt/set/charge will result in one on Sofar2mqtt/response/charge.
AND the message with 0xff to get the result code, which should be 0.

battery_save is a hybrid auto mode that will charge from excess solar but not discharge.

There will also be messages published to Sofar2mqtt/response/<type> when things happen
in the background, such as setting auto mode on startup and switching modes in battery_save mode.

(c)Colin McGerty 2021 colin@mcgerty.co.uk
Major version 2.0 rewrite by Adam Hill sidepipeukatgmaildotcom
Thanks to Rich Platts for hybrid model code and testing.
calcCRC by angelo.compagnucci@gmail.com and jpmzometa@gmail.com
*****/
#if (! defined INVERTER_ME3000) && ! defined INVERTER_HYBRID
#error You must specify the inverter type.
#endif

#include <Arduino.h>

#define SOFAR_SLAVE_ID          0x01

#ifdef INVERTER_ME3000
#define	MAX_POWER		3000		// ME3000 is 3000W max.
#elif defined INVERTER_HYBRID
#define MAX_POWER		6000
#endif

#define RS485_TRIES 8       // x 50mS to wait for RS485 input chars.
// Wifi parameters. Fill in your wifi network name and password.
#include <ESP8266WiFi.h>
const char* wifiName = WIFI_SSID;
WiFiClient wifi;

// MQTT parameters
#include <PubSubClient.h>
const char* mqttClientID = deviceName;
PubSubClient mqtt(wifi);

// SoftwareSerial is used to create a second serial port, which will be deidcated to RS485.
// The built-in serial port remains available for flashing and debugging.
#include <SoftwareSerial.h>
#define SERIAL_COMMUNICATION_CONTROL_PIN D5 // Transmission set pin
#define RS485_TX HIGH
#define RS485_RX LOW
#define RXPin        D6  // Serial Receive pin
#define TXPin        D7  // Serial Transmit pin
SoftwareSerial RS485Serial(RXPin, TXPin);

// Sofar run states
#define waiting 0
#define check 1

#ifdef INVERTER_ME3000
#define charging		2
#define checkDischarge		3
#define discharging		4
#define epsState		5
#define faultState		6
#define permanentFaultState	7

#define HUMAN_CHARGING		"Charging"
#define HUMAN_DISCHARGING	"Discharge"

#elif defined INVERTER_HYBRID
#define normal			2
#define epsState		3
#define faultState		4
#define permanentFaultState	5
#define normal1			6

// State names are a bit strange - makes sense to also match to these?
#define charging		2
#define discharging		6

#define HUMAN_CHARGING		"Normal"
#define HUMAN_DISCHARGING	"Normal1"
#endif

#define MAX_FRAME_SIZE          64
#define MODBUS_FN_READSINGLEREG 0x03
#define SOFAR_FN_PASSIVEMODE    0x42
#define SOFAR_PARAM_STANDBY     0x5555

unsigned int INVERTER_RUNNINGSTATE;

// Battery Save mode is a hybrid mode where the battery will charge from excess solar but not discharge.
bool BATTERYSAVE = false;

// SoFar ME3000 Information Registers
#define SOFAR_REG_RUNSTATE	0x0200
#define SOFAR_REG_GRIDV		0x0206
#define SOFAR_REG_GRIDA		0x0207
#define SOFAR_REG_GRIDFREQ	0x020c
#define SOFAR_REG_BATTW		0x020d
#define SOFAR_REG_BATTV		0x020e
#define SOFAR_REG_BATTA		0x020f
#define SOFAR_REG_BATTSOC	0x0210
#define SOFAR_REG_BATTTEMP	0x0211
#define SOFAR_REG_GRIDW		0x0212
#define SOFAR_REG_LOADW		0x0213
#define SOFAR_REG_PVW		0x0215
#define SOFAR_REG_PVDAY		0x0218
#define SOFAR_REG_EXPDAY	0x0219
#define SOFAR_REG_IMPDAY	0x021a
#define SOFAR_REG_LOADDAY	0x021b
#define SOFAR_REG_BATTCYC	0x022c
#define SOFAR_REG_PVA		0x0236
#define SOFAR_REG_INTTEMP	0x0238
#define SOFAR_REG_HSTEMP	0x0239
#define SOFAR_REG_PV1		0x0252
#define SOFAR_REG_PV2		0x0255

#define SOFAR_FN_STANDBY	0x0100
#define SOFAR_FN_DISCHARGE	0x0101
#define SOFAR_FN_CHARGE		0x0102
#define SOFAR_FN_AUTO		0x0103

struct mqtt_status_register
{
	uint16_t regnum;
	String    mqtt_name;
};

static struct mqtt_status_register  mqtt_status_reads[] =
{
	{ SOFAR_REG_RUNSTATE, "running_state" },
	{ SOFAR_REG_GRIDV, "grid_voltage" },
	{ SOFAR_REG_GRIDA, "grid_current" },
	{ SOFAR_REG_GRIDFREQ, "grid_freq" },
	{ SOFAR_REG_GRIDW, "grid_power" },
	{ SOFAR_REG_BATTW, "battery_power" },
	{ SOFAR_REG_BATTV, "battery_voltage" },
	{ SOFAR_REG_BATTA, "battery_current" },
	{ SOFAR_REG_BATTSOC, "batterySOC" },
	{ SOFAR_REG_BATTTEMP, "battery_temp" },
	{ SOFAR_REG_BATTCYC, "battery_cycles" },
	{ SOFAR_REG_LOADW, "consumption" },
	{ SOFAR_REG_PVW, "solarPV" },
	{ SOFAR_REG_PVA, "solarPVAmps" },
	{ SOFAR_REG_PVDAY, "today_generation" },
#ifdef INVERTER_ME3000
	{ SOFAR_REG_EXPDAY, "today_exported" },
	{ SOFAR_REG_IMPDAY, "today_purchase" },
#elif defined INVERTER_HYBRID
	{ SOFAR_REG_PV1, "Solarpv1" },
	{ SOFAR_REG_PV2, "Solarpv2" },
#endif
	{ SOFAR_REG_LOADDAY, "today_consumption" },
	{ SOFAR_REG_INTTEMP, "inverter_temp" },
	{ SOFAR_REG_HSTEMP, "inverter_HStemp" },
};

// This is the return object for the sendModbus() function. Since we are a modbus master, we
// are primarily interested in the responses to our commands.
struct modbusResponse
{
	uint8_t errorLevel;
	uint8_t data[MAX_FRAME_SIZE];
	uint8_t dataSize;
	char* errorMessage;
};

// These timers are used in the main loop.
#define HEARTBEAT_INTERVAL 9000
#define RUNSTATE_INTERVAL 5000
#define SEND_INTERVAL 10000
#define BATTERYSAVE_INTERVAL 3000

// Wemos OLED Shield set up. 64x48, pins D1 and D2
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);

/**
 * Check to see if the elapsed interval has passed since the passed in
 * millis() value. If it has, return true and update the lastRun. Note
 * that millis() overflows after 50 days, so we need to deal with that
 * too... in our case we just zero the last run, which means the timer
 * could be shorter but it's not critical... not worth the extra effort
 * of doing it properly for once in 50 days.
 */
bool checkTimer(unsigned long *lastRun, unsigned long interval)
{
	unsigned long now = millis();

	if(*lastRun > now)
		*lastRun = 0;

	if(now >= *lastRun + interval)
	{
		*lastRun = now;
		return true;
	}

	return false;
}

// Update the OLED. Use "NULL" for no change or "" for an empty line.
String oledLine1;
String oledLine2;
String oledLine3;
String oledLine4;

void updateOLED(String line1, String line2, String line3, String line4)
{
	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(0,0);

	if(line1 != "NULL")
	{
		display.println(line1);
		oledLine1 = line1;
	}
	else
		display.println(oledLine1);

	display.setCursor(0,12);

	if(line2 != "NULL")
	{
		display.println(line2);
		oledLine2 = line2;
	}
	else
		display.println(oledLine2);

	display.setCursor(0,24);

	if(line3 != "NULL")
	{
		display.println(line3);
		oledLine3 = line3;
	}
	else
		display.println(oledLine3);

	display.setCursor(0,36);

	if(line4 != "NULL")
	{
		display.println(line4);
		oledLine4 = line4;
	}
	else
		display.println(oledLine4);

	display.display();
}

// Connect to WiFi
void setup_wifi()
{
	// We start by connecting to a WiFi network
	Serial.println();
	Serial.print("Connecting to ");
	Serial.println(wifiName);
	updateOLED("NULL", "NULL", "WiFi..", "NULL");
	WiFi.mode(WIFI_STA);
	WiFi.begin(wifiName, WIFI_PASSWORD);

	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print(".");
		updateOLED("NULL", "NULL", "WiFi...", "NULL");
	}

	WiFi.hostname(deviceName);
	Serial.println("");
	Serial.print("WiFi connected - ESP IP address: ");
	Serial.println(WiFi.localIP());
	updateOLED("NULL", "NULL", "WiFi....", "NULL");
}

int addStateInfo(String &state, uint16_t reg, String human)
{
	unsigned int	val;
	modbusResponse	rs;

	if(readSingleReg(SOFAR_SLAVE_ID, reg, &rs))
		return -1;

	val = ((rs.data[0] << 8) | rs.data[1]);

	if (!( state == "{"))
		state += ",";

	state += "\"" + human + "\":" + String(val);
	return 0;
}

void sendData()
{
	static unsigned long	lastRun = 0;

	// Update all parameters and send to MQTT.
	if(checkTimer(&lastRun, SEND_INTERVAL))
	{
		int	l;
		String	state = "{";

		for(l = 0; l < sizeof(mqtt_status_reads)/sizeof(struct mqtt_status_register); l++)
			addStateInfo(state, mqtt_status_reads[l].regnum, mqtt_status_reads[l].mqtt_name);

		state = state+"}";

		//Prefix the mqtt topic name with deviceName.
		String topic(deviceName);
		topic += "/state";
		sendMqtt(const_cast<char*>(topic.c_str()), state);
	}
}

// This function is executed when an MQTT message arrives on a topic that we are subscribed to.
void mqttCallback(String topic, byte *message, unsigned int length)
{
	if(!topic.startsWith(String(deviceName) + "/set/"))
		return;

	Serial.print("Message arrived on topic: ");
	Serial.print(topic);
	Serial.print(". Message: ");
	String messageTemp;
	uint16_t fnCode = 0, fnParam = 0;
	String cmd = topic.substring(topic.lastIndexOf("/") + 1);

	for(int i = 0; i < length; i++)
	{
		Serial.print((char)message[i]);
		messageTemp += (char)message[i];
	}

	Serial.println();
	int   messageValue = messageTemp.toInt();
	bool  messageBool = (messageTemp != "false");

	if(cmd == "standby")
	{
		if(messageBool)
		{
			fnCode = SOFAR_FN_STANDBY;
			fnParam = SOFAR_PARAM_STANDBY;
		}
	}
	else if(cmd == "auto")
	{
		if(messageBool)
			fnCode = SOFAR_FN_AUTO;
		else if(messageTemp == "battery_save")
			BATTERYSAVE = true;
	}
	else if((messageValue > 0) && (messageValue <= MAX_POWER))
	{
		fnParam = messageValue;

		if(cmd == "charge")
			fnCode = SOFAR_FN_CHARGE;
		else if(cmd == "discharge")
			fnCode = SOFAR_FN_DISCHARGE;
	}

	if(fnCode)
	{
		BATTERYSAVE = false;
		sendPassiveCmd(SOFAR_SLAVE_ID, fnCode, fnParam, cmd);
	}
}

void batterySave()
{
	static unsigned long	lastRun = 0;

	if(checkTimer(&lastRun, BATTERYSAVE_INTERVAL) && BATTERYSAVE)
	{
		modbusResponse  rs;

		//Get grid power
		unsigned int	p = 0;

		if(!readSingleReg(SOFAR_SLAVE_ID, SOFAR_REG_GRIDW, &rs))
			p = ((rs.data[0] << 8) | rs.data[1]);
		else
			Serial.println("modbus error");

		Serial.print("Grid power: ");
		Serial.println(p);
		Serial.print("Battery save mode: ");

		// Switch to auto when any power flows to the grid.
		// We leave a little wriggle room because once you start charging the battery,
		// gridPower should be floating just above or below zero.
		if(p < 65535/2 || p > 65525)
		{
			//exporting to the grid
			if(!sendPassiveCmd(SOFAR_SLAVE_ID, SOFAR_FN_AUTO, 0, "bsave_auto"))
				Serial.println("auto");
		}
		else
		{
			//importing from the grid
			if(!sendPassiveCmd(SOFAR_SLAVE_ID, SOFAR_FN_STANDBY, SOFAR_PARAM_STANDBY, "bsave_standby"))
				Serial.println("standby");
		}
	}
}

// This function reconnects the ESP8266 to the MQTT broker
void mqttReconnect() 
{
	// Loop until we're reconnected
	while(true)
	{
		mqtt.disconnect();		// Just in case.
		delay(200);
		Serial.print("Attempting MQTT connection...");
		updateOLED("NULL", "connecting", "NULL", "MQTT.");
		delay(500);
		updateOLED("NULL", "NULL", "NULL", "MQTT..");

		// Attempt to connect
		if(mqtt.connect(mqttClientID, MQTT_USERNAME, MQTT_PASSWORD))
		{
			Serial.println("connected");
			delay(1000);
			updateOLED("NULL", "NULL", "NULL", "MQTT....");
			delay(1000);

			//Set topic names to include the deviceName.
			String standbyMode(deviceName);
			standbyMode += "/set/standby";
			String autoMode(deviceName);
			autoMode += "/set/auto";
			String chargeMode(deviceName);
			chargeMode += "/set/charge";
			String dischargeMode(deviceName);
			dischargeMode += "/set/discharge";

			// Subscribe or resubscribe to topics.
			if(
				mqtt.subscribe(const_cast<char*>(standbyMode.c_str())) &&
				mqtt.subscribe(const_cast<char*>(autoMode.c_str())) &&
				mqtt.subscribe(const_cast<char*>(chargeMode.c_str())) &&
				mqtt.subscribe(const_cast<char*>(dischargeMode.c_str())))
			{
				updateOLED("NULL", "NULL", "NULL", "");
				break;
			}
		}

		Serial.print("failed, rc=");
		Serial.print(mqtt.state());
		Serial.println(" try again in 5 seconds");
		updateOLED("NULL", "NULL", "NULL", "MQTT...");

		// Wait 5 seconds before retrying
		delay(5000);
	}
}

/**
 * Flush the RS485 buffers in both directions. The doc for Serial.flush() implies it only
 * flushes outbound characters now... I assume RS485Serial is the same.
 */
void flushRS485()
{
	RS485Serial.flush();
	delay(200);

	while(RS485Serial.available())
		RS485Serial.read();
}

int sendModbus(uint8_t frame[], byte frameSize, modbusResponse *resp)
{
	//Calculate the CRC and overwrite the last two bytes.
	calcCRC(frame, frameSize);

	// Make sure there are no spurious characters in the in/out buffer.
	flushRS485();

	//Send
	digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_TX);
	RS485Serial.write(frame, frameSize);

	// It's important to reset the SERIAL_COMMUNICATION_CONTROL_PIN as soon as
	// we finish sending so that the serial port can start to buffer the response.
	digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX);
	return listen(resp);
}

// Listen for a response.
int listen(modbusResponse *resp)
{
	uint8_t		inFrame[64];
	uint8_t		inByteNum = 0;
	uint8_t		inFrameSize = 0;
	uint8_t		inFunctionCode = 0;
	uint8_t		inDataBytes = 0;
	int		done = 0;
	modbusResponse	dummy;

	if(!resp)
		resp = &dummy;      // Just in case we ever want to interpret here.

	resp->dataSize = 0;
	resp->errorLevel = 0;

	while((!done) && (inByteNum < sizeof(inFrame)))
	{
		int tries = 0;

		while((!RS485Serial.available()) && (tries++ < RS485_TRIES))
			delay(50);

		if(tries >= RS485_TRIES)
		{
			Serial.println("Timeout waiting for RS485 response.");
			break;
		}

		inFrame[inByteNum] = RS485Serial.read();

		//Process the byte
		switch(inByteNum)
		{
			case 0:
				if(inFrame[inByteNum] != SOFAR_SLAVE_ID)   //If we're looking for the first byte but it dosn't match the slave ID, we're just going to drop it.
					inByteNum--;          // Will be incremented again at the end of the loop.
				break;

			case 1:
				//This is the second byte in a frame, where the function code lives.
				inFunctionCode = inFrame[inByteNum];
				break;

			case 2:
				//This is the third byte in a frame, which tells us the number of data bytes to follow.
				if((inDataBytes = inFrame[inByteNum]) > sizeof(inFrame))
				inByteNum = -1;       // Frame is too big?
				break;

			default:
				if(inByteNum < inDataBytes + 3)
				{
					//This is presumed to be a data byte.
					resp->data[inByteNum - 3] = inFrame[inByteNum];
					resp->dataSize++;
				}
				else if(inByteNum > inDataBytes + 3)
					done = 1;
		}

		inByteNum++;
	}

	inFrameSize = inByteNum;

	/**
	* Now check to see if the last two bytes are a valid CRC.
	* If we don't have a response pointer we don't care.
	**/
	if(inFrameSize < 5)
	{
		resp->errorLevel = 2;
		resp->errorMessage = "Response too short";
	}
	else if(checkCRC(inFrame, inFrameSize))
	{
		resp->errorLevel = 0;
		resp->errorMessage = "Valid data frame";
	}
	else
	{
		resp->errorLevel = 1;
		resp->errorMessage = "Error: invalid data frame";
	}

	if(resp->errorLevel)
		Serial.println(resp->errorMessage);

	return -resp->errorLevel;
}

int readSingleReg(uint8_t id, uint16_t reg, modbusResponse *rs)
{
	uint8_t	frame[] = { id, MODBUS_FN_READSINGLEREG, reg >> 8, reg & 0xff, 0, 0x01, 0, 0 };

	return sendModbus(frame, sizeof(frame), rs);
}

int sendPassiveCmd(uint8_t id, uint16_t cmd, uint16_t param, String pubTopic)
{
	modbusResponse	rs;
	uint8_t	frame[] = { id, SOFAR_FN_PASSIVEMODE, cmd >> 8, cmd & 0xff, param >> 8, param & 0xff, 0, 0 };
	int		err = -1;
	String		retMsg;

	if(sendModbus(frame, sizeof(frame), &rs))
		retMsg = rs.errorMessage;
	else if(rs.dataSize != 2)
		retMsg = "Reponse is " + String(rs.dataSize) + " bytes?";
	else
	{
		retMsg = String((rs.data[0] << 8) | (rs.data[1] & 0xff));
		err = 0;
	}

	String topic(deviceName);
	topic += "/response/" + pubTopic;
	sendMqtt(const_cast<char*>(topic.c_str()), retMsg);
	return err;
}

void sendMqtt(char* topic, String msg_str)
{
	char	msg[1000];

	mqtt.setBufferSize(512);
	msg_str.toCharArray(msg, msg_str.length() + 1); //packaging up the data to publish to mqtt

	if (!(mqtt.publish(topic, msg)))
		Serial.println("MQTT publish failed");
}

void heartbeat()
{
	static unsigned long  lastRun = 0;

	//Send a heartbeat
	if(checkTimer(&lastRun, HEARTBEAT_INTERVAL))
	{
		uint8_t	sendHeartbeat[] = {SOFAR_SLAVE_ID, 0x49, 0x22, 0x01, 0x22, 0x02, 0x00, 0x00};
		int	ret;

		Serial.println("Send heartbeat");

		// This just makes the dot on the first line of the OLED screen flash on and off with
		// the heartbeat and clears any previous RS485 error massage that might still be there.
		if(!(ret = sendModbus(sendHeartbeat, sizeof(sendHeartbeat), NULL)))
		{
			String flashDot;

			if (oledLine2 == "Online")
				flashDot = "Online.";

			if (oledLine2 == "Online.")
				flashDot = "Online";

			if (oledLine3 == "RS485")
				oledLine3 = "";

			if (oledLine4 == "ERROR")
				oledLine4 = "";

			updateOLED("NULL", flashDot, "NULL", "NULL");
		}
		else
		{
			Serial.print("Bad heartbeat ");
			Serial.println(ret);
			updateOLED("NULL", "NULL", "RS485", "ERROR");
		}

		//Flash the LED
		digitalWrite(LED_BUILTIN, LOW);
		delay(4);
		digitalWrite(LED_BUILTIN, HIGH);
	}
}

void updateRunstate()
{
	static unsigned long	lastRun = 0;

	//Check the runstate
	if(checkTimer(&lastRun, RUNSTATE_INTERVAL))
	{
		modbusResponse  response;

		Serial.print("Get runstate: ");

		if(!readSingleReg(SOFAR_SLAVE_ID, SOFAR_REG_RUNSTATE, &response))
		{
			INVERTER_RUNNINGSTATE = ((response.data[0] << 8) | response.data[1]);
			Serial.println(INVERTER_RUNNINGSTATE);

			switch(INVERTER_RUNNINGSTATE)
			{
				case waiting:
					if (BATTERYSAVE)
						updateOLED("NULL", "NULL", "Batt Save", "Waiting");
					else
						updateOLED("NULL", "NULL", "Standby", "");
					break;

				case check:
					updateOLED("NULL", "NULL", "Checking", "NULL");
					break;

				case charging:
					updateOLED("NULL", "NULL", HUMAN_CHARGING, String(batteryWatts())+"W");
					break;

#ifdef INVERTER_ME3000
				case checkDischarge:
					updateOLED("NULL", "NULL", "Check Dis", "NULL");
					break;
#endif
				case discharging:
					updateOLED("NULL", "NULL", HUMAN_DISCHARGING, String(batteryWatts())+"W");
					break;

				case epsState:
					updateOLED("NULL", "NULL", "EPS State", "NULL");
					break;

				case faultState:
					updateOLED("NULL", "NULL", "FAULT", "NULL");
					break;

				case permanentFaultState:
					updateOLED("NULL", "NULL", "PERMFAULT", "NULL");
					break;

				default:
					updateOLED("NULL", "NULL", "Runstate?", "NULL");
					break;
			}
		}
		else
		{
			Serial.println(response.errorMessage);
			updateOLED("NULL", "NULL", "CRC-FAULT", "NULL");
		}
	}
}

unsigned int batteryWatts()
{ 
	if(INVERTER_RUNNINGSTATE == charging || INVERTER_RUNNINGSTATE == discharging)
	{
		modbusResponse  response;

		if(!readSingleReg(SOFAR_SLAVE_ID, SOFAR_REG_BATTW, &response))
		{
			unsigned int w = ((response.data[0] << 8) | response.data[1]);

			switch(INVERTER_RUNNINGSTATE)
			{
				case charging:
					w = w*10;
					break;

				case discharging:
					w = (65535 - w)*10;
			}

			return w;
		}
		else
		{
			Serial.println(response.errorMessage);
			updateOLED("NULL", "NULL", "CRC-FAULT", "NULL");
		}
	}

	return 0;
}

void setup()
{
	Serial.begin(9600);
	pinMode(LED_BUILTIN, OUTPUT);

	pinMode(SERIAL_COMMUNICATION_CONTROL_PIN, OUTPUT);
	digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX);
	RS485Serial.begin(9600);

	delay(500);

	//Turn on the OLED
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize OLED with the I2C addr 0x3C (for the 64x48)
	display.clearDisplay();
	display.display();
	updateOLED(deviceName, "connecting", "", version);

	setup_wifi();

	mqtt.setServer(MQTT_SERVER, MQTT_PORT);
	mqtt.setCallback(mqttCallback);

	//Wake up the inverter and put it in auto mode to begin with.
	heartbeat();
	mqttReconnect();
	Serial.println("Set start up mode: Auto");
	sendPassiveCmd(SOFAR_SLAVE_ID, SOFAR_FN_AUTO, 0, "startup_auto");
}

void loop()
{
	//make sure mqtt is still connected
	if((!mqtt.connected()) || !mqtt.loop())
	{
		updateOLED("NULL", "Offline", "NULL", "NULL");
		mqttReconnect();
	}
	else
		updateOLED("NULL", "Online", "NULL", "NULL");

	//Send a heartbeat to keep the inverter awake
	heartbeat();

	//Check and display the runstate
	updateRunstate();

	//Transmit all data to MQTT
	sendData();

	//Set battery save state
	batterySave();

	delay(100);
}

//calcCRC and checkCRC are based on...
//https://github.com/angeloc/simplemodbusng/blob/master/SimpleModbusMaster/SimpleModbusMaster.cpp

void calcCRC(uint8_t frame[], byte frameSize) 
{
	unsigned int temp = 0xffff, flag;

	for(unsigned char i = 0; i < frameSize - 2; i++)
	{
		temp = temp ^ frame[i];

		for(unsigned char j = 1; j <= 8; j++)
		{
			flag = temp & 0x0001;
			temp >>= 1;

			if(flag)
				temp ^= 0xA001;
		}
	}

	// Bytes are reversed.
	frame[frameSize - 2] = temp & 0xff;
	frame[frameSize - 1] = temp >> 8;
}

bool checkCRC(uint8_t frame[], byte frameSize) 
{
	unsigned int calculated_crc, received_crc;

	received_crc = ((frame[frameSize-2] << 8) | frame[frameSize-1]);
	calcCRC(frame, frameSize);
	calculated_crc = ((frame[frameSize-2] << 8) | frame[frameSize-1]);
	return (received_crc = calculated_crc);
}
