// The device name is used as the MQTT base topic. If you need more than one Sofar2mqtt on your network, give them unique names.
const char* version = "v3.3-alpha12";

bool tftModel = true; //true means 2.8" color tft, false for oled version

bool calculated = true; //default to pre-calculated values before sending to mqtt

unsigned int screenDimTimer = 30; //dim screen after 30 secs
unsigned long lastScreenTouch = 0;


#include <DoubleResetDetect.h>
#define DRD_TIMEOUT 0.1
#define DRD_ADDRESS 0x00
DoubleResetDetect drd(DRD_TIMEOUT, DRD_ADDRESS);


#include <WiFiManager.h>
#include <EEPROM.h>
#define PORTAL_TIMEOUT 300 //reboots device if hotspot isn't configured after this time
#define WIFI_TIMEOUT 60 //try this long to connect to existing wifi before going to hotspot portal mode

// * To be filled with EEPROM data
char deviceName[64] = "Sofar";
char MQTT_HOST[64] = "";
char MQTT_PORT[6]  = "1883";
char MQTT_USER[32] = "";
char MQTT_PASS[32] = "";
#define MQTTRECONNECTTIMER 30000 //it takes 30 secs for each mqtt server reconnect attempt
unsigned long lastMqttReconnectAttempt = 0;



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
  systemIO_power (AC side of inverter)
  battery_power  (DC side of inverter)
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
#include <Arduino.h>

#define SOFAR_SLAVE_ID          0x01

#define MAX_POWER		3000 //maybe change in further models

#define RS485_TRIES 8       // x 50mS to wait for RS485 input chars.
// Wifi parameters.
#include <ESP8266WiFi.h>
WiFiClient wifi;

#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

char jsonstring[1000];

// MQTT parameters
#include <PubSubClient.h>
PubSubClient mqtt(wifi);

// SoftwareSerial is used to create a second serial port, which will be deidcated to RS485.
// The built-in serial port remains available for flashing and debugging.
#include <SoftwareSerial.h>
//for OLED version default to original sofar2mqtt ports
#define SERIAL_COMMUNICATION_CONTROL_PIN D5 // Transmission set pin OLED version
#define RS485_TX HIGH
#define RS485_RX LOW
#define OLEDRXPin    D6  // Serial Receive pin OLED version
#define OLEDTXPin    D7  // Serial Transmit pin OLED version
SoftwareSerial RS485Serial(OLEDRXPin, OLEDTXPin);

//for TFT verion we use the hardware serial (pin 3 and 1)
#define RXPin        3  // Serial Receive pin
#define TXPin        1  // Serial Transmit pin



unsigned int INVERTER_RUNNINGSTATE;

#define MAX_FRAME_SIZE          224
#define MODBUS_FN_READHOLDINGREG 0x03
#define MODBUS_FN_READINPUTREG 0x04
#define MODBUS_FN_WRITEMULREG 0x10
#define SOFAR_FN_PASSIVEMODE    0x42
#define SOFAR_PARAM_STANDBY     0x5555

// Battery Save mode is a hybrid mode where the battery will charge from excess solar but not discharge.
bool BATTERYSAVE = false;
// Peakshaving will allow the inverter to charge or discharge the battery depending on a limited export or import power value, everything above will be go to or from battery
bool PEAKSHAVING = false;

// This is the return object for the sendModbus() function. Since we are a modbus master, we
// are primarily interested in the responses to our commands.
struct modbusResponse
{
  uint8_t errorLevel;
  uint8_t data[MAX_FRAME_SIZE];
  uint8_t dataSize;
  char* errorMessage;
};

bool modbusError = true;

// SoFar Information Registers
#define ME3000_START 0x0200
#define ME3000_END 0x0239
#define HYBRID_START 0x0200
#define HYBRID_END 0x0255
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
#define SOFAR_REG_SYSIOW	0x0214
#define SOFAR_REG_PVW		0x0215
#define SOFAR_REG_PVDAY		0x0218
#define SOFAR_REG_EXPDAY	0x0219
#define SOFAR_REG_IMPDAY	0x021a
#define SOFAR_REG_LOADDAY	0x021b
#define SOFAR_REG_PVTOTAL   0x021c
#define SOFAR_REG_EXPTOTAL  0x021e
#define SOFAR_REG_IMPTOTAL  0x0220
#define SOFAR_REG_LOADTOTAL 0x0222
#define SOFAR_REG_CHARGDAY  0x0224
#define SOFAR_REG_DISCHDAY  0x0225
#define SOFAR_REG_CHARGTOTAL  0x0226
#define SOFAR_REG_DISCHTOTAL  0x0228
#define SOFAR_REG_BATTCYC	0x022c
#define SOFAR_REG_PVA		0x0236
#define SOFAR_REG_INTTEMP	0x0238
#define SOFAR_REG_HSTEMP	0x0239
#define SOFAR_REG_PV1		0x0252
#define SOFAR_REG_PV2		0x0255

#define SOFAR_ANTIREFLUX_CONTROL 0x1242
#define SOFAR_ANTIREFLUX_POWER 0x1243

#define SOFAR_FN_STANDBY	0x0100
#define SOFAR_FN_DISCHARGE	0x0101
#define SOFAR_FN_CHARGE		0x0102
#define SOFAR_FN_AUTO		0x0103

//for HYD-EP and HYD-KTL
//system (0x0400-0x047F)
#define SOFAR2_SYSTEM_BEGIN 0x0404
#define SOFAR2_SYSTEM_END 0x041A
#define SOFAR2_REG_RUNSTATE 0x0404
#define SOFAR2_REG_INTTEMP 0x0418
#define SOFAR2_REG_HSTEMP  0x041A
//on grid output (0x0480-0x04FF)
#define SOFAR2_GRID_BEGIN 0x0484
#define SOFAR2_GRID_END 0x04AF
#define SOFAR2_REG_GRIDFREQ  0x0484
#define SOFAR2_REG_ACTW   0x0485
#define SOFAR2_REG_EXPW  0x0488
#define SOFAR2_REG_GRIDV   0x048D
#define SOFAR2_REG_LOADW   0x04AF
//PV INPUT (0x0580-0x05FF)
#define SOFAR2_PV_BEGIN 0x0584
#define SOFAR2_PV_END 0x0589
#define SOFAR2_REG_VPV1   0x0584
#define SOFAR2_REG_APV1   0x0585
#define SOFAR2_REG_PV1   0x0586
#define SOFAR2_REG_VPV2   0x0587
#define SOFAR2_REG_APV2   0x0588
#define SOFAR2_REG_PV2   0x0589
//seperate
#define SOFAR2_REG_PVW   0x05C4
//Battery input (0x0600-0x067F)
#define SOFAR2_BAT_BEGIN 0x0604
#define SOFAR2_BAT_END 0x060A
#define SOFAR2_REG_BATTV   0x0604
#define SOFAR2_REG_BATTA   0x0605
#define SOFAR2_REG_BATTW   0x0606
#define SOFAR2_REG_BATTTEMP  0x0607
#define SOFAR2_REG_BATTSOC 0x0608
#define SOFAR2_REG_BATTCYC 0x060A
//Electric  Power (0x0680-0x06BF)
#define SOFAR2_POW_BEGIN 0x0684
#define SOFAR2_POW_END 0x069B //one more because of 32bit value stored
#define SOFAR2_REG_PVDAY   0x0684
#define SOFAR2_REG_PVTOTAL   0x0686
#define SOFAR2_REG_LOADDAY 0x0688
#define SOFAR2_REG_LOADTOTAL 0x068A
#define SOFAR2_REG_IMPDAY  0x068C
#define SOFAR2_REG_IMPTOTAL  0x068E
#define SOFAR2_REG_EXPDAY  0x0690
#define SOFAR2_REG_EXPTOTAL  0x0692
#define SOFAR2_REG_CHARGDAY  0x0694
#define SOFAR2_REG_CHARGTOTAL  0x0696
#define SOFAR2_REG_DISCHDAY  0x0698
#define SOFAR2_REG_DISCHTOTAL  0x069A
//end

#define SOFAR2_ANTIREFLUX_CONTROL 0x1023
#define SOFAR2_ANTIREFLUX_POWER 0x1024

#define SOFAR2_REG_STORAGEMODE  0x4368
#define SOFAR2_REG_PASSIVECONTROL 0x1187 //in decimal 4487-4492, write 3x 32BIT values with first 32BIT = 0x0000 and next two are same (actually low=high limit) for the value of passive control

enum calculatorT {NOCALC, DIV10, DIV100, MUL10, MUL100, COMBINE};
enum valueTypeT {U16, S16, U32, S32, FLOAT};
enum inverterModelT {ME3000, HYBRID, HYDV2};
inverterModelT inverterModel = ME3000; //default to ME3000

bool separateMqttTopics = false;

struct mqtt_status_register
{
  inverterModelT inverter;
  uint16_t regnum;
  String    mqtt_name;
  valueTypeT valueType;
  calculatorT calculator;
};

static struct mqtt_status_register  mqtt_status_reads[] =
{
  { ME3000, SOFAR_REG_RUNSTATE, "running_state", U16, NOCALC},
  { ME3000, SOFAR_REG_GRIDV, "grid_voltage", U16, DIV10},
  { ME3000, SOFAR_REG_GRIDA, "grid_current", S16, DIV100},
  { ME3000, SOFAR_REG_GRIDFREQ, "grid_freq", U16, DIV100 },
  { ME3000, SOFAR_REG_GRIDW, "grid_power", S16, MUL10 },
  { ME3000, SOFAR_REG_BATTW, "battery_power", S16, MUL10 },
  { ME3000, SOFAR_REG_BATTV, "battery_voltage", U16, DIV10 },
  { ME3000, SOFAR_REG_BATTA, "battery_current", S16, DIV100 },
  { ME3000, SOFAR_REG_SYSIOW, "inverter_power", S16, MUL10 },
  { ME3000, SOFAR_REG_BATTSOC, "batterySOC", U16, NOCALC },
  { ME3000, SOFAR_REG_BATTTEMP, "battery_temp", S16, NOCALC },
  { ME3000, SOFAR_REG_BATTCYC, "battery_cycles", U16, NOCALC },
  { ME3000, SOFAR_REG_LOADW, "consumption", S16, MUL10},
  { ME3000, SOFAR_REG_PVW, "solarPV", U16, MUL10 },
  { ME3000, SOFAR_REG_PVA, "solarPVAmps", U16, NOCALC },
  { ME3000, SOFAR_REG_EXPDAY, "today_exported", U16, DIV100 },
  { ME3000, SOFAR_REG_IMPDAY, "today_purchase", U16, DIV100 },
  { ME3000, SOFAR_REG_PVDAY, "today_generation", U16, DIV100 },
  { ME3000, SOFAR_REG_LOADDAY, "today_consumption", U16, DIV100 },
  { ME3000, SOFAR_REG_EXPTOTAL, "total_exported", U32, COMBINE },
  { ME3000, SOFAR_REG_IMPTOTAL, "total_purchase", U32, COMBINE },
  { ME3000, SOFAR_REG_PVTOTAL, "total_generation", U32, COMBINE },
  { ME3000, SOFAR_REG_LOADTOTAL, "total_consumption", U32, COMBINE },
  { ME3000, SOFAR_REG_CHARGDAY, "today_charged", U16, DIV100 },
  { ME3000, SOFAR_REG_DISCHDAY, "today_discharged", U16, DIV100 },
  { ME3000, SOFAR_REG_CHARGTOTAL, "total_charged", U32, COMBINE },
  { ME3000, SOFAR_REG_DISCHTOTAL, "total_discharged", U32, COMBINE },
  { ME3000, SOFAR_REG_INTTEMP, "inverter_temp", S16, NOCALC },
  { ME3000, SOFAR_REG_HSTEMP, "inverter_HStemp", S16, NOCALC },
  { HYBRID, SOFAR_REG_RUNSTATE, "running_state", U16, NOCALC },
  { HYBRID, SOFAR_REG_GRIDV, "grid_voltage", U16, DIV10 },
  { HYBRID, SOFAR_REG_GRIDA, "grid_current", S16, DIV100 },
  { HYBRID, SOFAR_REG_GRIDFREQ, "grid_freq", U16, DIV100 },
  { HYBRID, SOFAR_REG_GRIDW, "grid_power", S16, MUL10 },
  { HYBRID, SOFAR_REG_BATTW, "battery_power", S16, MUL10 },
  { HYBRID, SOFAR_REG_BATTV, "battery_voltage", U16, DIV10 },
  { HYBRID, SOFAR_REG_BATTA, "battery_current", S16, DIV100 },
  { HYBRID, SOFAR_REG_SYSIOW, "inverter_power", S16, MUL10 },
  { HYBRID, SOFAR_REG_BATTSOC, "batterySOC", U16, NOCALC },
  { HYBRID, SOFAR_REG_BATTTEMP, "battery_temp", S16, NOCALC },
  { HYBRID, SOFAR_REG_BATTCYC, "battery_cycles", U16, NOCALC },
  { HYBRID, SOFAR_REG_LOADW, "consumption", S16, MUL10 },
  { HYBRID, SOFAR_REG_PVW, "solarPV", U16, MUL10 },
  { HYBRID, SOFAR_REG_PVA, "solarPVAmps", U16, NOCALC },
  { HYBRID, SOFAR_REG_PV1, "solarPV1", U16, MUL10 },
  { HYBRID, SOFAR_REG_PV2, "solarPV2", U16, MUL10 },
  { HYBRID, SOFAR_REG_PVDAY, "today_generation", U16, DIV100 },
  { HYBRID, SOFAR_REG_LOADDAY, "today_consumption", U16, DIV100 },
  { HYBRID, SOFAR_REG_EXPDAY, "today_exported", U16, DIV100 },
  { HYBRID, SOFAR_REG_IMPDAY, "today_purchase", U16, DIV100 },
  { HYBRID, SOFAR_REG_CHARGDAY, "today_charged", U16, DIV100 },
  { HYBRID, SOFAR_REG_DISCHDAY, "today_discharged", U16, DIV100 },
  { HYBRID, SOFAR_REG_PVTOTAL, "total_generation", U32, COMBINE },
  { HYBRID, SOFAR_REG_LOADTOTAL, "total_consumption", U32, COMBINE },
  { HYBRID, SOFAR_REG_EXPTOTAL, "total_exported", U32, COMBINE },
  { HYBRID, SOFAR_REG_IMPTOTAL, "total_purchase", U32, COMBINE },
  { HYBRID, SOFAR_REG_CHARGTOTAL, "total_charged", U32, COMBINE },
  { HYBRID, SOFAR_REG_DISCHTOTAL, "total_discharged", U32, COMBINE },
  { HYBRID, SOFAR_REG_INTTEMP, "inverter_temp", S16, NOCALC },
  { HYBRID, SOFAR_REG_HSTEMP, "inverter_HStemp", S16, NOCALC },
  { HYDV2, SOFAR2_REG_RUNSTATE, "running_state", U16, NOCALC },
  { HYDV2, SOFAR2_REG_INTTEMP, "inverter_temp", S16, NOCALC },
  { HYDV2, SOFAR2_REG_HSTEMP, "inverter_HStemp", S16, NOCALC},
  { HYDV2, SOFAR2_REG_GRIDFREQ, "grid_freq", U16, DIV100 },
  { HYDV2, SOFAR2_REG_ACTW, "inverter_power", S16, MUL10 },
  { HYDV2, SOFAR2_REG_EXPW, "grid_power", S16, MUL10},
  { HYDV2, SOFAR2_REG_GRIDV, "grid_voltage", U16, DIV10 },
  { HYDV2, SOFAR2_REG_LOADW, "consumption", S16, MUL10 },
  { HYDV2, SOFAR2_REG_VPV1, "solarPV1Volt", U16, DIV10},
  { HYDV2, SOFAR2_REG_APV1, "solarPV1Current", U16, DIV100},
  { HYDV2, SOFAR2_REG_PV1, "solarPV1", U16, MUL10},
  { HYDV2, SOFAR2_REG_VPV2, "solarPV2Volt", U16, DIV10},
  { HYDV2, SOFAR2_REG_APV2, "solarPV2Current", U16, DIV100},
  { HYDV2, SOFAR2_REG_PV2, "solarPV2", U16, MUL10},
  { HYDV2, SOFAR2_REG_PVW, "solarPV", U16, MUL100 },
  { HYDV2, SOFAR2_REG_BATTV, "battery_voltage", U16, DIV10 },
  { HYDV2, SOFAR2_REG_BATTA, "battery_current", S16, DIV100 },
  { HYDV2, SOFAR2_REG_BATTW, "battery_power", S16, MUL10 },
  { HYDV2, SOFAR2_REG_BATTTEMP, "battery_temp", S16, NOCALC },
  { HYDV2, SOFAR2_REG_BATTSOC, "batterySOC", U16, NOCALC },
  { HYDV2, SOFAR2_REG_BATTCYC, "battery_cycles", U16, NOCALC },
  { HYDV2, SOFAR2_REG_PVDAY, "today_generation", U32, DIV100 },
  { HYDV2, SOFAR2_REG_PVTOTAL, "total_generation", U32, DIV10 },
  { HYDV2, SOFAR2_REG_LOADDAY, "today_consumption", U32, DIV100 },
  { HYDV2, SOFAR2_REG_LOADTOTAL, "total_consumption", U32, DIV10 },
  { HYDV2, SOFAR2_REG_IMPDAY, "today_purchase", U32, DIV100 },
  { HYDV2, SOFAR2_REG_IMPTOTAL, "total_purchase", U32, DIV10 },
  { HYDV2, SOFAR2_REG_EXPDAY, "today_exported", U32, DIV100 },
  { HYDV2, SOFAR2_REG_EXPTOTAL, "total_exported", U32, DIV10 },
  { HYDV2, SOFAR2_REG_CHARGDAY, "today_charged", U32, DIV100 },
  { HYDV2, SOFAR2_REG_CHARGTOTAL, "total_charged", U32, DIV10 },
  { HYDV2, SOFAR2_REG_DISCHDAY, "today_discharged", U32, DIV100 },
  { HYDV2, SOFAR2_REG_DISCHTOTAL, "total_discharged", U32, DIV10 }
};


// These timers are used in the main loop.
#define HEARTBEAT_INTERVAL 9000
#define RUNSTATE_INTERVAL 5000
#define SEND_INTERVAL 10000
#define BATTERYSAVE_INTERVAL 3000
#define PEAKSHAVING_INTERVAL 3000

// Wemos OLED Shield set up. 64x48, pins D1 and D2
#include <SPI.h>
#include <Wire.h>

#include "Sofar2mqtt.h"

//for the tft
#include <Adafruit_ILI9341.h>   // include Adafruit ILI9341 TFT library
#define TFT_CS    D1     // TFT CS  pin is connected to arduino pin 8
#define TFT_DC    D2     // TFT DC  pin is connected to arduino pin 10
#define TFT_LED   D8
// initialize ILI9341 TFT library
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

//for touch
#include <XPT2046_Touchscreen.h>
#define TCS_PIN  0
#define TIRQ_PIN  2
XPT2046_Touchscreen ts(TCS_PIN, TIRQ_PIN);

//for the oled
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);


#include <ArduinoOTA.h>

/**
   Check to see if the elapsed interval has passed since the passed in
   millis() value. If it has, return true and update the lastRun. Note
   that millis() overflows after 50 days, so we need to deal with that
   too... in our case we just zero the last run, which means the timer
   could be shorter but it's not critical... not worth the extra effort
   of doing it properly for once in 50 days.
*/
bool checkTimer(unsigned long *lastRun, unsigned long interval)
{
  unsigned long now = millis();

  if (*lastRun > now)
    *lastRun = 0;

  if (now >= *lastRun + interval)
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
  display.setCursor(0, 0);

  if (line1 != "NULL")
  {
    display.println(line1);
    oledLine1 = line1;
  }
  else
    display.println(oledLine1);

  display.setCursor(0, 12);

  if (line2 != "NULL")
  {
    display.println(line2);
    oledLine2 = line2;
  }
  else
    display.println(oledLine2);

  display.setCursor(0, 24);

  if (line3 != "NULL")
  {
    display.println(line3);
    oledLine3 = line3;
  }
  else
    display.println(oledLine3);

  display.setCursor(0, 36);

  if (line4 != "NULL")
  {
    display.println(line4);
    oledLine4 = line4;
  }
  else
    display.println(oledLine4);

  display.display();
}

// **********************************
// * EEPROM helpers                 *
// **********************************

String read_eeprom(int offset, int len)
{
  String res = "";
  for (int i = 0; i < len; ++i)
  {
    res += char(EEPROM.read(i + offset));
  }
  return res;
}

void write_eeprom(int offset, int len, String value)
{
  for (int i = 0; i < len; ++i)
  {
    if ((unsigned)i < value.length())
    {
      EEPROM.write(i + offset, value[i]);
    }
    else
    {
      EEPROM.write(i + offset, 0);
    }
  }
}

// * Gets called when WiFiManager enters configuration mode
void configModeCallback(WiFiManager *myWiFiManager)
{
  if (tftModel) {
    tft.println(F("Entering config mode"));
    tft.println(F("Connect your phone to WiFi: "));
    tft.println(myWiFiManager->getConfigPortalSSID());
    tft.println(F("And browse to: "));
    tft.println(WiFi.softAPIP());
  } else {
    updateOLED("NULL", "hotspot", "no config", "NULL");
  }

}


bool shouldSaveConfig = false;

// * Callback notifying us of the need to save config
void save_wifi_config_callback ()
{
  shouldSaveConfig = true;
}

void saveToEeprom() {
  write_eeprom(0, 1, "1");           // * 0 --> always "1"
  write_eeprom(1, 64, deviceName);   // * 1-64
  write_eeprom(65, 64, MQTT_HOST);   // * 65-128
  write_eeprom(129, 6, MQTT_PORT);   // * 129-134
  write_eeprom(135, 32, MQTT_USER);  // * 135-166
  write_eeprom(167, 32, MQTT_PASS);  // * 167-198
  EEPROM.write(199, inverterModel); // * 199
  EEPROM.write(200, tftModel); // * 200
  EEPROM.write(201, calculated); // * 201
  EEPROM.write(202, screenDimTimer); // * 202
  EEPROM.write(203, separateMqttTopics); // * 203
  EEPROM.commit();
  ESP.reset(); // reset after save to activate new settings
}

bool loadFromEeprom() {
  // * Get MQTT Server settings
  String settings_available = read_eeprom(0, 1);

  if (settings_available == "1")
  {
    read_eeprom(1, 64).toCharArray(deviceName, 64);  // * 1-64
    read_eeprom(65, 64).toCharArray(MQTT_HOST, 64);   // * 65-128
    read_eeprom(129, 6).toCharArray(MQTT_PORT, 6);    // * 129-134
    read_eeprom(135, 32).toCharArray(MQTT_USER, 32);  // * 135-166
    read_eeprom(167, 32).toCharArray(MQTT_PASS, 32); // * 167 -198
    inverterModel = ME3000;
    if (EEPROM.read(199) == 1) {
      inverterModel = HYBRID;
    } else if (EEPROM.read(199) == 2) {
      inverterModel = HYDV2;
    }
    tftModel = EEPROM.read(200);
    calculated = EEPROM.read(201);
    screenDimTimer = EEPROM.read(202);
    separateMqttTopics = EEPROM.read(203);
    WiFi.hostname(deviceName);
    return true;
  }
  return false;
}

void setup_wifi()
{

  WiFiManagerParameter CUSTOM_MY_HOST("device", "My hostname", deviceName, 64);
  WiFiManagerParameter CUSTOM_MQTT_HOST("mqtt", "MQTT hostname", MQTT_HOST, 64);
  WiFiManagerParameter CUSTOM_MQTT_PORT("port", "MQTT port",     MQTT_PORT, 6);
  WiFiManagerParameter CUSTOM_MQTT_USER("user", "MQTT user",     MQTT_USER, 32);
  WiFiManagerParameter CUSTOM_MQTT_PASS("pass", "MQTT pass",     MQTT_PASS, 32);

  const char *bufferStr = R"(
  <br/>
  <p>Select LCD screen type:</p>
  <input style='display: inline-block;' type='radio' id='TFT' name='lcd_selection' onclick='setHiddenValueLCD()'>
  <label for='TFT'>TFT</label><br/>
  <input style='display: inline-block;' type='radio' id='OLED' name='lcd_selection' onclick='setHiddenValueLCD()'>
  <label for='OLED'>OLED</label><br/>
  <br/>  
  <p>Select inverter type:</p>
  <input style='display: inline-block;' type='radio' id='ME3000' name='inverter_selection' onclick='setHiddenValueInverter()'>
  <label for='ME3000'>ME3000SP</label><br/>
  <input style='display: inline-block;' type='radio' id='HYBRID' name='inverter_selection' onclick='setHiddenValueInverter()'>
  <label for='HYBRID'>HYDxxxxES</label><br/>
  <input style='display: inline-block;' type='radio' id='HYDV2' name='inverter_selection' onclick='setHiddenValueInverter()'>
  <label for='HYDV2'>HYD EP/KTL</label><br/>  
  <br/>
  <p>Select data mode:</p>
  <input style='display: inline-block;' type='radio' id='RAW' name='mode_selection' onclick='setHiddenValueMode()'>
  <label for='RAW'>Raw data</label><br/>
  <input style='display: inline-block;' type='radio' id='CALCULATED' name='mode_selection' onclick='setHiddenValueMode()'>
  <label for='CALCULATED'>Calculated data</label><br/>
  <br/>
  <script>
  function setHiddenValueLCD() {
    var checkBox = document.getElementById('OLED');
    var hiddenvalue = document.getElementById('key_custom_lcd');
    if (checkBox.checked == true){
      hiddenvalue.value=0
    } else {
      hiddenvalue.value=1
    }
  }
  function setHiddenValueInverter() {
    var checkBoxME3000 = document.getElementById('ME3000');
    var hiddenvalue = document.getElementById('key_custom_inverter');
    if (checkBoxME3000.checked == true){
      hiddenvalue.value=0
    } else {
      var checkBoxHYBRID = document.getElementById('HYBRID');
       if (checkBoxME3000.checked == true){
         hiddenvalue.value=1 
       } else {
         hiddenvalue.value=2 
       }
      
    }
  }
  function setHiddenValueMode() {
    var checkBox = document.getElementById('RAW');
    var hiddenvalue = document.getElementById('key_custom_mode');
    if (checkBox.checked == true){
      hiddenvalue.value=0
    } else {
      hiddenvalue.value=1
    }
  }
  if (document.getElementById("key_custom_lcd").value === "1") {
    document.getElementById("TFT").checked = true  
  } else {
    document.getElementById("OLED").checked = true  
  }
  document.querySelector("[for='key_custom_lcd']").hidden = true;
  document.getElementById('key_custom_lcd').hidden = true;
  if (document.getElementById("key_custom_inverter").value === "1") {
    document.getElementById("HYBRID").checked = true  
  } else {
    document.getElementById("ME3000").checked = true  
  }
  document.querySelector("[for='key_custom_inverter']").hidden = true;
  document.getElementById('key_custom_inverter').hidden = true;
  if (document.getElementById("key_custom_mode").value === "1") {
    document.getElementById("CALCULATED").checked = true  
  } else {
    document.getElementById("RAW").checked = true  
  }
  document.querySelector("[for='key_custom_mode']").hidden = true;
  document.getElementById('key_custom_mode').hidden = true;
  </script>
  )";
  WiFiManagerParameter custom_html_inputs(bufferStr);
  char lcdModelString[6];
  sprintf(lcdModelString, "%u", uint8_t(tftModel));
  WiFiManagerParameter custom_hidden_lcd("key_custom_lcd", "LCD type hidden", lcdModelString, 2);
  char inverterModelString[6];
  sprintf(inverterModelString, "%u", uint8_t(inverterModel));
  WiFiManagerParameter custom_hidden_inverter("key_custom_inverter", "Inverter type hidden", inverterModelString, 2);
  char modeString[6];
  sprintf(modeString, "%u", uint8_t(calculated));
  WiFiManagerParameter custom_hidden_mode("key_custom_mode", "Mode type hidden", modeString, 2);

  WiFiManager wifiManager;
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setConfigPortalTimeout(PORTAL_TIMEOUT);
  wifiManager.setSaveConfigCallback(save_wifi_config_callback);
  wifiManager.addParameter(&custom_hidden_lcd);
  wifiManager.addParameter(&custom_hidden_inverter);
  wifiManager.addParameter(&custom_hidden_mode);
  wifiManager.addParameter(&custom_html_inputs);
  wifiManager.addParameter(&CUSTOM_MY_HOST);
  wifiManager.addParameter(&CUSTOM_MQTT_HOST);
  wifiManager.addParameter(&CUSTOM_MQTT_PORT);
  wifiManager.addParameter(&CUSTOM_MQTT_USER);
  wifiManager.addParameter(&CUSTOM_MQTT_PASS);


  wifiManager.setConnectTimeout(WIFI_TIMEOUT);
  if (!wifiManager.autoConnect("Sofar2Mqtt"))
  {
    if (tftModel) {
      tft.println(F("Failed to connect to WIFI and hit timeout"));
    } else {
      updateOLED("NULL", "NULL", "WiFi.!.", "NULL");
    }
    // * Reset and try again, or maybe put it to deep sleep
    ESP.reset();
  }

  // * Read updated parameters
  strcpy(deviceName, CUSTOM_MY_HOST.getValue());
  strcpy(MQTT_HOST, CUSTOM_MQTT_HOST.getValue());
  strcpy(MQTT_PORT, CUSTOM_MQTT_PORT.getValue());
  strcpy(MQTT_USER, CUSTOM_MQTT_USER.getValue());
  strcpy(MQTT_PASS, CUSTOM_MQTT_PASS.getValue());
  if (atoi(custom_hidden_lcd.getValue()) == 0) tftModel = false;
  if (atoi(custom_hidden_inverter.getValue()) == 1) inverterModel = HYBRID;
  if (atoi(custom_hidden_inverter.getValue()) == 2) inverterModel = HYDV2;

  // * Save the custom parameters to FS which will also initiate a reset to activate other lcd screen if necessary
  if (shouldSaveConfig) saveToEeprom();
  if (tftModel) {
    tft.println(F("Connected to WIFI..."));
    tft.println(WiFi.localIP());
  } else {
    updateOLED("NULL", "NULL", "WiFi...", "NULL");
  }
  delay(500);

}



void addStateInfo(String &state, unsigned int index, unsigned int dataindex, modbusResponse *rs)
{
  String stringVal;

  if (!calculated) {
    if ( (mqtt_status_reads[index].valueType == U16) || (mqtt_status_reads[index].valueType == S16)) {
      uint16_t  val;
      val = (uint16_t)((rs->data[dataindex] << 8) | rs->data[dataindex + 1]);
      stringVal = String(val);
    } else {
      uint32_t   val;
      val = (uint32_t)((rs->data[dataindex] << 24) | (rs->data[dataindex + 1] << 16) | (rs->data[dataindex + 2] << 8) | rs->data[dataindex + 3]);
      stringVal = String(val);
    }
  } else {
    if (mqtt_status_reads[index].valueType == U16) {
      uint16_t  val;
      val = (uint16_t)((rs->data[dataindex] << 8) | rs->data[dataindex + 1]);
      switch (mqtt_status_reads[index].calculator) {
        case DIV10: {
            stringVal = String((float)val / 10.0);
            break;
          }
        case DIV100: {
            stringVal = String((float)val / 100.0);
            break;
          }
        case MUL10: {
            stringVal = String(val * 10);
            break;
          }
        case MUL100: {
            stringVal = String(val * 100);
            break;
          }
        default: {
            stringVal = String(val);
            break;
          }
      }
    }
    if (mqtt_status_reads[index].valueType == S16) {
      int16_t  val;
      val = (int16_t)((rs->data[dataindex] << 8) | rs->data[dataindex + 1]);
      switch (mqtt_status_reads[index].calculator) {
        case DIV10: {
            stringVal = String((float)val / 10.0);
            break;
          }
        case DIV100: {
            stringVal = String((float)val / 100.0);
            break;
          }
        case MUL10: {
            stringVal = String(val * 10);
            break;
          }
        case MUL100: {
            stringVal = String(val * 100);
            break;
          }
        default: {
            stringVal = String(val);
            break;
          }
      }
    }
    if (mqtt_status_reads[index].valueType == U32) {
      uint32_t   val;
      val = (uint32_t)((rs->data[dataindex] << 24) | (rs->data[dataindex + 1] << 16) | (rs->data[dataindex + 2] << 8) | rs->data[dataindex + 3]);
      switch (mqtt_status_reads[index].calculator) {
        case DIV10: {
            stringVal = String((float)val / 10.0);
            break;
          }
        case DIV100: {
            stringVal = String((float)val / 100.0);
            break;
          }
        case MUL10: {
            stringVal = String(val * 10);
            break;
          }
        case MUL100: {
            stringVal = String(val * 100);
            break;
          }
        default: {
            stringVal = String(val);
            break;
          }
      }
    }
    if (mqtt_status_reads[index].valueType == S32) {
      int32_t   val;
      val = (int32_t)((rs->data[dataindex] << 24) | (rs->data[dataindex + 1] << 16) | (rs->data[dataindex + 2] << 8) | rs->data[dataindex + 3]);
      switch (mqtt_status_reads[index].calculator) {
        case DIV10: {
            stringVal = String((float)val / 10.0);
            break;
          }
        case DIV100: {
            stringVal = String((float)val / 100.0);
            break;
          }
        case MUL10: {
            stringVal = String(val * 10);
            break;
          }
        case MUL100: {
            stringVal = String(val * 100);
            break;
          }
        default: {
            stringVal = String(val);
            break;
          }
      }
    }
  }

  if (!( state == "{"))
    state += ",";

  state += "\"" + mqtt_status_reads[index].mqtt_name + "\":" + stringVal;

  if ((separateMqttTopics) && (mqtt.connected())) {
    String topic(deviceName);
    topic += "/state/" + mqtt_status_reads[index].mqtt_name;
    sendMqtt(const_cast<char*>(topic.c_str()), stringVal);
  }

  if ((mqtt_status_reads[index].mqtt_name == "batterySOC") && (tftModel)) {
    tft.setCursor(105, 70);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.println(stringVal + "%  ");
  }

}

void retrieveData()
{
  static unsigned long	lastRun = 0;

  // Update all parameters and send to MQTT.
  if (checkTimer(&lastRun, SEND_INTERVAL))
  {
    String	state = "{\"uptime\":" + String(millis()) + ",\"deviceName\": \"" + String(deviceName) + "\"";
    if (inverterModel == ME3000) {
      modbusResponse  rs;
      if ((!modbusError) && ( readBulkReg(SOFAR_SLAVE_ID, ME3000_START, (ME3000_END - ME3000_START + 1), &rs) == 0)) {
        for (unsigned int l = 0; l < sizeof(mqtt_status_reads) / sizeof(struct mqtt_status_register); l++)
          if (mqtt_status_reads[l].inverter == inverterModel) {
            addStateInfo(state, l, (mqtt_status_reads[l].regnum - ME3000_START) * 2, &rs);
            loopRuns(); //handle some other requests while building the state info
          }
      }
    } else if (inverterModel == HYBRID) {
      modbusResponse  rs;
      if ((!modbusError) && ( readBulkReg(SOFAR_SLAVE_ID, HYBRID_START, (HYBRID_END - HYBRID_START + 1), &rs) == 0)) {
        for (unsigned int l = 0; l < sizeof(mqtt_status_reads) / sizeof(struct mqtt_status_register); l++)
          if (mqtt_status_reads[l].inverter == inverterModel) {
            addStateInfo(state, l, (mqtt_status_reads[l].regnum - HYBRID_START) * 2, &rs);
            loopRuns(); //handle some other requests while building the state info
          }
      }
    } else if (inverterModel == HYDV2) {
      if (!modbusError) {
        modbusResponse rs;
        uint8_t cached = 0;
        for (unsigned int l = 0; l < sizeof(mqtt_status_reads) / sizeof(struct mqtt_status_register); l++)
          if (mqtt_status_reads[l].inverter == inverterModel) {
            if ((mqtt_status_reads[l].regnum >= SOFAR2_SYSTEM_BEGIN) && (mqtt_status_reads[l].regnum <= SOFAR2_SYSTEM_END)) {
              if ((cached == 1) || (readBulkReg(SOFAR_SLAVE_ID, SOFAR2_SYSTEM_BEGIN, SOFAR2_SYSTEM_END - SOFAR2_SYSTEM_BEGIN + 1, &rs) == 0) ) {
                cached == 1;
                addStateInfo(state, l, (mqtt_status_reads[l].regnum - SOFAR2_SYSTEM_BEGIN) * 2, &rs);
              } else {
                cached = 0;
              }
            }
            if ((mqtt_status_reads[l].regnum >= SOFAR2_GRID_BEGIN) && (mqtt_status_reads[l].regnum <= SOFAR2_GRID_END))  {
              if ((cached == 2) || (readBulkReg(SOFAR_SLAVE_ID, SOFAR2_GRID_BEGIN, SOFAR2_GRID_END - SOFAR2_GRID_BEGIN + 1, &rs) == 0) ) {
                cached == 2;
                addStateInfo(state, l, (mqtt_status_reads[l].regnum - SOFAR2_GRID_BEGIN) * 2, &rs);
              } else {
                cached = 0;
              }
            }
            if ((mqtt_status_reads[l].regnum >= SOFAR2_PV_BEGIN) && (mqtt_status_reads[l].regnum <= SOFAR2_PV_END)) {
              if ((cached == 3) || (readBulkReg(SOFAR_SLAVE_ID, SOFAR2_PV_BEGIN, SOFAR2_PV_END - SOFAR2_PV_BEGIN + 1, &rs) == 0) ) {
                cached == 3;
                addStateInfo(state, l, (mqtt_status_reads[l].regnum - SOFAR2_PV_BEGIN) * 2, &rs);
              } else {
                cached = 0;
              }
            }
            if ((mqtt_status_reads[l].regnum >= SOFAR2_BAT_BEGIN) && (mqtt_status_reads[l].regnum <= SOFAR2_BAT_END))  {
              if ((cached == 4) || (readBulkReg(SOFAR_SLAVE_ID, SOFAR2_BAT_BEGIN, SOFAR2_BAT_END - SOFAR2_BAT_BEGIN + 1, &rs) == 0) ) {
                cached == 4;
                addStateInfo(state, l, (mqtt_status_reads[l].regnum - SOFAR2_BAT_BEGIN) * 2, &rs);
              } else {
                cached = 0;
              }
            }
            if ((mqtt_status_reads[l].regnum >= SOFAR2_POW_BEGIN) && (mqtt_status_reads[l].regnum <= SOFAR2_POW_END))  {
              if ((cached == 5) || (readBulkReg(SOFAR_SLAVE_ID, SOFAR2_POW_BEGIN, SOFAR2_POW_END - SOFAR2_POW_BEGIN + 1, &rs) == 0) ) {
                cached == 5;
                addStateInfo(state, l, (mqtt_status_reads[l].regnum - SOFAR2_POW_BEGIN) * 2, &rs);
              } else {
                cached = 0;
              }
            }
            //for some unknown reason this register can't be get in BULK
            if (mqtt_status_reads[l].regnum == SOFAR2_REG_PVW) {
              cached == 0;
              if (readSingleReg(SOFAR_SLAVE_ID, SOFAR2_REG_PVW, &rs) == 0) {
                addStateInfo(state, l, 0, &rs);
              }
            }
            loopRuns(); //handle some other requests while building the state info
          }
      }
    }

    state = state + "}";

    { //Prefix the mqtt topic name with deviceName.
      String topic(deviceName);
      topic += "/state";
      if (mqtt.connected()) sendMqtt(const_cast<char*>(topic.c_str()), state);
      state.toCharArray(jsonstring, sizeof(jsonstring));
    }

  }
}

// This function is executed when an MQTT message arrives on a topic that we are subscribed to.
void mqttCallback(String topic, byte *message, unsigned int length)
{
  if (!topic.startsWith(String(deviceName) + "/set/"))
    return;

  String messageTemp;
  uint16_t fnCode = 0, fnParam = 0;
  String cmd = topic.substring(topic.lastIndexOf("/") + 1);

  for (int i = 0; i < length; i++)
  {
    messageTemp += (char)message[i];
  }

  if (cmd == "modbus") {

    modbusResponse rs;
    String retMsg;
    if (sendModbus(message, length, &rs))
      retMsg = rs.errorMessage;
    else if (rs.dataSize != 2)
      retMsg = "Reponse is " + String(rs.dataSize) + " bytes?";
    else
    {
      retMsg = String((rs.data[0] << 8) | (rs.data[1] & 0xff));
    }

    String topic(deviceName);
    topic += "/response/modbus";
    sendMqtt(const_cast<char*>(topic.c_str()), retMsg);
    return;
  }

  if (cmd == "mode_control") { //only for HYDV2
    if (inverterModel == HYDV2) {
      modbusResponse  rs;
      int16_t data = messageTemp.toInt();
      uint16_t addr = 0x1110;
      uint8_t frame[] = { SOFAR_SLAVE_ID, MODBUS_FN_WRITEMULREG, (addr >> 8) & 0xff, addr & 0xff, 0, 1, 2, 0, data, 0, 0};
      String retMsg;
      if (sendModbus(frame, sizeof(frame), &rs))
        retMsg = rs.errorMessage;
      else if (rs.dataSize != 2)
        retMsg = "Reponse is " + String(rs.dataSize) + " bytes?";
      else
      {
        retMsg = String((rs.data[0] << 8) | (rs.data[1] & 0xff));
      }

      String topic(deviceName);
      topic += "/response/mode_control";
      sendMqtt(const_cast<char*>(topic.c_str()), retMsg);
    }
    return;

  }

  if ((cmd == "threephaselimit") || (cmd == "antireflux")) { 
    uint16_t addr = inverterModel == HYDV2 ? SOFAR2_ANTIREFLUX_CONTROL : SOFAR_ANTIREFLUX_CONTROL;
    if (messageTemp == "off") {
      modbusResponse  rs;
      uint8_t frame[] = { SOFAR_SLAVE_ID, MODBUS_FN_WRITEMULREG, (addr >> 8) & 0xff, addr & 0xff, 0, 2, 4, 0, 0, 0, 0, 0, 0};
      String retMsg;
      if (sendModbus(frame, sizeof(frame), &rs))
        retMsg = rs.errorMessage;
      else if (rs.dataSize != 2)
        retMsg = "Reponse is " + String(rs.dataSize) + " bytes?";
      else
      {
        retMsg = String((rs.data[0] << 8) | (rs.data[1] & 0xff));
      }

      String topic(deviceName);
      topic += "/response/" + cmd;
      sendMqtt(const_cast<char*>(topic.c_str()), retMsg);
    } else {
      modbusResponse  rs;
      int16_t data = messageTemp.toInt();
      uint8_t control = ((cmd == "threephaselimit") && (inverterModel == HYDV2)) ? 2 : 1;
      uint8_t frame[] = { SOFAR_SLAVE_ID, MODBUS_FN_WRITEMULREG, (addr >> 8) & 0xff, addr & 0xff, 0, 2, 4, 0, control, (data >> 8) & 0xff, data & 0xff, 0, 0};
      String retMsg;
      if (sendModbus(frame, sizeof(frame), &rs))
        retMsg = rs.errorMessage;
      else if (rs.dataSize != 2)
        retMsg = "Reponse is " + String(rs.dataSize) + " bytes?";
      else
      {
        retMsg = String((rs.data[0] << 8) | (rs.data[1] & 0xff));
      }

      String topic(deviceName);
      topic += "/response/" + cmd;
      sendMqtt(const_cast<char*>(topic.c_str()), retMsg);
    }
    return;
  }



  if (cmd == "remote_control") { //only for HYDV2
    if (inverterModel == HYDV2) {
      modbusResponse  rs;
      int16_t data = messageTemp.toInt();
      uint16_t addr = 0x1104;
      uint8_t frame[] = { SOFAR_SLAVE_ID, MODBUS_FN_WRITEMULREG, (addr >> 8) & 0xff, addr & 0xff, 0, 1, 2, 0, data, 0, 0};
      String retMsg;
      if (sendModbus(frame, sizeof(frame), &rs))
        retMsg = rs.errorMessage;
      else if (rs.dataSize != 2)
        retMsg = "Reponse is " + String(rs.dataSize) + " bytes?";
      else
      {
        retMsg = String((rs.data[0] << 8) | (rs.data[1] & 0xff));
      }

      String topic(deviceName);
      topic += "/response/remote_control";
      sendMqtt(const_cast<char*>(topic.c_str()), retMsg);
    }
    return;
  }

  int   messageValue = messageTemp.toInt();
  bool  messageBool = ((messageTemp != "false") && (messageTemp != "battery_save"));

  switch (inverterModel) {
    case HYDV2: {
        if (cmd == "standby") {
          sendPassiveCmdV2(SOFAR_SLAVE_ID, SOFAR2_REG_PASSIVECONTROL, 0, cmd);
        } else if (cmd == "auto") {
        } else if ((cmd == "charge") || (cmd == "discharge")) {
          if (cmd == "discharge") {
            messageValue = messageValue * -1;
          }
          sendPassiveCmdV2(SOFAR_SLAVE_ID, SOFAR2_REG_PASSIVECONTROL, messageValue, cmd);
        }
        break;
      }
    default: {
        if (cmd == "standby")
        {
          if (messageBool)
          {
            fnCode = SOFAR_FN_STANDBY;
            fnParam = SOFAR_PARAM_STANDBY;
          }
        }
        else if (cmd == "auto")
        {
          if (messageBool)
            fnCode = SOFAR_FN_AUTO;
          else if (messageTemp == "battery_save")
            BATTERYSAVE = true;
        }
        else if (messageValue > 0)
        {
          fnParam = messageValue;

          if (cmd == "charge")
            fnCode = SOFAR_FN_CHARGE;
          else if (cmd == "discharge")
            fnCode = SOFAR_FN_DISCHARGE;
        }

        if (fnCode)
        {
          BATTERYSAVE = false;
          sendPassiveCmd(SOFAR_SLAVE_ID, fnCode, fnParam, cmd);
        }
        break;
      }

  }
}

int peakShavingLimitImport = 0;
int peakShavingLimitExport = 0;
int peakShavingBatteryPower = 0;
void peakShaving()
{
  static unsigned long  lastRun = 0;

  if (checkTimer(&lastRun, PEAKSHAVING_INTERVAL) && PEAKSHAVING)
  {
    modbusResponse  rs;

    //Get grid power
    int16_t  gridPower = 0;

    if (readSingleReg(SOFAR_SLAVE_ID, SOFAR_REG_GRIDW, &rs) == 0) {
      gridPower = (int16_t)((rs.data[0] << 8) | rs.data[1]) * 10;
      if (gridPower < peakShavingLimitImport) {
        peakShavingBatteryPower += (gridPower - peakShavingLimitImport);
      } else if (gridPower > peakShavingLimitExport) {
        peakShavingBatteryPower += (gridPower - peakShavingLimitExport);
      }
      if (peakShavingBatteryPower > 0) {
        //charge
        if (peakShavingBatteryPower > MAX_POWER) {
          peakShavingBatteryPower == MAX_POWER;
        }
        sendPassiveCmd(SOFAR_SLAVE_ID, SOFAR_FN_CHARGE, (uint16_t)peakShavingBatteryPower, "charge");
      } else if (peakShavingBatteryPower < 0) {
        //discharge
        if (peakShavingBatteryPower < (-1 * MAX_POWER)) {
          peakShavingBatteryPower == (-1 * MAX_POWER);
        }
        sendPassiveCmd(SOFAR_SLAVE_ID, SOFAR_FN_DISCHARGE, (uint16_t)(peakShavingBatteryPower * -1), "discharge");
      } else {
        //standby
        sendPassiveCmd(SOFAR_SLAVE_ID, SOFAR_FN_STANDBY, SOFAR_PARAM_STANDBY, "standby");
      }
    }
  }
}

void batterySave()
{
  static unsigned long	lastRun = 0;

  if (checkTimer(&lastRun, BATTERYSAVE_INTERVAL) && BATTERYSAVE)
  {
    modbusResponse  rs;

    //Get grid power
    unsigned int	p = 0;

    if (readSingleReg(SOFAR_SLAVE_ID, SOFAR_REG_GRIDW, &rs) == 0) {
      p = ((rs.data[0] << 8) | rs.data[1]);

      // Switch to auto when any power flows to the grid.
      // We leave a little wriggle room because once you start charging the battery,
      // gridPower should be floating just above or below zero.
      if ((p < 65535 / 2 || p > 65525) && (((inverterModel == ME3000) && (INVERTER_RUNNINGSTATE != 4)) || ((inverterModel == HYBRID) && (INVERTER_RUNNINGSTATE != 6))) )
      {
        //exporting to the grid
        sendPassiveCmd(SOFAR_SLAVE_ID, SOFAR_FN_AUTO, 0, "bsave_auto");

      }
      else
      {
        //importing from the grid
        sendPassiveCmd(SOFAR_SLAVE_ID, SOFAR_FN_STANDBY, SOFAR_PARAM_STANDBY, "bsave_standby");
      }
    }
  }
}

// This function reconnects the ESP8266 to the MQTT broker
void mqttReconnect()
{
  unsigned long now = millis();
  if ((lastMqttReconnectAttempt == 0) || ((unsigned long)(now - lastMqttReconnectAttempt) > MQTTRECONNECTTIMER)) { //only try reconnect each MQTTRECONNECTTIMER seconds or on boot when lastMqttReconnectAttempt is still 0
    lastMqttReconnectAttempt = now;
    if (tftModel) {
      tft.fillCircle(220, 290, 10, ILI9341_RED);
    } else {
      updateOLED("NULL", "Offline", "NULL", "NULL");
    }
    mqtt.disconnect();		// Just in case.
    // Attempt to connect
    if (mqtt.connect(deviceName, MQTT_USER, MQTT_PASS))
    {
      if (tftModel) {
        tft.fillCircle(220, 290, 10, ILI9341_GREEN);
      } else {
        updateOLED("NULL", "Online", "NULL", "NULL");
      }
      //subscribe to set topics
      String settopics(deviceName);
      settopics += "/set/#";
      // Subscribe or resubscribe to topics.
      mqtt.subscribe(const_cast<char*>(settopics.c_str()));
    }
  }
}

/**
   Flush the RS485 buffers in both directions. The doc for Serial.flush() implies it only
   flushes outbound characters now... I assume RS485Serial is the same.
*/
void flushRS485()
{
  if (tftModel) {
    Serial.flush();
    delay(200);

    while (Serial.available())
      Serial.read();
  } else {
    RS485Serial.flush();
    delay(200);

    while (RS485Serial.available())
      RS485Serial.read();
  }
}

int sendModbus(uint8_t frame[], byte frameSize, modbusResponse *resp)
{
  //Calculate the CRC and overwrite the last two bytes.
  calcCRC(frame, frameSize);

  // Make sure there are no spurious characters in the in/out buffer.
  flushRS485();

  //Send
  if (tftModel) {
    Serial.write(frame, frameSize);
  } else {
    digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_TX);
    RS485Serial.write(frame, frameSize);
    digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX);
  }

  return listen(resp);
}

// Listen for a response.
int listen(modbusResponse *resp)
{
  uint8_t		inFrame[MAX_FRAME_SIZE];
  uint8_t		inByteNum = 0;
  uint8_t		inFrameSize = 0;
  uint8_t		inFunctionCode = 0;
  uint8_t		inDataBytes = 0;
  int		done = 0;
  modbusResponse	dummy;

  if (!resp)
    resp = &dummy;      // Just in case we ever want to interpret here.

  resp->dataSize = 0;
  resp->errorLevel = 0;

  while ((!done) && (inByteNum < sizeof(inFrame)))
  {
    int tries = 0;

    if (tftModel) {
      while ((!Serial.available()) && (tries++ < RS485_TRIES))
        delay(50);
    } else {
      while ((!RS485Serial.available()) && (tries++ < RS485_TRIES))
        delay(50);
    }

    if (tries >= RS485_TRIES)
    {
      break;
    }

    if (tftModel) {
      inFrame[inByteNum] = Serial.read();
    } else {
      inFrame[inByteNum] = RS485Serial.read();
    }

    //Process the byte
    switch (inByteNum)
    {
      case 0:
        if (inFrame[inByteNum] != SOFAR_SLAVE_ID)  //If we're looking for the first byte but it dosn't match the slave ID, we're just going to drop it.
          inByteNum--;          // Will be incremented again at the end of the loop.
        break;

      case 1:
        //This is the second byte in a frame, where the function code lives.
        inFunctionCode = inFrame[inByteNum];
        break;

      case 2:
        //This is the third byte in a frame, which tells us the number of data bytes to follow.
        if ((inDataBytes = inFrame[inByteNum]) > sizeof(inFrame))
          inByteNum = -1;       // Frame is too big?
        break;

      default:
        if (inByteNum < inDataBytes + 3)
        {
          //This is presumed to be a data byte.
          resp->data[inByteNum - 3] = inFrame[inByteNum];
          resp->dataSize++;
        }
        else if (inByteNum > inDataBytes + 3)
          done = 1;
    }

    inByteNum++;
  }

  inFrameSize = inByteNum;

  /**
    Now check to see if the last two bytes are a valid CRC.
    If we don't have a response pointer we don't care.
  **/
  if (inFrameSize < 5)
  {
    resp->errorLevel = 2;
    resp->errorMessage = "Response too short";
  }
  else if (checkCRC(inFrame, inFrameSize))
  {
    resp->errorLevel = 0;
    resp->errorMessage = "Valid data frame";
  }
  else
  {
    resp->errorLevel = 1;
    resp->errorMessage = "Error: invalid data frame";
  }

  return -resp->errorLevel;
}

int readBulkReg(uint8_t id, uint16_t reg, uint8_t bulkSize, modbusResponse *rs)
{
  uint8_t  frame[] = { id, MODBUS_FN_READHOLDINGREG, reg >> 8, reg & 0xff, 0, bulkSize, 0, 0 };
  return sendModbus(frame, sizeof(frame), rs);
}

int readSingleReg(uint8_t id, uint16_t reg, modbusResponse *rs)
{
  uint8_t	frame[] = { id, MODBUS_FN_READHOLDINGREG, reg >> 8, reg & 0xff, 0, 0x01, 0, 0 };
  return sendModbus(frame, sizeof(frame), rs);
}

int sendPassiveCmdV2(uint8_t id, uint16_t cmd, int32_t param, String pubTopic) {
  /*SOFAR2_REG_PASSIVECONTROL
    need to be finished and checked
    writes to 4487 - 4492 with 6x 32-bit integers
    4487 = desired PPC passive power
    4489 = min passive power
    4491 = max passive power
    but 4487 isn't for forced passive mode. Set min and max to same value for that. Negative is discharging
  */
  modbusResponse  rs;
  uint8_t frame[] = { id, MODBUS_FN_WRITEMULREG, (cmd >> 8) & 0xff, cmd & 0xff, 0, 6, 12, 0, 0, 0, 0, (param >> 24) & 0xff, (param >> 16) & 0xff, (param >> 8) & 0xff, param & 0xff, (param >> 25) & 0xff, (param >> 16) & 0xff, (param >> 8) & 0xff, param & 0xff, 0, 0 };
  int   err = -1;
  String    retMsg;

  if (sendModbus(frame, sizeof(frame), &rs))
    retMsg = rs.errorMessage;
  else if (rs.dataSize != 2)
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

int sendPassiveCmd(uint8_t id, uint16_t cmd, uint16_t param, String pubTopic)
{
  if (inverterModel == HYDV2) return 0; //no commands yet
  modbusResponse	rs;
  uint8_t	frame[] = { id, SOFAR_FN_PASSIVEMODE, cmd >> 8, cmd & 0xff, param >> 8, param & 0xff, 0, 0 };
  int		err = -1;
  String		retMsg;

  if (sendModbus(frame, sizeof(frame), &rs))
    retMsg = rs.errorMessage;
  else if (rs.dataSize != 2)
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
  char	msg[2000];

  mqtt.setBufferSize(2048);
  msg_str.toCharArray(msg, msg_str.length() + 1); //packaging up the data to publish to mqtt
  if (!(mqtt.publish(topic, msg)))
    printScreen("MQTT publish failed");
}


void heartbeat()
{
  if (inverterModel != HYDV2) { //no heartbeat
    static unsigned long  lastRun = 0;

    //Send a heartbeat
    if (checkTimer(&lastRun, HEARTBEAT_INTERVAL))
    {
      uint8_t	sendHeartbeat[] = {SOFAR_SLAVE_ID, 0x49, 0x22, 0x01, 0x22, 0x02, 0x00, 0x00};
      int	ret;

      sendModbus(sendHeartbeat, sizeof(sendHeartbeat), NULL);

    }
  }
}


void runStateME3000() {
  switch (INVERTER_RUNNINGSTATE)
  {
    case 0:
      printScreen("Standby");
      if (BATTERYSAVE) {
        if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_LIGHTGREY);
      }
      else  {
        if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_WHITE);
      }
      break;

    case 1:
      printScreen("Check");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_YELLOW );
      break;

    case 2:
      printScreen("Charging", String(batteryWatts()) + "W");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_BLUE);
      break;

    case 3:
      printScreen("Check dis");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_GREEN);
      break;

    case 4:
      printScreen("Discharging", String(-1 * batteryWatts()) + "W");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_GREEN);
      break;

    case 5:
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_PURPLE);
      break;

    case 6:
      printScreen("EPS state");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_RED);
      break;

    case 7:
      printScreen("FAULT");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_RED);
      break;

    default:
      printScreen("?");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_BLACK);
      break;
  }

}

void runStateHYBRID() { //same for v2
  switch (INVERTER_RUNNINGSTATE)
  {
    case 0:
      printScreen("Standby");
      if (BATTERYSAVE) {
        if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_LIGHTGREY);
      }
      else  {
        if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_WHITE);
      }
      break;

    case 1:
      printScreen("Check");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_YELLOW );
      break;

    case 2:
      {
        int16_t w = batteryWatts();
        if (w == 0) {
          printScreen("Normal");
          if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_WHITE);
        } else if (w > 0) {
          printScreen("Charging", String(w) + "W");
          if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_BLUE);
        } else {
          printScreen("Discharging", String(w * -1) + "W");
          if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_GREEN);
        }
      }
      break;

    case 3:
      printScreen("EPS state");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_PURPLE);
      break;

    case 4:
      printScreen("FAULT");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_RED);
      break;

    case 5:
      printScreen("PERM FAULT");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_RED);
      break;

    case 6:
      printScreen("Upgrading");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_RED);
      break;

    case 7:
      printScreen("Self Charging");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_RED);
      break;

    default:
      printScreen("?");
      if (tftModel) tft.fillCircle(120, 290, 10, ILI9341_BLACK);
      break;
  }

}


void updateRunstate()
{
  static unsigned long	lastRun = 0;

  //Check the runstate
  if (checkTimer(&lastRun, RUNSTATE_INTERVAL))
  {
    modbusResponse  response;
    uint16_t reg = inverterModel == HYDV2 ? SOFAR2_REG_RUNSTATE : SOFAR_REG_RUNSTATE;
    if (!readSingleReg(SOFAR_SLAVE_ID, reg, &response)) // 0 response is no error
    {
      INVERTER_RUNNINGSTATE = ((response.data[0] << 8) | response.data[1]);
      if (inverterModel == 0) { //only ME3000 has different runstates
        runStateME3000();
      } else  {
        runStateHYBRID();
      }
      if (modbusError) { //fixed previous modbus error
        modbusError = false;
        if (tftModel) {
          tft.fillCircle(20, 290, 10, ILI9341_GREEN);
        }
      }
    }
    else
    {
      if (!modbusError) { //new modbus error
        modbusError = true;
        if (tftModel) {
          tft.fillCircle(20, 290, 10, ILI9341_RED);
        }
        printScreen("RS485 fault");
      }
    }
  }
}

int16_t batteryWatts()
{
  uint16_t reg = inverterModel == HYDV2 ? SOFAR2_REG_BATTW : SOFAR_REG_BATTW;
  modbusResponse  response;
  if (!readSingleReg(SOFAR_SLAVE_ID, reg, &response))
  {
    int16_t  w = (int16_t)((response.data[0] << 8) | response.data[1]) * 10;
    return w;
  }
  return 0;
}


void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {

  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      if (i & 7) byte <<= 1;
      else      byte   = pgm_read_byte(bitmap + j * byteWidth + i / 8);
      if (byte & 0x80) tft.drawPixel(x + i, y + j, color);
    }
  }
}

void printScreen(String text) {
  if (text.length() > 10) {
    int index = text.lastIndexOf(' ');
    String text1 = text.substring(0, index);
    String text2 = text.substring(index + 1);
    printScreen(text1, text2);
  } else {
    if (tftModel) {
      tft.fillRect(40, 135, 159, 64, ILI9341_CYAN);
      int pos = 115 - 12 * (text.length() / 2);
      tft.setCursor(pos, 160);
      tft.setTextSize(2);
      tft.setTextColor(ILI9341_BLACK, ILI9341_CYAN);
      tft.println(text);
    } else {
      updateOLED("NULL", "NULL", text, "NULL");
    }
  }
}

void printScreen(String text1, String text2) {
  if (tftModel) {
    tft.fillRect(40, 135, 159, 64, ILI9341_CYAN);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK, ILI9341_CYAN);
    { int pos = 115 - 12 * (text1.length() / 2);
      tft.setCursor(pos, 145);
      tft.println(text1);
    }
    { int pos = 115 - 12 * (text2.length() / 2);
      tft.setCursor(pos, 175);
      tft.println(text2);
    }
  } else {
    updateOLED("NULL", "NULL", text1, text2);
  }
}

void setupOTA() {
  ArduinoOTA.setHostname(deviceName);
  // ArduinoOTA.setPassword("admin");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    //Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    //Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      //Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      //Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      //Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      //Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      //Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

// Webserver root page
void handleRoot() {
  httpServer.send_P(200, "text/html", index_html);
}

// Webserver root page
void handleSettings() {
  httpServer.send_P(200, "text/html", settings_html_new);
}

void handleJson()
{
  httpServer.send(200, "application/json", jsonstring);
}

void handleJsonSettings()
{
  String jsonsettingsstring;
  jsonsettingsstring = "{";
  jsonsettingsstring += "\"deviceName\": \"" + String(deviceName) + "\"";
  jsonsettingsstring += ",";
  jsonsettingsstring += "\"mqtthost\": \"" + String(MQTT_HOST) + "\"";
  jsonsettingsstring += ",";
  jsonsettingsstring += "\"mqttport\": \"" + String(MQTT_PORT) + "\"";
  jsonsettingsstring += ",";
  jsonsettingsstring += "\"mqttuser\": \"" + String(MQTT_USER) + "\"";
  jsonsettingsstring += ",";
  jsonsettingsstring += "\"mqttpass\": \"" + String(MQTT_PASS) + "\"";
  jsonsettingsstring += ",";
  jsonsettingsstring += "\"inverterModel\": \"" + String(inverterModel) + "\"";
  jsonsettingsstring += ",";
  jsonsettingsstring += "\"tftModel\": \"" + String(tftModel) + "\"";
  jsonsettingsstring += ",";
  jsonsettingsstring += "\"calculated\": \"" + String(calculated) + "\"";
  jsonsettingsstring += ",";
  jsonsettingsstring += "\"screendimtimer\": \"" + String(screenDimTimer) + "\"";
  jsonsettingsstring += ",";
  jsonsettingsstring += "\"separateMqttTopics\": \"" + String(separateMqttTopics) + "\"";
  jsonsettingsstring += "}";

  httpServer.send(200, "application/json", jsonsettingsstring);
}

void handleCommand() {
  int num = httpServer.args();
  bool saveEeprom = false;
  String message = "";
  for (int i = 0 ; i < num ; i++) {
    if ((httpServer.argName(i) == "reset") || (httpServer.argName(i) == "restart") || (httpServer.argName(i) == "reboot") || ((httpServer.argName(i) == "reload"))) {
      httpServer.send(200, "text/html", "<html><head><meta http-equiv=\"refresh\" content=\"2; URL=/\" /></head><body>Restarting!</body></html>");
      delay(1000);
      ESP.reset();
    } else if (httpServer.argName(i) == "factoryreset") {
      httpServer.send(200, "text/html", "<html><head><meta http-equiv=\"refresh\" content=\"2; URL=/\" /></head><body>Factory reset! Please reconfig using wifi hotspot!</body></html>");
      delay(1000);
      resetConfig();
    } else if (httpServer.argName(i) == "deviceName") {
      String value =  httpServer.arg(i);
      message += "Setting devicename to: " + value + "<br>";
      value.toCharArray(deviceName, sizeof(deviceName));
      saveEeprom = true;
    } else if (httpServer.argName(i) == "mqtthost") {
      String value =  httpServer.arg(i);
      message += "Setting MQTT host to: " + value + "<br>";
      value.toCharArray(MQTT_HOST, sizeof(MQTT_HOST));
      saveEeprom = true;
    } else if (httpServer.argName(i) == "mqttport") {
      String value =  httpServer.arg(i);
      message += "Setting MQTT port to: " + value + "<br>";
      value.toCharArray(MQTT_PORT, sizeof(MQTT_PORT));
      saveEeprom = true;
    } else if (httpServer.argName(i) == "mqttuser") {
      String value =  httpServer.arg(i);
      message += "Setting MQTT username to: " + value + "<br>";
      value.toCharArray(MQTT_USER, sizeof(MQTT_USER));
      saveEeprom = true;
    } else if (httpServer.argName(i) == "mqttpass") {
      String value =  httpServer.arg(i);
      message += "Setting MQTT password to: " + value + "<br>";
      value.toCharArray(MQTT_PASS, sizeof(MQTT_PASS));
      saveEeprom = true;
    } else if (httpServer.argName(i) == "inverterModel") {
      String value =  httpServer.arg(i);
      message += "Setting inverter type to: " + value + "<br>";
      if (value == "me3000") {
        inverterModel = ME3000;
      } else if (value == "hybrid") {
        inverterModel = HYBRID;
      } else if (value == "hydv2") {
        inverterModel = HYDV2;
      }
      saveEeprom = true;
    } else if (httpServer.argName(i) == "tftModel") {
      String value =  httpServer.arg(i);
      message += "Setting lcd type to: " + value + "<br>";
      if (value == "oled") {
        tftModel = false;
      } else if (value == "tft") {
        tftModel = true;
      }
      saveEeprom = true;
    } else if (httpServer.argName(i) == "calculated") {
      String value =  httpServer.arg(i);
      message += "Setting calculated mode to: " + value + "<br>";
      if (value == "true") {
        calculated = true;
      } else {
        calculated = false;
      }
      saveEeprom = true;
    } else if (httpServer.argName(i) == "screendimtimer") {
      String value =  httpServer.arg(i);
      message += "Setting screen dim timer to: " + value + "<br>";
      screenDimTimer = value.toInt();
      saveEeprom = true;
    } else if (httpServer.argName(i) == "separateMqttTopics") {
      String value =  httpServer.arg(i);
      message += "Setting separateMqttTopics to: " + value + "<br>";
      if (value == "true") {
        separateMqttTopics = true;
      } else {
        separateMqttTopics = false;
      }
      saveEeprom = true;
    }

  }

  if (saveEeprom) {
    httpServer.send(200, "text/html", "<html><head><meta http-equiv=\"refresh\" content=\"5; URL=/\" /></head><body>" + message + "</body></html>");
    delay(1000);
    saveToEeprom();
  } else {
    httpServer.send(200, "text/html", "<html><head><meta http-equiv=\"refresh\" content=\"2; URL=/\" /></head><body>Nothing to do!</body></html>");
  }
}



void resetConfig() {
  //initiate debug led indication for factory reset
  pinMode(2, FUNCTION_0); //set it as gpio
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW); //blue led on
  if (tftModel) {
    analogWrite(TFT_LED, 32); //PWM on led pin to dim screen
    tft.fillScreen(ILI9341_RED);
    tft.fillScreen(ILI9341_BLACK);
    tft.setScrollMargins(1, 10);
    tft.setTextColor(ILI9341_RED, ILI9341_BLACK); // Red on black
    tft.println("Double reset detected, clearing config.");
  }
  WiFi.persistent(true);
  WiFi.disconnect();
  WiFi.persistent(false);
  WiFiManager wifiManager;
  wifiManager.resetSettings();
  EEPROM.begin(512);
  write_eeprom(0, 1, "0");
  EEPROM.commit();

  if (tftModel) {
    tft.println("Config cleared. Please reset to configure this device...");
  }

  while (true) {
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
  }
}

void doubleResetDetect() {
  if (drd.detect()) {
    if (tftModel) {
      tft.begin();
      tft.setRotation(2);
    }
    resetConfig();
  }
}

void setup()
{
  // * Configure EEPROM an get initial settings
  EEPROM.begin(512);
  if (!loadFromEeprom()) { //we don't have config yet, switch between lcd models after each reset
    tftModel = true;
    if (EEPROM.read(200)) tftModel = false; //previous reboot we selected TFT model, now switch to OLED
    EEPROM.write(200, tftModel); // * 200
    EEPROM.commit();
  }
  doubleResetDetect(); //detect factory reset first

  if (tftModel) {
    tft.begin();
    tft.setRotation(2);
    analogWrite(TFT_LED, 32); //PWM on led pin to dim screen
    tft.fillScreen(ILI9341_CYAN);
    tft.fillScreen(ILI9341_BLACK);
    tft.setScrollMargins(1, 10);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); // White on black
    tft.println("Sofar2mqtt starting...");
    ts.begin();
    ts.setRotation(1);

  } else {
    //Turn on the OLED
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize OLED with the I2C addr 0x3C (for the 64x48)
    display.clearDisplay();
    display.display();
    updateOLED(deviceName, "starting", "WiFi..", version);
  }

  if (tftModel) {
    Serial.begin(9600);
  } else {
    pinMode(SERIAL_COMMUNICATION_CONTROL_PIN, OUTPUT);
    digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX);
    RS485Serial.begin(9600);
  }
  delay(500);
  setup_wifi(); //set wifi and get settings, so first thing to do

  if (tftModel) {
    tft.print("Running inverter model: ");
    if (inverterModel == ME3000) {
      tft.println("ME3000");
    } else if (inverterModel == HYBRID) {
      tft.println("HYBRID");
    } else {
      tft.println("HYD EP/KTL");
    }
  }
  delay(1000);

  mqtt.setServer(MQTT_HOST, atoi(MQTT_PORT));
  mqtt.setCallback(mqttCallback);

  setupOTA();
  MDNS.begin(deviceName);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  httpServer.on("/", handleRoot);
  httpServer.on("/settings", handleSettings);
  httpServer.on("/json", handleJson);
  httpServer.on("/jsonsettings", handleJsonSettings);
  httpServer.on("/command", handleCommand);

  if (tftModel) {
    tft.fillScreen(ILI9341_BLACK);
    drawBitmap(0, 0, background, 240, 320, ILI9341_WHITE);
    printScreen("Started");
    tft.fillCircle(20, 290, 10, ILI9341_RED); //turn modbus icon to red first
  }
  heartbeat();
  mqttReconnect();
}

int brightness = 32;
bool touchedBefore = false;
void tsLoop() {
  if (ts.tirqTouched()) {
    if (ts.touched()) { //this will run update() and therefore reset the tirqTouched flag if touch is released
      if (!touchedBefore) {
        touchedBefore = true;
        brightness == 32 ? brightness = 0 : brightness = 32;
        analogWrite(TFT_LED, brightness);
        lastScreenTouch = millis();
        delay(100);
      }
    } else {
      touchedBefore = false;
    }
  }
  if ((screenDimTimer > 0) && (brightness > 0) && ((unsigned long)(millis() - lastScreenTouch) > (1000 * screenDimTimer))) {
    brightness--;
    analogWrite(TFT_LED, brightness);
    delay(50);
  }

}

void loopRuns() {
  ArduinoOTA.handle();
  httpServer.handleClient();
  MDNS.update();
  if (tftModel) tsLoop();
}

void loop()
{
  loopRuns();

  //Check and display the runstate and update modbusError boolean
  updateRunstate();

  //Send a heartbeat to keep the inverter awake
  if (!modbusError) heartbeat();

  //make sure mqtt is still connected
  if ((!mqtt.connected()) || !mqtt.loop())
  {
    mqttReconnect();
  }

  //Get all data and send to MQTT
  retrieveData();

  //Set battery save state
  if (!modbusError) batterySave();
}

//calcCRC and checkCRC are based on...
//https://github.com/angeloc/simplemodbusng/blob/master/SimpleModbusMaster/SimpleModbusMaster.cpp

void calcCRC(uint8_t frame[], byte frameSize)
{
  unsigned int temp = 0xffff, flag;

  for (unsigned char i = 0; i < frameSize - 2; i++)
  {
    temp = temp ^ frame[i];

    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;

      if (flag)
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

  received_crc = ((frame[frameSize - 2] << 8) | frame[frameSize - 1]);
  calcCRC(frame, frameSize);
  calculated_crc = ((frame[frameSize - 2] << 8) | frame[frameSize - 1]);
  return (received_crc = calculated_crc);
}
