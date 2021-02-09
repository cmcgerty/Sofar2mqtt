# Sofar2MQTT
## A smart home interface for Sofar solar and battery inverters.

Supported models:  

ME3000SP - Full support  
HYD-xx00-ES - Partial support

![Sofar2MQTT](pics/Sofar2MQTT.jpg)

Sofar2MQTT is a remote control interface for Sofar solar and battery inverters.
It allows remote control of the inverter and reports the invertor status, power usage, battery state etc for integration with smart home systems such as [Home Assistant](https://www.home-assistant.io/) and [Node-Red](https://nodered.org/).  
For read only mode, it will send status messages without the inverter needing to be in passive mode.  
It's designed to run on an ESP8266 microcontroller with a TTL to RS485 module such as MAX485 or MAX3485.  
Designed to work with TTL modules with or without the DR and RE flow control pins. If your TTL module does not have these pins then just ignore the wire from D5. 

Subscribe your MQTT client to:

sofar/state

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

sofar/set/standby   - send value "true"  
sofar/set/auto   - send value "true" or "battery_save"  
sofar/set/charge   - send value in the range 0-3000 (watts)  
sofar/set/discharge   - send value in the range 0-3000 (watts) 

battery_save is a hybrid auto mode that will charge from excess solar but not discharge.

(c)Colin McGerty 2021 colin@mcgerty.co.uk  
calcCRC by angelo.compagnucci@gmail.com and jpmzometa@gmail.com

# How To Build

Parts List:
1. ESP8266 Microcontroller
2. MAX485 or MAX3485 TTL to RS485 board
3. Wemos 64x48 OLED Screen (optional)
4. A small project board
5. A few wires and a little solder

![Parts](pics/parts.jpg)

Cut the project board to a convenient size.

![Board](pics/board.jpg)

Wire the components according to this circuit diagram.

![Wiring Diagram](pics/diagram.jpg)

I tend to keep the wires on top of the board, poke them through and solder underneath. Your approach may be better and your soldering will almost certainly be better than mine!

![Wiring](pics/wiring.jpg)

Make sure you connect the DR and RE pins together. The red arrow below shows where a single wire from D5 connects to both DR and RE. (If you are using a TTL board without the DR and RE pins, ignore this step.)

![Short these pins](pics/short.jpg)

Use long pinned mounts on your ESP8266 if you are stacking the optional OLED on top.

![Chips](pics/ICs.jpg)

Here's how it looks when completed.

![Finished](pics/Sofar2MQTT_completed.jpg)

# Flashing

Open Sofar2MQTT.ino in the Arduino IDE.

Edit Sofar2MQTT.ino with your wifi network name and password and your mqtt server details. If you need more than one Sofar2MQTT on your network, make sure you give them unique device names.  

You'll need the libraries for the ESP8266. Follow [this guide](https://randomnerdtutorials.com/how-to-install-esp8266-board-arduino-ide/) if you haven't completed that step before.

Add a few more libraries using the Manage Libraries menu:
1. PubSubClient
2. Adafruit GFX
3. Adafruit SSD1306 Wemos Mini OLED

(Even if you are not using the OLED screen, you should install the Adafruit libraries or it will not compile.)

...and upload.

Run it on the desktop, not connected to you invertor, to test that wifi and mqtt are connected and see some messages in the serial monitor.
The OLED screen should show "Online" to indicate a connection to WiFi and MQTT. It will alternate between "RS485 Error" and "CRC-FAULT" to indicate that the inverter is not connected.

# Connect to Inverter

Connect the Sofar2MQTT unit to a 5v micro USB power supply.
Now connect wires A and B to the two wire RS485 input of your inverter, which is marked as 485s on the image of the ME3000SP below.

![ME3000SP Data Connections](pics/485s.jpg)

# Troubleshooting

Here's what the various things on the OLED screen tell you:

Line 1 is just the project name, nothing else.  
Line 2 will display "Connecting" during start up and lines 3 and 4 will show WIFI and MQTT getting connected.  
Line 2 also has a dot that slowly flashes, once every few seconds. This is when a heartbeat message is being sent to the inverter.  
If a message is read from the inverter that fails the CRC checksum, line 3 will display "CRC-FAULT". This could be caused by a loose or bad RS485 wire or by unsupported features.  
If no response is received to a heartbeat message, lines 3 and 4 show "RS485 ERROR". This could be caused by disconnected or reversed RS485 wires.  
During start-up, line 4 shows the Sofar2MQTT software version. Check that you have the latest version at https://github.com/cmcgerty/Sofar2MQTT  
In normal operation, line 2 shows "Online" which indicates that both WIFI and MQTT are still connected.  
In normal operation, line 3 shows the inverter run state, Standby, Charging, Discharging etc.  
In normal operation, line 4 shows the power in W in or out of the batteries when charging or discharging.  



