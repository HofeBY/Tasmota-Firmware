/*
  user_config_override.h - user configuration overrides my_user_config.h for Tasmota

  Copyright (C) 2021  Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _USER_CONFIG_OVERRIDE_H_
#define _USER_CONFIG_OVERRIDE_H_

/*****************************************************************************************************\
 * USAGE:
 *   To modify the stock configuration without changing the my_user_config.h file:
 *   (1) copy this file to "user_config_override.h" (It will be ignored by Git)
 *   (2) define your own settings below
 *
 ******************************************************************************************************
 * ATTENTION:
 *   - Changes to SECTION1 PARAMETER defines will only override flash settings if you change define CFG_HOLDER.
 *   - Expect compiler warnings when no ifdef/undef/endif sequence is used.
 *   - You still need to update my_user_config.h for major define USE_MQTT_TLS.
 *   - All parameters can be persistent changed online using commands via MQTT, WebConsole or Serial.
\*****************************************************************************************************/

/*
Examples :

// -- Master parameter control --------------------
#undef  CFG_HOLDER
#define CFG_HOLDER        4617                   // [Reset 1] Change this value to load SECTION1 configuration parameters to flash

// -- Setup your own Wifi settings  ---------------
#undef  STA_SSID1
#define STA_SSID1         "YourSSID"             // [Ssid1] Wifi SSID

#undef  STA_PASS1
#define STA_PASS1         "YourWifiPassword"     // [Password1] Wifi password

// -- Setup your own MQTT settings  ---------------
#undef  MQTT_HOST
#define MQTT_HOST         "your-mqtt-server.com" // [MqttHost]

#undef  MQTT_PORT
#define MQTT_PORT         1883                   // [MqttPort] MQTT port (10123 on CloudMQTT)

#undef  MQTT_USER
#define MQTT_USER         "YourMqttUser"         // [MqttUser] Optional user

#undef  MQTT_PASS
#define MQTT_PASS         "YourMqttPass"         // [MqttPassword] Optional password

// You might even pass some parameters from the command line ----------------------------
// Ie:  export PLATFORMIO_BUILD_FLAGS='-DUSE_CONFIG_OVERRIDE -DMY_IP="192.168.1.99" -DMY_GW="192.168.1.1" -DMY_DNS="192.168.1.1"'

#ifdef MY_IP
#undef  WIFI_IP_ADDRESS
#define WIFI_IP_ADDRESS     MY_IP                // Set to 0.0.0.0 for using DHCP or enter a static IP address
#endif

#ifdef MY_GW
#undef  WIFI_GATEWAY
#define WIFI_GATEWAY        MY_GW                // if not using DHCP set Gateway IP address
#endif

#ifdef MY_DNS
#undef  WIFI_DNS
#define WIFI_DNS            MY_DNS               // If not using DHCP set DNS IP address (might be equal to WIFI_GATEWAY)
#endif

#ifdef MY_DNS2
#undef  WIFI_DNS2
#define WIFI_DNS2           MY_DNS2              // If not using DHCP set DNS IP address (might be equal to WIFI_GATEWAY)
#endif

// !!! Remember that your changes GOES AT THE BOTTOM OF THIS FILE right before the last #endif !!!
*/
//byHofeBY unterhalb eingetragen

// Treiber die erst mal entfernt gehören, weil sie unsinniger Weise in .\tasmota\my_user_config.h aktiviert wurden
 #undef USE_VEML6070
 #undef USE_VEML6070_RSET
 #undef USE_VEML6070_SHOW_RAW
 #undef USE_APDS9960
 #undef USE_APDS9960_GESTURE
 #undef USE_APDS9960_PROXIMITY
 #undef USE_APDS9960_COLOR
 #undef USE_APDS9960_STARTMODE
 #undef USE_DISPLAY_MODES1TO5
 #undef USE_DISPLAY_LCD
 #undef USE_DISPLAY_SSD1306
 #undef USE_DISPLAY_MATRIX
 #undef USE_DISPLAY_SEVENSEG
 #undef USE_DISPLAY_ILI9341
 #undef USE_AHT1x
 #undef USE_AHT2x

//==========================================================================
// Auswahl der Unterscheidung SML/SCRIPT oder RULE & MATTER falls CPU ESP32x
// Entweder USE_SCRIPT oder USE_RULES aktiv wählen !
 #undef USE_SCRIPT  // SML benötigt SCRIPT, also standardmäßig eingeschaltet
#define USE_SCRIPT
 #undef USE_RULES   // Sobald RULES aktiv wird auch MATTER aktiv, es muss SCRIPT & SML deaktiviert werden
//#define USE_RULES
 #undef USE_I2C
#define USE_I2C // Eigentlich bei allen ESP mit I2C Unterstützung
//==========================================================================

#if defined USE_SCRIPT // RULES & MATTER weg, aktivieren von MATTER, sofern SCRIPT aktiv ist
 #undef USE_RULES
 #undef USE_MATTER_DEVICE
 #undef SCRIPT_POWER_SECTION
#define SCRIPT_POWER_SECTION
 #undef USE_SML_M
#define USE_SML_M
 #undef USE_SCRIPT_WEB_DISPLAY
#define USE_SCRIPT_WEB_DISPLAY
 #undef USE_SCRIPT_JSON_EXPORT
#define USE_SCRIPT_JSON_EXPORT
 #undef USE_SCRIPT_STATUS
#define USE_SCRIPT_STATUS
 #undef USE_SCRIPT_GLOBVARS
#define USE_SCRIPT_GLOBVARS
 #undef USE_SCRIPT_FATFS_EXT
#define USE_SCRIPT_FATFS_EXT
 #undef SML_REPLACE_VARS
#define SML_REPLACE_VARS
 #undef USE_SML_SCRIPT_CMD
#define USE_SML_SCRIPT_CMD
 #undef SML_MAX_VARS
#define SML_MAX_VARS
#endif // USE_SCRIPT 

#if defined USE_RULES // aktivieren von MATTER, deaktivieren von SCRIPT/SML
 #undef USE_MATTER_DEVICE
#define USE_MATTER_DEVICE
 #undef USE_SCRIPT
 #undef SCRIPT_POWER_SECTION
 #undef USE_SML_M
 #undef USE_SCRIPT_WEB_DISPLAY
 #undef USE_SCRIPT_JSON_EXPORT
 #undef USE_SCRIPT_STATUS
 #undef USE_SCRIPT_GLOBVARS
 #undef USE_SCRIPT_FATFS_EXT
 #undef SML_REPLACE_VARS
 #undef USE_SML_SCRIPT_CMD
 #undef SML_MAX_VARS
#endif // USE_RULES 

// neue Option USE_GPIO_VIEWER für alle ESP möglich:
 #undef USE_GPIO_VIEWER
#define USE_GPIO_VIEWER

//KNX IP Protocol für alle ESP möglich
 #undef USE_KNX
#define USE_KNX			// Enable KNX IP Protocol Support (+9.4k code, +3k7 mem)
 #undef USE_KNX_WEB_MENU
#define USE_KNX_WEB_MENU	// Enable KNX WEB MENU (+8.3k code, +144 mem)

//DOMOTICZ Protocol für alle ESP möglich
 #undef USE_DOMOTICZ
#define USE_DOMOTICZ

//==========================================================================
// Einstellungen je nach Prozessorwahl im VSC Compiler also tasmota,tasmota32,tasmota32s2,tasmota32c2,tasmota32c3,tasmota32c6,tasmota32s3
 #undef USER_TEMPLATE
//==========================================================================

#if defined ESP8266 // Gültig für ESP8285 und ESP8266 im VSC Compiler also tasmota
#define USER_TEMPLATE "{\"NAME\":\"Generic\",\"GPIO\":[1,1,1,1,1,1,1,1,1,1,1,1,1,1],\"FLAG\":0,\"BASE\":18}"  // [Template] Set JSON template
//ESP8266 -DOTA_URL='"http://ota.tasmota.com/tasmota/release/tasmota.bin.gz"'
 #undef OTA_URL
//#define OTA_URL "https://github.com/HofeBY/Tasmota-Firmware/tasmota.bin"
#define OTA_URL "http://ota.tasmota.com/tasmota/release/tasmota.bin.gz/tasmota-minimal.bin.gz"
 #undef USE_MATTER_DEVICE // Matter ist mit ESP8285 & ESP8266 nicht möglich
#endif  // ESP8266

#if defined ESP32 // ESP32 Generationen im VSC Compiler also tasmota32,tasmota32s2,tasmota32c2,tasmota32c3,tasmota32c6,tasmota32s3
#define USER_TEMPLATE "{\"NAME\":\"ESP32\",\"GPIO\":[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],\"FLAG\":0,\"BASE\":1}"
//ESP32 -DOTA_URL='"http://ota.tasmota.com/tasmota32/release/tasmota32.bin"'
 #undef OTA_URL
#define OTA_URL "http://ota.tasmota.com/tasmota32/release/tasmota32-saveboot.bin"
//#define OTA_URL "https://github.com/HofeBY/Tasmota-Firmware/tasmota32.bin"

// ESP32 CPU Temperature and optional Hall Effect sensor
// To allow for not updating the global temperature by the ESP32 temperature sensor this
// driver needs to be the highest numbered driver (currently 127)
// ESP32 internal Hall Effect sensor connected to both GPIO36 and GPIO39
// To enable set both as following
// GPIO36 as HallEffect 1 und GPIO39 as HallEffect 2 einstellen
// Temperatursensor CPU    anzeigen mit "SetOption146 1" in Kommandokonsole
// Temperatursensor CPU ausschalten mit "SetOption146 0" in Kommandokonsole
// Temperatursensor&Hall in CPU   aktivieren mit "SetSensor127 1" in Kommandokonsole
// Temperatursensor&Hall in CPU deaktivieren mit "SetSensor127 0" in Kommandokonsole
// siehe auch https://tasmota.github.io/docs/ESP32/#cpu-temperature-sensor

// -- Influxdb ------------------------------------
#define USE_INFLUXDB			// Enable influxdb support (+5k code)
#define INFLUXDB_STATE     0		// [Ifx] Influxdb initially Off (0) or On (1)
#define INFLUXDB_VERSION   1		// Version of Influxdb 1 or 2
#define INFLUXDB_HOST      "influxdb"	// [IfxHost] Influxdb hostname or IP address
#define INFLUXDB_PORT      8086		// [IfxPort] Influxdb port number
#define INFLUXDB_ORG       ""		// [IfxUser, IfxOrg] Influxdb v1 username or v2 organisation
#define INFLUXDB_TOKEN     ""		// [IfxPassword, IfxToken] Influxdb v1 password or v2 token
#define INFLUXDB_BUCKET    "db"		// [IfxDatabase, IfxBucket] Influxdb v1 database or v2 bucket
#define INFLUXDB_RP        ""		// [IfxRP] Influxdb retention policy
#endif // ESP32 Generationen

#if defined ESP32-S2 // im VSC Compiler also tasmota32s2
// Prozessorspezifika
#endif // ESP32-S2

#if defined ESP32-C2 // im VSC Compiler also tasmota32c2
// Prozessorspezifika
#define NO_USE_SML_CANBUS // weil er keinen CANBUS hat
#endif // ESP32-C2

#if defined ESP32-C3 // im VSC Compiler also tasmota32c3
// Prozessorspezifika
#endif // ESP32-C3

#if defined ESP32-C6 // im VSC Compiler also tasmota32c6
// Prozessorspezifika
#endif // ESP32-C6

#if defined ESP32-S3 // im VSC Compiler also tasmota32s3
// Prozessorspezifika
#endif // ESP32-S3

//#undef  GUI_SHOW_HOSTNAME
//#define GUI_SHOW_HOSTNAME true //[SetOption53] Show hostname and IP address in GUI main menu

// -- Time - Up to three NTP servers in your region
 #undef NTP_SERVER1
#define NTP_SERVER1	"192.53.103.108"	// [fritz.box besser die IP] 
 #undef NTP_SERVER2
#define NTP_SERVER2	"192.53.103.104"	// ptbtime1.ptb.de[192.53.103.108] ptbtime2.ptb.de[192.53.103.104] ptbtime3.ptb.de[192.53.103.103]
 #undef NTP_SERVER3
#define NTP_SERVER3	"0.de.pool.ntp.org"	// [NtpServerPool]

 #undef MDNS_ENABLED
#define MDNS_ENABLED	true

// -- Time - Start Daylight Saving Time and timezone offset from UTC in minutes
 #undef TIME_DST_HEMISPHERE
#define TIME_DST_HEMISPHERE    North	// [TimeDst] Hemisphere (0 or North, 1 or South)
 #undef TIME_DST_WEEK
#define TIME_DST_WEEK          Last	// Week of month (0 or Last, 1 or First, 2 or Second, 3 or Third, 4 or Fourth)
 #undef TIME_DST_DAY
#define TIME_DST_DAY           Sun	// Day of week (1 or Sun, 2 or Mon, 3 or Tue, 4 or Wed, 5 or Thu, 6 or Fri, 7 or Sat)
 #undef TIME_DST_MONTH
#define TIME_DST_MONTH         Mar	// Month (1 or Jan, 2 or Feb, 3 or Mar, 4 or Apr, 5 or May, 6 or Jun, 7 or Jul, 8 or Aug, 9 or Sep, 10 or Oct, 11 or Nov, 12 or Dec)
 #undef TIME_DST_HOUR
#define TIME_DST_HOUR          2	// Hour (0 to 23)
 #undef TIME_DST_OFFSET
#define TIME_DST_OFFSET        +120	// Offset from UTC in minutes (-780 to +780)

// -- Time - Start Standard Time and timezone offset from UTC in minutes
 #undef TIME_STD_HEMISPHERE
#define TIME_STD_HEMISPHERE    North	// [TimeStd] Hemisphere (0 or North, 1 or South)
 #undef TIME_STD_WEEK
#define TIME_STD_WEEK          Last	// Week of month (0 or Last, 1 or First, 2 or Second, 3 or Third, 4 or Fourth)
 #undef TIME_STD_DAY
#define TIME_STD_DAY           Sun	// Day of week (1 or Sun, 2 or Mon, 3 or Tue, 4 or Wed, 5 or Thu, 6 or Fri, 7 or Sat)
 #undef TIME_STD_MONTH
#define TIME_STD_MONTH         Oct	// Month (1 or Jan, 2 or Feb, 3 or Mar, 4 or Apr, 5 or May, 6 or Jun, 7 or Jul, 8 or Aug, 9 or Sep, 10 or Oct, 11 or Nov, 12 or Dec)
 #undef TIME_STD_HOUR
#define TIME_STD_HOUR          3	// Hour (0 to 23)
 #undef TIME_STD_OFFSET
#define TIME_STD_OFFSET        +60	// Offset from UTC in minutes (-780 to +780)

// -- Application ---------------------------------
//Timezone 99 // muss auf Console einmalig gesetzt werden
 #undef APP_TIMEZONE
#define APP_TIMEZONE 99 // [Timezone] +1 hour (Amsterdam) (-13 .. 14 = hours from UTC, 99 = use TIME_DST/TIME_STD)

// -- Location------------------------------------
 #undef LATITUDE
#define LATITUDE	49.845915	// [Latitude] Your location Creußen/Gemeinde to be used with sunrise and sunset
 #undef LONGITUDE
#define LONGITUDE	11.626169	// [Longitude] Your location Creußen/Gemeinde to be used with sunrise and sunset

// -- Localization --------------------------------
  // If non selected the default en-GB will be used
 #undef MY_LANGUAGE
#define MY_LANGUAGE de_DE // German in Germany

// -- Setup your own Wifi settings  ---------------
 #undef STA_SSID1
#define STA_SSID1	""

 #undef STA_PASS1
#define STA_PASS1	""

 #undef STA_SSID2
#define STA_SSID2	""

 #undef STA_PASS2
#define STA_PASS2	""

// IR z.Z. nicht zu compilieren TM14.0.0.4
// #undef FIRMWARE_IR
//#define FIRMWARE_IR		// Create tasmota-ir with IR full protocols activated, and many sensors disabled

// -- Lights --------------------------------------
 #undef WS2812_LEDS
#define WS2812_LEDS            30                // [Pixels] Number of WS2812 LEDs to start with (max is 512)
 #undef LIGHT_MODE
#define LIGHT_MODE             true              // [SetOption15] Switch between commands PWM or COLOR/DIMMER/CT/CHANNEL
 #undef LIGHT_CLOCK_DIRECTION
#define LIGHT_CLOCK_DIRECTION  false             // [SetOption16] Switch WS2812 clock between clockwise or counter-clockwise
 #undef LIGHT_COLOR_RADIX
#define LIGHT_COLOR_RADIX      false             // [SetOption17] Switch between decimal or hexadecimal color output (false = hexadecimal, true = decimal)
 #undef LIGHT_PAIRS_CO2
#define LIGHT_PAIRS_CO2        false             // [SetOption18] Enable Pair light signal with CO2 sensor
 #undef LIGHT_POWER_CONTROL
#define LIGHT_POWER_CONTROL    false             // [SetOption20] Enable power control in relation to Dimmer/Color/Ct changes
 #undef LIGHT_CHANNEL_MODE
#define LIGHT_CHANNEL_MODE     false             // [SetOption68] Enable multi-channels PWM instead of Color PWM
 #undef LIGHT_SLIDER_POWER
#define LIGHT_SLIDER_POWER     false             // [SetOption77] Do not power off if slider moved to far left
 #undef LIGHT_ALEXA_CT_RANGE
#define LIGHT_ALEXA_CT_RANGE   false             // [SetOption82] Reduced CT range for Alexa
 #undef LIGHT_PWM_CT_MODE
#define LIGHT_PWM_CT_MODE      false             // [SetOption92] Set PWM Mode from regular PWM to ColorTemp control (Xiaomi Philips ...) a.k.a. module 48 mode
 #undef LIGHT_WHITE_BLEND_MODE
#define LIGHT_WHITE_BLEND_MODE false             // [SetOption105] White Blend Mode - used to be `RGBWWTable` last value `0`, now deprecated in favor of this option
 #undef LIGHT_VIRTUAL_CT
#define LIGHT_VIRTUAL_CT       false             // [SetOption106] Virtual CT - Creates a virtual White ColorTemp for RGBW lights
 #undef LIGHT_VIRTUAL_CT_CW
#define LIGHT_VIRTUAL_CT_CW    false             // [SetOption107] Virtual CT Channel - signals whether the hardware white is cold CW (true) or warm WW (false)
 #undef LIGHT_VIRTUAL_CT_POINTS
#define LIGHT_VIRTUAL_CT_POINTS 3                // Number of reference points for Virtual CT (min 2, default 3)
 #undef USE_AC_ZERO_CROSS_DIMMER
#define USE_AC_ZERO_CROSS_DIMMER                 // Requires USE_COUNTER and USE_LIGHT

// -- One wire sensors ----------------------------
 #undef USE_DS18x20
#define USE_DS18x20		// Add support for DS18x20 sensors with id sort, single scan and read retry (+2k6 code)
 #undef W1_PARASITE_POWER
#define W1_PARASITE_POWER	// Optimize for parasite powered sensors
 #undef DS18x20_USE_ID_ALIAS
#define DS18x20_USE_ID_ALIAS	// Add support aliasing for DS18x20 sensors. See comments in xsns_05 files (+0k5 code)
//Kommandos für DS18x...
// DS18Alias B33C6CE347110815,gotemp // DS-Adressse aus ConsoleLOG

//#if defined USE_I2C // Moegliche I2C Geraete Treiber siehe my_user_config.h
 #undef USE_RTC_CHIPS
#define USE_RTC_CHIPS		// Enable RTC chip support and NTP server - Select only one #undef USE_DS3231
 #undef USE_DS3231
#define USE_DS3231		//I2C Support für DS1307 oder DS3231 und NTP-Server Funktion
 #undef DS3231_ENABLE_TEMP
#define DS3231_ENABLE_TEMP	//   In DS3231 driver, enable the internal temperature senso
// RtcNtpServer<x> // 0 = disabled; 1 = enabled Use Tasmota NTP server when enabled by define RTC_NTP_SERVER
 #undef USE_24C256
#define USE_24C256

 #undef USE_BMP
#define USE_BMP		// [I2cDriver10] Enable BMP085/BMP180/BMP280/BME280 sensors (ab 8.-€ Cn RH/T/P I2C Sensor addresses 0x76 and 0x77) (+4k4 code)
 #undef USE_BME68X
#define USE_BME68X	// Enable support for BME680/BME688 sensor using Bosch BME68x library (ab 11.-€ Cn RH/T/P/IAQ I2C Sensor +6k9 code)
 #undef USE_AHT2x
#define USE_AHT2x	// [I2cDriver43] Enable AHT20/AM2301B instead of AHT1x humidity and temperature sensor (ab 1.70€ Cn RH/T 2-5,5V I2C address 0x38) (+0k8 code)
 #undef USE_ENS16x
#define USE_ENS16x      // [I2cDriver85] Enable ENS160 and ENS161 CO2 sensor (ab 4,50€ Cn CO2 mit AHT21 I2C addresses 0x52 and 0x53) (+1.9kB of code and 12B of RAM)
 #undef USE_HTU
#define USE_HTU
 #undef USE_SCD40
#define USE_SCD40
 #undef USE_SGP40
#define USE_SGP40       // [I2cDriver69] Enable SGP40 sensor (I2C address 0x59) (+1k4 code)

 #undef USE_INA219
//#define USE_INA219	// [I2cDriver14] Enable INA219 (I2C address 0x40, 0x41 0x44 or 0x45) Low voltage and current sensor (+1k code)
//#define INA219_SHUNT_RESISTOR (0.100)	// 0.1 Ohm default shunt resistor, can be overriden in user_config_override or using Sensor13

// I2C Driver Nr 22 only one device usable
 #undef USE_MCP230xx
//#define USE_MCP230xx		 // [I2cDriver22] Enable MCP23008/MCP23017 - Must define I2C Address in #define USE_MCP230xx_ADDR below - range 0x20 - 0x27 (+5k1 code)
 #undef USE_MCP230xx_ADDR
//#define USE_MCP230xx_ADDR 0x20 // Enable MCP23008/MCP23017 I2C Address to use (Must be within range 0x20 through 0x26 - set according to your wired setup)
 #undef USE_MCP230xx_OUTPUT 
//#define USE_MCP230xx_OUTPUT	 // Enable MCP23008/MCP23017 OUTPUT support through sensor29 commands (+2k2 code)
 #undef USE_MCP230xx_DISPLAYOUTPUT
//#define USE_MCP230xx_DISPLAYOUTPUT	// Enable MCP23008/MCP23017 to display state of OUTPUT pins on Web UI (+0k2 code)

// I2C Driver Nr 77 multiple devices !
 #undef USE_MCP23XXX_DRV
#define USE_MCP23XXX_DRV	 // multiple devices [I2cDriver77] Enable MCP23xxx support as virtual switch/button/relay (+3k(I2C)/+5k(SPI) code)

 #undef USE_PCF8574
#define USE_PCF8574	// [I2cDriver2] Enable PCF8574 I/O Expander (I2C addresses 0x20 - 0x26 and 0x39 - 0x3F) (+2k1 code)
 #undef USE_PCF8574_MODE2
#define USE_PCF8574_MODE2	// Enable Mode2 virtual relays/buttons/switches (+2k3 code)
 #undef USE_PCF8574_SENSOR
#define USE_PCF8574_SENSOR	// Enable Mode1 inputs and outputs in SENSOR message (+0k2 code)
 #undef USE_PCF8574_DISPLAYINPUT
#define USE_PCF8574_DISPLAYINPUT// Enable Mode1 inputs display in Web page (+0k2 code)
 #undef USE_PCF8574_MQTTINPUT
#define USE_PCF8574_MQTTINPUT	// Enable Mode1 MQTT message & rule process on input change detection : stat/%topic%/PCF8574_INP = {"Time":"2021-03-07T16:19:23+01:00","PCF8574-1_INP":{"D1":1}} (+0k5 code)
 #undef PCF8574_ADDR1
#define PCF8574_ADDR1 0x20	// First address to search for PCF8574
 #undef PCF8574_ADDR1_COUNT
#define PCF8574_ADDR1_COUNT 7	// Number of addresses to search for PCF8574 - Default to 0x20 to 0x26
 #undef PCF8574_ADDR2
#define PCF8574_ADDR2 0x39	// First address to search for PCF8574A
 #undef PCF8574_ADDR2_COUNT
#define PCF8574_ADDR2_COUNT 6	// Number of addresses to search for PCF8574A - Default to 0x39 to 0x3E

 #undef USE_PCA9557
#define USE_PCA9557	// [I2cDriver81] Enable PCA9557 8-bit I/O Expander (I2C addresses 0x18 - 0x1F) (+2k5 code)

 #undef USE_PCA9685
#define USE_PCA9685		// [I2cDriver1] Enable PCA9685 I2C HW PWM Driver - Must define I2C Address in #define USE_PCA9685_ADDR below - range 0x40 - 0x47 (+1k4 code)
 #undef USE_PCA9685_ADDR
#define USE_PCA9685_ADDR 0x40	// Enable PCA9685 I2C Address to use (Must be within range 0x40 through 0x47 - set according to your wired setup)
 #undef USE_PCA9685_FREQ 
#define USE_PCA9685_FREQ 50	// Define default PWM frequency in Hz to be used (must be within 24 to 1526) - If other value is used, it will rever to 50Hz

// PCA9685 Befehle für die Eingabeconsole
// driver15 pwmf,frequency	// where frequency is the PWM frequency from 24 to 1526 in Hz
// driver15 pwm,pin,pwmvalue	// where pin = LED pin 0 through 15 and pwmvalue is the pulse width between 0 and 4096
// driver15 pwm,pin,ON		// Fully turn a specific pin/LED ON
// driver15 pwm,pin,OFF		// Fully turn a specific pin/LED OFF
// driver15 reset		// Reset to power-up settings - i.e. F=50hz and all pins in OFF state
// driver15 status		// Will return a JSON string containing all the current settings / parameters
//#endif  // USE_I2C

// ESP8266 und ESP32
#define USE_EXPRESSION

#if defined ESP32	// Only for ESP32 x devices
// -- ESP-NOW Nur ESP32 ! -------------------------
 #undef USE_TASMESH
//#define USE_TASMESH		// Enable Tasmota Mesh using ESP-NOW (+11k code)

// WiFI Range extender  Nur ESP32 !? 
 #undef USE_WIFI_RANGE_EXTENDER
//#define USE_WIFI_RANGE_EXTENDER
 #undef USE_WIFI_RANGE_EXTENDER_NAPT
//#define USE_WIFI_RANGE_EXTENDER_NAPT

// #undef WIFI_RGX_SSID
//#define WIFI_RGX_SSID ""
// #undef WIFI_RGX_PASSWORD
//#define WIFI_RGX_PASSWORD ""
// #undef WIFI_RGX_IP_ADDRESS
//#define WIFI_RGX_IP_ADDRESS "192.168.99.254"
// #undef WIFI_RGX_SUBNETMASK
//#define WIFI_RGX_SUBNETMASK "255.255.255.0"
// #undef WIFI_RGX_STATE
//#define WIFI_RGX_STATE 1

// Commands für WIFI_RANGE_EXTENDER:
// RgxSSId RE ; REDW
// RgxPassword xyz
// RgxAddress 192.168.99.1 bis 254
// RgxState 1
// RgxNAPT 1
//
// RgxClients //listet aktive Client´s auf
//

#endif  // Ende Only for ESP32 x devices


// -- Matter Protocol -freischalten mit SetOption151 1 ----------
//Matter ist nicht mit SML kombinierbar. Deshalb wird SCRIPT deaktiviert, sofern "USE_MATTER_DEVICE" vorhanden ist !!!
#if defined USE_MATTER_DEVICE	// Only for MATTER devices
 #undef USE_DISCOVERY
#define USE_DISCOVERY
// Enable all the crypto required by Matter
 #undef USE_BERRY_CRYPTO_EC_P256
#define USE_BERRY_CRYPTO_EC_P256
 #undef USE_BERRY_CRYPTO_HMAC_SHA256
#define USE_BERRY_CRYPTO_HMAC_SHA256
 #undef USE_BERRY_CRYPTO_HKDF_SHA256
#define USE_BERRY_CRYPTO_HKDF_SHA256
 #undef USE_BERRY_CRYPTO_AES_CCM
#define USE_BERRY_CRYPTO_AES_CCM
 #undef USE_BERRY_CRYPTO_AES_CTR
#define USE_BERRY_CRYPTO_AES_CTR
 #undef USE_BERRY_CRYPTO_PBKDF2_HMAC_SHA256
#define USE_BERRY_CRYPTO_PBKDF2_HMAC_SHA256
 #undef USE_BERRY_CRYPTO_SPAKE2P_MATTER
#define USE_BERRY_CRYPTO_SPAKE2P_MATTER
#endif  // Only for MATTER devices without SCRIPT

 #undef SUPPORT_MQTT_EVENT
#define SUPPORT_MQTT_EVENT	// enables suppoer for subscribe an unsubscribe

 #undef USE_SENDMAIL
#define USE_SENDMAIL		// >m section for sending e-Mail
#undef SET_ESP32_STACK_SIZE
#define SET_ESP32_STACK_SIZE (12 * 1024)
// #undef EMAIL_USER
//#define EMAIL_USER ""
// #undef EMAIL_PASSWORD
//#define EMAIL_PASSWORD ""
// #undef EMAIL_FROM
//#define EMAIL_FROM ""
// #undef EMAIL_SERVER
//#define EMAIL_SERVER "mail.gmx.net"
// #undef EMAIL_PORT
//#define EMAIL_PORT 465
 #undef MAIL_TIMEOUT
#define MAIL_TIMEOUT 2000

 #undef USE_BUTTON_EVENT
#define USE_BUTTON_EVENT	// >b Section detect button state changes

//https://github.com/arendst/Tasmota/issues/7021
 #undef USE_EXPRESSION
#define USE_EXPRESSION

 #undef SUPPORT_IF_STATEMENT
#define SUPPORT_IF_STATEMENT

// Speichern auf FAT System intern als auch auf SD-Karte
// Dateien, welche vorkommen können (IP durch eigene ersetzen)
// http://192.168.190.21/ufs/AMT681-2024.csv
// _matter_device.json
// _persist.json
// script.txt
// .drvset003
// .settings.lkg
// .settings
// Info aus dem Start des ESP32 in Konsole Log:
// .autoconf
// preinit.be
// autoexec.be
// autoexec.bat

 #undef USE_UFILESYS
#define USE_UFILESYS

 #undef GUI_TRASH_FILE
#define GUI_TRASH_FILE

 #undef USE_SDCARD
#define USE_SDCARD

 #undef USE_PING
#define USE_PING

 #undef MAXSVARS
#define MAXSVARS 5

 #undef MAXFILT
#define MAXFILT 5

// erzeugt Compiler Fehler
// #undef USE_SCRIPT_GLOBVARS
//define USE_SCRIPT_GLOBVARS

// erzeugt Compiler Fehler
// #undef SCRIPT_FULL_WEBPAGE
//#define SCRIPT_FULL_WEBPAGE

// erzeugt Compiler Fehler
// #undef USE_WEBSEND_RESPONSE
//define USE_WEBSEND_RESPONSE

// erzeugt Compiler Fehler
// #undef USE_ENERGY_SENSOR
//define USE_ENERGY_SENSOR

// erzeugt Compiler Fehler
// #undef USE_SCRIPT_SUB_COMMAND
//define USE_SCRIPT_SUB_COMMAND

 #undef SUPPORT_MQTT_EVENT
#define SUPPORT_MQTT_EVENT
 #undef MQTT_EVENT_MSIZE
#define MQTT_EVENT_MSIZE 256 // (default is 256)
 #undef MQTT_EVENT_JSIZE
#define MQTT_EVENT_JSIZE 400 // (default is 400) 

// Script ursprünglicher Basisinhalt
//>D
//script error must start with >D

//byHofeBY oberhalb eingetragen


#endif  // _USER_CONFIG_OVERRIDE_H_
