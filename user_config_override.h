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

// Schnelle Prozessorwahl ESP8266 oser ESP32 mit universal Port deffiniton
//#define USER_TEMPLATE "{\"NAME\":\"ESP8266\",\"GPIO\":[1,1,1,1,1,1,0,0,1,1,1,1,1,1],\"FLAG\":0,\"BASE\":18}"  // [Template] Set JSON template
#define USER_TEMPLATE "{\"NAME\":\"ESP32\",\"GPIO\":[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],\"FLAG\":0,\"BASE\":1}"

// -- HTTP ----------------------------------------
//#undef WEB_SERVER
// #define WEB_SERVER             2                 // [WebServer] Web server (0 = Off, 1 = Start as User, 2 = Start as Admin)
//#undef WEB_PASSWORD
// #define WEB_PASSWORD           ""                // [WebPassword] Web server Admin mode Password for WEB_USERNAME (empty string = Disable)

//#undef  GUI_SHOW_HOSTNAME
//#define GUI_SHOW_HOSTNAME      true	// [SetOption53] Show hostname and IP address in GUI main menu

// ESP32 CPU Temperature and optional Hall Effect sensor
// To allow for not updating the global temperature by the ESP32 temperature sensor this
// driver needs to be the highest numbered driver (currently 127)
// ESP32 internal Hall Effect sensor connected to both GPIO36 and GPIO39
// To enable set both as following
// GPIO36 as HallEffect 1
// GPIO39 as HallEffect 2
// Setoption146 1 // für CPU Temperatur & Hall anzeigen
// SO146 1 // Alternativbefehl für CPU Temperatur & Hall anzeigen

// -- Time - Up to three NTP servers in your region
 #undef NTP_SERVER1
#define NTP_SERVER1	"192.53.103.108"	// ntpserver1 192.53.103.108 [SML-LeserESP32; fritz.box oder besser die IP] 
 #undef NTP_SERVER2
#define NTP_SERVER2	"192.53.103.104"	// ntpserver2 192.53.103.104 ptbtime1.ptb.de[192.53.103.108] ptbtime2.ptb.de[192.53.103.104] ptbtime3.ptb.de[192.53.103.103]
 #undef NTP_SERVER3
#define NTP_SERVER3	"0.de.pool.ntp.org"	// ntpserver3 0.de.pool.ntp.org [NtpServerPool]

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
 #undef APP_TIMEZONE
#define APP_TIMEZONE           99	// timezone 99 [Timezone] +1 hour (Amsterdam) (-13 .. 14 = hours from UTC, 99 = use TIME_DST/TIME_STD)

// -- Location ------------------------------------
 #undef LATITUDE
#define LATITUDE	49.845915	// latitude 49.845915 [Latitude] This example location Creußen/Gemeinde to be used with sunrise and sunset. Look in Routing SW.
 #undef LONGITUDE
#define LONGITUDE	11.626169	// Longitude 11.626169 [Longitude] This example location Creußen/Gemeinde to be used with sunrise and sunset. Look in Routing SW.

// -- Setup your own Wifi settings  ---------------
 #undef STA_SSID1
//#define STA_SSID1	"IhreSSID1"	// [Ssid1] Wifi SSID
#define STA_SSID1	""

 #undef STA_PASS1
//#define STA_PASS1	"IhrPasswort1"	// [Password1] Wifi password
#define STA_PASS1         ""

 #undef STA_SSID2
//#define STA_SSID2	"IhreSSID2"		// [Ssid2] Wifi SSID
#define STA_SSID2         ""

 #undef STA_PASS2
//#define STA_PASS2	"IhrPasswort2"	// [Password2] Wifi password
#define STA_PASS2         ""

// -- Localization --------------------------------
  // If non selected the default en-GB will be used
 #undef MY_LANGUAGE
#define MY_LANGUAGE	de_DE	// German in Germany

// #undef FIRMWARE_IR	// IR Bringt Fehler bei Aktivierung im Compilerlauf !
//#define FIRMWARE_IR	// Create tasmota-ir with IR full protocols activated, and many sensors disabled

#undef USE_IR_REMOTE	// lässt sich compilieren aber noch ungetestet.
#define USE_IR_REMOTE	// Send IR remote commands using library IRremoteESP8266 and ArduinoJson (+4k code, 0k3 mem, 48 iram)
#undef USE_IR_RECEIVE
#define USE_IR_RECEIVE	// Support for IR receiver (+5k5 code, 264 iram)

// -- One wire Temperature sensors ----------------------------
#undef USE_DS18x20
#define USE_DS18x20		// Add support for DS18x20 sensors with id sort, single scan and read retry (+2k6 code)
#undef W1_PARASITE_POWER
#define W1_PARASITE_POWER	// Optimize for parasite powered sensors
#undef DS18x20_USE_ID_ALIAS
#define DS18x20_USE_ID_ALIAS	// Add support aliasing for DS18x20 sensors. See comments in xsns_05 files (+0k5 code)

//#ifdef USE_I2C		// Moegliche I2S Geraete Treiber siehe my_user_config.h
// Treiber welche zu deaktivieren sind
#undef USE_VEML6070
//#define USE_VEML6070			//I2CDriver12 0 [I2cDriver12] Enable VEML6070 sensor (I2C addresses 0x38 and 0x39) (+1k5 code)
#undef USE_VEML6070_RSET 
//#define USE_VEML6070_RSET 270000	// VEML6070, Rset in Ohm used on PCB board, default 270K = 270000ohm, range for this sensor: 220K ... 1Meg
#undef USE_VEML6070_SHOW_RAW
//#define USE_VEML6070_SHOW_RAW		// VEML6070, shows the raw value of UV-A
#undef USE_APDS9960
//#define USE_APDS9960			// [I2cDriver21] Enable APDS9960 Proximity Sensor (I2C address 0x39). Disables SHT and VEML6070 (+4k7 code)
#undef USE_APDS9960_GESTURE
//#define USE_APDS9960_GESTURE		// Enable APDS9960 Gesture feature (+2k code)
#undef USE_APDS9960_PROXIMITY
//#define USE_APDS9960_PROXIMITY	// Enable APDS9960 Proximity feature (>50 code)
#undef USE_APDS9960_COLOR
//#define USE_APDS9960_COLOR		// Enable APDS9960 Color feature (+0.8k code)
#undef USE_APDS9960_STARTMODE
//#define USE_APDS9960_STARTMODE 0	// Default to enable Gesture mode
#undef USE_ADE7953
//#define USE_ADE7953			// [I2cDriver7] Enable ADE7953 Energy monitor as used on Shelly 2.5 (I2C address 0x38) (+1k5)

#undef RTC_NTP_SERVER
#define RTC_NTP_SERVER		// Enable NTP-RTC Server Service by Command "RtcNtpServer 1" also without RTC Chip DS3231
#undef USE_RTC_CHIPS
#define USE_RTC_CHIPS		// Enable RTC chip support and NTP server - Select only one #undef USE_DS3231
#undef USE_DS3231
#define USE_DS3231		// I2C Support für DS1307 oder DS3231 und NTP-Server Funktion (I2C addresse 0x68)
#undef DS3231_ENABLE_TEMP
#define DS3231_ENABLE_TEMP	// In DS3231 driver, enable the internal temperature sensor

#undef USE_BMP
#define USE_BMP		// [I2cDriver10] Enable BMP085/BMP180/BMP280/BME280 sensors (ab 8.-€ Cn RH/T/P I2C Sensor addresses 0x76 and 0x77) (+4k4 code)
#undef USE_BME68X
//#define USE_BME68X	// Enable support for BME680/BME688 sensor using Bosch BME68x library (ab 11.-€ Cn RH/T/P/IAQ I2C Sensor +6k9 code)
#undef USE_AHT2x
#define USE_AHT2x	//I2CDriver43 1 [I2cDriver43] Enable AHT20/AM2301B instead of AHT1x humidity and temperature sensor (ab 1.70€ Cn RH/T 2-5,5V I2C address 0x38) (+0k8 code)

#undef USE_INA219
//#define USE_INA219	// [I2cDriver14] Enable INA219 (I2C address 0x40, 0x41 0x44 or 0x45) Low voltage and current sensor (+1k code)
//#define INA219_SHUNT_RESISTOR (0.100)	// 0.1 Ohm default shunt resistor, can be overriden in user_config_override or using Sensor13

// MCP23008/MCP23017 8/16Bit I2C zu TTL IO Adapter 
// 00:00:00.085 TFS: File 'mcp23x.dat' not found wird in Logfile vermekt. evtl. Configfile
#undef USE_MCP230xx
#define USE_MCP230xx		// [I2cDriver22] Enable MCP23008/MCP23017 - Must define I2C Address in #define USE_MCP230xx_ADDR below - range 0x20 - 0x27 (+5k1 code)
#undef USE_MCP230xx_ADDR
#define USE_MCP230xx_ADDR 0x20	// Enable MCP23008/MCP23017 I2C Address to use (Must be within range 0x20 through 0x26 - set according to your wired setup)
#undef USE_MCP230xx_OUTPUT 
#define USE_MCP230xx_OUTPUT	// Enable MCP23008/MCP23017 OUTPUT support through sensor29 commands (+2k2 code)
#undef USE_MCP230xx_DISPLAYOUTPUT
#define USE_MCP230xx_DISPLAYOUTPUT	// Enable MCP23008/MCP23017 to display state of OUTPUT pins on Web UI (+0k2 code)

// PCF8574 8Bit I2C zu TTL IO Adapter
#define USE_PCF8574			// [I2cDriver2] Enable PCF8574 I/O Expander (I2C addresses 0x20 - 0x26 and 0x39 - 0x3F) (+2k1 code)
#define USE_PCF8574_MODE2		// Enable Mode2 virtual relays/buttons/switches (+2k3 code)
#define USE_PCF8574_SENSOR		// Enable Mode1 inputs and outputs in SENSOR message (+0k2 code)
#define USE_PCF8574_DISPLAYINPUT	// Enable Mode1 inputs display in Web page (+0k2 code)
#define USE_PCF8574_MQTTINPUT		// Enable Mode1 MQTT message & rule process on input change detection : stat/%topic%/PCF8574_INP = {"Time":"2021-03-07T16:19:23+01:00","PCF8574-1_INP":{"D1":1}} (+0k5 code)
#define PCF8574_ADDR1 0x20		// First address to search for PCF8574
#define PCF8574_ADDR1_COUNT 7		// Number of addresses to search for PCF8574 - Default to 0x20 to 0x26
#define PCF8574_ADDR2 0x39		// First address to search for PCF8574A
#define PCF8574_ADDR2_COUNT 6		// Number of addresses to search for PCF8574A - Default to 0x39 to 0x3E

// PCA9685 16Bit I2C zu TTL PWM IO Adapter z.B.Servomotoren oder LED
#undef USE_PCA9685
#define USE_PCA9685			// [I2cDriver1] Enable PCA9685 I2C HW PWM Driver - Must define I2C Address in #define USE_PCA9685_ADDR below - range 0x40 - 0x47 (+1k4 code)
#undef USE_PCA9685_ADDR
#define USE_PCA9685_ADDR 0x40		// Enable PCA9685 I2C Address to use (Must be within range 0x40 through 0x47 - set according to your wired setup)
#undef USE_PCA9685_FREQ 
#define USE_PCA9685_FREQ 50		// Define default PWM frequency in Hz to be used (must be within 24 to 1526) - If other value is used, it will rever to 50Hz

// PCA9685 Befehle für die Eingabekonsole
// driver15 pwmf,frequency	// where frequency is the PWM frequency from 24 to 1526 in Hz
// driver15 pwm,pin,pwmvalue	// where pin = LED pin 0 through 15 and pwmvalue is the pulse width between 0 and 4096
// driver15 pwm,pin,ON		// Fully turn a specific pin/LED ON
// driver15 pwm,pin,OFF		// Fully turn a specific pin/LED OFF
// driver15 reset		// Reset to power-up settings - i.e. F=50hz and all pins in OFF state
// driver15 status		// Will return a JSON string containing all the current settings / parameters
//#endif  // USE_I2C

//#ifdef ESP32
// -- ESP-NOW Nur ESP32 ! -------------------------
#undef USE_TASMESH
#define USE_TASMESH		// Enable Tasmota Mesh using ESP-NOW (+11k code)

// WiFI Range extender Nur ESP32 ! Commands "RgxClients" "RgxPort"
// #ifndef USE_WIFI_RANGE_EXTENDER
#undef USE_WIFI_RANGE_EXTENDER
#define USE_WIFI_RANGE_EXTENDER
// #endif

// -- Matter Protocol -freischalten mit SetOption151 1 -auch SO151 1 ---------
#undef USE_MATTER_DEVICE
#define USE_MATTER_DEVICE
// Enable all the crypto required by Matter
#undef  USE_BERRY_CRYPTO_EC_P256
#define USE_BERRY_CRYPTO_EC_P256
#undef  USE_BERRY_CRYPTO_HMAC_SHA256
#define USE_BERRY_CRYPTO_HMAC_SHA256
#undef  USE_BERRY_CRYPTO_HKDF_SHA256
#define USE_BERRY_CRYPTO_HKDF_SHA256
#undef  USE_BERRY_CRYPTO_AES_CCM
#define USE_BERRY_CRYPTO_AES_CCM
#undef  USE_BERRY_CRYPTO_AES_CTR
#define USE_BERRY_CRYPTO_AES_CTR
#undef  USE_BERRY_CRYPTO_PBKDF2_HMAC_SHA256
#define USE_BERRY_CRYPTO_PBKDF2_HMAC_SHA256
#undef  USE_BERRY_CRYPTO_SPAKE2P_MATTER
#define USE_BERRY_CRYPTO_SPAKE2P_MATTER

//#endif  // ESP32

// -- KNX IP Protocol -----------------------------
 #undef USE_KNX
#define USE_KNX			// Enable KNX IP Protocol Support (+9.4k code, +3k7 mem)
 #undef USE_KNX_WEB_MENU
#define USE_KNX_WEB_MENU	// Enable KNX WEB MENU (+8.3k code, +144 mem)

// DOMOTICZ
 #undef USE_DOMOTICZ
#define USE_DOMOTICZ

// -- SML-Zaehler Konfiguration --------------------
 #undef USE_RULES

 #undef USE_SCRIPT
#define USE_SCRIPT

 #undef SCRIPT_POWER_SECTION
#define SCRIPT_POWER_SECTION	// >P section execute on power changes

 #undef SUPPORT_MQTT_EVENT
#define SUPPORT_MQTT_EVENT	// enables suppoer for subscribe an unsubscribe

 #undef USE_SENDMAIL
#define USE_SENDMAIL		// >m section for sending e-Mail

 #undef USE_SML_M
#define USE_SML_M		// >M section

 #undef USE_BUTTON_EVENT
#define USE_BUTTON_EVENT	// >b Section detect button state changes

 #undef USE_SCRIPT_WEB_DISPLAY
#define USE_SCRIPT_WEB_DISPLAY

 #undef USE_SCRIPT_JSON_EXPORT
#define USE_SCRIPT_JSON_EXPORT	// >J section JSON payload

 #undef USE_SCRIPT_STATUS
#define USE_SCRIPT_STATUS	// >U section receive JSON payloads

 #undef USE_SCRIPT_GLOBVARS
#define USE_SCRIPT_GLOBVARS	// >G enables global variables

//https://github.com/arendst/Tasmota/issues/7021
 #undef USE_EXPRESSION
#define USE_EXPRESSION

 #undef SUPPORT_IF_STATEMENT
#define SUPPORT_IF_STATEMENT

// Speichern auf FAT System (ESP32)intern als auch auf SD-Karte
// Dateien, welche vorkommen können (IP durch eigene ersetzen)
// http://192.168.190.21/ufs/AMT681-2023.csv //byHofeBY dies ist meine Logdatei der SML-Zählerwerte
// http://192.168.190.21/ufs/AMT681-2023B.csv  //byHofeBY dies ist meine Logdatei der SML-Zählerwerte
// http://192.168.190.21/ufs/_matter_device.json
// http://192.168.190.21/ufs/_persist.json //hier werden Variablen mit Inhalt gespeichert
// http://192.168.190.21/ufs/script.txt
// http://192.168.190.21/ufs/.drvset003
// http://192.168.190.21/ufs/.settings.lkg
// http://192.168.190.21/ufs/.settings
// Info aus dem Start des ESP32 in Konsole Log:
// http://192.168.190.21/ufs/.autoconf
// http://192.168.190.21/ufs/preinit.be
// http://192.168.190.21/ufs/autoexec.be
// http://192.168.190.21/ufs/autoexec.bat

 #undef USE_UFILESYS
#define USE_UFILESYS
 #undef GUI_TRASH_FILE
#define GUI_TRASH_FILE
 #undef USE_SDCARD
#define USE_SDCARD

 #undef USE_PING
#define USE_PING

// -- Variablen in Konsole erlauben ----------------
 #undef SML_REPLACE_VARS
#define SML_REPLACE_VARS

 #undef USE_SML_SCRIPT_CMD
#define USE_SML_SCRIPT_CMD

 #undef SML_MAX_VARS
#define SML_MAX_VARS 60

 #undef MAXSVARS
#define MAXSVARS 5

 #undef MAXFILT
#define MAXFILT 5

// Influx-DB webinterface
 #undef USE_WEBCLIENT_HTTPS
#define USE_WEBCLIENT_HTTPS

// // erzeugt Fehler
// // #undef USE_SCRIPT_GLOBVARS
// //define USE_SCRIPT_GLOBVARS

// // erzeugt Fehler
// // #undef SCRIPT_FULL_WEBPAGE
// //#define SCRIPT_FULL_WEBPAGE

// // erzeugt Fehler
// // #undef USE_WEBSEND_RESPONSE
// //define USE_WEBSEND_RESPONSE

// // erzeugt Fehler
// // #undef USE_ENERGY_SENSOR
// //define USE_ENERGY_SENSOR

// // erzeugt Fehler
// // #undef USE_SCRIPT_SUB_COMMAND
// //define USE_SCRIPT_SUB_COMMAND

 #undef SUPPORT_MQTT_EVENT
#define SUPPORT_MQTT_EVENT
 #undef MQTT_EVENT_MSIZE
#define MQTT_EVENT_MSIZE 256 // (default is 256)
 #undef MQTT_EVENT_JSIZE
#define MQTT_EVENT_JSIZE 400 // (default is 400) 

// Script Basisinhalt
//>D
//script error must start with >D

//byHofeBY oberhalb eingetragen

#endif  // _USER_CONFIG_OVERRIDE_H_
