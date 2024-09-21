#include <WiFiNINA.h>
#include <utility/wifi_drv.h>
#include <ArduinoRS485.h>  // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <RTCZero.h>
#include <time.h>
#include <TimeLib.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <DS18B20.h>

#include <SPI.h>
#include <SD.h>

#include <WiFiUdp.h>
#include <ArduinoMDNS.h>

#include <Wire.h>
#include <DS3231.h>

#include <hd44780.h>                        // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h>  // i2c expander i/o class header

#include <INA226.h>
#include <Adafruit_MCP23X17.h>

//#include <MemoryFree.h>
//#include <pgmStrToRAM.h>
//#include <Arduino_MKRMEM.h>


// Shunt : 75 mV and 50 A, 0.0015 ohm, 1.5 mohm
//IFS = 81.92mV/1.5mOhm = 54.6133 A
// Resolution = 54.6133/32768=0.001666: 1.67 mA


#define TIMER_INTERRUPT_DEBUG 0
#define _TIMERINTERRUPT_LOGLEVEL_ 0
#define USING_TIMER_TCC true
#include "SAMDTimerInterrupt.h"
#include "SAMD_ISR_Timer.h"

#define HW_TIMER_INTERVAL_MS 10

SAMDTimer ITimer(TIMER_TCC);
SAMD_ISR_Timer ISR_Timer;

#define TIMER_INTERVAL_60s 60000L
#define TIMER_INTERVAL_10s 10000L

#define BUTTONPRESS_LED 0
#define SPARE_LED 1
#define HTTP_LED 2
#define MODBUS_LED 3
#define SD_LED 6
#define RTC_EXT_LED 7
#define SHUNT1_LED 8
#define SHUNT2_LED 9
#define NTP_LED 10
#define WiFi_LED 11

const int chipSelect = 4;

#define SECRET_SSID "SKYF97BA"


#define RTCAddress 0x68
#define mcpAddress 0x24
#define lcdAddress 0x27
#define INA0Address 0x40
#define INA1Address 0x45

time_t ntp;

bool charging;
bool wifiBegun(false);
bool SDAvailable = false;
bool rtc_ext_exist;
bool h12Flag, pmFlag;
bool century = false;
bool rtc_ext_time_good;
bool screen_0, screen_1, screen_2;
bool modbus_client_status;
bool shunt1_exist, shunt2_exist;

char c, d;
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
char html[3000];
char config[500];
char Dayfilename[] = "00000000.CSV";
char Monthfilename[] = "00000000.CSV";
char configfilename[] = "config.csv";

byte busStatus, error, address;

const byte modbus_status = 2, web_connect = 3;
const byte green = 0, red = 1, blue = 2, white = 3;

int numDevices, nDevices;
int DevCount = 1;
int comm_count;
int filesize, a, b;
int ntp_try, WiFi_try;
int NTP_success, WiFi_success;
int setup_oncerun = 0;
int keyIndex = 0;
int wifi_status = WL_IDLE_STATUS;
int day_change_flag, month_change_flag, hour_change_flag;
int LED;
int cal1, cal2, cal3, cal4, cal5, cal6, cal7;
int lcd_status;
int delay_line = 750;
int delay_screen = 750;
int left_count, right_count, up_count, down_count, back_count, enter_count, line_count = 0, tab_count = 0, page_count = 0, cursor_count = 0, current_page, last_page = 1, current_tab, last_tab;

const int LCD_COLS = 20;
const int LCD_ROWS = 4;
uint current_year, current_minute, current_hour, current_second, current_day, current_month, previous_hour, previous_month, previous_day;

float current, bat_voltage_1, bat_voltage_2, power, energy, Whour, Ahour;
float lipo_bat_voltage, rtc_bat_voltage, gas_sensor_voltage;
float voltage_scale = 1, current_scale = 1;
float batTemp1, batTemp2, batTemp3, batTemp4;

long rssi;
unsigned long StartTime, BootTime, EndTime, ElapsedTime, LoopTime, CurrentSampleTime, PreviousSampleTime, ElapsedSampleTime;

volatile unsigned long button_time = 0, last_button_time = 0;
volatile int adc_count, print_ISR, button_delay = 500;
volatile int up, down, left, right, enter, home, back;
volatile int raw0, raw1, raw2, raw3, raw4, raw5, raw6, raw7;
volatile bool adc_capture;

String str;

Adafruit_MCP23X17 mcp;

OneWire oneWire_1(4);
OneWire oneWire_2(3);
OneWire oneWire_3(2);
OneWire oneWire_4(A4);


DallasTemperature Bat1Sensor(&oneWire_1);
DallasTemperature Bat2Sensor(&oneWire_2);
DallasTemperature Bat3Sensor(&oneWire_3);
DallasTemperature Bat4Sensor(&oneWire_4);

DeviceAddress Bat1_Address, Bat2_Address, Bat3_Address, Bat4_Address;

hd44780_I2Cexp lcd(lcdAddress);  // declare lcd object: auto locate & auto config expander chip

INA226 INA0(INA0Address);
INA226 INA1(INA1Address);

WiFiServer wifi_Modbus_Server(502);
WiFiServer wifi_web_server(80);
ModbusTCPServer modbusTCPServer;

RTCZero rtc_int;
DS3231 rtc_ext;

File DayFile, MonthFile, ConfigFile;
SDLib::File webPage;
SDLib::File root;
String HTTP_req;  // stores the HTTP request

WiFiUDP udp;
MDNS mdns(udp);

void setup() {
  setup_oncerun = 1;
  StartTime = millis();
  Serial.begin(115200);

  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  adc_count = 1;
  HTTP_req = "";
  rtc_ext_exist = 1;
  LED = 1;

  delay(2500);  // Serial port

  Wire.begin();
  rtc_int.begin();  // initialize RTC

  bootScreen();
  splashScreen();
  setup2time();
  setup3temp();
  setup4INA226();
  setup5analog();
  setup6SD();

  setup7WiFi();
  if (wifi_status = WL_CONNECTED) {
    setup8NTP();
    setup9Modbus();
  }

  attachInterrupt(digitalPinToInterrupt(A1), LeftKey, RISING);
  attachInterrupt(digitalPinToInterrupt(A2), UpKey, RISING);
  attachInterrupt(digitalPinToInterrupt(0), HomeKey, FALLING);
  attachInterrupt(digitalPinToInterrupt(1), BackKey, FALLING);
  attachInterrupt(digitalPinToInterrupt(5), RightKey, FALLING);
  attachInterrupt(digitalPinToInterrupt(6), DownKey, FALLING);
  //attachInterrupt(digitalPinToInterrupt(7), EnterKey, FALLING);

  setup10ISR();

  WiFi.setHostname("UPS_Monitor");
  mdns.begin(WiFi.localIP(), "UPSmonitor");
  mdns.addServiceRecord("Arduino mDNS Webserver Example._http", 80, MDNSServiceTCP);

  EndTime = millis();
  BootTime = EndTime - StartTime;

  CurrentSampleTime = PreviousSampleTime = ElapsedSampleTime = millis();

  if (SDAvailable) {
    mcp.digitalWrite(SD_LED, HIGH);
  } else {
    mcp.digitalWrite(SD_LED, LOW);
  }

  if (rtc_ext_exist) {
    mcp.digitalWrite(RTC_EXT_LED, HIGH);
  } else {
    mcp.digitalWrite(RTC_EXT_LED, LOW);
  }

  if (shunt1_exist) {
    mcp.digitalWrite(SHUNT1_LED, HIGH);
  } else {
    mcp.digitalWrite(SHUNT1_LED, LOW);
  }

  if (shunt2_exist) {
    mcp.digitalWrite(SHUNT2_LED, HIGH);
  } else {
    mcp.digitalWrite(SHUNT2_LED, LOW);
  }

  int x = INA0.setMaxCurrentShunt(54.6133, 0.0015);
  INA0.setBusVoltageConversionTime(0);
  Serial.println("normalized = true (default)");
  Serial.println(x);
  Serial.print("LSB:\t");
  Serial.println(INA0.getCurrentLSB(), 10);
  Serial.print("LSB_uA:\t");
  Serial.println(INA0.getCurrentLSB_uA(), 3);
  Serial.print("shunt:\t");
  Serial.println(INA0.getShunt(), 3);
  Serial.print("maxCur:\t");
  Serial.println(INA0.getMaxCurrent(), 3);
  Serial.println();

  x = INA1.setMaxCurrentShunt(54.6133, 0.0015);
  INA1.setBusVoltageConversionTime(0);
  Serial.println("normalized = false");
  Serial.println(x);
  Serial.print("LSB:\t");
  Serial.println(INA1.getCurrentLSB(), 10);
  Serial.print("LSB_uA:\t");
  Serial.println(INA1.getCurrentLSB_uA(), 3);
  Serial.print("shunt:\t");
  Serial.println(INA1.getShunt(), 3);
  Serial.print("maxCur:\t");
  Serial.println(INA1.getMaxCurrent(), 3);
  Serial.println();

  str = "";

  if (Serial) {
    Serial.println(str + "Setup completed in: " + (BootTime) + " ms");
  }
  if (lcd_status) {
    clearScreen();
    lcd.setCursor(0, 0);
    lcd.print("Setup Completed in");
    lcd.print(BootTime / 1000);
    lcd.print("s");
  }
  setup_oncerun = 0;
}

void loop() {

  StartTime = millis();

  main_loop_led();
  main_loop_keypress();
  main_loop_progress();
  main_loop_rollover();

  if (Bat1Sensor.isConversionComplete()) {
    batTemp1 = Bat1Sensor.getTempCByIndex(0);
  }

  if (Bat2Sensor.isConversionComplete()) {
    batTemp2 = Bat2Sensor.getTempCByIndex(0);
  }

  if (Bat3Sensor.isConversionComplete()) {
    batTemp3 = Bat3Sensor.getTempCByIndex(0);
  }

  if (Bat4Sensor.isConversionComplete()) {
    batTemp4 = Bat4Sensor.getTempCByIndex(0);
  }

  /*
  Bat1Sensor.requestTemperatures();
  Bat2Sensor.requestTemperatures();
  Bat3Sensor.requestTemperatures();
  Bat4Sensor.requestTemperatures();
  */

  if (line_count > 3) {
    if (line_count <= 7) {
      page_count = 1;
    } else {
      page_count = 2;
    }
  } else {
    page_count = 0;
  }

  if (tab_count == 0) {
    if ((last_page != page_count) || (screen_0)) {
      main_loop_screen_0(page_count, line_count);
      last_page = page_count;
      screen_0 = false;
    }

    lcd_call(0, 0, " ", 0);
    lcd_call(0, 1, " ", 0);
    lcd_call(0, 2, " ", 0);
    lcd_call(0, 3, " ", 0);
    lcd_call(0, (line_count - page_count * 4), ">", 0);
  }

  if (tab_count == 1) {
    //if ((last_tab != tab_count)) {
    //  Serial.println("new tab");
    main_loop_screen_1(line_count);
    //   last_tab = tab_count;
    // }
  }

  if (tab_count == 2) {
    //if ((last_tab != tab_count)) {
    //  Serial.println("new tab");
    main_loop_screen_2(line_count);
    //   last_tab = tab_count;
    // }
  }
  /*
  sensors.requestTemperatures();  // Send the command to get temperatures

  // Loop through each device, print out temperature data
  for (int i = 0; i < numDevices; i++) {
    // Search the wire for address
    if (sensors.getAddress(tempDeviceAddress, i)) {
      modbusTCPServer.inputRegisterWrite((20 + i), int(sensors.getTempC(tempDeviceAddress) * 10));
    }
  }*/

  if (wifi_status != WL_CONNECTED) {
    if (Serial) {
      Serial.print("/");
    }
    setup7WiFi();
  }

  mdns.run();

  if (adc_capture == 1) {
    ElapsedSampleTime = (CurrentSampleTime - PreviousSampleTime) / 1000;
    current = raw0 * 1 * current_scale;
    bat_voltage_1 = raw3 * 1 * current_scale;
    bat_voltage_2 = raw4 * 1 * voltage_scale;
    power = (bat_voltage_1 + bat_voltage_2) * current;
    /* if (Serial) {
      str = "";
      Serial.println(".");
      Serial.println(str + "Battery 1 " + " : " + bat_voltage_1 + "Battery 2 " + " : " + bat_voltage_2 + " | " + "Current" + ":" + current + " | " + "Power" + ":" + power);
      str = "";
      Serial.println(str + "ADC Count" + ":" + adc_count + ":" + "Communication Count" + ":" + comm_count);
    }*/

    modbusTCPServer.holdingRegisterWrite(8, adc_count);
    modbusTCPServer.inputRegisterWrite(0, int(current));
    modbusTCPServer.inputRegisterWrite(1, int(bat_voltage_1));
    modbusTCPServer.inputRegisterWrite(2, int(bat_voltage_2));
    modbusTCPServer.inputRegisterWrite(3, int(bat_voltage_1 + bat_voltage_2));
    modbusTCPServer.inputRegisterWrite(4, int(power));
    modbusTCPServer.inputRegisterWrite(5, int(energy));

    adc_capture = 0;
  }

  modbusTCPServer.holdingRegisterWrite(0, rtc_int.getYear());
  modbusTCPServer.holdingRegisterWrite(1, rtc_int.getMonth());
  modbusTCPServer.holdingRegisterWrite(2, rtc_int.getDay());
  modbusTCPServer.holdingRegisterWrite(3, rtc_int.getHours());
  modbusTCPServer.holdingRegisterWrite(4, rtc_int.getMinutes());
  modbusTCPServer.holdingRegisterWrite(5, rtc_int.getSeconds());
  modbusTCPServer.holdingRegisterWrite(6, abs(int(WiFi.RSSI())));

  modbusTCPServer.holdingRegisterWrite(10, rtc_int.getYear());
  modbusTCPServer.holdingRegisterWrite(11, rtc_int.getMonth());
  modbusTCPServer.holdingRegisterWrite(12, rtc_int.getDay());
  modbusTCPServer.holdingRegisterWrite(13, rtc_int.getHours());
  modbusTCPServer.holdingRegisterWrite(14, rtc_int.getMinutes());
  modbusTCPServer.holdingRegisterWrite(15, rtc_int.getSeconds());

  WiFiClient modbus_client = wifi_Modbus_Server.available();

  if (modbus_client) {
    modbusTCPServer.accept(modbus_client);

    modbus_client_status = true;
    if (modbus_client.connected()) {
      // poll for Modbus TCP requests, while client connected
      if (modbus_client.available()) {
        modbusTCPServer.poll();
        comm_count++;
        modbusTCPServer.holdingRegisterWrite(7, comm_count);
        if (Serial) {
          Serial.print("+");
        }
        mcp.digitalWrite(MODBUS_LED, HIGH);
        if (comm_count > 10000) {
          if (Serial) {
            Serial.println("count reset to 0 after 10000");
            comm_count = 0;
          }
        }
      }
    }
  } else {
    mcp.digitalWrite(MODBUS_LED, LOW);
    modbus_client_status = false;
  }

  WiFiClient web_client = wifi_web_server.available();

  if (web_client) {
    // an HTTP request ends with a blank line
    if (Serial) {
      Serial.print("O");
    }
    mcp.digitalWrite(HTTP_LED, HIGH);
    bool current_line_is_blank = true;
    while (web_client.connected()) {

      if (web_client.available()) {

        c = web_client.read();
        // if we've gotten to the end of the line (received a newline
        // character) and the line is blank, the HTTP request has ended,
        // so we can send a reply

        // if (HTTP_req.length() < 120)
        // HTTP_req += c;

        if (c == '\n' && current_line_is_blank) {
          // send a standard HTTP response header
          web_client.println("HTTP/1.1 200 OK");
          web_client.println("Content-Type: text/html");
          web_client.println("Connection: close");
          // web_client.println("Refresh: 5");
          web_client.println();
          //webPage = SD.open("index.htm");
          //if (webPage) {
          for (a = 0; a <= filesize; a++) {
            d = html[a];
            web_client.write(d);  // send web page to client
          }
          webPage.close();
          //}
          break;
        }
        if (c == '\n') {
          // we're starting a new line
          current_line_is_blank = true;
        } else if (c != '\r') {
          // we've gotten a character on the current line
          current_line_is_blank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(50);
    web_client.stop();
    //HTTP_req = "";
    // if (Serial) {
    //   Serial.println("q");
    // }
  } else {
    mcp.digitalWrite(HTTP_LED, LOW);
  }

  EndTime = millis();
  LoopTime = EndTime - StartTime;
  modbusTCPServer.holdingRegisterWrite(8, int(LoopTime));
}

void TimerHandler(void) {
  ISR_Timer.run();
}

void Print_Routine() {
  print_ISR = 1;
  //WiFiDrv::analogWrite(25, 0);
  //WiFiDrv::analogWrite(26, 0);
  //WiFiDrv::analogWrite(27, 0);
}

void ADC_Routine() {
  /*
  switch (adc_count) {
    case 1:
      WiFiDrv::analogWrite(25, 0);
      WiFiDrv::analogWrite(26, 0);
      WiFiDrv::analogWrite(27, 50);
      break;
    case 2:
      WiFiDrv::analogWrite(25, 50);
      WiFiDrv::analogWrite(26, 0);
      WiFiDrv::analogWrite(27, 0);
      break;
    case 3:
      WiFiDrv::analogWrite(25, 0);
      WiFiDrv::analogWrite(26, 50);
      WiFiDrv::analogWrite(27, 0);
      break;
  }
  adc_count++;
  if (adc_count > 3) {
    adc_count = 1;
  }*/
  PreviousSampleTime = CurrentSampleTime;
  CurrentSampleTime = millis();
  raw0 = analogRead(A0);
  raw3 = analogRead(A3);
  raw4 = analogRead(A4);
  raw5 = analogRead(A5);
  raw6 = analogRead(A6);
  //raw7 = analogRead(A7);

  Bat1Sensor.requestTemperaturesByAddress(Bat1_Address);
  Bat2Sensor.requestTemperaturesByAddress(Bat2_Address);
  Bat3Sensor.requestTemperaturesByAddress(Bat3_Address);
  Bat4Sensor.requestTemperaturesByAddress(Bat4_Address);

  adc_capture = 1;
}

void printWifiStatus() {
  str = "";
  if (Serial) {
    Serial.println(str + "SSID: " + WiFi.SSID() + " | IP Address: " + WiFi.localIP() + " | Signal strength (RSSI): " + WiFi.RSSI() + " dBm");
  }
  IPAddress ip = WiFi.localIP();
  rssi = WiFi.RSSI();
  // colour_blink(green);
}

void colour_blink(byte colour) {

  switch (colour) {
    case green:
      WiFiDrv::analogWrite(25, 50);
      WiFiDrv::analogWrite(26, 0);
      WiFiDrv::analogWrite(27, 0);
      break;
    case red:
      WiFiDrv::analogWrite(25, 0);
      WiFiDrv::analogWrite(26, 50);
      WiFiDrv::analogWrite(27, 0);
      break;
    case blue:
      WiFiDrv::analogWrite(25, 0);
      WiFiDrv::analogWrite(26, 0);
      WiFiDrv::analogWrite(27, 50);
      break;
    case white:
      WiFiDrv::analogWrite(25, 10);
      WiFiDrv::analogWrite(26, 10);
      WiFiDrv::analogWrite(27, 10);
      break;
  }

  delay(50);
  WiFiDrv::analogWrite(25, 0);
  WiFiDrv::analogWrite(26, 0);
  WiFiDrv::analogWrite(27, 0);
  delay(50);
}

void holding_register_update() {
  modbusTCPServer.holdingRegisterWrite(0, rtc_int.getYear());
  modbusTCPServer.holdingRegisterWrite(1, rtc_int.getMonth());
  modbusTCPServer.holdingRegisterWrite(2, rtc_int.getDay());
  modbusTCPServer.holdingRegisterWrite(3, rtc_int.getHours());
  modbusTCPServer.holdingRegisterWrite(4, rtc_int.getMinutes());
  modbusTCPServer.holdingRegisterWrite(5, rtc_int.getSeconds());
  modbusTCPServer.holdingRegisterWrite(6, abs(int(WiFi.RSSI())));
  modbusTCPServer.holdingRegisterWrite(7, comm_count);
  modbusTCPServer.holdingRegisterWrite(8, adc_count);
}

void input_register_update() {
  modbusTCPServer.inputRegisterWrite(0, raw0);
  modbusTCPServer.inputRegisterWrite(1, raw1);
  modbusTCPServer.inputRegisterWrite(2, raw2);
  modbusTCPServer.inputRegisterWrite(3, raw3);
  modbusTCPServer.inputRegisterWrite(4, raw4);
  modbusTCPServer.inputRegisterWrite(5, raw5);
  modbusTCPServer.inputRegisterWrite(6, raw6);
  modbusTCPServer.inputRegisterWrite(7, raw7);
}

void coil_update() {
  modbusTCPServer.coilWrite(0, home);
  modbusTCPServer.coilWrite(1, up);
  modbusTCPServer.coilWrite(2, down);
  modbusTCPServer.coilWrite(3, left);
  modbusTCPServer.coilWrite(4, right);
  modbusTCPServer.coilWrite(5, enter);
  modbusTCPServer.coilWrite(6, back);
}

void printDirectory(File dir, int numTabs) {

  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      break;
    }

    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }

    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {

      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void ajaxRequest(WiFiClient web_client) {
  for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
    int sensorReading = analogRead(analogChannel);
    web_client.print("analog input ");
    web_client.print(analogChannel);
    web_client.print(" is ");
    web_client.print(sensorReading);
    web_client.println("<br />");
  }
}

void ledChangeStatus(WiFiClient web_client) {
  int state = digitalRead(1);
  Serial.println(state);
}

void getDayFileName() {

  Dayfilename[0] = ((rtc_int.getYear() + 2000) / 1000) % 10 + '0';  //To get 1st digit from year()
  Dayfilename[1] = (rtc_int.getYear() / 100) % 10 + '0';            //To get 2nd digit from year()
  Dayfilename[2] = (rtc_int.getYear() / 10) % 10 + '0';             //To get 3rd digit from year()
  Dayfilename[3] = rtc_int.getYear() % 10 + '0';                    //To get 4th digit from year()
  Dayfilename[4] = rtc_int.getMonth() / 10 + '0';                   //To get 1st digit from month()
  Dayfilename[5] = rtc_int.getMonth() % 10 + '0';                   //To get 2nd digit from month()
  Dayfilename[6] = rtc_int.getDay() / 10 + '0';                     //To get 1st digit from day()
  Dayfilename[7] = rtc_int.getDay() % 10 + '0';                     //To get 2nd digit from day()
}

void getMonthFileName() {

  Monthfilename[0] = ((rtc_int.getYear() + 2000) / 1000) % 10 + '0';  //To get 1st digit from year()
  Monthfilename[1] = (rtc_int.getYear() / 100) % 10 + '0';            //To get 2nd digit from year()
  Monthfilename[2] = (rtc_int.getYear() / 10) % 10 + '0';             //To get 3rd digit from year()
  Monthfilename[3] = rtc_int.getYear() % 10 + '0';                    //To get 4th digit from year()
  Monthfilename[4] = rtc_int.getMonth() / 10 + '0';                   //To get 1st digit from month()
  Monthfilename[5] = rtc_int.getMonth() % 10 + '0';                   //To get 2nd digit from month()
  Monthfilename[6] = 'A';                                             //To get 1st digit from day()
  Monthfilename[7] = 'B';                                             //To get 2nd digit from day()
}

void createDayFileName() {

  if (SD.exists(Dayfilename)) {
    if (Serial) {
      str = "";
      Serial.println(str + "  Found Daily log File" + "[" + Dayfilename + "]");
    }
  }

  else {
    if (Serial) {
      Serial.print("Day Log File doesn't exist.");
      Serial.print("Creating new file..");
      Serial.println(Dayfilename);
    }
    updateTime();
    SdFile::dateTimeCallback(dateTime);
    DayFile = SD.open(Dayfilename, FILE_WRITE);
    DayFile.close();
  }
}

void createMonthFileName() {

  if (SD.exists(Monthfilename)) {
    str = "";
    if (Serial) {
      Serial.print(str + "Found Monthly Log File" + "[" + Monthfilename + "]");
    }
  }

  else {
    if (Serial) {
      Serial.print("Monthly config File doesn't exist.");
      Serial.print("Creating new file...");
      Serial.println(Monthfilename);
    }
    updateTime();
    SdFile::dateTimeCallback(dateTime);
    MonthFile = SD.open(Monthfilename, FILE_WRITE);
    MonthFile.close();
  }
}

void dateTime(uint16_t* date, uint16_t* time) {
  *date = FAT_DATE(current_year, current_month, current_day);
  *time = FAT_TIME(current_hour, current_minute, current_second);
}

void updateTime() {

  current_year = 2000 + rtc_int.getYear();
  current_month = rtc_int.getMonth();
  current_day = rtc_int.getDay();
  current_hour = rtc_int.getHours();
  current_minute = rtc_int.getMinutes();
  current_second = rtc_int.getSeconds();
}

void clearLine(int line) {
  lcd.setCursor(0, line);
  lcd.print("                    ");
}

void clearScreen() {
  clearLine(0);
  clearLine(1);
  clearLine(2);
  clearLine(3);
}

void bootScreen() {

  lcd_status = lcd.begin(LCD_COLS, LCD_ROWS);
  if (lcd_status)  // non zero status means it was unsuccesful
  {
    hd44780::fatalError(lcd_status);  // does not return
    if (Serial) {
      Serial.println("LCD Display Not Present ");
    }
  } else {
    if (Serial) {
      Serial.println("LCD Display Present ");
      Serial.println("Testing Sequence LED");
    }

    lcd.clear();
    lcd_call(0, 0, "********************", 0);
    lcd_call(0, 1, "********************", 0);
    lcd_call(0, 2, "********************", 0);
    lcd_call(0, 3, "********************", delay_screen);

    if (!mcp.begin_I2C(mcpAddress)) {
      if (Serial) {
        Serial.println("Error.");
      }
    } else {
      for (int i = 0; i < 16; i++) {
        mcp.pinMode(i, OUTPUT);
        mcp.digitalWrite(i, HIGH);
      }

      delay(delay_screen);

      lcd.noBacklight();
      for (int i = 0; i < 16; i++) {
        mcp.digitalWrite(i, LOW);
      }
      delay(delay_screen);
      lcd.backlight();
    }
  }
}

void splashScreen() {
  if (Serial) {
    Serial.println("UPS Monitoring Boot.. ");
    Serial.println("Starting Setup...");
    Serial.print(F("Arduino MKR 1010: CPU Frequency = "));
    Serial.print(F_CPU / 1000000);
    Serial.println(F(" MHz"));
  }

  // initalization was successful, the backlight should be on now
  if (!lcd_status) {
    lcd.clear();
    lcd_call(0, 0, "UPS Monitor Booting", delay_screen);
    /* lcd_call(0, 1, "Arduino MKR 1010", 0);
    lcd_call(0, 2, "CPU Freq = ", 0);
    lcd.print(F_CPU / 1000000);
    lcd.print(F(" MHz"));
    lcd_call(0, 3, "Starting Setup..", delay_screen);
    lcd.clear();*/
  }
}

void setup2time() {
  /*
  if (!lcd_status) {
    lcd.clear();
    lcd_call(0, 0, "********************", 0);
    lcd_call(0, 1, "   Setup...2 of 11  ", 0);
    lcd_call(0, 2, "   Setting Time     ", 0);
    lcd_call(0, 3, "********************", delay_screen);
    lcd.clear();
  }*/

  if (!lcd_status) {
    lcd_call(0, 1, "External RTC", 0);
    lcd.setCursor(17, 1);
    lcd.print((char)91);
    lcd.setCursor(19, 1);
    lcd.print((char)93);
    delay(delay_screen);
  }

  Wire.beginTransmission(RTCAddress);
  //Wire.write(0x0E); //pointing Control Register
  //Wire.write(0x80);
  byte busStatus = Wire.endTransmission();
  if (busStatus == 0) {
    if (Serial) {
      Serial.println("DS3231 Clock Module Found...!");
    }

    if (!lcd_status) {
      lcd.setCursor(18, 1);
      lcd.print((char)255);
      delay(delay_screen);
    }
    rtc_ext_exist = 1;
  } else {
    if (Serial) {
      Serial.println("DS3231 Clock Module not Present...!");
    }
    //clearLine(0);
    if (!lcd_status) {
      //  lcd.Setcursor(18, 0);
      //  lcd.print((char));
      //  lcd_call(0, 1, "Ext RTC not Found", delay_screen);
    }
    rtc_ext_exist = 0;
  }


  if (!lcd_status) {
    lcd_call(0, 2, "External RTC Sync", 0);
    lcd.setCursor(17, 2);
    lcd.print((char)91);
    lcd.setCursor(19, 2);
    lcd.print((char)93);
    delay(delay_screen);
  }

  if (rtc_ext_exist) {
    if (Serial) {
      Serial.print("Reading from external RTC...");
    }
    /*
    if (!lcd_status) {
      lcd_call(0, 2, "Ext RTC Sync", delay_screen);
    }*/

    if (!rtc_ext.getYear() == 2000) {
      current_year = rtc_ext.getYear();
      current_month = rtc_ext.getMonth(century);
      current_day = rtc_ext.getDate();
      current_hour = rtc_ext.getHour(h12Flag, pmFlag);
      current_minute = rtc_ext.getMinute();
      current_second = rtc_ext.getSecond();
      rtc_ext_time_good = 1;
      rtc_int.setMinutes(current_minute);
      rtc_int.setSeconds(current_second);
      rtc_int.setYear(current_year);
      rtc_int.setMonth(current_month);
      rtc_int.setDay(current_day);
      rtc_int.setHours(current_hour);
      /*if (!lcd_status) {

        lcd_call(0, 3, "Date: ", 0);
        lcd.print(current_day);
        lcd.print(":");
        lcd.print(current_month);
        lcd.print(":");
        lcd.print(2000 + current_year);

        delay(delay_screen);

        lcd_call(0, 0, "Time: ", 0);
        lcd.print(current_hour);
        lcd.print(":");
        lcd.print(current_minute);
        lcd.print(":");
        lcd.print(current_second);

        lcd_call(0, 1, "Internal RTC Set", 0);
      }*/
      if (!lcd_status) {
        lcd.setCursor(18, 2);
        lcd.print((char)255);
        delay(delay_screen);
      }

      if (Serial) {
        str = "";
        Serial.println(str + " | " + "Year - " + (2000 + current_year) + ": " + "Month - " + current_month + ": " + "Date -  " + current_day + " | " + "Hour - " + current_hour + ": " + "Minutes - " + current_minute + ": " + "Seconds - " + current_second);
        Serial.print("Setting Internal RTC...");
        Serial.println("Done!");
      }
    } else {
      rtc_ext_time_good = 0;
      /*
      if (!lcd_status) {
        lcd_call(0, 3, "Ext RTC Not Good.   ", delay_screen);
      }*/
      if (Serial) {
        Serial.println("External RTC Time not initialised");
      }
    }
    delay(delay_screen);
  }
}

void setup3temp() {

  Bat1Sensor.begin();
  Bat2Sensor.begin();
  Bat3Sensor.begin();
  Bat4Sensor.begin();


  if (Bat1Sensor.getAddress(Bat1_Address, 0)) {
    numDevices++;
  }
  if (Bat2Sensor.getAddress(Bat2_Address, 0)) {
    numDevices++;
  }
  if (Bat3Sensor.getAddress(Bat3_Address, 0)) {
    numDevices++;
  }
  if (Bat4Sensor.getAddress(Bat4_Address, 0)) {
    numDevices++;
  }

  if (!lcd_status) {

    lcd_call(0, 3, "1W Temp Sensors", delay_screen);

    lcd.setCursor(17, 3);
    lcd.print((char)91);

    lcd.setCursor(18, 3);
    lcd.print(numDevices);

    lcd.setCursor(19, 3);
    lcd.print((char)93);

    delay(delay_screen);
  }
  if (Serial) {
    Serial.print("Discovered ");
    Serial.print(numDevices);
    Serial.println(" of 1 Wire devices");

    Serial.print("Address 1: ");
    printAddress(Bat1_Address);
    Serial.print("  |  ");
    Serial.print("Resolution: ");
    Serial.print(Bat1Sensor.getResolution(Bat1_Address), DEC);

    Serial.print(" | Address 2: ");
    printAddress(Bat2_Address);
    Serial.print("  |  ");
    Serial.print("Resolution: ");
    Serial.print(Bat2Sensor.getResolution(Bat2_Address), DEC);

    Serial.print(" | Address 3: ");
    printAddress(Bat3_Address);
    Serial.print("  |  ");
    Serial.print("Resolution: ");
    Serial.print(Bat3Sensor.getResolution(Bat3_Address), DEC);

    Serial.print(" | Address 4: ");
    printAddress(Bat4_Address);
    Serial.print("  |  ");
    Serial.print("Resolution: ");
    Serial.print(Bat4Sensor.getResolution(Bat4_Address), DEC);

    Serial.println("");
  }
}

void setup4INA226() {
  /*
    if (!lcd_status) {
      lcd.clear();
      lcd_call(0, 0, "********************", 0);
      lcd_call(0, 1, "   Setup...4 of 11  ", 0);
      lcd_call(0, 2, "     INA 226        ", 0);
      lcd_call(0, 3, "********************", delay_screen);
      lcd.clear();
    }*/

  if (!lcd_status) {
    lcd.clear();
    lcd_call(0, 0, "Shunt Sensor 1", 0);
    lcd.setCursor(17, 0);
    lcd.print((char)91);

    lcd.setCursor(19, 0);
    lcd.print((char)93);

    delay(delay_screen);
    lcd_call(0, 1, "Shunt Sensor 2", 0);
    lcd.setCursor(17, 1);
    lcd.print((char)91);

    lcd.setCursor(19, 1);
    lcd.print((char)93);

    delay(delay_screen);
  }

  if (INA0.begin()) {
    shunt1_exist = 1;
    if (!lcd_status) {
      lcd.setCursor(18, 0);
      lcd.print((char)255);
      delay(delay_screen);
    }

    if (Serial) {
      // Serial.print("40");
      Serial.print("\t");
      Serial.print(INA0.getBusVoltage(), 3);
      Serial.print("\t");
      Serial.print(INA0.getShuntVoltage_mV(), 3);
      Serial.print("\t");
      Serial.print(INA0.getCurrent_mA(), 3);
      Serial.print("\t");
      Serial.print(INA0.getPower_mW(), 3);
      Serial.println();
    }
  } else {
    /* if (!lcd_status) {
      lcd_call(0, 1, "Shunt Sensor 1 X", delay_screen);
    }*/
  }


  if (INA1.begin()) {
    shunt2_exist = 1;
    if (!lcd_status) {
      lcd.setCursor(18, 1);
      lcd.print((char)255);
      delay(delay_screen);
    }
    /*
      if (!lcd_status) {
        lcd.clear();
        lcd_call(0, 0, "Sensor 2", 0);
        lcd_call(0, 1, "Bus: ", 0);
        lcd.print(INA1.getBusVoltage(), 3);
        lcd_call(8, 1, "Shunt: ", 0);
        lcd.print(INA1.getShuntVoltage_mV(), 3);
        lcd_call(0, 2, "Current: ", 0);
        lcd.print(INA1.getCurrent_mA(), 3);
        lcd.print(" mA");
        lcd_call(0, 3, "Power: ", 0);
        lcd.print(INA1.getPower_mW(), 3);
        lcd.print(" W");
        delay(delay_screen);
        INA0.setMaxCurrentShunt(1, 0.002);
      }*/

    if (Serial) {
      Serial.print("45");
      Serial.print("\t");
      Serial.print(INA1.getBusVoltage(), 3);
      Serial.print("\t");
      Serial.print(INA1.getShuntVoltage_mV(), 3);
      Serial.print("\t");
      Serial.print(INA1.getCurrent_mA(), 3);
      Serial.print("\t");
      Serial.print(INA1.getPower_mW(), 3);
      Serial.println();
    }
  } else {
    /*
    if (!lcd_status) {
      lcd_call(0, 2, "Shunt Sensor 2 X", delay_screen);
    }*/
  }
}

void setup5analog() {
  /*
    if (!lcd_status) {
      lcd.clear();
      lcd_call(0, 0, "********************", 0);
      lcd_call(0, 1, "   Setup...5 of 11  ", 0);
      lcd_call(0, 2, "  Analogue Inputs   ", 0);
      lcd_call(0, 3, "********************", delay_screen);
      lcd.clear();
    }*/
  lipo_bat_voltage = analogRead(ADC_BATTERY) * (4.3 / 1023.0);
  rtc_bat_voltage = analogRead(A4) * (3.3 / 1023.0);
  gas_sensor_voltage = analogRead(A6) * (3.3 / 1023.0);
  /*
    if (!lcd_status) {
      lcd.clear();
      lcd_call(0, 0, "External Battery:", 0);
      lcd.print(lipo_bat_voltage);
      lcd.print(" V");

      lcd_call(0, 1, "RTC Bkp Bat: ", 0);
      lcd.print(rtc_bat_voltage);
      lcd.print(" V");

      lcd_call(0, 2, "Gas Sensor: ", 0);
      lcd.print(gas_sensor_voltage);
      lcd.print(" V");

      delay(delay_screen);
    }*/

  if (Serial) {
    Serial.print("Back up Battery Voltage - ");
    Serial.print(lipo_bat_voltage);
    Serial.println(" V");

    Serial.print("RTC Bkp Bat - ");
    Serial.print(rtc_bat_voltage);
    Serial.println(" V");

    Serial.print("Gas Sensor - ");
    Serial.print(gas_sensor_voltage);
    Serial.println(" V");
  }
}

void setup6SD() {
  /*
    if (!lcd_status) {
      lcd.clear();
      lcd_call(0, 0, "********************", 0);
      lcd_call(0, 1, "   Setup...6 of 11  ", 0);
      lcd_call(0, 2, "      SD CARD       ", 0);
      lcd_call(0, 3, "********************", delay_screen);
      lcd.clear();
    }*/

  if (!lcd_status) {

    lcd_call(0, 2, "SD Card", 0);
    lcd.setCursor(17, 2);
    lcd.print((char)91);

    lcd.setCursor(19, 2);
    lcd.print((char)93);

    delay(delay_screen);
  }

  SDAvailable = SD.begin(chipSelect);

  if (SDAvailable) {
    modbusTCPServer.holdingRegisterWrite(9, 100);
    if (Serial) {
      Serial.println("Card working: " + String(SDAvailable));
    }
    if (!lcd_status) {
      lcd.setCursor(18, 2);
      lcd.print((char)255);
      delay(delay_screen);
    }
    //root = SD.open("/");
    //printDirectory(root, 0);
    filesize = 0;
    if ((!SD.exists("index.htm"))) {
      if (Serial) {
        Serial.println("index.htm file not found!");
      }
      /*
      if (!lcd_status) {
        lcd_call(0, 2, "index.htm file not found!", 0);
      }*/
    } else {
      if (Serial) {
        Serial.print("index.htm file found, reading contents...");
      }                                /*
      if (!lcd_status) {
        lcd_call(0, 2, "index.htm found!", 0);
      }*/
      webPage = SD.open("index.htm");  // open web page file
      filesize = webPage.available();
      for (b = 0; b <= filesize; b++) {
        html[b] = webPage.read();
      }
      if (Serial) {
        Serial.println("moved file to memory successfully");
      } /*
      if (!lcd_status) {
        lcd_call(0, 3, "Moved index.htm to mem", delay_screen);
      }*/
    }
    getMonthFileName();
    getDayFileName();

    if (rtc_ext_time_good) { /*
      if (!lcd_status) {
        lcd.clear();
        lcd_call(0, 1, "creating file", delay_screen);
        lcd.clear();
      }*/
      if (Serial) {
        Serial.println("Creating File(s)");
      }
      createMonthFileName();
      createDayFileName();
    } else { /*
      if (!lcd_status) {
        lcd.clear();
        lcd_call(0, 1, "Skipping file", delay_screen);
        lcd.clear();
      }*/
      if (Serial) {
        Serial.println("Skipping File(s)");
      }
    }

    if (SD.exists("config.csv")) {
      if (Serial) {
        Serial.print("Found Configuration File. Reading values to Memory...");
      } /*
      if (!lcd_status) {
        lcd_call(0, 3, "Found config.csv", delay_screen);
      }*/
      ConfigFile = SD.open("config.csv");
      String temp;
      temp = ConfigFile.readStringUntil('\r');
      temp.toCharArray(Monthfilename, (temp.length() + 1));
      // Serial.println(temp.length());
      Serial.println(Monthfilename);
      temp = ConfigFile.readStringUntil('\r');
      temp.toCharArray(Dayfilename, (temp.length() + 1));
      // Serial.println(temp.length());
      Serial.println(Dayfilename);
      //ConfigFile.readStringUntil('\r').toCharArray(Dayfilename, (ConfigFile.readStringUntil('\r')).length());
      cal1 = (ConfigFile.readStringUntil('\r')).toInt();
      cal2 = (ConfigFile.readStringUntil('\r')).toInt();
      cal3 = (ConfigFile.readStringUntil('\r')).toInt();
      cal4 = (ConfigFile.readStringUntil('\r')).toInt();
      cal5 = (ConfigFile.readStringUntil('\r')).toInt();
      cal6 = (ConfigFile.readStringUntil('\r')).toInt();
      cal7 = (ConfigFile.readStringUntil('\r')).toInt();
      str = "";
      if (Serial) {
        Serial.println(str + "From config: - " + Monthfilename + "  " + Dayfilename + " " + cal1 + "  " + cal2 + "  " + cal3 + "  " + cal4 + "  " + cal5 + " " + cal6 + " " + cal7);
      }
      /*
      if (!lcd_status) {
        lcd.clear();
        lcd_call(0, 0, "From config.csv", 0);
        lcd_call(0, 1, Monthfilename, 0);
        lcd_call(0, 2, Dayfilename, 0);
        lcd_call(0, 3, "Reading Cal Values", delay_screen);

        lcd.clear();
        lcd_call(0, 0, "Cal 1: ", 0);
        lcd.print(cal1);

        lcd_call(0, 1, "Cal 2: ", 0);
        lcd.print(cal2);

        lcd_call(0, 2, "Cal 3: ", 0);
        lcd.print(cal3);

        lcd_call(0, 3, "Cal 3: ", delay_screen);
        lcd.print(cal4);

        lcd.clear();
        lcd_call(0, 0, "Cal 4: ", 0);
        lcd.print(cal4);

        lcd_call(0, 1, "Cal 5: ", 0);
        lcd.print(cal5);

        lcd_call(0, 2, "Cal 6: ", 0);
        lcd.print(cal6);

        lcd_call(0, 3, "Cal 7: ", 0);
        lcd.print(cal7);
        delay(delay_screen);
      }*/

    } else {
      if (Serial) {
        Serial.print("Config File doesn't exist.");
        Serial.print("Creating new file...[CONFIG.CSV]");
        Serial.println("Preparting Configuration File for first write");
      }
      /*
      if (!lcd_status) {
        lcd.clear();
        lcd_call(0, 1, "Creating new [CONFIG.CSV]", delay_screen);
        lcd.clear();
      }*/

      updateTime();
      SdFile::dateTimeCallback(dateTime);

      ConfigFile = SD.open(configfilename, FILE_WRITE);
      ConfigFile.println(Monthfilename);
      ConfigFile.println(Dayfilename);
      ConfigFile.println(cal1);
      ConfigFile.println(cal2);
      ConfigFile.println(cal3);
      ConfigFile.println(cal4);
      ConfigFile.println(cal5);
      ConfigFile.println(cal6);
      ConfigFile.println(cal7);
      ConfigFile.close();
    }

  } else {
    if (Serial) {
      Serial.println("Card not present");
    } /*
    if (!lcd_status) {
      lcd_call(0, 3, "SD Card Not Found", delay_screen);
    }*/
    modbusTCPServer.holdingRegisterWrite(9, 101);
  }
}

void setup7WiFi() {
  if (setup_oncerun) {
    /*
    if (!lcd_status) {
      lcd.clear();
      lcd_call(0, 0, "********************", 0);
      lcd_call(0, 1, "   Setup...7 of 11   ", 0);
      lcd_call(0, 2, "WiFi: ", 0);
      lcd.print(ssid);
      lcd_call(0, 3, "********************", delay_screen);
      lcd.clear();
    }*/

    if (!lcd_status) {
      //  lcd.clear();
      lcd_call(0, 3, "WiFi(", 0);
      lcd.print(ssid);
      lcd.print(")");
      lcd.setCursor(17, 3);
      lcd.print((char)91);

      lcd.setCursor(19, 3);
      lcd.print((char)93);

      delay(delay_screen);
    }


    if (Serial) {
      Serial.print("Connecting to WiFi");
    }
    /*
    if (!lcd_status) {
      clearScreen();
      lcd_call(0, 0, "WiFi    ", delay_screen);
      lcd.print(ssid);
    }*/
  }

  WiFi_try = 0;
  do {

    if (setup_oncerun) {
      if (Serial) {
        Serial.print(".");
      } /*
      if (!lcd_status) {
        lcd.print(".");
      }*/
    }

    wifi_status = WiFi.begin(ssid, pass);

    delay(delay_screen);
    //colour_blink(red);
    WiFi_try++;
    if (WiFi_try == 10) {
      break;
    }
  } while (wifi_status != WL_CONNECTED);

  if (WiFi_try == 10) {
    WiFi_success = 0;
    if (setup_oncerun) {
      if (Serial) {
        Serial.println("Unable to connect to WiFi - attempts : " + WiFi_try);
      }
      /*
      if (!lcd_status) {
        lcd_call(0, 0, "WiFi Not Connected", delay_screen);
      }*/
    }
  } else {
    WiFi_success = 1;
    IPAddress ip = WiFi.localIP();
    rssi = WiFi.RSSI();
    str = "";
    if (setup_oncerun) {
      if (!lcd_status) {
        // clearScreen();

        lcd.setCursor(18, 3);
        lcd.print((char)255);
        delay(delay_screen);

        //lcd_call(0, 0, "WiFi Connected", delay_screen);

        /* lcd.setCursor(0, 1);
        lcd.print("IP Address:");
        lcd.setCursor(0, 2);
        lcd.print(ip);
        lcd.setCursor(0, 3);
        lcd.print("RSSI: ");
        lcd.print(WiFi.RSSI());
        lcd.print(" dBm");
        delay(delay_screen);*/
      }

      if (Serial) {
        Serial.print(str + "Success!! SSID: " + WiFi.SSID());
        Serial.print("| IP Address: ");
        Serial.print(ip);
        Serial.print(" | Signal strength (RSSI): ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
      }
    }
  }
}

void setup8NTP() {

  if (setup_oncerun) {
    /*
    if (!lcd_status) {
      lcd.clear();
      lcd_call(0, 0, "********************", 0);
      lcd_call(0, 1, "   Setup...8 of 11  ", 0);
      lcd_call(0, 2, "     NTP Request    ", 0);
      lcd_call(0, 3, "********************", delay_screen);
      lcd.clear();
    }*/

    if (!lcd_status) {
      lcd.clear();
      lcd_call(0, 0, "NTP Sync", 0);

      lcd.setCursor(17, 0);
      lcd.print((char)91);

      lcd.setCursor(19, 0);
      lcd.print((char)93);

      delay(delay_screen);
    }
  }
  /*
  if (!lcd_status) {

    lcd_call(0, 1, "NTP Check", delay_screen);
  }*/
  ntp_try = 1;
  if (Serial) {
    Serial.print("Getting Time from NTP Server ");
  }

  do {
    if (Serial) {
      Serial.print(".");
    }
    /*
    if (!lcd_status) {
      lcd.print(".");
    }*/

    ntp = WiFi.getTime();
    ntp_try++;

    if (ntp_try == 10) {
      if (Serial) {
        Serial.print("Exceeded attempts, breaking routine");
      } /*
      if (!lcd_status) {

        lcd_call(0, 1, "NTP Unsuccessful..", delay_screen);
      }*/
      break;
    }
    delay(5000);
  } while ((year(ntp) == 1970));

  rtc_int.setEpoch(ntp);
  if (rtc_ext_exist) {
    rtc_ext.setEpoch(ntp);
  }

  str = "";
  if (Serial) {
    Serial.print(str + "(" + ntp_try + ")");
  }
  str = "";
  if (Serial) {
    Serial.print(str + "Updated available RTC clocks from timeserver");
  }

  if (ntp_try == 10) {
    // colour_blink(red);
    NTP_success = 1;

    if (Serial) {
      Serial.print(str + "Failed to retrieve time from time server : " + "(" + ntp_try + ")");
    }
  } else {
    // colour_blink(blue);
    current_day = rtc_int.getDay();
    previous_day = current_day;

    current_hour = rtc_int.getHours();
    previous_hour = current_hour;

    current_month = rtc_int.getMonth();
    previous_month = current_month;
    NTP_success = 1;
    str = "";
    if (Serial) {

      Serial.println(str + " | " + "Year - " + (2000 + rtc_int.getYear()) + ": " + "Month - " + current_month + ": " + "Date -  " + current_day + " | " + "Hour - " + current_hour + ": " + "Minutes - " + rtc_int.getMinutes() + ": " + "Seconds - " + rtc_int.getSeconds());
    }

    if (!lcd_status) {
      if (setup_oncerun) {


        lcd.setCursor(18, 0);
        lcd.print((char)255);
        delay(delay_screen);
      }
    }
  }
}

void setup9Modbus() {

  /*

  if (!lcd_status) {

    lcd.clear();
    lcd_call(0, 0, "********************", 0);
    lcd_call(0, 1, "   Setup...9 of 11  ", 0);
    lcd_call(0, 2, "    Modbus Server   ", 0);
    lcd_call(0, 3, "********************", delay_screen);
    lcd.clear();
  }*/

  if (!lcd_status) {
    //  lcd.clear();
    lcd_call(0, 1, "Modbus Server", 0);

    lcd.setCursor(17, 1);
    lcd.print((char)91);

    lcd.setCursor(19, 1);
    lcd.print((char)93);

    delay(delay_screen);
  }

  wifi_Modbus_Server.begin();
  /*
  if (!lcd_status) {
    lcd.clear();
    lcd_call(0, 0, "********************", 0);
    lcd_call(0, 1, "Start Modbus Server ", delay_screen);
    lcd.clear();
  }*/
  if (!modbusTCPServer.begin()) {
    if (Serial) {
      Serial.println("Failed to start Modbus TCP Server!");
    }
    /*
    if (!lcd_status) {
      lcd_call(0, 2, "Failed ", 0);
    }*/
  }

  if (!lcd_status) {
    //    lcd_call(0, 3, "Successful ", delay_screen);

    lcd.setCursor(18, 1);
    lcd.print((char)255);
    delay(delay_screen);
  }

  modbusTCPServer.configureCoils(0x00, 10);
  modbusTCPServer.configureDiscreteInputs(0x00, 10);
  modbusTCPServer.configureInputRegisters(0x00, 30);
  modbusTCPServer.configureHoldingRegisters(0x00, 20);

  wifi_web_server.begin();

  modbusTCPServer.holdingRegisterWrite(0, rtc_int.getYear());
  modbusTCPServer.holdingRegisterWrite(1, rtc_int.getMonth());
  modbusTCPServer.holdingRegisterWrite(2, rtc_int.getDay());
  modbusTCPServer.holdingRegisterWrite(3, rtc_int.getHours());
  modbusTCPServer.holdingRegisterWrite(4, rtc_int.getMinutes());
  modbusTCPServer.holdingRegisterWrite(5, rtc_int.getSeconds());
  modbusTCPServer.holdingRegisterWrite(6, abs(int(WiFi.RSSI())));
}

void setup10ISR() {

  /*
  if (!lcd_status) {
    lcd.clear();
    lcd_call(0, 0, "********************", 0);
    lcd_call(0, 1, "   Setup..10 of 11  ", 0);
    lcd_call(0, 2, "    Setup ISR       ", 0);
    lcd_call(0, 3, "********************", delay_screen);
    lcd.clear();
  }*/

  if (!lcd_status) {
    //  lcd.clear();
    lcd_call(0, 2, "Setting up ISR", 0);

    lcd.setCursor(17, 2);
    lcd.print((char)91);

    lcd.setCursor(19, 2);
    lcd.print((char)93);

    delay(delay_screen);
  }

  if (Serial) {
    Serial.println("Starting Interrupt Routine");
  }

  if (ITimer.attachInterruptInterval_MS(HW_TIMER_INTERVAL_MS, TimerHandler)) {
    if (Serial) {
      Serial.println("Starting Interrupt ");
    }
    if (!lcd_status) {

      lcd.setCursor(18, 2);
      lcd.print((char)255);
      delay(delay_screen);
    }


  } else {
    if (Serial) {
      Serial.println("Unable to start Interrupt Routine");
    } /*
    if (!lcd_status) {
      lcd.clear();
      lcd_call(0, 2, "Unable to start ISR", delay_screen);
      lcd.clear();
    }*/
  }

  ISR_Timer.setInterval(TIMER_INTERVAL_10s, ADC_Routine);
  ISR_Timer.setInterval(TIMER_INTERVAL_60s, Print_Routine);

  lcd_call(0, 3, "Start Main Routine", delay_screen);
  delay(delay_screen);
}

void UpKey() {
  // mcp.digitalWrite(BUTTONPRESS_LED, HIGH);
  button_time = millis();
  if (button_time - last_button_time > button_delay) {
    if (!up) {
      up = 1;
      detachInterrupt(digitalPinToInterrupt(A2));
      last_button_time = button_time;
    }
  }
}

void DownKey() {
  // mcp.digitalWrite(BUTTONPRESS_LED, HIGH);
  button_time = millis();
  if (button_time - last_button_time > button_delay) {
    if (!down) {
      down = 1;
      detachInterrupt(digitalPinToInterrupt(6));
      last_button_time = button_time;
    }
  }
}

void LeftKey() {
  // mcp.digitalWrite(BUTTONPRESS_LED, HIGH);
  button_time = millis();
  if (button_time - last_button_time > button_delay) {
    if (!left) {
      left = 1;
      detachInterrupt(digitalPinToInterrupt(A1));
      last_button_time = button_time;
    }
  }
}

void RightKey() {
  //mcp.digitalWrite(BUTTONPRESS_LED, HIGH);
  button_time = millis();
  if (button_time - last_button_time > button_delay) {
    if (!right) {
      right = 1;
      detachInterrupt(digitalPinToInterrupt(5));
      last_button_time = button_time;
    }
  }
}

void EnterKey() {
  // mcp.digitalWrite(BUTTONPRESS_LED, HIGH);
  button_time = millis();
  if (button_time - last_button_time > button_delay) {
    if (!enter) {
      enter = 1;
      detachInterrupt(digitalPinToInterrupt(7));
      last_button_time = button_time;
    }
  }
}

void HomeKey() {
  // mcp.digitalWrite(BUTTONPRESS_LED, HIGH);
  button_time = millis();
  if (button_time - last_button_time > button_delay) {
    if (!home) {
      home = 1;
      detachInterrupt(digitalPinToInterrupt(0));
      last_button_time = button_time;
    }
  }
}

void BackKey() {
  //  mcp.digitalWrite(BUTTONPRESS_LED, HIGH);
  button_time = millis();
  if (button_time - last_button_time > button_delay) {
    if (!back) {
      back = 1;
      detachInterrupt(digitalPinToInterrupt(1));
      last_button_time = button_time;
    }
  }
}

void lcd_call(int column, int row, char print_text[], int delay_refresh) {
  lcd.setCursor(column, row);
  lcd.print(print_text);
  delay(delay_refresh);
}

void main_loop_led() {


  if (NTP_success) {
    mcp.digitalWrite(NTP_LED, HIGH);
  } else {
    mcp.digitalWrite(NTP_LED, LOW);
  }



  if (wifi_status != WL_CONNECTED) {
    mcp.digitalWrite(WiFi_LED, LOW);
  } else {
    mcp.digitalWrite(WiFi_LED, HIGH);
  }


  /*

  if (web_client) {
    mcp.digitalWrite(HTTP_LED, HIGH);
  } else {
    mcp.digitalWrite(HTTP_LED, HIGH);
  }

  if (modbus_client.connected()) {
   mcp.digitalWrite(MODBUS_LED, HIGH);
  } else {
    mcp.digitalWrite(MODBUS_LED, LOW);
  }
  */
}

void main_loop_keypress() {

  if (up) {
    up_count++;
    if (line_count >= 1) {
      line_count--;
    }

    if (Serial) {
      Serial.print("UP: ");
      Serial.print(up_count);

      Serial.print("  Line:  ");
      Serial.print(line_count);

      Serial.print("  Last Page:  ");
      Serial.print(last_page);

      Serial.print("  Current Page:  ");
      Serial.print(page_count);

      Serial.print("  Tab:  ");
      Serial.println(tab_count);
    }
    up = 0;
    attachInterrupt(digitalPinToInterrupt(A2), UpKey, RISING);
    mcp.digitalWrite(BUTTONPRESS_LED, LOW);
  }

  if (down) {
    down_count++;
    if (line_count < 12) {
      line_count++;
    }
    if (Serial) {
      Serial.print("Down: ");
      Serial.print(down_count);

      Serial.print("  Line:  ");
      Serial.print(line_count);

      Serial.print("  Last Page:  ");
      Serial.print(last_page);

      Serial.print("  Current Page:  ");
      Serial.print(page_count);

      Serial.print("  Tab:  ");
      Serial.println(tab_count);
    }
    down = 0;
    attachInterrupt(digitalPinToInterrupt(6), DownKey, FALLING);
    mcp.digitalWrite(BUTTONPRESS_LED, LOW);
  }

  if (left) {
    left_count++;

    if (tab_count >= 1) {
      tab_count--;
      if (tab_count == 0) {
        screen_0 = true;
      }

      if (tab_count == 1) {
        screen_1 = true;
      }
    }
    if (Serial) {
      Serial.print("Left: ");
      Serial.print(left_count);

      Serial.print("  Line: ");
      Serial.print(line_count);

      Serial.print("  Last Page:  ");
      Serial.print(last_page);

      Serial.print("  Current Page: ");
      Serial.print(page_count);

      Serial.print("  Tab: ");
      Serial.println(tab_count);
    }
    left = 0;
    attachInterrupt(digitalPinToInterrupt(A1), LeftKey, RISING);
    mcp.digitalWrite(BUTTONPRESS_LED, LOW);
  }

  if (right) {
    right_count++;
    if (tab_count < 2) {
      tab_count++;
    }
    if (Serial) {
      Serial.print("Right");
      Serial.print(right_count);

      Serial.print("  Line: ");
      Serial.print(line_count);

      Serial.print("  Last Page:  ");
      Serial.print(last_page);

      Serial.print("  Current Page: ");
      Serial.print(page_count);

      Serial.print("  Tab: ");
      Serial.println(tab_count);
    }
    right = 0;
    attachInterrupt(digitalPinToInterrupt(5), RightKey, FALLING);
    mcp.digitalWrite(BUTTONPRESS_LED, LOW);
  }

  if (enter) {
    enter_count++;
    if (Serial) {
      Serial.print("Enter");
      Serial.println(enter_count);
    }
    enter = 0;
    attachInterrupt(digitalPinToInterrupt(7), EnterKey, FALLING);
    mcp.digitalWrite(BUTTONPRESS_LED, LOW);
  }

  if (home) {
    left_count = 0;
    right_count = 0;
    up_count = 0;
    down_count = 0;
    enter_count = 0;
    line_count = 0;
    last_page = 1;
    page_count = 0;
    tab_count = 0;
    if (Serial) {
      Serial.print("Home");

      Serial.print("  Line: ");
      Serial.print(line_count);

      Serial.print("  Last Page:  ");
      Serial.print(last_page);

      Serial.print("  Current Page: ");
      Serial.print(page_count);

      Serial.print("  Tab: ");
      Serial.println(tab_count);
    }
    home = 0;
    attachInterrupt(digitalPinToInterrupt(0), HomeKey, FALLING);
    mcp.digitalWrite(BUTTONPRESS_LED, LOW);
  }

  if (back) {
    back_count++;
    if (Serial) {
      Serial.print("Back");
      Serial.println(back_count);
    }
    back = 0;
    attachInterrupt(digitalPinToInterrupt(1), BackKey, FALLING);
    mcp.digitalWrite(BUTTONPRESS_LED, LOW);
  }
}

void main_loop_progress() {

  if (tab_count == 0) {
    switch (LED) {
      case 1:
        lcd_call(19, 0, "-", 0);
        break;
      case 2:
        lcd_call(19, 0, "|", 0);
        break;
      case 3:
        lcd_call(19, 0, "-", 0);
        break;
      case 4:
        lcd_call(19, 0, "|", 0);
        break;
    }
  }
  LED++;
  if (LED > 4) {
    LED = 1;
  }
}

void main_loop_screen_0(int current_page, int cursor) {

  //Serial.print("screen refresh");

  lcd.clear();
  if (current_page == 0) {
    lcd_call(1, 0, "1. System", 0);
    lcd_call(1, 1, "2. WiFi", 0);
    lcd_call(1, 2, "3. SD Card", 0);
    lcd_call(1, 3, "4. Modbus", 0);
  }

  if (current_page == 1) {
    lcd_call(1, 0, "5. INA 0", 0);
    lcd_call(1, 1, "6. INA 1", 0);
    lcd_call(1, 2, "7. Analogues", 0);
    lcd_call(1, 3, "8. LED Test", 0);
  }
  if (current_page == 2) {
    lcd_call(1, 0, "9. T Sensor 1", 0);
    lcd_call(1, 1, "10.T Sensor 2", 0);
    lcd_call(1, 2, "11.T Sensor 3", 0);
    lcd_call(1, 3, "12.T Sensor 4", 0);
  }
}

void main_loop_screen_1(int cursor) {

  lcd.clear();
  lcd.setCursor(18, 0);
  lcd.print((char)127);
  lcd.print((char)126);

  if (cursor == 0) {
    lcd_call(0, 0, "Arduino MKR 1010", 0);
    lcd_call(0, 1, "CPU Freq = ", 0);
    lcd.print(F_CPU / 1000000);
    lcd.print(F(" MHz"));
    lcd_call(0, 2, "Looptime: ", 0);
    lcd.print(LoopTime);
    lcd.print(" ms");
    lcd_call(0, 3, "Boot Time: ", 0);
    lcd.print(BootTime);
    lcd.print(" ms");
    //  lcd_call(19, 3, char(142), 0);
    //lcd_call(19, 4, "V", 0);
  }

  if (cursor == 1) {
    IPAddress ip = WiFi.localIP();
    rssi = WiFi.RSSI();
    if (WiFi_success) {
      lcd_call(0, 0, ssid, 0);
      lcd.setCursor(0, 1);
      lcd.print(ip);
      lcd.setCursor(0, 2);
      lcd.print("RSSI: ");
      lcd.print(WiFi.RSSI());
      lcd.print(" dBm");
    }
  }

  if (cursor == 2) {
    lcd_call(0, 0, "Screen 2/Tab 1", 0);
  }

  if (cursor == 3) {
    if (modbus_client_status) {
      lcd_call(0, 0, "Modbus Connected", 0);
    } else {
      lcd_call(0, 0, "Modbus not Connected", 0);
    }

    lcd_call(0, 1, "Comm Count: ", 0);
    lcd.print(comm_count);
    lcd_call(0, 2, "Press Enter to Reset", 0);
    if (!digitalRead(7)) {
      comm_count = 0;
    }
  }

  if (cursor == 4) {

    lcd_call(0, 0, "Sensor 1", 0);
    lcd_call(0, 1, "Bus: ", 0);
    lcd.print(INA0.getBusVoltage(), 1);
    lcd.print(" V");
    lcd_call(0, 2, "Shunt: ", 0);
    lcd.print(INA0.getShuntVoltage_mV(), 1);
    lcd.print(" V");
    lcd_call(0, 3, "I: ", 0);
    lcd.print(INA0.getCurrent(), 1);
    lcd.print(" A");
    lcd_call(10, 3, "Pow: ", 0);
    lcd.print(INA0.getPower(), 1);
    lcd.print(" W");
  }

  if (cursor == 5) {

    lcd_call(0, 0, "Sensor 2", 0);
    lcd_call(0, 1, "Bus: ", 0);
    lcd.print(INA1.getBusVoltage(), 1);
    lcd.print(" V");
    lcd_call(0, 2, "Shunt: ", 0);

    lcd.print(INA1.getShuntVoltage_mV(), 1);
    lcd.print(" V");
    lcd_call(0, 3, "I: ", 0);
    lcd.print(INA1.getCurrent(), 1);
    lcd.print(" A");
    lcd_call(10, 3, "Power: ", 0);
    lcd.print(INA1.getPower(), 1);
    lcd.print(" W");
  }

  if (cursor == 6) {
    lcd_call(0, 0, "Analog 0:  ", 0);
    lcd.print(analogRead(A0));
    lcd_call(0, 1, "Analog 1:  ", 0);
    lcd.print("N.A.");
    lcd_call(0, 2, "Analog 2:  ", 0);
    lcd.print("N.A.");
    lcd_call(0, 3, "Analog 3:  ", 0);
    lcd.print(analogRead(A3));
  }

  if (cursor == 7) {
    lcd_call(0, 0, "Press and Hold", 0);
    lcd_call(0, 1, "Enter for LED ON", 0);
    if (!digitalRead(7)) {
      for (int i = 0; i < 16; i++) {
        mcp.digitalWrite(i, HIGH);
      }
    }
  }

  if (cursor == 8) {

    lcd_call(0, 0, "Battery Sensor 1", 0);
    lcd.setCursor(0, 1);
    for (uint8_t i = 0; i < 8; i++) {
      // zero pad the address if necessary
      if (Bat1_Address[i] < 16) Serial.print("0");
      Serial.print(Bat1_Address[i], HEX);
    }
    for (uint8_t i = 0; i < 8; i++) {
      // zero pad the address if necessary
      if (Bat1_Address[i] < 16) lcd.print("0");
      lcd.print(Bat1_Address[i], HEX);
    }
    lcd.setCursor(0, 2);
    lcd.print("Resolution: ");
    lcd.print(Bat1Sensor.getResolution());
    Serial.print("Resolution: ");
    Serial.println(Bat1Sensor.getResolution());
    lcd.setCursor(0, 3);
    Serial.print("Power Mode: ");
    lcd.print("Power: ");
    if (Bat1Sensor.isParasitePowerMode()) {
      if (Serial) {
        Serial.println("External");
      }

      if (!lcd_status) {
        lcd.print("External");
      }
    } else {
      if (Serial) {
        Serial.println("Parasite");
      }
      if (!lcd_status) {
        lcd.print("Internal");
      }
    }
  }

  if (cursor == 9) {

    lcd_call(0, 0, "Battery Sensor 2", 0);
    lcd.setCursor(0, 1);
    for (uint8_t i = 0; i < 8; i++) {
      // zero pad the address if necessary
      if (Bat2_Address[i] < 16) Serial.print("0");
      Serial.print(Bat2_Address[i], HEX);
    }
    for (uint8_t i = 0; i < 8; i++) {
      // zero pad the address if necessary
      if (Bat2_Address[i] < 16) lcd.print("0");
      lcd.print(Bat2_Address[i], HEX);
    }
    lcd.setCursor(0, 2);
    lcd.print("Resolution: ");
    lcd.print(Bat2Sensor.getResolution());
    Serial.print("Resolution: ");
    Serial.println(Bat2Sensor.getResolution());
    lcd.setCursor(0, 3);
    Serial.print("Power Mode: ");
    lcd.print("Power: ");
    if (Bat1Sensor.isParasitePowerMode()) {
      if (Serial) {
        Serial.println("External");
      }

      if (!lcd_status) {
        lcd.print("External");
      }
    } else {
      if (Serial) {
        Serial.println("Parasite");
      }
      if (!lcd_status) {
        lcd.print("Internal");
      }
    }
  }

  if (cursor == 10) {


    lcd_call(0, 0, "Battery Sensor 3", 0);
    lcd.setCursor(0, 1);
    for (uint8_t i = 0; i < 8; i++) {
      // zero pad the address if necessary
      if (Bat3_Address[i] < 16) Serial.print("0");
      Serial.print(Bat3_Address[i], HEX);
    }
    for (uint8_t i = 0; i < 8; i++) {
      // zero pad the address if necessary
      if (Bat3_Address[i] < 16) lcd.print("0");
      lcd.print(Bat3_Address[i], HEX);
    }
    lcd.setCursor(0, 2);
    lcd.print("Resolution: ");
    lcd.print(Bat3Sensor.getResolution());
    Serial.print("Resolution: ");
    Serial.println(Bat3Sensor.getResolution());
    lcd.setCursor(0, 3);
    Serial.print("Power Mode: ");
    lcd.print("Power: ");
    if (Bat3Sensor.isParasitePowerMode()) {
      if (Serial) {
        Serial.println("External");
      }

      if (!lcd_status) {
        lcd.print("External");
      }
    } else {
      if (Serial) {
        Serial.println("Parasite");
      }
      if (!lcd_status) {
        lcd.print("Internal");
      }
    }
  }


  if (cursor == 11) {

    lcd_call(0, 0, "Battery Sensor 4", 0);
    lcd.setCursor(0, 1);
    for (uint8_t i = 0; i < 8; i++) {
      // zero pad the address if necessary
      if (Bat4_Address[i] < 16) Serial.print("0");
      Serial.print(Bat4_Address[i], HEX);
    }
    for (uint8_t i = 0; i < 8; i++) {
      // zero pad the address if necessary

      if (Bat4_Address[i] < 16) lcd.print("0");
      lcd.print(Bat4_Address[i], HEX);
    }
    lcd.setCursor(0, 2);
    lcd.print("Resolution: ");
    lcd.print(Bat4Sensor.getResolution());
    Serial.print("Resolution: ");
    Serial.println(Bat4Sensor.getResolution());
    lcd.setCursor(0, 3);
    Serial.print("Power Mode: ");
    lcd.print("Power: ");
    if (Bat4Sensor.isParasitePowerMode()) {
      if (Serial) {
        Serial.println("External");
      }

      if (!lcd_status) {
        lcd.print("External");
      }
    } else {
      if (Serial) {
        Serial.println("Parasite");
      }
      if (!lcd_status) {
        lcd.print("Internal");
      }
    }
  }
}

void main_loop_screen_2(int cursor) {

  lcd.clear();
  lcd.setCursor(19, 0);
  lcd.print((char)127);
  //  lcd_call(19, 0, (char)231, 0);
  if (cursor == 0) {
    nDevices = 0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd_call(0, 0, "No. of Devices: ", 0);
    Serial.print("I2C device found at address: ");
    lcd.setCursor(0, 1);
    for (address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0) {
        if (address < 16) {
          Serial.print("0");
          lcd.print("0");
        }

        Serial.print(address, HEX);
        Serial.print("|");
       
        lcd.print(address, HEX);
        lcd.print("|");
        nDevices++;
      } else if (error == 4) {
        Serial.print("Unknown error at address 0x");
        if (address < 16)
        Serial.print("  0");
        Serial.println(address, HEX);
      }
    }

    lcd.setCursor(17, 0);
    lcd.print(nDevices);
    if (nDevices == 0)
      Serial.println("No I2C devices found\n");
    else
      Serial.println("done\n");
    //  delay(5000);  // wait 5 seconds for next scan
  }

  if (cursor == 1) {
    lcd_call(0, 0, "Screen 1/Tab 2", 0);
    lcd_call(19, 3, "^", 0);
    lcd_call(19, 4, "v", 0);
  }

  if (cursor == 2) {
    lcd_call(0, 0, "Screen 2/Tab 2", 0);
  }

  if (cursor == 3) {
    lcd_call(0, 0, "Screen 3/Tab2", 0);
  }

  if (cursor == 4) {

    lcd_call(0, 0, "Sensor 1 | ", 0);
    lcd.print("Avg: ");
    //lcd_call(0, 1, "Avg: ", 0);
    lcd.print(INA0.getAverage(), DEC);
    lcd_call(0, 1, "Mfr: ", 0);
    lcd.print(INA0.getManufacturerID(), DEC);
    lcd_call(11, 1, "ID: ", 0);
    lcd.print(INA0.getDieID(), DEC);
    lcd_call(0, 2, "Config: ", 0);
    lcd.print(INA0.getRegister(0), DEC);
    lcd_call(0, 3, "Cal: ", 0);

    lcd.print(INA0.getRegister(5), DEC);
  }

  if (cursor == 5) {
    lcd_call(0, 0, "Sensor 2 | ", 0);
    lcd.print("Avg: ");
    //lcd_call(0, 1, "Avg: ", 0);
    lcd.print(INA1.getAverage(), DEC);
    lcd_call(0, 1, "Mfr: ", 0);
    lcd.print(INA1.getManufacturerID(), DEC);
    lcd_call(11, 1, "ID: ", 0);
    lcd.print(INA1.getDieID(), DEC);
    lcd_call(0, 2, "Config: ", 0);
    lcd.print(INA1.getRegister(0), DEC);
    lcd_call(0, 3, "Cal: ", 0);
    lcd.print(INA0.getRegister(5), DEC);
  }



  if (cursor == 6) {
    lcd_call(0, 0, "Analog 4:  ", 0);
    lcd.print(analogRead(A4));
    lcd_call(0, 1, "Analog 5:  ", 0);
    lcd.print(analogRead(A5));
    lcd_call(0, 2, "Analog 6:  ", 0);
    lcd.print(analogRead(A6));
    lcd_call(0, 3, "LiPo Battery:  ", 0);
    lcd.print(analogRead(ADC_BATTERY));
  }

  if (cursor == 7) {
    lcd_call(0, 0, "Press and Hold", 0);
    lcd_call(0, 1, "Enter for LED Off", 0);
    if (!digitalRead(7)) {
      for (int i = 0; i < 16; i++) {
        mcp.digitalWrite(i, LOW);
      }
    }
  }

  if (cursor == 8) {
    // Bat1Sensor.requestTemperatures();
    if (!lcd_status) {
      lcd_call(0, 0, "Battery Sensor 1", 0);
      lcd.setCursor(0, 1);
      lcd.print("Temp.: ");
      lcd.print(batTemp1);
      lcd.print(" C  ");
    }

    if (Serial) {
      Serial.print("Temp.: ");
      Serial.print(batTemp1);
      Serial.print(" C ");
    }
  }

  if (cursor == 9) {
    // Bat2Sensor.requestTemperatures();
    if (!lcd_status) {
      lcd_call(0, 0, "Battery Sensor 2", 0);
      lcd.setCursor(0, 1);
      lcd.print("Temp.: ");
      lcd.print(batTemp2);
      lcd.print(" C");
    }

    if (Serial) {
      Serial.print("Temp.: ");
      Serial.print(batTemp2);
      Serial.print(" C");
    }
  }

  if (cursor == 10) {

    if (!lcd_status) {
      lcd_call(0, 0, "Battery Sensor 3", 0);
      lcd.setCursor(0, 1);
      lcd.print("Temp.: ");
      lcd.print(batTemp3);
      lcd.print(" C");
    }

    if (Serial) {
      Serial.print("Temp.: ");
      Serial.print(batTemp3);
      Serial.print(" C");
    }
  }

  if (cursor == 11) {
    //Bat4Sensor.requestTemperatures();
    if (!lcd_status) {
      lcd_call(0, 0, "Battery Sensor 4", 0);
      lcd.setCursor(0, 1);
      lcd.print("Temp.: ");
      lcd.print(batTemp4);
      lcd.print(" C");
    }

    if (Serial) {
      Serial.print("Temp.: ");
      Serial.print(batTemp4);
      Serial.print(" C");
    }
  }
}

void main_loop_rollover() {

  current_day = rtc_int.getDay();
  if (current_day != previous_day) {
    day_change_flag = 1;
    previous_day = current_day;
    setup8NTP();
    getDayFileName();
    createDayFileName();
  }

  current_month = rtc_int.getMonth();
  if (current_month != previous_month) {
    month_change_flag = 1;
    previous_month = current_month;
    setup8NTP();
    getMonthFileName();
    createMonthFileName();
  }

  current_hour = rtc_int.getHours();
  if (current_hour != previous_hour) {
    hour_change_flag = 1;
    previous_hour = current_hour;
    updateTime();
    SdFile::dateTimeCallback(dateTime);
    ConfigFile = SD.open(configfilename, FILE_WRITE);
    ConfigFile.println(Monthfilename);
    ConfigFile.println(Dayfilename);
    ConfigFile.println(cal1);
    ConfigFile.println(cal2);
    ConfigFile.println(cal3);
    ConfigFile.println(cal4);
    ConfigFile.println(cal5);
    ConfigFile.println(cal6);
    ConfigFile.println(cal7);
    ConfigFile.close();
  }
}

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}