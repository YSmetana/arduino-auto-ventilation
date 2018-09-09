/*
 * Inspired by:
 * http://forum.amperka.ru/threads/%D0%9A%D0%BE%D0%BD%D1%82%D1%80%D0%BE%D0%BB%D1%8C-%D0%B2%D0%BB%D0%B0%D0%B6%D0%BD%D0%BE%D1%81%D1%82%D0%B8-%D0%B8-%D1%82%D0%B5%D0%BC%D0%BF%D0%B5%D1%80%D0%B0%D1%82%D1%83%D1%80%D1%8B-%D0%B2-%D0%BF%D0%BE%D0%B3%D1%80%D0%B5%D0%B1%D0%B5-%D0%BF%D0%BE%D0%B4%D0%B2%D0%B0%D0%BB%D0%B5.9557/
 * http://arduino.ru/forum/proekty/kontrol-vlazhnosti-podvala-arduino-pro-mini
 * http://arduinolab.pw/index.php/2017/05/04/upravlenie-vytyazhkoj-v-pogrebe-ili-podvale/
 */
#define DEBUG false
#define Serial if(DEBUG)Serial  // https://forum.arduino.cc/index.php?topic=155268.0

#define NODE_ID_ME 1          // PJON device ID
#define NODE_ID_OUTSIDE 10
#define NODE_ID_BASEMENT 20

#define BASEMENT_TEMP_MIN 5   // minimum temperature in basement
#define BASEMENT_TEMP_MAX 30  // maximum temperature in basement
#define HUM_DELTA 0.5         // difference between outside and basement abs. hum. to run vent.

#define Node_Request 0.5 *60*1000000  // minutes, how often request data from a node (PJON microseconds)
#define Node_Timeout 1 *60*1000UL     // minutes, time after which node considered unreachable (Arduino milliseconds)

#define Vent_Check_Every 1 *60*1000UL         // minutes, how often calculate if we need vent.
#define Vent_Running_Max_Time 8 *60*60*1000UL // hours, maximum time keeping the vent running.
#define Vent_Rest_After_Run 5 *60*1000UL      // hours, maximum time keeping the vent running.

/*
 * Other settings
 */
#define BLINK_DUR 500UL             // when blinking icons on the dispay
#define DISPLAY_SAVER 1 *60*1000UL  // minutes, display goes off after this time
#define PROXIMITY_PIN 2             // digital pin of proximity sensor
#define VENT_COUNTER_RESET false    // Reset ven. worked minutes in EEPROM. Do not forget to set to false.
#define SECONDS_MINUTE    60
#define SECONDS_HOUR      60*60
#define SECONDS_DAY       60*60*24
#define SECONDS_MONTH     60*60*24*30
#define SECONDS_99MONTHS  60*60*24*99
bool Display_On = true;             // display state
bool Receiving = false;             // data receiving status
bool Fan_On = false;                // fan is working
bool Node_Outside_Online = false;   // node data up to date
bool Node_Basement_Online = false;  // node data up to date
uint16_t Vent_Work_Total;           // total work time of ventilator
byte Receiving_From_Node;           // last data received from this node ID

/*
 * Timers
 */
unsigned long Timer_Page_Display = millis();  // page on display
unsigned long Timer_Blink = millis();         // blinking icons
unsigned long Timer_Receiving = millis();     // receiving icon
unsigned long Timer_Check_Vent;               // to check if its is time to check ventilation process
unsigned long Timer_Vent_Running = 0;         // how long is it running already
unsigned long Timer_Node_Outside = millis();  // node timeout
unsigned long Timer_Node_Basement = millis(); // node timeout
unsigned long Timer_Display_On = millis();    // node timeout

/*
 * Data structures
 */
struct SENSORS{         // for sending data
  float temperature;
  float temperature2;
  float humidity;
  float pressure;
};

struct CONTROL {        // for controlling nodes
  char command[3+1];
  char subcommand[3+1];
  int16_t value;
};

CONTROL Command;        // temp data
SENSORS Sensors;        // temp data

SENSORS SENS_OUTSIDE;   // data from outside (street) sensors
SENSORS SENS_BASEMENT;  // data from basement sensors

/*
 * PJON - Arduino compatible, multi-master, multi-media network protocol.
 * https://github.com/gioblu/PJON
 */
#define TS_BYTE_TIME_OUT 20000        // 80 ms max
#define TS_RESPONSE_TIME_OUT 1000000  // micro sec., PJON timeout, if operating at less than 9600Bd should be longer
#define PJON_PACKET_MAX_LENGTH 32
#define TS_MAX_ATTEMPTS 1
#define PJON_INCLUDE_TS
#include <PJON.h>
PJON<ThroughSerial> bus(NODE_ID_ME);  // <Strategy name> bus(selected device id)

/*
 * U8g2 is a monochrome graphics library for embedded devices.
 * https://github.com/olikraus/u8g2/wiki
 * OLED 0.96" I2C 128x64, https://arduino.ua/prod1263-oled-displei-modyl-belii
 */
#include <U8g2lib.h>
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // full buffer, consumes more RAM
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // 1 page mode
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // 2 pages mode
// For proper text align.
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGH 64

/*
 * HC-12 433 MHz UART transiver
 * Datasheet: https://www.elecrow.com/download/HC-12.pdf
 */
#include <AltSoftSerial.h>  // https://github.com/PaulStoffregen/AltSoftSerial , https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
AltSoftSerial HC12;         // Arduino Pro Mini: HC-12_TX-PIN8_RX, HC-12_RX-PIN9_TX; Arduino Mega: HC-12_TX-PIN48_RX, HC-12_RX-PIN46_TX;
#define HC12_SPEED 2400

#include <EEPROM.h>

#include "images.h"  // icons

/*
 * Functions
 */
void pageTemperature();
void pageHumidity();
void pageAbsHumidity();
void pagePressure();
void pageInfo();
void pageInfo2();
void blink(char icon[3]);
void float2char (float value, char * input, bool sign=false);
unsigned int center_x(char* message, int margin_left=0, int margin_right=0);
unsigned int right_x(char* message, int margin_right=0);
bool run_ventilation();

/*
 * Pages on display
 */
int current_page = 0;
void (*pages[])() = {pageTemperature, pageHumidity, pageAbsHumidity, pagePressure, pageInfo, pageInfo2};
unsigned int pages_count = 6;
int page_duration[] = {5000UL, 5000UL, 5000UL, 3000UL, 5000UL, 5000UL};

/*
 * SETUP
 */
void setup() {
  Serial.begin(9600);
  Serial.println(F("Go"));

  Serial.print(F("ID ")); Serial.print(NODE_ID_ME,DEC); Serial.println(F("."));
  Serial.print(F("Query nodes  ")); Serial.print(Node_Request/60/1000000); Serial.println(F("m."));
  Serial.print(F("N. timeout ")); Serial.print(Node_Timeout/60/1000); Serial.println(F("m."));
  Serial.print(F("Check V ")); Serial.print(Vent_Check_Every/60/1000); Serial.println(F("m."));
  Serial.print(F("V max. dur ")); Serial.print(Vent_Running_Max_Time/60/60/1000); Serial.println(F("h."));
  Serial.print(F("Bas. min. T ")); Serial.print(BASEMENT_TEMP_MIN); Serial.println(F("°C."));
  Serial.print(F("Bas. max T ")); Serial.print(BASEMENT_TEMP_MAX); Serial.println(F("°C."));
  Serial.print(F("A. H delta ")); Serial.print(HUM_DELTA); Serial.println(F(" g/m3."));
  Serial.print(F("V count. ")); Serial.print(EEPROM_read_long(0)); Serial.println(F("m."));
  Serial.print(F("Reset V count. ")); Serial.println(VENT_COUNTER_RESET);
  Serial.println();

  if (VENT_COUNTER_RESET) {
    // set ventilation counter in EEPROM to 0
    EEPROM_write_long(0,0);
  }

  Vent_Work_Total = EEPROM_read_long(0);

  pinMode(PROXIMITY_PIN, INPUT);

  // Display
  u8g2.begin();
  u8g2.setColorIndex(1);
  u8g2.setFontMode(0);    // enable transparent mode, which is faster

  // Wireless
  HC12.begin(HC12_SPEED);       // Set HC12 baudrate (you must use the one configured in HC12, default 9600)
  //delay(1000);
  //HC12.begin(9600);
  //HC12.write("AT+B2400");
  //delay(1000);

  // PJON protocol
  bus.strategy.set_serial(&HC12); // Pass the HC12 Serial instance you want to use it for PJON communication
  bus.set_error(error_handler);
  bus.set_receiver(receiver_function);
  //bus.set_synchronous_acknowledge(false);
  bus.begin();

  // Repeatedly asking nodes to send data
  CONTROL Command = {"get","all",0};
  bus.send_repeatedly(NODE_ID_OUTSIDE,  (const char*)&Command, sizeof(CONTROL), Node_Request); // seconds * 1000000
  delay(TS_RESPONSE_TIME_OUT/1000 * 5); // to avoid inteference with response from the previous node
  bus.send_repeatedly(NODE_ID_BASEMENT, (const char*)&Command, sizeof(CONTROL), Node_Request); // seconds * 1000000
  delay(TS_RESPONSE_TIME_OUT/1000 * 5);

  Timer_Check_Vent = millis(); // to check if its is time to calc ventilation process
};

void error_handler(uint8_t code, uint16_t data, void *custom_pointer) {
  /*
   * Pjon errors handlers
   */
  if(code == PJON_CONNECTION_LOST) {
    Serial.print(F("No conn., ID "));
    Serial.print(bus.packets[data].content[0], DEC);
  }
  if(code == PJON_PACKETS_BUFFER_FULL) {
    Serial.print(F("P. buff. "));  // Packet buff. is full, has now a length of
    Serial.println(data, DEC);
    Serial.println(F("Bad config?")); // Possible wrong bus configuration!
                                      // higher PJON_MAX_PACKETS if necessary.
  }
  if(code == PJON_CONTENT_TOO_LONG) {
    Serial.print(F("Cont. too long: "));
    Serial.println(data);
  }
};

void receiver_function(uint8_t *payload, uint16_t length, const PJON_Packet_Info &packet_info) {
  /*
   * If data are being receiving
   */
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  Timer_Receiving = millis();
  Receiving = true;
  Receiving_From_Node = packet_info.sender_id;

  Serial.print(F("Rec., ID "));
  Serial.println(packet_info.sender_id);
  Serial.print(F("Cont. len. "));
  Serial.println(length);

  String content = "";
  char character;

  byte* infop = (byte*)&Sensors;

  for(uint8_t i = 0; i < length; i++) {
      character = (char)payload[i];
      content.concat(character);
      *(infop++) = payload[i];
  }

  // Outside module
  if (packet_info.sender_id == NODE_ID_OUTSIDE) {
    Node_Outside_Online = true;
    Timer_Node_Outside = millis();  // update timeout timer

    SENS_OUTSIDE = Sensors;         // assign received values to global struct

    Serial.print(F("Outs. T (DS): "));
    Serial.println(SENS_OUTSIDE.temperature);
    Serial.print(F("Outs. T (BMP): "));
    Serial.println(SENS_OUTSIDE.temperature2);
    Serial.print(F("Outs. H: "));
    Serial.println(SENS_OUTSIDE.humidity);
    Serial.print(F("Outs. P: "));
    Serial.println(SENS_OUTSIDE.pressure);
    Serial.println();
  }

  // Basement module
  if (packet_info.sender_id == NODE_ID_BASEMENT) {
    Node_Basement_Online = true;
    Timer_Node_Basement = millis(); // update timeout timer

    SENS_BASEMENT = Sensors;        // assign received values to global struct

    Serial.print(F("Bas. T (DS): "));
    Serial.println(SENS_BASEMENT.temperature);
    Serial.print(F("Bas. T (BMP): "));
    Serial.println(SENS_BASEMENT.temperature2);
    Serial.print(F("Bas. H: "));
    Serial.println(SENS_BASEMENT.humidity);
    Serial.print(F("Bas. P: "));
    Serial.println(SENS_BASEMENT.pressure);
    Serial.println();
  }

  digitalWrite(LED_BUILTIN, LOW);    // end of receiving
};


void EEPROM_write_long(uint8_t address, uint16_t value){
  /*
  * Write a 4 byte (32bit) long to the eeprom at
  * the specified address to address + 3.
  */
     //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
}

uint16_t EEPROM_read_long(uint8_t address){
  /*
  * Read the 4 bytes from the eeprom memory from the specified address
  */
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

float rel2abs_hum(float t, float h) {
  /*
   * Rel humidity to abs. humidity
   * https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
   * This formula is accurate to within 0.1% over the temperature range –30°C to +35°C.
   */
    double tmp = pow(2.718281828,(17.67*t)/(t+243.5));
    float res = (6.112*tmp*h*2.1674)/(273.15+t); // g of water in m3

    if (res < 0) {
      res = 0;
    }

    return res;
}

bool time_to_check_vent () {
  /*
   * Calc if we need ventilation not often than Vent_Check_Every minutes.
   */
    if (millis()-Timer_Check_Vent >= Vent_Check_Every) {
      Timer_Check_Vent = millis();  // reset counter
      return true;
    } else {
      return false;
  }
}

bool vent_too_long() {
  if (millis()-Timer_Vent_Running > Vent_Running_Max_Time) {
    return true;
  } else {
    return false;
  }
}

bool run_ventilation() {
  /*
   * Check all parameters and allow to run ventilation if needed.
   */

  if (Fan_On) {
    // increase fan work counter, minutes
    Vent_Work_Total = EEPROM_read_long(0)+(Vent_Check_Every/60/1000);
    EEPROM_write_long(0, Vent_Work_Total);

    Serial.print("V total ");
    Serial.println(EEPROM_read_long(0), DEC);
  }

  bool run = true;  // allowed by default

  Serial.println(F("Check V")); // Checking ventilation

  if (!Node_Outside_Active()) {
    Serial.println(F("Outs. no upd.")); // Outside module data are not up to date.
    run = false;
  }

  if (!Node_Basement_Active()) {
    Serial.println(F("Bas. no upd.")); // Basement module data are not up to date.
    run = false;
  }

  if (vent_too_long()) {
    Serial.println(F("Vent. too long"));  // TODO: power off period // Ventilator has been working too long already.
    run = false;
  }

  if (SENS_OUTSIDE.humidity == 0 || SENS_BASEMENT.humidity == 0) {
    Serial.println(F("Outs. or bas. H=0")); // Outside or basement humidity has wrong value (0).
  }

  if (SENS_OUTSIDE.temperature == -99.9 || SENS_BASEMENT.temperature == -99.9) {
    Serial.println(F("Outs. or bas. T=-99.9")); // Outside or basement temperature has wrong value (-99.9)."
  }

  if(rel2abs_hum(SENS_OUTSIDE.humidity, SENS_BASEMENT.temperature2)  >
     rel2abs_hum(SENS_BASEMENT.humidity, SENS_OUTSIDE.temperature2) -  HUM_DELTA) {
    Serial.println(F("Outs. AH > bas. AH - (H_DELTA)")); // Outside abs. hum  > basement abs. hum. - HUM_DELTA
    run = false;
  }

  if (SENS_OUTSIDE.temperature > BASEMENT_TEMP_MAX) {
    Serial.print(F("Outs. T > bas. max. T: ")); // Outside temp is higher then basement max. temp:
    Serial.print(BASEMENT_TEMP_MAX);
    Serial.println(F("."));
    run = false;
  }

  if (SENS_OUTSIDE.temperature < BASEMENT_TEMP_MIN) {
    Serial.print(F("Outs. T < bas. min. T: ")); // Outside temp is lower then basement min. temp:
    Serial.print(BASEMENT_TEMP_MIN);
    Serial.println(F("."));
    run = false;
  }

  if (!run) {
    Serial.println(F("No V")); // Deny to run ventilator.
    Serial.println();
  } else {
    Serial.println(F("Run V")); // All conditions are good to run the ventilator.
    Serial.println();
  }

  return run;
}

float hpa2mmhg (float hpa) {
  /*
   * Hectopascal to mm Hg convertor
   */
   return hpa *  0.7500616827;
}

void float2char(float number, char *output, bool sign=false) {
  /*
   * [-]XX.YY float only
   */
  char str_temp[6+1];
  //dtostrf(number, 5, 1, output);
  if (number>=0) {
    dtostrf(number, 4, 1, str_temp);
    if (sign) {
      sprintf(output, "+%s", str_temp);
    } else {
      sprintf(output, "%s", str_temp);
    }
  }
  else {
    dtostrf(number*(-1), 4, 1, str_temp);
    sprintf(output, "-%s", str_temp);
  }
}

void drawArrows() {
  /*
  * Small arrows on the right side explaining the outside and inside sensors data.
  */
  if (Node_Outside_Online) {
    // permanent icon
    u8g2.drawXBMP( 120, 0, arrow_outside_8_width, arrow_outside_8_height, arrow_outside_8_bits);
  } else {
    // blinking icon
    blink("arO");
  }

  if (Node_Basement_Online) {
    // permanent icon
    u8g2.drawXBMP( 120, 56, arrow_inside_8_width, arrow_inside_8_height, arrow_inside_8_bits);
  }
    else {
      // blinking icon
      blink("arB");
    }

}

void time_and_units(char *output, unsigned long seconds) {
  /*
   * Converts time in seconds into seconds, minutes, hours, days with the short units name.
   * 99 month is maximum.
   * 35 -> "35s",
   * 70 -> "1m",
   * 3650 -> "1h" ,
   * ...
   * 8553600 -> "3M" (months).
   */
  char units;
  int new_value;

  if(seconds >= SECONDS_99MONTHS) {         // months
      new_value = seconds/SECONDS_MONTH;
      units = 'M';
  } else if(seconds >= SECONDS_DAY) {       // days
      new_value = seconds/SECONDS_DAY;
      units = 'd';
  } else if(seconds >= SECONDS_HOUR) {      // hours
      new_value = seconds/SECONDS_HOUR;
      units = 'h';
  } else if(seconds >= SECONDS_MINUTE) {    // minutes
    new_value = seconds/SECONDS_MINUTE;
    units = 'm';
  } else  {                                 // seconds
      new_value = seconds;
      units = 's';
  }

  if (new_value > 99) {
    new_value = 99; // avoid char overrun
  }

  sprintf(output, "%2d%c", new_value, units);
}

void pageTemperature(){
  char outside_temp_str[5+1];   // +99.9
  char basement_temp_str[5+1];

  //float2char(sensors.temperature, outside_temp_str);
  float2char(SENS_OUTSIDE.temperature, outside_temp_str, true);
  //float2char(-99.9, outside_temp_str);

  //char outside_temp[] = dtostrf(myFloat, 1, 4, buffer);String(outside_temp);
  //char inside_temp[] = "-15.2";
  float2char(SENS_BASEMENT.temperature, basement_temp_str, true);

  u8g2.drawXBMP( 0, 16, temperature_width, temperature_height, temperature_bits);

  u8g2.setFont(u8g2_font_logisoso24_tn);
  u8g2.drawStr( right_x(outside_temp_str,32), 26, outside_temp_str);
  u8g2.drawStr( right_x(basement_temp_str,32), 62, basement_temp_str);

  u8g2.setFont(u8g2_font_pxplusibmvga8_mr);
  u8g2.drawCircle(104, 28, 2, U8G2_DRAW_ALL); // Degree symbol :), bacause font is truncated. +200 bytes.
  u8g2.drawStr(110, 36, "C");

  drawArrows();
  return 0;
}
void pageHumidity() {
  char outside_hum[6+1];  // 100.0
  char basement_hum[6+1];  // 100.0

  //itoa(round(sensors.humidity), outside_hum, 10);
  float2char(SENS_OUTSIDE.humidity, outside_hum);
  float2char(SENS_BASEMENT.humidity, basement_hum);


  u8g2.drawXBMP( 0, 16, humidity_32_width, humidity_32_height, humidity_32_bits);
  u8g2.setFont(u8g2_font_pxplusibmvga8_mr);
  u8g2.drawStr(8, 64, "RH");

  u8g2.setFont(u8g2_font_logisoso24_tr);
  u8g2.drawStr(right_x(outside_hum, 32), 26, outside_hum);
  u8g2.drawStr(right_x(basement_hum, 32), 64, basement_hum);


  u8g2.setFont(u8g2_font_pxplusibmvga8_mr);
  u8g2.drawStr(104, 36, "%");

  drawArrows();
  return 0;
}

void pageAbsHumidity() {
  char outside_hum[4+1]; // 99.9
  char basement_hum[4+1]; // 99.9

  //itoa(round(rel2abs_hum(sensors.humidity, sensors.temperature)), outside_hum, 10);
  float2char(rel2abs_hum(SENS_OUTSIDE.humidity, SENS_OUTSIDE.temperature2 ), outside_hum);
  float2char(rel2abs_hum(SENS_BASEMENT.humidity, SENS_BASEMENT.temperature2), basement_hum);

  u8g2.drawXBMP( 0, 16, humidity_abs_32_width, humidity_abs_32_height, humidity_abs_32_bits);
  u8g2.setFont(u8g2_font_pxplusibmvga8_mr);
  u8g2.drawStr(8, 64, "AH");

  u8g2.setFont(u8g2_font_logisoso24_tr);
  u8g2.drawStr(right_x(outside_hum, 32), 26, outside_hum);
  u8g2.drawStr(right_x(basement_hum, 32), 64, basement_hum);


  u8g2.setFont(u8g2_font_pxplusibmvga8_mr);
  u8g2.drawStr(104, 28, "g/");
  u8g2.drawStr(104, 44, "m3");

  drawArrows();
  return 0;
}

void pagePressure() {
  char outside_press[3+1];
  char basement_press[3+1];

  itoa(round(hpa2mmhg(SENS_OUTSIDE.pressure)), outside_press, 10);
  itoa(round(hpa2mmhg(SENS_BASEMENT.pressure)), basement_press, 10);

  u8g2.drawXBMP( 0, 16, pressure_32_width, pressure_32_height, pressure_32_bits);

  u8g2.setFont(u8g2_font_logisoso24_tn);

  u8g2.drawStr(right_x(outside_press, 32), 24, outside_press);
  u8g2.drawStr(right_x(basement_press, 32), 64, basement_press);

  u8g2.setFont(u8g2_font_pxplusibmvga8_mr);
  u8g2.drawStr(104, 28, "mm");
  u8g2.drawStr(104, 44, "Hg");

  drawArrows();
  return 0;
}

void pageInfo() {
  char temp_char[3+1];

  u8g2.setFont(u8g2_font_pxplusibmvga8_mr);

  u8g2.drawStr(0, 24, "V tot (m)");   // total hours of vent. runned
  u8g2.setCursor(104, 24);
  //u8g2.print(Vent_Work_Total);
  //sprintf(v_tot, "%dh", Vent_Work_Total);
  time_and_units(temp_char, Vent_Work_Total*60);  // need Vent_Work_Total minutes to seconds conversions
  u8g2.print(temp_char);

  u8g2.drawStr(0, 40, "Next chk"); // next check, seconds
  u8g2.setCursor(104, 40);
  time_and_units(temp_char, (Vent_Check_Every-(millis()-Timer_Check_Vent))/1000);
  u8g2.print(temp_char);

  //u8g2.drawStr(0, 56, "Last chk");
  //u8g2.setCursor(104, 56);
  //u8g2.print((millis()-Timer_Check_Vent)/1000);
}

void pageInfo2() {
  char temp_char[3+1];

  u8g2.setFont(u8g2_font_pxplusibmvga8_mr);

  u8g2.drawStr(72, 24, "Out Bas");  // Outside, Basement
  u8g2.drawStr(0, 40, "Last chk"); // last checked, seconds

  u8g2.setCursor(72, 40);
  time_and_units(temp_char, (millis()-Timer_Node_Outside)/1000);
  u8g2.print(temp_char);

  u8g2.setCursor(104, 40);
  time_and_units(temp_char, (millis()-Timer_Node_Basement)/1000);
  u8g2.print(temp_char);
}

unsigned int center_x(char* message, int margin_left=0, int margin_right=0) {
  /*
   * Centers the string.
   */
  return (DISPLAY_WIDTH-margin_left-margin_right-u8g2.getStrWidth(message))/2+margin_left;
}

unsigned int right_x(char* message, int margin_right=0) {
  return DISPLAY_WIDTH-margin_right-u8g2.getStrWidth(message);
}

void blink(char icon[3]) {
  /*
   * Blink icons.
   */

  if (millis()-Timer_Blink < BLINK_DUR) {

      if (icon == "fan") {
        u8g2.drawXBMP(0, 0, fan_8_width, fan_8_height, fan_8_bits);
      } else if (icon == "arO") {
        u8g2.drawXBMP( 120, 0, arrow_outside_8_width, arrow_outside_8_height, arrow_outside_8_bits);
      } else if (icon == "arB") {
        u8g2.drawXBMP( 120, 56, arrow_inside_8_width, arrow_inside_8_height, arrow_inside_8_bits);
      }

  }
  if (millis()-Timer_Blink >= BLINK_DUR*2) {
    Timer_Blink = millis();
  }
}

void blink_receiving() {
  /*
   * Blink receiving icon.
   */
  if (millis()-Timer_Receiving < BLINK_DUR/2) {
    u8g2.drawXBMP(8, 0, arrow_receive_8_width, arrow_receive_8_height, arrow_receive_8_bits);
    u8g2.setFont(u8g2_font_profont12_mn);
    u8g2.setCursor(16, 8);
    u8g2.print(Receiving_From_Node);
  } else {
    Receiving = false;
    //Timer_Receiving = millis();
  }
}


bool Node_Outside_Active() {
  /*
   * Check if the Node data are up to date.
   */
  if (millis()-Timer_Node_Outside >= Node_Timeout) {
    Node_Outside_Online = false;
    return false;
  } else {
    Node_Outside_Online = true;
    return true;
  }
}

bool Node_Basement_Active() {
  /*
   * Check if the Node data are up to date.
   */
  if (millis()-Timer_Node_Basement >= Node_Timeout) {
    Node_Basement_Online = false;
    return false;
  } else {
    Node_Basement_Online = true;
    return true;
  }
}

void oled_display_state(bool turn_on=true) {
  /*
  * Turns OLED display on or off.
  */
  if (turn_on) {
    u8g2.setPowerSave(false);
    Timer_Display_On = millis();
    Display_On = true;
  } else {
    u8g2.setPowerSave(true);
    Display_On = false;
  }
}

void oled_display_on(bool state=false) {
  /*
  * Turn OLED display on.
  */
  oled_display_state(true);
}

void oled_display_off () {
  /*
  * Turn OLED display off.
  */
  oled_display_state(false);
}


void loop() {

  // wireless updates
  bus.receive(50000);
  bus.update();

  //u8g2.clearBuffer(); // if full buffer mode
  u8g2.firstPage();     // page mode
  do {
    if (Receiving) {
      //blink_fan();
      blink_receiving();
    }

    if (Fan_On) {
      blink("fan");
    }

    if (Display_On) {
        (*pages[current_page])();
        //u8g2.sendBuffer();  // if full buffer mode

      if (millis()-Timer_Page_Display >= page_duration[current_page]) {
        current_page++;
        if (current_page > pages_count-1)
          current_page=0;

        Timer_Page_Display = millis();
      }
    }
  } while ( u8g2.nextPage() );

  // check if we need to run vetilation
  if (time_to_check_vent()) {
    if (run_ventilation()) {
      Timer_Vent_Running = millis(); // reset counter
      // send signal to run
      CONTROL Command = {"fan","on",0};
      bus.send(NODE_ID_BASEMENT, (const char*)&Command, sizeof(CONTROL));
      Fan_On = true;
    } else {
      CONTROL Command = {"fan","off",0};
      bus.send(NODE_ID_BASEMENT, (const char*)&Command, sizeof(CONTROL));
      Fan_On = false;
    }
  }

  if (millis()-Timer_Display_On >= DISPLAY_SAVER && Display_On) {
    oled_display_off();
  }

  if (!Display_On) {
    if (digitalRead(PROXIMITY_PIN) == LOW) {
      current_page=0; // start displaying pages from the begining
      Timer_Page_Display = millis(); // reset current page time
      oled_display_on();
    }
  }

};
