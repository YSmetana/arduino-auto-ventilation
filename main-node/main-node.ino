/*
 * Inspired by:
 * http://forum.amperka.ru/threads/%D0%9A%D0%BE%D0%BD%D1%82%D1%80%D0%BE%D0%BB%D1%8C-%D0%B2%D0%BB%D0%B0%D0%B6%D0%BD%D0%BE%D1%81%D1%82%D0%B8-%D0%B8-%D1%82%D0%B5%D0%BC%D0%BF%D0%B5%D1%80%D0%B0%D1%82%D1%83%D1%80%D1%8B-%D0%B2-%D0%BF%D0%BE%D0%B3%D1%80%D0%B5%D0%B1%D0%B5-%D0%BF%D0%BE%D0%B4%D0%B2%D0%B0%D0%BB%D0%B5.9557/
 * http://arduino.ru/forum/proekty/kontrol-vlazhnosti-podvala-arduino-pro-mini
 * http://arduinolab.pw/index.php/2017/05/04/upravlenie-vytyazhkoj-v-pogrebe-ili-podvale/
 */

#define NODE_ID_ME 1          // PJON device ID
#define NODE_ID_OUTSIDE 10
#define NODE_ID_BASEMENT 20

#define BASEMENT_TEMP_MIN 5   // minimum temperature in basement
#define BASEMENT_TEMP_MAX 30  // maximum temperature in basement
#define HUM_DELTA 0.5         // difference between outside and basement abs. hum. to run vent.

#define Node_Request 0.5 *60*1000000  // minutes, how often request data from a node (PJON microseconds)
#define Node_Timeout 1 *60*1000UL   // minutes, time after which node considered unreachable (Arduino milliseconds)

#define Vent_Check_Every 1 *60*1000UL         // minutes, how often calculate if we need vent.
#define Vent_Running_Max_Time 8 *60*60*1000UL // hours, maximum time keeping the vent running.
#define Vent_Rest_After_Run 5 *60*1000UL      // hours, maximum time keeping the vent running.

/*
 * Other settings
 */
#define BLINK_DUR 500UL             // when blinking icons on the dispay
#define DISPLAY_SAVER 1 *60*1000UL  // minutes, display goes off after this time
#define PROXIMITY_PIN 2
bool Display_On = true;             // display state
bool Receiving = false;             // data receiving status
byte Receiving_From_Node;           // last data received from this node
bool Fan_On = false;                // fan is working



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
unsigned long Timer_Display_On = millis(); // node timeout

/*
 * Data structures
 */
struct SENSORS{       // for sending data
  float temperature;
  float temperature2;
  float humidity;
  float pressure;
};

struct CONTROL {       // for controlling nodes
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
 */
#include <U8g2lib.h>
// OLED 0.96" I2C 128x64, https://arduino.ua/prod1263-oled-displei-modyl-belii
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // full buffer
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

#include "images.h"  // icons

/*
 * Functions
 */
void pageTemperature();
void pageHumidity();
void pageAbsHumidity();
void pagePressure();
void pageTimeDay();
void blink_fan();
void float2char (float value, char * input, bool sign=false);
unsigned int center_x(char* message, int margin_left=0, int margin_right=0);
unsigned int right_x(char* message, int margin_right=0);
bool run_ventilation();

/*
 * Pages on display
 */
int current_page = 0;
void (*pages[])() = {pageTemperature, pageHumidity, pageAbsHumidity, pagePressure, pageTimeDay};
unsigned int pages_count = 4;
int page_duration[] = {3000UL, 3000UL, 3000UL, 3000UL, 1000UL};

/*
 * SETUP
 */
void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  Serial.print("My ID is "); Serial.print(NODE_ID_ME); Serial.println(".");
  Serial.print("Req. data from nodes every "); Serial.print(Node_Request/60/1000000); Serial.println("m.");
  Serial.print("Node timeout is "); Serial.print(Node_Timeout/60/1000); Serial.println("m.");
  Serial.print("Calc. if we need vent. every "); Serial.print(Vent_Check_Every/60/1000); Serial.println("m.");
  Serial.print("Max. dur. of vent. is "); Serial.print(Vent_Running_Max_Time/60/60/1000); Serial.println("h.");
  Serial.print("Min. temp in bas. is "); Serial.print(BASEMENT_TEMP_MIN); Serial.println("°C.");
  Serial.print("Max. temp in bas. is "); Serial.print(BASEMENT_TEMP_MAX); Serial.println("°C.");
  Serial.print("Abs hum. delta is "); Serial.print(HUM_DELTA); Serial.println(" g/m3.");
  Serial.println();

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
  delay(TS_RESPONSE_TIME_OUT/1000 * 5); // 2 times more then PJON responce time out
  bus.send_repeatedly(NODE_ID_BASEMENT, (const char*)&Command, sizeof(CONTROL), Node_Request); // seconds * 1000000
  delay(TS_RESPONSE_TIME_OUT/1000 * 5); // 2 times more then PJON responce time out

  Timer_Check_Vent = millis(); // to check if its is time to calc ventilation process
};

void error_handler(uint8_t code, uint16_t data, void *custom_pointer) {
  /*
   * Pjon errors handlers
   */
  if(code == PJON_CONNECTION_LOST) {
    Serial.print("Conn. with ID ");
    Serial.print(bus.packets[data].content[0], DEC);
    Serial.println(" lost.");
  }
  if(code == PJON_PACKETS_BUFFER_FULL) {
    Serial.print("Packet buff. is full, has now a length of ");
    Serial.println(data, DEC);
    Serial.println("Possible wrong bus configuration!");
    Serial.println("higher PJON_MAX_PACKETS if necessary.");
  }
  if(code == PJON_CONTENT_TOO_LONG) {
    Serial.print("Content is too long, length: ");
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

  Serial.print("Rec. from ID ");
  Serial.println(packet_info.sender_id);
  Serial.print("Cont. len. ");
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
    Timer_Node_Outside = millis();  // update timeout timer

    SENS_OUTSIDE = Sensors;         // assign received values to global struct

    Serial.print("Outside temp: ");
    Serial.println(SENS_OUTSIDE.temperature);
    Serial.print("Outside hum: ");
    Serial.println(SENS_OUTSIDE.humidity);
    Serial.print("Outside press: ");
    Serial.println(SENS_OUTSIDE.pressure);
    Serial.println();
  }

  // Basement module
  if (packet_info.sender_id == NODE_ID_BASEMENT) {
    Timer_Node_Basement = millis(); // update timeout timer

    SENS_BASEMENT = Sensors;        // assign received values to global struct

    Serial.print("Basement temp: ");
    Serial.println(SENS_BASEMENT.temperature);
    Serial.print("Basement hum: ");
    Serial.println(SENS_BASEMENT.humidity);
    Serial.print("Basement press: ");
    Serial.println(SENS_BASEMENT.pressure);
    Serial.println();
  }

  digitalWrite(LED_BUILTIN, LOW);    // end of receiving
};

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
    //return true;
  } else {
    return false;
  }
}

bool run_ventilation() {
  /*
   * Check all parameters and allow to run ventilation if needed.
   */
  bool run = true;  // allowed by default

  Serial.println("Check if we can run ventilator...");

  if (!Node_Outside_Active()) {
    Serial.println("Outside module data are not up to date.");
    run = false;
  }

  if (!Node_Basement_Active()) {
    Serial.println("Basement module data are not up to date.");
    run = false;
  }

  if (vent_too_long()) {
    Serial.println("Ventilator has been working too long already.");  // TODO: power off period
    run = false;
  }

  if (SENS_OUTSIDE.humidity == 0 || SENS_BASEMENT.humidity == 0) {
    Serial.println("Outside or basement humidity has wrong value (0).");
  }

  if (SENS_OUTSIDE.temperature == -99.9 || SENS_BASEMENT.temperature == -99.9) {
    Serial.println("Outside or basement temperature has wrong value (-99.9).");
  }

  if(rel2abs_hum(SENS_OUTSIDE.humidity, SENS_OUTSIDE.temperature2) -
     rel2abs_hum(SENS_BASEMENT.humidity, SENS_BASEMENT.temperature2)
     >= HUM_DELTA) {
    Serial.println("Outside abs. hum > basement abs. hum.");
    run = false;
  }

  if (SENS_OUTSIDE.temperature > BASEMENT_TEMP_MAX) {
    Serial.print("Outside temp is higher then basement max. temp: ");
    Serial.print(BASEMENT_TEMP_MAX);
    Serial.println(".");
    run = false;
  }

  if (SENS_OUTSIDE.temperature < BASEMENT_TEMP_MIN) {
    Serial.print("Outside temp is lower then basement min. temp: ");
    Serial.print(BASEMENT_TEMP_MIN);
    Serial.println(".");
    run = false;
  }

  if (!run) {
    Serial.println("Deny to run ventilator.");
    Serial.println();
  } else {
    Serial.println("All conditions are good to run the ventilator.");
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

void pageTemperature(){
  char outside_temp_str[5+1]; // +99.9
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

  u8g2.setFont(u8g2_font_pxplusibmvga8_mf);
  u8g2.drawUTF8(104, 36, "°C");

  u8g2.drawXBMP( 120, 0, arrow_outside_8_width, arrow_outside_8_height, arrow_outside_8_bits);
  u8g2.drawXBMP( 120, 56, arrow_inside_8_width, arrow_inside_8_height, arrow_inside_8_bits);
  return 0;
}
void pageHumidity() {
  char outside_hum[6+1];  // 100.0
  char basement_hum[6+1];  // 100.0

  //itoa(round(sensors.humidity), outside_hum, 10);
  float2char(SENS_OUTSIDE.humidity, outside_hum);
  float2char(SENS_BASEMENT.humidity, basement_hum);


  u8g2.drawXBMP( 0, 16, humidity_32_width, humidity_32_height, humidity_32_bits);
  u8g2.setFont(u8g2_font_pxplusibmvga8_mf);
  u8g2.drawUTF8(8, 64, "RH");

  u8g2.setFont(u8g2_font_logisoso24_tr);
  u8g2.drawStr(right_x(outside_hum, 32), 26, outside_hum);
  u8g2.drawStr(right_x(basement_hum, 32), 64, basement_hum);


  u8g2.setFont(u8g2_font_pxplusibmvga8_mf);
  u8g2.drawUTF8(104, 36, "%");

  u8g2.drawXBMP( 120, 0, arrow_outside_8_width, arrow_outside_8_height, arrow_outside_8_bits);
  u8g2.drawXBMP( 120, 56, arrow_inside_8_width, arrow_inside_8_height, arrow_inside_8_bits);
  return 0;
}

void pageAbsHumidity() {
  char outside_hum[4+1]; // 99.9
  char basement_hum[4+1]; // 99.9

  //itoa(round(rel2abs_hum(sensors.humidity, sensors.temperature)), outside_hum, 10);
  float2char(rel2abs_hum(SENS_OUTSIDE.humidity, SENS_OUTSIDE.temperature2 ), outside_hum);
  float2char(rel2abs_hum(SENS_BASEMENT.humidity, SENS_BASEMENT.temperature2), basement_hum);

  u8g2.drawXBMP( 0, 16, humidity_abs_32_width, humidity_abs_32_height, humidity_abs_32_bits);
  u8g2.setFont(u8g2_font_pxplusibmvga8_mf);
  u8g2.drawUTF8(8, 64, "AH");

  u8g2.setFont(u8g2_font_logisoso24_tr);
  u8g2.drawStr(right_x(outside_hum, 32), 26, outside_hum);
  u8g2.drawStr(right_x(basement_hum, 32), 64, basement_hum);


  u8g2.setFont(u8g2_font_pxplusibmvga8_mf);
  u8g2.drawUTF8(104, 28, "g/");
  u8g2.drawUTF8(104, 44, "m3");

  u8g2.drawXBMP( 120, 0, arrow_outside_8_width, arrow_outside_8_height, arrow_outside_8_bits);
  u8g2.drawXBMP( 120, 56, arrow_inside_8_width, arrow_inside_8_height, arrow_inside_8_bits);
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

  u8g2.setFont(u8g2_font_pxplusibmvga8_mf);
  u8g2.drawUTF8(104, 28, "mm");
  u8g2.drawUTF8(104, 44, "Hg");

  u8g2.drawXBMP( 120, 0, arrow_outside_8_width, arrow_outside_8_height, arrow_outside_8_bits);
  u8g2.drawXBMP( 120, 56, arrow_inside_8_width, arrow_inside_8_height, arrow_inside_8_bits);
  return 0;
}

void pageTimeDay() {
  char time_now[] = "08:48";
  char date_now[] = "22.07.18";

  u8g2.setFont(u8g2_font_inb30_mn);
  u8g2.drawStr( center_x(time_now), 40, time_now);

  u8g2.setFont(u8g2_font_pxplusibmvga8_mf);
  u8g2.drawUTF8(center_x(date_now), 64, date_now);
  return 0;
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

void blink_fan() {
  /*
   * Blink fan icon.
   */
  if (millis()-Timer_Blink < BLINK_DUR) {
    u8g2.drawXBMP(0, 0, fan_8_width, fan_8_height, fan_8_bits);
  }
  if (millis()-Timer_Blink >= BLINK_DUR && millis()-Timer_Blink < BLINK_DUR*2) {
    //u8g2.drawXBMP(0, 0, fan_8_width, fan_8_height, fan_8_bits);
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
   * Check if the Noda data are up to date.
   */
  if (millis()-Timer_Node_Outside >= Node_Timeout) {
    return false;
  } else {
    return true;
  }
}

bool Node_Basement_Active() {
  /*
   * Check if the Noda data are up to date.
   */
  if (millis()-Timer_Node_Basement >= Node_Timeout) {
    return false;
  } else {
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

  u8g2.clearBuffer();

  if (Receiving) {
    //blink_fan();
    blink_receiving();
  }

  if (Fan_On) {
    blink_fan();
  }

  if (Display_On) {
      (*pages[current_page])();
      u8g2.sendBuffer();

    if (millis()-Timer_Page_Display >= page_duration[current_page]) {
      current_page++;
      if (current_page > pages_count-1)
        current_page=0;

      Timer_Page_Display = millis();
    }
  }

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
