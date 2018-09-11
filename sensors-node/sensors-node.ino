// TODO: Check if all sensors are available
#define DEBUG false
#define Serial if(DEBUG)Serial  // https://forum.arduino.cc/index.php?topic=155268.0

#define MY_PJON_ID 20 // PJON device ID

#define FAN_CONTROL_PIN 4
#define HUM_CORR 0 // correction to humidity result

#define NODE_TIMEOUT 2 *60*1000UL   // minutes, time after which data considered obsolete (Arduino milliseconds)
unsigned long Timer_Fan_Control = millis();

bool Fan_On = false;

/*
 * DS18B20 temperature sensor
 * Datasheet: https://cdn.sparkfun.com/datasheets/Sensors/Temp/DS18B20.pdf
 * Tutorial:  https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806
 *            http://mypractic.ru/ds18b20-datchik-temperatury-s-interfejsom-1-wire-opisanie-na-russkom-yazyke.html#22z
 */
#include <OneWire.h>
#define ONE_WIRE_BUS 2
#include <DallasTemperature.h>  // https://github.com/milesburton/Arduino-Temperature-Control-Library
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
#define DS18B20_PRECISION 12    // 9-12. 12 = 0,0625 (1/16) °C

/*
 * BME280 weather sensor
 * Datasheet: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280_DS001-12.pdf
 */
#include <Adafruit_Sensor.h>            // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_BME280.h>            // https://github.com/adafruit/Adafruit_BME280_Library
#define SEALEVELPRESSURE_HPA (1013.25)  // for altitude measurment
Adafruit_BME280 bme;                    // I2C
#define BME280_ADDRESS (0x76)           // I2C address

/*
 * HC-12 433 MHz UART transiver
 * Datasheet: https://www.elecrow.com/download/HC-12.pdf
 * http://radiolaba.ru/microcotrollers/podklyuchenie-radiomoduley-hc-12-na-osnove-transivera-si4463.html
 */
#include <AltSoftSerial.h>  // https://github.com/PaulStoffregen/AltSoftSerial , https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
AltSoftSerial HC12;         // Arduino Pro Mini: HC-12_TX-PIN8_RX, HC-12_RX-PIN9_TX; Arduino Mega: HC-12_TX-PIN48_RX, HC-12_RX-PIN46_TX;
#define HC12_SPEED 2400

#define TS_BYTE_TIME_OUT 20000 // 80 ms max
#define TS_RESPONSE_TIME_OUT 1000000
#define PJON_PACKET_MAX_LENGTH 32
#define PJON_INCLUDE_TS
#define TS_MAX_ATTEMPTS 1
#include <PJON.h>
// <Strategy name> bus(selected device id)
PJON<ThroughSerial> bus(MY_PJON_ID);


struct CONTROL{       // for controlling nodes
  char command[4];    // 3 bytes + 1 byte for array termination \0
  char subcommand[4];
  int16_t value;
};

CONTROL control;

struct SENSORS{       // for sending data
  float temperature;
  float temperature2;
  float humidity;
  float pressure;
};

void log(char message[64], bool new_line = true) {
  if (DEBUG) {
    Serial.print(message);
    if (new_line) {
      Serial.println();
    }
  }
}

void setup() {
  pinMode(FAN_CONTROL_PIN, OUTPUT);

  // Initialize LED 13 to be off
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // Serial port
  Serial.begin(9600);
  Serial.println("Start");

  Serial.print("My ID is ");
  Serial.print(MY_PJON_ID);
  Serial.println(".");

  // DS18B20 temperature sensor
  // locate devices on the bus
  DS18B20.begin();
  if (DS18B20.getDeviceCount() == 0) {
    Serial.println("No device found!!!");
    Serial.println("Stopping.");
    while(1);
  } else {
    DS18B20.setResolution(DS18B20_PRECISION);
    Serial.print("DS18B20 sensor started (");
    Serial.print(DS18B20.getDeviceCount(), DEC);
    Serial.println(").");
  }

  // BME280 sensor
  if (!bme.begin(BME280_ADDRESS)) {
      Serial.println("Could not find BME280 sensor, check wiring!");
      Serial.println("No device found!!!");
      Serial.println("Stopping.");
      while (1);
  } else {
    Serial.println("BME280 sensor started.");
  }

  //BME280 scenario
  Serial.println("Weather Station Scenario:");
  //Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
  //Serial.println("filter off.");
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);
                  //Adafruit_BME280::STANDBY_MS_1000);


  // Set HC12 baudrate (you must use the one configured in HC12, default 9600)
  HC12.begin(HC12_SPEED);
  //delay(1000);
  //HC12.begin(9600);
  //HC12.write("AT+B2400");
  //delay(1000);

  // Pass the HC12 Serial instance you want to use for PJON communication
  bus.strategy.set_serial(&HC12);
  bus.set_error(error_handler);
  bus.set_receiver(receiver_function);
  //bus.set_synchronous_acknowledge(false);
  bus.begin();
};

void fan_on() {
  Serial.println("Fan ON.");
  digitalWrite(FAN_CONTROL_PIN, HIGH);
  Fan_On = true;
}

void fan_off() {
  Serial.println("Fan OFF.");
  digitalWrite(FAN_CONTROL_PIN, LOW);
  Fan_On = false;
}

void error_handler(uint8_t code, uint16_t data, void *custom_pointer) {
  if(code == PJON_CONNECTION_LOST) {
    Serial.print("Conn. with ID ");
    Serial.print(bus.packets[data].content[0], DEC);
    Serial.println(" is lost.");
  }
  if(code == PJON_PACKETS_BUFFER_FULL) {
    Serial.print("Packet buffer is full, has now a length of ");
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
  /* Make use of the payload before sending something, the buffer where payload points to is
     overwritten when a new message is dispatched */

  Serial.println();

  Serial.print("Receiving from node ");
  Serial.println(packet_info.sender_id);

  Serial.print("Content length - ");
  Serial.println(length);

  byte* cont_byte = (byte*)&control;

  // receive indicator ON
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

  Serial.print("Received content: ");

  for(uint8_t i = 0; i < length; i++) {
      *(cont_byte++) = payload[i];
      Serial.print((char)payload[i]);
  }
  Serial.println();

  // receive indicator OFF
  digitalWrite(LED_BUILTIN, LOW);

  Serial.print("Cmd: ");
  Serial.println(control.command);

  Serial.print("Subcmd: ");
  Serial.println(control.subcommand);

  Serial.print("Val: ");
  Serial.println(control.value);

  if (strcmp(control.command, "get") == 0 && strcmp(control.subcommand, "all") == 0) {
    bme.takeForcedMeasurement();
    SENSORS outside_sensors = { DS18B20_temp() , bme_temp(), bme_hum() , bme_press() };

    if(!bus.update()) {
      Serial.println("Waiting before response.");
      delay(TS_RESPONSE_TIME_OUT/1000);
      Serial.println("Sending resp.");
      bus.send(1, (const char*)&outside_sensors, sizeof(SENSORS));
    }
  }

  if (strcmp(control.command, "fan") == 0 && strcmp(control.subcommand, "on") == 0) {
    Timer_Fan_Control = millis();   // update timer
    fan_on();
  }

  if (strcmp(control.command, "fan") == 0 && strcmp(control.subcommand, "off") == 0) {
    fan_off();
  }
};

float bme_temp() {
    float temp;
    temp = bme.readTemperature();

    Serial.print("BME temp = ");
    Serial.print(temp);
    Serial.println(" *C");

    return temp;
}

float bme_press() {
  float press;
  press = bme.readPressure() / 100.0F;

  Serial.print("Press = ");
  Serial.print(press);
  Serial.println(" hPa");

  return press;
};

float bme_hum() {
  float hum;
  hum = bme.readHumidity();

  Serial.print("Hum = ");
  Serial.print(hum);
  Serial.println(" %");

  return hum + HUM_CORR;
};


float DS18B20_temp() {
  float temp;

  Serial.println("Requesting temp...");
  DS18B20.requestTemperatures(); // Send the command to get temperature readings
  //delay (750+50);

  temp = DS18B20.getTempCByIndex(0);

  if (temp == -127) {
    Serial.println("Wrong temp. from DS18B20 (-127).");
    temp = -99.9;
  }

  Serial.print("DS18B20 temperature is: ");
  Serial.println(temp);
  return temp;
};

void loop() {
  bus.receive(50000);
  bus.update();

  if (Fan_On) {
    if (millis() - Timer_Fan_Control >= NODE_TIMEOUT) {
      Serial.println("No conf. of fan run from main node.");
      fan_off();
    }
  }

};
