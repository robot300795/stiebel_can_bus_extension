/**
 * stiebel eltron can telegramm structure
 * length           --> 7 byte as hex values
 * example          --> send (A1 00 fa 07 49) --> revice (D2 00 fa 07 49 xx xx)
 * can-id           --> 8*(A0 & f0) + (00 & 0f) --> 8*(<Digit 1>) + (<Digit 4>) --> can-id 500
 * can-id           --> 8*(d0 & f0) + (00 & 0f) --> 8*(<Digit 1>) + (<Digit 4>) --> can-id 780
 * digit 2.         --> value 1 request, value 2 reponse
 * hex value 2.     --> broadcast --> 0x79 (at regular time intervalls)
 * hex value 3.     --> 0xfa for extention frame (not used)
 * hex value 4.-5.  --> data index values
 * hex value 6.-7.  --> data values (int16)
 * http://juerg5524.ch/list_data.php
 */

/**
 * allowed sender can id: 680 or 700 or 780
 * BE CAREFUL WITH THIS !!!
 * 
 * some values are split between two telegramms
 */

/** Circuit:
  * Spi      --> pins MOSI 16, MISO 14, SCK 15
  * Ethernet --> pins CS 10 
  * SD Card  --> pins CS 4 
  * Can wpm3 --> pins CS 4, INT 2
  * Can fek  --> pins CS 5, INT 3
  * Display  --> pins 20(SDA), 21(SCL)
  * Encoder  --> pins 
*/

// 1664 2 17 282

#include <Arduino.h>
#include <Ethernet.h>
#include <SD.h>
#include <SPI.h>
// Modbus
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
// Can
#include <ACAN2517FD.h>
// digital potentiometer
#include <Adafruit_DS3502.h>
// encoder and display
#include <Wire.h>
#include <menu.h>
#include <menuIO/chainStream.h>
#include <menuIO/encoderIn.h>
#include <menuIO/keyIn.h>
#include <menuIO/u8g2Out.h>

using namespace Menu;

#define pins_led          13
#define pins_watchdog     40
#define pins_relay_can    41
#define pins_relay_sensor 42

#define pins_ethernet_cs  10
// #define pins_ethernet_int 2
#define pins_sd_card_cs   4

#define pins_encoder_clk  19
#define pins_encoder_dt   18
#define pins_encoder_sw   22

#define pins_can_wpm3_cs  53
#define pins_can_wpm3_int 2
#define pins_can_fek_cs   48
#define pins_can_fek_int  3

#define WIRE              Wire
#define fontName          u8g2_font_t0_11_tf
#define fontX             6
#define fontY             11
#define offsetX           0
#define offsetY           3
#define U8_Width          128
#define U8_Height         64
#define MAX_DEPTH         1

const colorDef<uint8_t> colors[6] MEMMODE={
  {{0,0},{0,1,1}},  // bgColor
  {{1,1},{1,0,0}},  // fgColor
  {{1,1},{1,0,0}},  // valColor
  {{1,1},{1,0,0}},  // unitColor
  {{0,1},{0,0,1}},  // cursorColor
  {{1,1},{1,0,0}},  // titleColor
};

// define Modbus server/slave 
// input Mac and IP address
int modbus_slave_id      = 3;
int num_Coils            = 1;
int num_HoldingRegisters = 3;
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 178, 102);
EthernetServer  ethServer(502);
ModbusTCPServer modbusTCPServer;

// define Can bus
unsigned long can_baudrate = 20100;
bool can_logging_wpm3      = true;
bool can_logging_fek       = true;
ACAN2517FD can_wpm3 (pins_can_wpm3_cs, SPI, pins_can_wpm3_int);
ACAN2517FD can_fek  (pins_can_fek_cs, SPI, pins_can_fek_int);

// define digital potentiometer
int i2c_adress_potentiometer = 0x28;
Adafruit_DS3502 potentiometer = Adafruit_DS3502();

// define SD card and logger file
bool status_sd_card_detected = false;
File File_Datalogger;

// define display and encoder
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
encoderIn<pins_encoder_clk, pins_encoder_dt> encoder;
encoderInStream<pins_encoder_clk, pins_encoder_dt> encStream(encoder,4);

// keyboard with only one key as the encoder button
keyMap encBtn_map[]={{-pins_encoder_sw, defaultNavCodes[enterCmd].ch}}; //negative pin numbers use internal pull-up, this is on when low
keyIn<1> encButton(encBtn_map);                                         //1 is the number of keys

MENU(mainMenu,
  "Main menu",      doNothing, noEvent, wrapStyle,
  OP("Status",      doNothing, noEvent),
  OP("IP Adress",   doNothing, noEvent),
  OP("Mac Adress",  doNothing, noEvent),
  EXIT("Exit")
);

MENU_INPUTS(in, &encStream, &encButton);

MENU_OUTPUTS(out,MAX_DEPTH,
  U8G2_OUT(display, colors, fontX, fontY, offsetX, offsetY, {0, 0, U8_Width/fontX, U8_Height/fontY}),
  NONE
);

NAVROOT(nav, mainMenu, MAX_DEPTH, in, out);

// internal values
unsigned long baudrate_serial = 115200;
unsigned long watchdog_timer  = millis();
bool watchdog_value           = 0;
bool mode_passthrough         = 0;
int retry                     = 0;


void watchdog(unsigned int interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - watchdog_timer >= interval) {
    // save the last time you blinked the LED
    watchdog_timer = currentMillis;
    // change watchdog state
    if (watchdog_value == LOW) {
      watchdog_value = HIGH;
    } else {
      watchdog_value = LOW;
    }
    // set the LED with the watchdog state
    digitalWrite(pins_led, watchdog_value);
  }
}

/**
 * [0] --> can-id
 * [1] --> message flage (0 broadcast, 1 request, value 2 reponse, 3 invalid)
 * [2] --> data index value
 * [3] --> data (int16)
 */
unsigned int *decode_can_data(CANFDMessage &can_data) {
  static unsigned int decoded_can_data[4];
  if (can_data.len == 7) {
    // [0] --> can-id
    decoded_can_data[0] = (8*(can_data.data[0] & 0xf0)) + (can_data.data[1] & 0x0f);
    // [1] --> message flage (0 broadcast, 1 request, value 2 reponse)
    if ((can_data.data[0] & 0x0f) == 1) {
      decoded_can_data[1] = 1;
    }
    else if ((can_data.data[0] & 0x0f) == 2) {
      decoded_can_data[1] = 2;
    }
    else if (can_data.data[2] == 0x79) {
      decoded_can_data[1] = 0;
    }
    // [2] --> data index value
    decoded_can_data[2] = (can_data.data[3] << 8) + can_data.data[4];
    // [3] --> data (int16)
    decoded_can_data[3] = (can_data.data[5] << 8) + can_data.data[6];
    return decoded_can_data;
  }
  else {
    Serial.println("Warning: Invalid decoding length!");
    static unsigned int decoded_can_data[] = {0x00, 0x03, 0x00, 0x00};
    return decoded_can_data;
  }  
}

void update_can_data(CANFDMessage &can_data, unsigned int data_value) {
  //data value
  can_data.data[5] = highByte(data_value);
  can_data.data[6] = lowByte(data_value);
}

void logging_can_data(CANFDMessage &can_data, const char *filename) {
  if (status_sd_card_detected) {
    unsigned long timestamp = millis();
    File_Datalogger = SD.open(filename, FILE_WRITE);
    if (File_Datalogger) {
      File_Datalogger.print(timestamp); 
      File_Datalogger.print(",");
      File_Datalogger.print(can_data.data[0], HEX); 
      File_Datalogger.print(",");
      File_Datalogger.print(can_data.data[1], HEX); 
      File_Datalogger.print(",");
      File_Datalogger.print(can_data.data[2], HEX); 
      File_Datalogger.print(",");
      File_Datalogger.print(can_data.data[3], HEX); 
      File_Datalogger.print(",");
      File_Datalogger.print(can_data.data[4], HEX); 
      File_Datalogger.print(",");
      File_Datalogger.print(can_data.data[5], HEX); 
      File_Datalogger.print(",");
      File_Datalogger.println(can_data.data[6], HEX);
      File_Datalogger.close();
    }
    else {
      Serial.print("Error: Opening ");
      Serial.print(filename);
      Serial.println("!");
    }
  }
}

void serial_can(CANFDMessage &can_data) {
  Serial.print(can_data.data[0], HEX); 
  Serial.print(" ");
  Serial.print(can_data.data[1], HEX); 
  Serial.print(" ");
  Serial.print(can_data.data[2], HEX); 
  Serial.print(" ");
  Serial.print(can_data.data[3], HEX);
  Serial.print(" ");
  Serial.print(can_data.data[4], HEX);
  Serial.print(" ");
  Serial.print(can_data.data[5], HEX);
  Serial.print(" ");
  Serial.println(can_data.data[6], HEX);
}

void serial_can_decoded(unsigned int decoded_can_data[]) {
  Serial.print(decoded_can_data[0], HEX); 
  Serial.print(" ");
  Serial.print(decoded_can_data[1]); 
  Serial.print(" ");
  Serial.print(decoded_can_data[2]); 
  Serial.print(" ");
  Serial.println(decoded_can_data[3]);
}

void setup() {
  // *** start serial communications and wait for port to open ***
  Serial.begin(baudrate_serial);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // *** start spi and i2c communications ***
  SPI.begin();
  Wire.begin();
 
   // *** start ethernet communications and the server ***
  Ethernet.init(pins_ethernet_cs);
  Ethernet.begin(mac, ip);

  // check for ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("ERROR: Ethernet hardware was not found!");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  else {
    Serial.println("INFO: Ethernet hardware was found.");
  }

  // check if ethernet cable is connected
  // only available when using the W5200 and W5500 Ethernet controller chips
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Warning: Ethernet cable is not connected.");
  }

  // start the server
  ethServer.begin();

  // *** start the Modbus TCP server ***
  if (!modbusTCPServer.begin(modbus_slave_id)) {
    Serial.println("ERROR: Failed to start Modbus TCP Server!");
    while (true) {
      delay(1); // do nothing, no point running without Modbus TCP server
    }
  }
  else {
    Serial.println("INFO: Modbus TCP Server started.");
  }

  // *** configure can communications ***
  Serial.println("Configure ACAN2517FD");
  ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_40MHz, can_baudrate, DataBitRateFactor::x1);
  // Default values are too high for an Arduino Uno that contains 2048 bytes of RAM: reduce them
  settings.mDriverTransmitFIFOSize = 1;
  settings.mDriverReceiveFIFOSize = 1;
  // RAM Usage
  Serial.print("MCP2517FD RAM Usage: ");
  Serial.print(settings.ramUsage());
  Serial.println(" bytes");

  // *** start can communications for wpm3 side ***
  const uint32_t errorCode_wpm3 = can_wpm3.begin(settings, []{can_wpm3.isr();});
  if (errorCode_wpm3 == 0) {
    Serial.println("INFO: Can Bus for WPM3 side started with settings: ");
    Serial.print("Bit Rate prescaler: ");
    Serial.println(settings.mBitRatePrescaler);
    Serial.print("Arbitration Phase segment 1: ");
    Serial.println(settings.mArbitrationPhaseSegment1);
    Serial.print("Arbitration Phase segment 2: ");
    Serial.println(settings.mArbitrationPhaseSegment2);
    Serial.print("Arbitration SJW:");
    Serial.println(settings.mArbitrationSJW);
    Serial.print("Actual Arbitration Bit Rate: ");
    Serial.print(settings.actualArbitrationBitRate ());
    Serial.println(" bit/s") ;
    Serial.print("Exact Arbitration Bit Rate ? ");
    Serial.println(settings.exactArbitrationBitRate () ? "yes" : "no");
    Serial.print("Arbitration Sample point: ");
    Serial.print(settings.arbitrationSamplePointFromBitStart ());
    Serial.println("%") ;
  } else {
    Serial.print("ERROR: Failed to start Can Bus for WPM3 side! Configuration error 0x");
    Serial.println(errorCode_wpm3, HEX);
    while (true) {
      delay(1); // do nothing, no point running without can bus
    }
  }

  // *** start can communications for wpm3 side ***
  const uint32_t errorCode_fek = can_fek.begin(settings, []{can_fek.isr();});
  if (errorCode_wpm3 == 0) {
    Serial.println("INFO: Can Bus for WPM3 side started with settings: ");
    Serial.print("Bit Rate prescaler: ");
    Serial.println(settings.mBitRatePrescaler);
    Serial.print("Arbitration Phase segment 1: ");
    Serial.println(settings.mArbitrationPhaseSegment1);
    Serial.print("Arbitration Phase segment 2: ");
    Serial.println(settings.mArbitrationPhaseSegment2);
    Serial.print("Arbitration SJW:");
    Serial.println(settings.mArbitrationSJW);
    Serial.print("Actual Arbitration Bit Rate: ");
    Serial.print(settings.actualArbitrationBitRate ());
    Serial.println(" bit/s") ;
    Serial.print("Exact Arbitration Bit Rate ? ");
    Serial.println(settings.exactArbitrationBitRate () ? "yes" : "no");
    Serial.print("Arbitration Sample point: ");
    Serial.print(settings.arbitrationSamplePointFromBitStart ());
    Serial.println("%") ;
  } else {
    Serial.print("ERROR: Failed to start Can Bus for WPM3 side! Configuration error 0x");
    Serial.println(errorCode_fek, HEX);
    while (true) {
      delay(1); // do nothing, no point running without can bus
    }
  }

  // *** start digital potentiometer
  /* if (!potentiometer.begin(i2c_adress_potentiometer, &Wire)) {
    Serial.println("ERROR: Couldn't find potentiometer DS3502!");
    while (true) {
      delay(1); // do nothing, no point running without can bus
    }
  }
  else {
    Serial.println("INFO: Potentiometer DS3502 found.");
  } */

  // *** start SD card ***
  // https://docs.arduino.cc/learn/programming/sd-guide
  if (!SD.begin(pins_sd_card_cs)) {
    Serial.println("Warning: No SD Card detected. Logging deactivated!");
    status_sd_card_detected = false;
  }
  else {
    Serial.println("INFO: SD Card detected. Logging activated.");
    status_sd_card_detected = true;
  }

  // configure Modbus 
  // single coil at address 0x00
  // https://github.com/arduino-libraries/ArduinoModbus/blob/master/examples/TCP/EthernetModbusServerLED/EthernetModbusServerLED.ino
  modbusTCPServer.configureCoils(0x00, num_Coils);
  modbusTCPServer.configureHoldingRegisters(0x00, num_HoldingRegisters);

  // configure pins
  pinMode(pins_led, OUTPUT);
  pinMode(pins_relay_can, OUTPUT);
  pinMode(pins_relay_sensor, OUTPUT);

  // activate can relay
  digitalWrite(pins_relay_can, 1);

  // *** setup display ***
  display.begin();
  display.setFont(fontName);
  // disable second option
  mainMenu[1].enabled=disabledStatus;
}

void loop() {
  // listen for incoming clients
  EthernetClient client = ethServer.available();
  
  if (client) {
    // let the Modbus TCP accept the connection 
    modbusTCPServer.accept(client);

    Serial.println("Modbus client connected!");
    while (client.connected()) {
      // watchdog ok --> led blick 1 sec
      watchdog(1000);

      // display input check 
      nav.doInput();

      // poll for Modbus TCP requests, while client connected
      // maybe time delay
      modbusTCPServer.poll();

      // 0. check modbus for new data
      bool status_change_temperatue_outside = (bool) ModbusRTUServer.coilRead(0);
      double value_room_temperature         = (double) ModbusRTUServer.holdingRegisterRead(0) * 0.1;
      double value_room_humidity            = (double) ModbusRTUServer.holdingRegisterRead(1) * 0.1;

      // 1. get can data from wpm3 side
      CANFDMessage can_data_wpm3;
      if (can_wpm3.receive(can_data_wpm3)) { 
        // logging
        if (can_logging_wpm3) {
          logging_can_data(can_data_wpm3, "can_wpm.txt");
        }

        // send data to fek side, with 5 retrys
        retry = 0;
        while(TRUE) {
          if ((can_fek.tryToSend(can_data_wpm3)) || (retry >= 5)) {
            break;
          }
          else {
            retry += 1;
            Serial.print("Transmitted failed: Can WPM3 to Can FEK. Retry ");
            Serial.println(retry);
            unsigned int *decoded_can_data = decode_can_data(can_data_wpm3);
            serial_can_decoded(decoded_can_data);
          }
        }
      }

      CANFDMessage can_data_fek;
      if (can_fek.receive(can_data_fek)) {
        // logging
        if (can_logging_fek) {
          logging_can_data(can_data_fek, "can_fek.txt");
        }

        // change can data 
        unsigned int *decoded_can_data = decode_can_data(can_data_fek);
        if (decoded_can_data[0] == 1538) {
          Serial.println("FEK Side");
          serial_can(can_data_fek);
          serial_can_decoded(decoded_can_data);

          // temperature
          if (can_data_fek.data[2] == 17) { 
            can_data_fek.data[3] = highByte((int) (value_room_temperature * 10));
            can_data_fek.data[4] = lowByte((int) (value_room_temperature * 10));
          }
          // humidity
          if (can_data_fek.data[2] == 117) { 
            can_data_fek.data[3] = highByte((int) (value_room_humidity * 10));
            can_data_fek.data[4] = lowByte((int) (value_room_humidity * 10));
          }
        }

        // send fek data to wpm3 side, with 5 retrys
        retry = 0;
        while(TRUE) {
          if ((can_wpm3.tryToSend(can_data_fek)) || (retry >= 5)) {
            break;
          }
          else {
            retry += 1;
            Serial.print("Transmitted failed: Can FEK to Can WPM3. Retry ");
            Serial.println(retry);
            unsigned int *decoded_can_data = decode_can_data(can_data_fek);
            serial_can_decoded(decoded_can_data);
          }
        }
      }

      // outside temperature change
      if (status_change_temperatue_outside) {
        digitalWrite(pins_relay_sensor, 1);
        double value_temperatue_outside = (double) ModbusRTUServer.holdingRegisterRead(2) * 0.1;
        // digital poti output change
      }
      else {
        // passthrough temperature sensor
        digitalWrite(pins_relay_sensor, 0);
      }

      // 7. update display
      //only draw if menu changed for gfx device
      if (nav.changed(0)) {
        //change checking leaves more time for other tasks
        display.firstPage();
        do nav.doOutput(); 
        while(display.nextPage());
      }
    }
    Serial.println("Modbus client disconnected!");
  }
  else {
    // Serial.println("No Modbus client connected! Passthrough mode!");
    // watchdog ok --> led blick 0.5 sec
    watchdog(250);

    // display input check 
    nav.doInput();

    // 1. get can data from wpm3 side
    CANFDMessage can_data_wpm3;
    if (can_wpm3.receive(can_data_wpm3)) {      
      // logging
      if (can_logging_wpm3) {
        logging_can_data(can_data_wpm3, "can_wpm.txt");
      }

      // send data to fek side, with 5 retrys
      retry = 0;
      while(TRUE) {
        if ((can_fek.tryToSend(can_data_wpm3)) || (retry >= 5)) {
          break;
        }
        else {
          retry += 1;
          Serial.print("Transmitted failed: Can WPM3 to Can FEK. Retry ");
          Serial.println(retry);
          unsigned int *decoded_can_data = decode_can_data(can_data_wpm3);
          serial_can_decoded(decoded_can_data);
        }
      }
    }

    CANFDMessage can_data_fek;
    if (can_fek.receive(can_data_fek)) {
      // logging
      if (can_logging_fek) {
        logging_can_data(can_data_fek, "can_fek.txt");
      }

      // change data 
      unsigned int *decoded_can_data = decode_can_data(can_data_fek);
      if (decoded_can_data[0] == 1538) {
        Serial.println("FEK Side");
        serial_can(can_data_fek);
        serial_can_decoded(decoded_can_data);

        if (can_data_fek.data[2] == 17) { 
          can_data_fek.data[3] = highByte(245);
          can_data_fek.data[4] = lowByte(245);

          serial_can(can_data_fek);
        }
      }

      // send fek data to wpm3 side, with 5 retrys
      retry = 0;
      while(TRUE) {
        if ((can_wpm3.tryToSend(can_data_fek)) || (retry >= 5)) {
          break;
        }
        else {
          retry += 1;
          Serial.print("Transmitted failed: Can FEK to Can WPM3. Retry ");
          Serial.println(retry);
          unsigned int *decoded_can_data = decode_can_data(can_data_fek);
          serial_can_decoded(decoded_can_data);
        }
      }     
    }

    // passthrough temperature sensor
    digitalWrite(pins_relay_sensor, 0);

    // 7. update display
    //only draw if menu changed for gfx device
    if (nav.changed(0)) {
      //change checking leaves more time for other tasks
      display.firstPage();
      do nav.doOutput(); 
      while(display.nextPage());
    }
  }
}