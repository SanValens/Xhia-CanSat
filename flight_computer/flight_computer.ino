#include <math.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
#include "AltimeterController.h"
#include "MPUController.h"
#include "LoRaController.h"

typedef struct struct_message {
  float longitude;
  float latitude;
  float altitude;
  float refresh;
} struct_message;

struct_message second_stage;

Servo myservo;
AltimeterController bmp;
MPUController mpu;
LoRaController lora;
TinyGPSPlus gps;
HardwareSerial neogps(1);

#define RXD2 25
#define TXD2 26

#define servo_pin 2

#define gps_pss 27
#define gps_engage_led 23


#define cs_pin 22
#define miso_pin 19
#define mosi_pin 21
#define sck_pin 18
#define rst_lora_pin 4
#define inte_lora_pin 5

bool undeployed = true;
float latitude = 1, altitude = 1, longitud = 1, velocity, reference_gps_altitude, previus_altitude, apogee = 0;
int servo_state = 0;
int counter = 0;

int max_turn_servo = 70;

volatile bool new_data = false;

volatile byte command;

bool on_flight = false;
bool gps_second_load_engaged = false;
bool gps_engaged = false;

const int interval_length = 100;

long previus_time, init_time, last_gps_2nd_update, last_gps_update;

const double EARTH_RADIUS = 6372795;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    Serial.println("Error in serial initializing");
  }
  Serial.println("Flight computer");

  SPI.begin(sck_pin, miso_pin, mosi_pin, cs_pin);  //SCK, MISO, MOSI, CS;
  LoRa.setPins(cs_pin, rst_lora_pin, inte_lora_pin);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa wiring confirmed");
  if (!lora.begin()) while (1);
  if (!mpu.begin()) while (1); 
  if (!bmp.begin()) while (1);
  myservo.attach(servo_pin);
  pinMode(gps_pss, INPUT);
  pinMode(gps_engage_led, OUTPUT);
  digitalWrite(gps_engage_led, HIGH);
  delay(500);
  digitalWrite(gps_engage_led, LOW);
  delay(500);
  digitalWrite(gps_engage_led, HIGH);
  delay(500);
  digitalWrite(gps_engage_led, LOW);
  delay(500);
  digitalWrite(gps_engage_led, HIGH);
  delay(500);
  digitalWrite(gps_engage_led, LOW);
  digitalWrite(gps_engage_led, !gps_engaged);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  LoRa.onReceive(on_data_rec_lora);
  LoRa.receive();
  Serial.println("FLIGHT DATA");
  lora.data_send("Ready for command");
}

void on_data_rec_lora(int packet_size) {
  command = LoRa.read();
}

void hear_command(){
  bool both_engaged = true;
  switch(command) {
    case 0b10101010:
      lora.data_send("Entering CALIBRATION");
      mpu.calibrate_mpu();
      lora.data_send("Calibration finished successfully");
      break;

    case 0b01010101:
      lora.data_send("Getting reference pressure");
      bmp.getReferencePressure();
      lora.data_send("Refernce pressure at: " + String(bmp.referencePressure));
      break;

    case 0b111111:
      lora.data_send("Closing hatch");
      servo_state = 1;
      myservo.write(max_turn_servo); //Close hatch function
      break;

    case 0b01110111:
      lora.data_send("Pulling GPS Info:");
      lora.data_send("--- PRIMARY LOAD ---");
      if(gps_engaged){
        lora.data_send("Latitude: "+ String(latitude));
        lora.data_send("Longitude: " + String(longitud));
        lora.data_send("Last update: " + String((millis() - last_gps_update)/1000));
      } else {
        lora.data_send("Conexión no conseguida");
        both_engaged = false;
      }
      break;

    case 0b011011:
      lora.data_send("Openning hatch");
      servo_state = 0;
      myservo.write(0); //Close hatch function
      break;

    case 0b10100010:
      lora.data_send("Openning hatch");
      myservo.write(0); //Close hatch function
      break;

    case 0b11001100:
      lora.data_send("Entering FLIGH MODE");
      previus_time = init_time = millis();
      on_flight = true;
      break;

    default:
      break;
  }
  command = 0b00000000;
} 

void read_gps() {
  if (gps.location.isValid() == 1) {
    latitude = gps.location.lat();
    longitud = gps.location.lng();
    altitude = gps.altitude.meters();
    velocity = gps.speed.mps();
    gps_engaged = true;
    last_gps_update = millis();
  }
  digitalWrite(gps_engage_led, !gps_engaged);
}

void flight_mode() {
  if (millis() - previus_time > interval_length) {
    long time = millis();
    mpu.update();
    read_gps();
    float delta_time = float(time - previus_time) / 1000;
    float barometric_altitude =  bmp.readRelAltitude();
    float vertical_velocity = (bmp.readRelAltitude() - previus_altitude) / delta_time;
    String data = "/*" + String(counter) + "," + String(delta_time) + "," +
    String(bmp.readTemperature(0)) + "," + String(bmp.readPressure()) + "," +
    String(barometric_altitude) + "," + String(mpu.accel_data[0]) + "," +
    String(mpu.accel_data[1]) + "," + String(mpu.accel_data[2]) + "," +
    String(mpu.angle_data[0]) + "," + String(mpu.angle_data[1]) + "," +
    String(mpu.angle_data[2]) + "," + String(vertical_velocity) + "," +
    String(apogee) + ",";
  
    lora.data_send(data);

    previus_time = time;
    previus_altitude = barometric_altitude;
    counter++;
  }
}

void loop() {
  if(on_flight) {
    flight_mode();
  }
  if(command != 0 ) {
    hear_command();
  }
  while (neogps.available()) {
    if (gps.encode(neogps.read())) {
      new_data = true;
      read_gps();
    }
  }
}