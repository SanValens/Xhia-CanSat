#include <math.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
#include "AltimeterController.h"
#include "MPUController.h"
#include "LoRaController.h"
#include <esp_now.h>
#include <WiFi.h>

/* 

DUE: Course and haversine check.
Document the code
Use a switch case on hear_command
Consider adding retry mechanisms for failed LoRa transmissions.

*/

typedef struct struct_message {
  float longitude;
  float latitude;
  float altitude;
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

float latitude = 1, altitude = 1, longitud = 1, velocity, reference_gps_altitude;

int counter = 0;

volatile bool new_command = false;

volatile bool new_data = false;;

volatile byte command;

bool on_flight = false;

const int interval_length = 150;

long last_time, init_time;

double lat1 = 6.1412943;
double long1 = -75.4019269;
double lat2 = 6.1418871;
double long2 = -75.3962987;

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
  myservo.write(90);
  delay(1500);
  myservo.write(0);
  pinMode(gps_pss, INPUT);
  pinMode(gps_engage_led, OUTPUT);
  digitalWrite(gps_engage_led, HIGH);

  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  LoRa.onReceive(on_data_rec_lora);
  LoRa.receive();
  Serial.println("FLIGHT DATA");
  lora.data_send("Ready for command");
  //Init ESP-NOW
  WiFi.mode(WIFI_STA);
  //esp_wifi_set_protocol(WIFI_STA, WIFI_PROTOCOL_LR);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(on_data_rec_espnow);
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS0_LGI);
}

void on_data_rec_espnow(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&second_stage, incomingData, sizeof(second_stage));
  Serial.print("Data in #");
  Serial.println(second_stage.latitude,8);
  counter++;
}

void on_data_rec_lora(int packet_size) {
  command = LoRa.read();
  new_command = true;
}

void hear_command(){
  if(command == 0b10101010) {
    lora.data_send("Entering CALIBRATION");
    mpu.calibrate_mpu();
    lora.data_send("Calibration finished successfully");
  } else if (command == 0b01010101) {
    lora.data_send("Getting reference pressure");
    bmp.getReferencePressure();
    lora.data_send("Refernce pressure at: " + String(bmp.referencePressure));
  } else if (command == 0b111111) {
    lora.data_send("Closing hatch");
    myservo.write(60); //Close hatch function
  } else if (command == 0b01110111) {
    lora.data_send("Pulling GPS Info");
  } else if (command == 0b011011) {
    lora.data_send("Openning hatch");
    myservo.write(0); //Close hatch function
  } else if (command == 0b11001100) {
    lora.data_send("Entering FLIGH MODE");
    last_time = init_time = millis();
    on_flight = true;
  }
} 

void read_gps() {
  if (gps.location.isValid() == 1) {
    latitude = gps.location.lat();
    longitud = gps.location.lng();
    altitude = gps.altitude.meters();
    velocity = gps.speed.mps();
    digitalWrite(gps_engage_led, LOW);
  } else {
    digitalWrite(gps_engage_led, HIGH);
  }
}

void flight_mode() {
  if (millis() - last_time > interval_length) {
    long time = millis();
    mpu.update();
    read_gps();
    String data = "/*" + String(counter) + "," + String(time - last_time) + "," +
    String(bmp.readTemperature(0)) + "," + String(bmp.readPressure()) + "," +
    String(bmp.readRelAltitude()) + "," + String(mpu.accel_data[0]) + "," +
    String(mpu.accel_data[1]) + "," + String(mpu.accel_data[2]) + "," +
    String(mpu.angle_data[0]) + "," + String(mpu.angle_data[1]) + "," +
    String(mpu.angle_data[2]) + "," + String(velocity) + "," + String(second_stage.latitude) + "," + String(second_stage.longitude) +"*/";
    lora.data_send(data);
    Serial.println(data);
    last_time = time;
    counter++;
  }
  while (neogps.available()) {
    if (gps.encode(neogps.read())) {
      new_data = true;
    }
  }
}

double distancia_haversine(double lt1, double lg1, double lt2, double lg2) {
    double d_lt = radians(lt2) - radians(lt1);
    double d_lg = radians(lg2) - radians(lg1);
    double a = (sin(d_lt/2)) * (sin(d_lt/2)) + cos(radians(lt1)) * cos(radians(lt2)) * (sin(d_lg/2)) * (sin(d_lg/2));
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return c * EARTH_RADIUS;
}

double course(double lt1, double lg1, double lt2, double lg2) {
    lt1 = radians(lt1);
    lt2 = radians(lt2);
    lg1 = radians(lg1);
    lg2 = radians(lg2);
    
    double d_lg = lg2 - lg1;
    
    double y = sin(lg2 - lg1) * cos(lt2);
    double x = cos(lt1) * sin(lt2) - sin(lt1) * cos(lt2) * cos(d_lg);

    double course = atan2(y,x) * 180 / M_PI;
    
    course = fmod((course + 360), 360);

    return course;
}


void loop() {
  if(new_command){
    new_command = false;
    hear_command();
  }
  if(on_flight) {
    flight_mode();
  }
  while (neogps.available()) {
    if (gps.encode(neogps.read())) {
      new_data = true;
      read_gps();
    }
  }
}