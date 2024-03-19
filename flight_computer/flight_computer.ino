#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
#include "AltimeterController.h"
#include "MPUController.h"
#include "LoRaController.h"

Servo myservo;
AltimeterController bmp;
MPUController mpu;
LoRaController lora;
TinyGPSPlus gps;
HardwareSerial neogps(1);

#define RXD2 16
#define TXD2 17

#define servo_pin 33
#define gps_pss 27

float latitude, altitude = 0, longitud, velocity, reference_gps_altitude;

int counter = 0;

volatile bool new_command = false;

volatile bool new_data = false;;

volatile byte command;

bool on_flight = false;

const int interval_length = 150;

long last_time, init_time;


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    Serial.println("Error in serial initializing");
  }
  Serial.println("Flight computer");

  SPI.begin(23, 19, 18, 5);  //SCK, MISO, MOSI, CS;
  LoRa.setPins(5, 15, 2);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa wiring confirmed");
  if (!lora.begin()) while (1);
  if (!mpu.begin()) while (1); 
  if (!bmp.begin()) while (1);

/*   myservo.attach(servo_pin);
  myservo.write(0);
  delay(500);
  myservo.write(170);
  pinMode(27, INPUT);

  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  attachInterrupt(digitalPinToInterrupt(27), read_gps_int, RISING);
  
*/
  LoRa.onReceive(on_receive_int);
  LoRa.receive();
  Serial.println("FLIGHT DATA");
  lora.data_send("Ready for command");
}

void on_receive_int(int packet_size) {
  command = LoRa.read();
  new_command = true;
}

void hear_command(){
  if(command == 0b10101010) {
    lora.data_send("Entering CALIBRATION");
    mpu.calibrate_mpu();
    lora.data_send("Calibration finished successfully");
  }  else if (command == 0b01010101) {
    lora.data_send("Getting reference pressure");
    bmp.getReferencePressure();
    lora.data_send("Refernce pressure at: " + String(bmp.referencePressure));
  } else if (command == 0b11001100) {
    lora.data_send("Entering FLIGH MODE");
    last_time = init_time = millis();
    on_flight = true;
  }
} 

void flight_mode() {
  if (new_data) {
    new_data = false;
    if (gps.location.isValid() == 1) {
      latitude = gps.location.lat();
      longitud = gps.location.lng();
      altitude = gps.altitude.meters();
      velocity = gps.speed.mps();
    }
  }
  if (millis() - last_time > interval_length) {
    long time = millis();
    mpu.update();
    String data = "/*" + String(counter) + "," + String(time - last_time) + "," + String(bmp.readTemperature(0)) + "," + String(bmp.readPressure()) + "," + String(bmp.readRelAltitude()) + "," + String(mpu.accel_data[0]) + "," + String(mpu.accel_data[1]) + "," + String(mpu.accel_data[2]) + "," + String(mpu.angle_data[0]) + "," + String(mpu.angle_data[1]) + "," + String(mpu.angle_data[2]) + "," + String(velocity) + "*/";
    lora.data_send(data);
    last_time = time;
    counter++;
  }
}

void read_gps_int() {
  while (neogps.available()) {
    if (gps.encode(neogps.read())) {
      new_data = true;
    }
  }
}

void loop() {
  if(new_command){
    new_command = false;
    hear_command();
  }
  if(on_flight) {
    flight_mode();
  }
}