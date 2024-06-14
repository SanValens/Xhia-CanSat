#include <LoRa.h>

byte net_id = 0b10101010;
String command_names[6] = { "cali", "rfp", "cls", "opn", "gpull", "itf" };
String command_desc[6] = { "Calibration", "Reference pressure", "Closing hatch", "Opening hatch", "GPS info pull", "Flight initialization" };
byte command_id[6] = { 0b10101010, 0b01010101, 0b111111, 0b011011, 0b01110111, 0b11001100 };

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {}
  Serial.setTimeout(200);
  Serial.println("Terrain station: ESP32 - CanSat UNAM 2024");
  LoRa.setPins(5,4,2);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    delay(10000);
  }
  Serial.println("Avaliable commands are: ");
  Serial.println("- cali: calibration");
  Serial.println("- rfp: read reference pressure");
  Serial.println("- cls: close hatch (60 deg)");
  Serial.println("- opn: close hatch (0 deg)");
  Serial.println("- gpull: pull GPS info both for first and second load");
  Serial.println("- itf: INIT FLIGHT MODE (MAKE SURE REFERENCE PRESSURE WAS SUCCESFULLY TAKEN)");
  LoRa.receive();
  LoRa.onReceive(onReceive);
}

void onReceive(int packetSize) {
  if (packetSize) {
    byte packt_id = LoRa.read();
    if (packt_id == net_id) {
      Serial.print("CALLBACK: ");
      while (LoRa.available()) {
        Serial.print((char)LoRa.read());
      }
      Serial.print(LoRa.packetRssi());
      Serial.println("*/");
    }
  }
}

void loop() {
  command_handle();
  delay(10);
}

void command_handle() {
  if (Serial.available() > 0) {
    String command = Serial.readString();
    command.remove(command.length() - 1);
    int j = 7;
    for (int i = 0; i < 6; i++) {
      if (command == command_names[i]) {
        j = i;
      }
    }
    if(j < 7) {
      LoRa.beginPacket();
      LoRa.write(command_id[j]);
      LoRa.endPacket();
      LoRa.receive();
      Serial.print(command_desc[j]);
      Serial.println(" command sent");
      Serial.println("Wating for callback...");
    } else {
      Serial.println("No te entendí tu comando '" + command + "', bro");
    }
  }
}
