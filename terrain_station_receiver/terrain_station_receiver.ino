#include <LoRa.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {}
  Serial.setTimeout(200);
  Serial.println("Terrain station: ESP32 - CanSat UNAM 2024");
  LoRa.setPins(15,4,5);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.receive();
  LoRa.onReceive(onReceive);
}

void onReceive(int packetSize) {
  if (packetSize) {
    // received a packet
    if(LoRa.read() == 0b10101010) {
      Serial.print("CALLBACK: ");
      while (LoRa.available()) {
        Serial.print((char)LoRa.read());
      }
      Serial.println("");
    }
  }
}

void command_prompt() {
  if(Serial.available() > 0){
    String command = Serial.readString();
    command.remove(command.length()-1);
    if(command == "calibrate"){
      Serial.println("Calibration command sent");
      Serial.println("Wating for confirmation...");
      LoRa.beginPacket();
      LoRa.write(0b10101010);
      LoRa.endPacket();
      LoRa.receive();
    } else if(command == "refpress"){
      Serial.println("Reference pressure command sent");
      Serial.println("Wating for confirmation...");
      LoRa.beginPacket();
      LoRa.write(0b01010101);
      LoRa.endPacket();
      LoRa.receive();
    }
    else if(command == "init_flightmode") {
      Serial.println("Flight initialization command sent");
      Serial.println("Wating for confirmation...");
      LoRa.beginPacket();
      LoRa.write(0b11001100);
      LoRa.endPacket();
      LoRa.receive();
    } else {
      Serial.println("No te entendí tu comando '" + command + "', bro");
    }
  }

  delay(1000);
}

void loop() {
  command_prompt();
}
