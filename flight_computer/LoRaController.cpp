#include <Arduino.h>
#include "LoRaController.h"
#include <LoRa.h>

LoRaController::LoRaController() {
}

bool LoRaController::begin() {
  return 1;
}


void LoRaController::data_send(String value) {
  LoRa.beginPacket();
  LoRa.write(lora_net_address);
  LoRa.print(value);
  LoRa.endPacket();
  LoRa.receive();
}