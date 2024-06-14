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
  LoRa.write(LORA_NET_ADDRESS);
  LoRa.print(value);
  LoRa.endPacket();
  LoRa.receive();
}