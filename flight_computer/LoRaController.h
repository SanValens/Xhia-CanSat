#ifndef LoRaController_h
#define LoRaController_h

#include <LoRa.h>
#include <Arduino.h>


class LoRaController {
  public:
    LoRaController();
    byte LORA_NET_ADDRESS = 0b10101010;
    bool begin();
    void data_send(String value);
};

#endif