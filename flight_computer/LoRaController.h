#ifndef LoRaController_h
#define LoRaController_h

#include <LoRa.h>
#include <Arduino.h>


class LoRaController {
  public:
    LoRaController();
    byte lora_net_address = 0b10101010;
    byte lora_cb_address = 0b10111011;
    bool begin();
    void data_send(String value);
};

#endif