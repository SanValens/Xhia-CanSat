#include <espnow.h>
#include <ESP8266WiFi.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// REPLACE WITH RECEIVER MAC Address
uint8_t espAddress[] = { 0xD8, 0xBC, 0x38, 0xF9, 0xC9, 0x8C };

static const int RXPin = D7, TXPin = D2;
const int engage_led_pin = D3;
static const uint32_t GPSBaud = 9600;
bool onblink = true;
long blink_interval;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

typedef struct struct_message {
  float longitude;
  float latitude;
  float altitude;
} struct_message;

// Create a struct_message called myData
struct_message myData;

unsigned long lastTime = 0;
unsigned long previousMillis  = 0;
int counter = 0;
unsigned long timerDelay = 1000;  // send readings timer
int ledState = LOW;

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.print("Success #");
    Serial.println(counter);
  } else {
    Serial.print("Fail #");
    Serial.println(counter);
  }
  counter++;
}

void setup() {
  // Init Serial Monitor
  Serial.begin(9600);
  while (!Serial) {
    delay(100000);
  }
  pinMode(engage_led_pin, OUTPUT);
  digitalWrite(engage_led_pin, LOW);
  //Init ESP-NOW:
  WiFi.mode(WIFI_STA);
  //esp_wifi_set_protocol(WIFI_STA, WIFI_PROTOCOL_LR);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  esp_now_add_peer(espAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  myData.latitude = 1.234;
  myData.longitude = -5.678;
  myData.altitude = 2000.789;
  //Init GPS:
  ss.begin(GPSBaud);
}

void loop() {
  if ((millis() - lastTime) >= timerDelay) {
    // Set values to send
    if (gps.location.isValid()) {
      myData.latitude = gps.location.lat();
      myData.longitude = gps.location.lng();
      myData.altitude = gps.altitude.meters();
      digitalWrite(engage_led_pin, HIGH);
    } else if(gps.charsProcessed() < 10){
      Serial.println("EFE");
    }
    esp_now_send(espAddress, (uint8_t *)&myData, sizeof(myData));
    lastTime = millis();
  }
  while (ss.available()) {
    gps.encode(ss.read());
  }
}