#define DEBUG_ALL
#define FORCE_TEMPLATED_NOPS

#include "ESP32-USB-Soft-Host.h"
#include "BleDevice.h"

BleDevice bleDevice;
uint8_t keyboardDevice = -1;

void USB_IfaceDesc_Detect(uint8_t usbNum, int cfgCount, int sIntfCount, void *Intf, size_t len) {
  Serial.printf("\nUSB Device on port %d\n", usbNum);
  
  if (cfgCount == 1 && sIntfCount == 1) {
    sIntfDesc *sIntf = (sIntfDesc *)Intf;
    if (sIntf->iProto == 1) {
      Serial.println("Keyboard Found!");
      keyboardDevice = usbNum;
    }
  }
}

void USB_Data_Handler(uint8_t usbNum, uint8_t byte_depth, uint8_t *data, uint8_t data_len) {
  if (data_len > 4) {
    bleDevice.sendKeyboardReport(data, data_len);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Reset USB pins
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  digitalWrite(16, LOW);
  digitalWrite(17, LOW);
  delay(100);
  
  // Set as inputs
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  
  // Initialize USB
  USH.setOnIfaceDescCb(USB_IfaceDesc_Detect);
  USH.setTaskPriority(5);
  USH.setTaskCore(0);
  
  if (!USH.init(USB_Data_Handler)) {
    Serial.println("USB Init Failed!");
    while(1) {
      Serial.printf("D+: %d, D-: %d\n", digitalRead(16), digitalRead(17));
      delay(1000);
    }
  }

  bleDevice.begin();
}

void loop() {
  delay(10);
}
