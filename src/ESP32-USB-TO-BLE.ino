// Pin definitions must come before including the USB host library
#define USB_HOST_DP0 16  // D+ for mouse
#define USB_HOST_DM0 17  // D- for mouse
#define USB_HOST_DP1 22  // D+ for keyboard (try on working port)
#define USB_HOST_DM1 23  // D- for keyboard (try on working port)
#define USB_HOST_DP2 -1
#define USB_HOST_DM2 -1
#define USB_HOST_DP3 -1
#define USB_HOST_DM3 -1

// Power configuration
#define USB_HOST_RESET_TIME 20  // ms
#define USB_HOST_POWER_ON_WAIT 100  // ms
#define USB_HOST_SETTLE_TIME 200  // ms

#include <Arduino.h>
#include "ESP32-USB-Soft-Host.h"
#include "BleDevice.h"

BleDevice bleDevice;
uint8_t keyboardDevice = 0xFF;
uint8_t mouseDevice = 0xFF;

void USB_IfaceDesc_Detect(uint8_t usbNum, int cfgCount, int sIntfCount, void *Intf, size_t len) {
  if (!Intf || len < sizeof(sIntfDesc)) {
    Serial.println("Invalid interface descriptor!");
    return;
  }

  sIntfDesc *sIntf = (sIntfDesc *)Intf;
  
  // Print all device info
  Serial.printf("\nUSB Device %d Details:\n", usbNum);
  Serial.printf("  Config: %d, Interface: %d\n", cfgCount, sIntfCount);
  Serial.printf("  Class: %d (0x%02X)\n", sIntf->iClass, sIntf->iClass);
  Serial.printf("  SubClass: %d (0x%02X)\n", sIntf->iSub, sIntf->iSub);
  Serial.printf("  Protocol: %d (0x%02X)\n", sIntf->iProto, sIntf->iProto);

  // Check for HID devices (class 3)
  if (sIntf->iClass == 3) {  // HID Class
    if (sIntf->iProto == 1) {
      Serial.printf("Keyboard detected on USB%d\n", usbNum);
      Serial.printf("  SubClass: %d, Protocol: %d\n", sIntf->iSub, sIntf->iProto);
      keyboardDevice = usbNum;
    } else if (sIntf->iProto == 2) {
      Serial.printf("Mouse detected on USB%d\n", usbNum);
      mouseDevice = usbNum;
    } else {
      Serial.printf("Unknown HID device on USB%d (Protocol: %d)\n", usbNum, sIntf->iProto);
    }
  } else {
    Serial.printf("Non-HID device detected on USB%d (Class: %d)\n", usbNum, sIntf->iClass);
  }
}

void USB_Data_Handler(uint8_t usbNum, uint8_t byte_depth, uint8_t *data, uint8_t data_len) {
  // Print raw data for debugging
  Serial.printf("USB%d Data (len=%d): ", usbNum, data_len);
  for (int i = 0; i < data_len; i++) {
    Serial.printf("%02X ", data[i]);
  }
  Serial.println();

  if (usbNum == keyboardDevice && data_len > 0) {
    Serial.println("Processing keyboard data");
    bleDevice.sendKeyboardReport(data, data_len);
  } else if (usbNum == mouseDevice && data_len > 0) {
    Serial.println("Processing mouse data");
    bleDevice.sendMouseReport(data, data_len);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32 USB Host starting up...");

  // Power cycle USB ports
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  
  // Pull all pins low
  digitalWrite(22, LOW);
  digitalWrite(23, LOW);
  digitalWrite(16, LOW);
  digitalWrite(17, LOW);
  delay(USB_HOST_RESET_TIME);
  
  // Set pins to input with pullup
  pinMode(22, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  delay(USB_HOST_POWER_ON_WAIT);

  // Configure USB Host
  USH.setOnIfaceDescCb(USB_IfaceDesc_Detect);
  if (!USH.init(USB_Data_Handler)) {
    Serial.println("USB Host initialization failed!");
    while (1) delay(100);
  }
  Serial.println("USB Host initialized successfully");

  // Initialize BLE
  Serial.println("Initializing BLE...");
  if (!bleDevice.begin()) {
    Serial.println("BLE initialization failed!");
    while (1) delay(100);
  }
  Serial.println("BLE initialized successfully");
}

void loop() {
  delay(100);  // Add a small delay to prevent watchdog issues
}
