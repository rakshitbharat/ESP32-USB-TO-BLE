//#define DEBUG_ALL
#define FORCE_TEMPLATED_NOPS

#include "ESP32-USB-Soft-Host.h"
#include "BleDevice.h"

BleDevice bleDevice; // Initialize an instance of the BleDevice class

// Initialize variables to store the USB device numbers for keyboard and mouse
uint8_t keyboardDevice = -1;
uint8_t mouseDevice = -1;

// Callback function called when a USB interface descriptor is detected
void USB_IfaceDesc_Detect(uint8_t usbNum, int cfgCount, int sIntfCount, void *Intf, size_t len) {
  // Only check first configuration and interface in the device
  if (cfgCount == 1 && sIntfCount == 1) {
    sIntfDesc *sIntf = (sIntfDesc *)Intf;
    // Determine if the interface protocol indicates a keyboard or a mouse
    if (sIntf->iProto == 1) {
      keyboardDevice = usbNum; // Assign the USB device number to the keyboard
    } else if (sIntf->iProto == 2) {
      mouseDevice = usbNum; // Assign the USB device number to the mouse
    }
  }
}

// Callback function called when USB data is received
void USB_Data_Handler(uint8_t usbNum, uint8_t byte_depth, uint8_t *data, uint8_t data_len) {
  #ifdef DEBUG_ALL
    // Print debug information if DEBUG_ALL is defined
    printf("%s data: ", usbNum == mouseDevice ? "Mouse" : "Keyboard");
    for (int k = 0; k < data_len; k++) {
      printf("0x%02x ", data[k]);
    }
    printf("\n");
  #endif

  // Check if the USB data belongs to the keyboard or the mouse
  if (data_len > 4) {
    #ifdef DEBUG_ALL
      printf("sendKeyboardReport\n");
    #endif
    bleDevice.sendKeyboardReport(data, data_len); // Send keyboard report via BLE
  } else {
    #ifdef DEBUG_ALL
      printf("sendMouseReport\n");
    #endif
    bleDevice.sendMouseReport(data, data_len); // Send mouse report via BLE
  }
}

// Setup function called once at the beginning of the program
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Add debug info
  Serial.println("\nTesting USB connections:");
  
  // Test D+ (GPIO 16)
  pinMode(16, INPUT);
  Serial.println("\nD+ Line (GPIO 16):");
  Serial.printf("- Voltage level: %d\n", digitalRead(16));
  Serial.printf("- Should be HIGH (1) due to 1.5K pull-up to 3.3V\n");
  
  // Test D- (GPIO 17)
  pinMode(17, INPUT);
  Serial.println("\nD- Line (GPIO 17):");
  Serial.printf("- Voltage level: %d\n", digitalRead(17));
  Serial.printf("- Should be LOW (0) due to 15K pull-down to GND\n");
  
  // Test GND connections
  Serial.println("\nChecking connections:");
  Serial.println("- If D+ is LOW (0): Check 1.5K resistor and 3.3V connection");
  Serial.println("- If D- is HIGH (1): Check 15K resistor and GND connection");
  Serial.println("- If both are wrong: Check all GND connections are common\n");
  
  // Initialize USB Host
  USH.setOnIfaceDescCb(USB_IfaceDesc_Detect);
  USH.init(USB_Data_Handler);

  // Begin BLE device communication
  bleDevice.begin();
}

// Loop function called repeatedly after setup
void loop() {
  vTaskDelete(NULL); // Delete the current task
}
