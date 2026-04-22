/*
 * clarq_rf_arduino.ino - CLARQ RF Bridge (Arduino Nano)
 * CLEAN VERSION - Minimal debug output
 */

#include <SPI.h>
#include <RF24.h>

// ── NRF24L01 Config ──────────────────────────────────────────
#define CE_PIN  9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

// Must match clarq_rf_listener.py exactly
const uint8_t  RF_CHANNEL = 100;
const uint64_t WRITE_PIPE = 0x5152414C43LL;  // "CLARQ" in hex
const uint64_t READ_PIPE  = 0x304E53544ALL;  // "JTSN0" in hex

// ── Command IDs ──────────────────────────────────────────────
#define CMD_ID_PING            1
#define CMD_ID_LAND            2
#define CMD_ID_START_TAG       3
#define CMD_ID_STOP_TAG        4
#define CMD_ID_STOP_ALL        5
#define CMD_ID_LAUNCH          6
#define CMD_ID_LAUNCH_SIM      7
#define CMD_ID_SET_HOME        8
#define CMD_ID_START_SCAN      9
#define CMD_ID_SAVE_END        10
#define CMD_ID_GO_HOME         11
#define CMD_ID_START_3D_FUSION 12
#define CMD_ID_STOP_3D_FUSION  13
#define CMD_ID_START_PHOTO     14
#define CMD_ID_START_VIDEO     15
#define CMD_ID_STOP_CAPTURE    16
#define CMD_ID_SET_GIMBAL      17

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  
  // Wait for serial and USB to stabilize
  delay(2000);
  
  // Clear any garbage in serial buffer
  while(Serial.available()) {
    Serial.read();
  }
  
  // Initialize NRF24L01
  radio.begin();
  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  
  // Open pipes
  radio.openWritingPipe(WRITE_PIPE);
  radio.openReadingPipe(1, READ_PIPE);
  
  // Start listening
  radio.startListening();
  
  // Send ready signal to GUI (ONLY THIS, NOTHING ELSE)
  Serial.println("CLARQ_RF_READY");
}

// ── Handle Commands from GUI ─────────────────────────────────
void handleSerialCommand() {
  String line = Serial.readStringUntil('\n');
  line.trim();
  
  // Expected format: "CMD:<id>"
  if (!line.startsWith("CMD:")) {
    return;  // Silently ignore invalid format
  }
  
  // Extract command ID
  int cmd_id = line.substring(4).toInt();
  
  // Validate command ID
  if (cmd_id < 1 || cmd_id > 17) {
    return;  // Silently ignore invalid ID
  }
  
  // Build packet: [cmd_id, checksum]
  uint8_t packet[2];
  packet[0] = cmd_id;
  packet[1] = cmd_id ^ 0xAA;  // Simple XOR checksum
  
  // Send to Jetson via nRF
  radio.stopListening();
  delay(1);  // Allow radio time to settle before transmitting
  bool ok = radio.write(packet, 2);
  radio.startListening();
  
  // Report to GUI
  if (ok) {
    Serial.print("TX_OK:");
    Serial.println(cmd_id);
  } else {
    Serial.print("TX_FAIL:");
    Serial.println(cmd_id);
  }
}

// ── Handle Responses from Jetson ────────────────────────────
void handleRadioResponse() {
  uint8_t packet[2];
  
  // Read the data
  radio.read(packet, 2);
  
  uint8_t status_id = packet[0];
  uint8_t checksum  = packet[1];
  
  // Validate checksum
  if (checksum != (status_id ^ 0xAA)) {
    return;  // Silently ignore bad checksum
  }
  
  // Forward to GUI
  Serial.print("RX:");
  Serial.println(status_id);
}

// ── Main Loop ────────────────────────────────────────────────
void loop() {
  // Check for commands from GUI (USB Serial)
  if (Serial.available()) {
    handleSerialCommand();
  }
  
  // Check for responses from Jetson (nRF)
  if (radio.available()) {
    handleRadioResponse();
  }
  
  delay(10);
}
