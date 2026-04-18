// clarq_rf_arduino.ino
// Arduino Nano — CLARQ RF Bridge
// Reads command IDs from USB serial (from CLARQ_GUI.py)
// Transmits over NRF24L01 to Jetson
//
// Wiring (Arduino Nano):
//   NRF24L01   Arduino Nano
//   ─────────  ────────────
//   VCC     →  3.3V
//   GND     →  GND
//   CE      →  D9
//   CSN     →  D10
//   SCK     →  D13
//   MOSI    →  D11
//   MISO    →  D12
//
// Install library: RF24 by TMRh20 (Arduino Library Manager)

#include <SPI.h>
#include <RF24.h>

// ── NRF24L01 pins ─────────────────────────────────────────────
#define CE_PIN   9
#define CSN_PIN  10

RF24 radio(CE_PIN, CSN_PIN);

// ── RF channel and pipe address ───────────────────────────────
// Must match Jetson side exactly
const byte RF_CHANNEL   = 100;
const byte WRITE_PIPE[] = "CLARQ";   // GUI → Jetson
const byte READ_PIPE[]  = "JTSN0";   // Jetson → GUI (ACK/PONG)

// ── Command IDs — must match CLARQ_GUI.py ─────────────────────
#define CMD_PING        1
#define CMD_LAND        2
#define CMD_START_TAG   3
#define CMD_STOP_TAG    4
#define CMD_STOP_ALL    5
#define CMD_LAUNCH      6
#define CMD_LAUNCH_SIM  7
#define CMD_SET_HOME    8
#define CMD_START_SCAN  9
#define CMD_SAVE_END    10
#define CMD_GO_HOME     11

// ── Packet structure ──────────────────────────────────────────
// Keep it simple — just 2 bytes
// byte 0: command ID
// byte 1: checksum (XOR of byte 0 with 0xAA)
struct ClarqPacket {
  uint8_t cmd_id;
  uint8_t checksum;
};

// ── LED feedback ──────────────────────────────────────────────
#define LED_PIN LED_BUILTIN

void blink(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(80);
    digitalWrite(LED_PIN, LOW);
    delay(80);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Init NRF24L01
  if (!radio.begin()) {
    Serial.println("NRF24L01 not found!");
    // Blink rapidly to indicate error
    while (true) {
      blink(5);
      delay(500);
    }
  }

  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF24_PA_MAX);       // Max power for range
  radio.setDataRate(RF24_250KBPS);     // 250kbps for best range
  radio.setRetries(5, 15);             // 5 retries, 15*250us delay
  radio.setCRCLength(RF24_CRC_16);     // 16-bit CRC

  radio.openWritingPipe(WRITE_PIPE);
  radio.openReadingPipe(1, READ_PIPE);

  radio.stopListening();               // Start in TX mode

  Serial.println("CLARQ_RF_READY");
  blink(3);
}

void loop() {
  // ── Read command from GUI over USB serial ──────────────────
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (!line.startsWith("CMD:")) return;

    int cmd_id = line.substring(4).toInt();

    if (cmd_id < 1 || cmd_id > 11) {
      Serial.println("ERR:INVALID_CMD");
      return;
    }

    // Build packet
    ClarqPacket pkt;
    pkt.cmd_id   = (uint8_t)cmd_id;
    pkt.checksum = pkt.cmd_id ^ 0xAA;

    // Transmit over RF
    radio.stopListening();
    bool ok = radio.write(&pkt, sizeof(pkt));

    if (ok) {
      Serial.print("TX_OK:");
      Serial.println(cmd_id);
      blink(1);
    } else {
      Serial.print("TX_FAIL:");
      Serial.println(cmd_id);
      blink(3);
    }

    // Switch to RX briefly to catch PONG response
    radio.startListening();
    unsigned long start = millis();
    while (millis() - start < 200) {
      if (radio.available()) {
        ClarqPacket resp;
        radio.read(&resp, sizeof(resp));
        // Validate checksum
        if (resp.checksum == (resp.cmd_id ^ 0xAA)) {
          Serial.print("RX:");
          Serial.println(resp.cmd_id);
          blink(2);
        }
        break;
      }
    }
    radio.stopListening();
  }
}
