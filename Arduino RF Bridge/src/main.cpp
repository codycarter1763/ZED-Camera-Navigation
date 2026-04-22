/*
 * clarq_rf_arduino.ino - CLARQ RF Bridge (Arduino Uno)
 * RYLR998 LoRa Module (868/915 MHz) via SoftwareSerial
 * Baud rate lowered to 9600 for reliable SoftwareSerial on Uno
 */

#include <SoftwareSerial.h>
#include <Arduino.h>
// ── RYLR998 UART Config ──────────────────────────────────────
// D10 = RX (connect to RYLR998 TX)
// D11 = TX (connect to RYLR998 RX)
// Avoid D0/D1 — those are shared with USB Serial on the Uno
#define LORA_RX_PIN 10
#define LORA_TX_PIN 11

SoftwareSerial loraSerial(LORA_RX_PIN, LORA_TX_PIN);

// RYLR998 AT settings — must match receiver side exactly
#define LORA_NETWORK_ID   18
#define LORA_ADDRESS      10          // This node (Arduino)
#define LORA_DEST_ADDR    11          // Remote node (Jetson)
#define LORA_BAND         915000000   // Hz — use 868000000 for EU
#define LORA_SF           9
#define LORA_BANDWIDTH    7
#define LORA_CR           1
#define LORA_PREAMBLE     4
#define LORA_POWER        22

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

// ── AT Command Helper ────────────────────────────────────────
// Sends an AT command and blocks until +OK, +ERR, or timeout.
// Returns true on +OK.
bool sendAT(const String& cmd, unsigned long timeoutMs = 3000) {
  // Flush any stale bytes before sending
  while (loraSerial.available()) loraSerial.read();

  loraSerial.println(cmd);

  unsigned long start = millis();
  String resp = "";

  while (millis() - start < timeoutMs) {
    while (loraSerial.available()) {
      resp += (char)loraSerial.read();
    }
    if (resp.indexOf("+OK")  >= 0) return true;
    if (resp.indexOf("+ERR") >= 0) return false;
  }
  return false;  // Timeout
}

// ── LoRa Transmit ────────────────────────────────────────────
// Encodes 2-byte payload as hex and sends via AT+SEND
bool loraSend(uint8_t cmd_id) {
  uint8_t checksum = cmd_id ^ 0xAA;

  char hexPayload[5];
  snprintf(hexPayload, sizeof(hexPayload), "%02X%02X", cmd_id, checksum);

  String atCmd = "AT+SEND=";
  atCmd += LORA_DEST_ADDR;
  atCmd += ",2,";
  atCmd += hexPayload;

  return sendAT(atCmd, 4000);  // Slightly longer timeout at 9600 baud
}

// ── LoRa Init ────────────────────────────────────────────────
void initLora() {
  loraSerial.begin(9600);  // 9600 is reliable on Uno SoftwareSerial
  delay(1000);             // Give module time to boot

  // Reset and re-configure
  sendAT("AT+RESET");
  delay(1500);             // Wait for module to come back after reset

  sendAT("AT+ADDRESS="   + String(LORA_ADDRESS));
  sendAT("AT+NETWORKID=" + String(LORA_NETWORK_ID));
  sendAT("AT+BAND="      + String(LORA_BAND));

  String param = "AT+PARAMETER=";
  param += LORA_SF;        param += ",";
  param += LORA_BANDWIDTH; param += ",";
  param += LORA_CR;        param += ",";
  param += LORA_PREAMBLE;
  sendAT(param);

  sendAT("AT+CRFOP=" + String(LORA_POWER));

  // Set module baud to 9600 (in case it was previously set differently)
  // AT+IPR=<baud> — persists across resets
  sendAT("AT+IPR=9600");
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);  // USB Serial to GUI — Uno hardware UART is fine here
  delay(2000);

  while (Serial.available()) Serial.read();  // Clear garbage

  initLora();

  Serial.println("CLARQ_RF_READY");
}

// ── Handle Commands from GUI ─────────────────────────────────
void handleSerialCommand() {
  String line = Serial.readStringUntil('\n');
  line.trim();

  if (!line.startsWith("CMD:")) return;

  int cmd_id = line.substring(4).toInt();
  if (cmd_id < 1 || cmd_id > 17) return;

  bool ok = loraSend((uint8_t)cmd_id);

  if (ok) {
    Serial.print("TX_OK:");
    Serial.println(cmd_id);
  } else {
    Serial.print("TX_FAIL:");
    Serial.println(cmd_id);
  }
}

// ── Handle Incoming LoRa Packets from Jetson ─────────────────
// RYLR998 unsolicited receive format:
//   +RCV=<addr>,<len>,<data>,<RSSI>,<SNR>\r\n
void handleLoraResponse() {
  String line = loraSerial.readStringUntil('\n');
  line.trim();

  if (!line.startsWith("+RCV=")) return;

  // +RCV=<src>,<len>,<data>,<RSSI>,<SNR>
  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  int c3 = line.indexOf(',', c2 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0) return;

  String hexData = line.substring(c2 + 1, c3);
  if (hexData.length() < 4) return;

  uint8_t status_id = (uint8_t)strtol(hexData.substring(0, 2).c_str(), nullptr, 16);
  uint8_t checksum  = (uint8_t)strtol(hexData.substring(2, 4).c_str(), nullptr, 16);

  if (checksum != (status_id ^ 0xAA)) return;  // Bad checksum, silently drop

  Serial.print("RX:");
  Serial.println(status_id);
}

// ── Main Loop ────────────────────────────────────────────────
void loop() {
  // SoftwareSerial can't receive while transmitting, so check LoRa first
  // when not actively sending a command.
  if (loraSerial.available()) {
    handleLoraResponse();
  }

  if (Serial.available()) {
    handleSerialCommand();
  }

  delay(10);
}