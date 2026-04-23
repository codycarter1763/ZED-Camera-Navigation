#include <Arduino.h>
#include <HardwareSerial.h>

// ── RYLR998 UART Config ──────────────────────────────────────
#define LORA_RX_PIN 19   
#define LORA_TX_PIN 18   

HardwareSerial loraSerial(1);

// ── RYLR998 AT settings ──────────────────────────────────────
#define LORA_NETWORK_ID   18
#define LORA_ADDRESS      10
#define LORA_DEST_ADDR    11
#define LORA_BAND         915000000
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
bool sendAT(const String& cmd, unsigned long timeoutMs = 3000) {
  while (loraSerial.available()) loraSerial.read();
  loraSerial.println(cmd);

  unsigned long start = millis();
  String resp = "";

  while (millis() - start < timeoutMs) {
    while (loraSerial.available()) {
      resp += (char)loraSerial.read();
    }
    if (resp.indexOf("+OK")    >= 0) return true;
    if (resp.indexOf("+READY") >= 0) return true;
    if (resp.indexOf("+ERR")   >= 0) return false;
  }
  Serial.print("[LoRa] No response to: ");
  Serial.print(cmd);
  Serial.print(" | raw: ");
  Serial.println(resp);
  return false;
}

// ── LoRa Transmit ────────────────────────────────────────────
bool loraSend(uint8_t cmd_id) {
  uint8_t checksum = cmd_id ^ 0xAA;
  char hexPayload[5];
  snprintf(hexPayload, sizeof(hexPayload), "%02X%02X", cmd_id, checksum);

  String atCmd = "AT+SEND=";
  atCmd += LORA_DEST_ADDR;
  atCmd += ",2,";
  atCmd += hexPayload;

  return sendAT(atCmd, 4000);
}

// ── LoRa Init ────────────────────────────────────────────────
void initLora() {
  loraSerial.begin(115200, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(1000);

  // First ping
  bool alive = sendAT("AT", 2000);
  if (!alive) {
    Serial.println("[LoRa] WARN: module not responding on first ping");
    Serial.println("[LoRa] Check wiring: TX→GPIO15, RX→GPIO17, VDD→3.3V");
    Serial.println("CLARQ_RF_READY");
    return;
  }
  Serial.println("[LoRa] Module alive ✓");

  // Reset and wait for module to reboot
  sendAT("AT+RESET", 3000);
  delay(2000);

  // Re-ping after reset
  bool alive2 = sendAT("AT", 2000);
  if (!alive2) {
    Serial.println("[LoRa] WARN: module lost after reset");
    Serial.println("CLARQ_RF_READY");
    return;
  }
  Serial.println("[LoRa] Post-reset ping ✓");

  // Configure
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
  Serial.println("[LoRa] Configured ✓");

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
void handleLoraResponse() {
  String line = loraSerial.readStringUntil('\n');
  line.trim();

  if (!line.startsWith("+RCV=")) return;

  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  int c3 = line.indexOf(',', c2 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0) return;

  String hexData = line.substring(c2 + 1, c3);
  if (hexData.length() < 4) return;

  uint8_t status_id = (uint8_t)strtol(hexData.substring(0, 2).c_str(), nullptr, 16);
  uint8_t checksum  = (uint8_t)strtol(hexData.substring(2, 4).c_str(), nullptr, 16);

  if (checksum != (status_id ^ 0xAA)) return;

  Serial.print("RX:");
  Serial.println(status_id);
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);
  while (Serial.available()) Serial.read();
  initLora();
}

// ── Main Loop ────────────────────────────────────────────────
void loop() {
  if (loraSerial.available()) {
    handleLoraResponse();
  }
  if (Serial.available()) {
    handleSerialCommand();
  }
  delay(10);
}