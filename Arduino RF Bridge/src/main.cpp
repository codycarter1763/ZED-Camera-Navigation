#include <Arduino.h>
#include <HardwareSerial.h>

#define LORA_RX_PIN 19
#define LORA_TX_PIN 18

HardwareSerial loraSerial(1);

#define LORA_ADDRESS   10
#define LORA_DEST_ADDR 11

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

// ── Send plain text command ───────────────────────────────────
bool loraSend(uint8_t cmd_id) {
  // Format: AT+SEND=<dest>,<len>,<data>
  // Data is just the command number as a string e.g. "8"
  String data   = String(cmd_id);
  String atCmd  = "AT+SEND=" + String(LORA_DEST_ADDR) + "," + String(data.length()) + "," + data;

  loraSerial.println(atCmd);

  // Wait for +OK or +ERR
  unsigned long start = millis();
  String resp = "";
  while (millis() - start < 4000) {
    while (loraSerial.available()) {
      resp += (char)loraSerial.read();
    }
    if (resp.indexOf("+OK")  >= 0) return true;
    if (resp.indexOf("+ERR") >= 0) return false;
  }
  return false;
}

// ── Handle incoming from Jetson ───────────────────────────────
// +RCV=<addr>,<len>,<data>,<RSSI>,<SNR>
void handleLoraResponse() {
  String line = loraSerial.readStringUntil('\n');
  line.trim();

  if (!line.startsWith("+RCV=")) return;

  // Extract data field
  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  int c3 = line.indexOf(',', c2 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0) return;

  String data = line.substring(c2 + 1, c3);
  data.trim();

  // Data is just the status id as plain text e.g. "8"
  int status_id = data.toInt();
  if (status_id < 1 || status_id > 17) return;

  Serial.print("RX:");
  Serial.println(status_id);
}

// ── Handle commands from GUI ──────────────────────────────────
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

void setup() {
  Serial.begin(115200);
  delay(2000);
  while (Serial.available()) Serial.read();

  loraSerial.begin(115200, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(1000);

  // Basic init — no reset, just set address
  loraSerial.println("AT");                              delay(500);
  loraSerial.println("AT+ADDRESS=" + String(LORA_ADDRESS)); delay(500);
  while (loraSerial.available()) loraSerial.read();      // flush responses

  Serial.println("CLARQ_RF_READY");
}

void loop() {
  if (loraSerial.available()) {
    handleLoraResponse();
  }
  if (Serial.available()) {
    handleSerialCommand();
  }
  delay(10);
}