#include <PacketSerial.h>
#include "commands.h"
#include "motor_driver.h"

// ===================== 可調參數 =====================
static const uint32_t SERIAL_BAUD = 115200;

// // 取樣間隔（用來估測轉速）
// static const uint16_t SAMPLE_PERIOD_MS = 50;

// // 你的編碼器每圈計數數（用於你要換算 RPM 可用；本版先回 ticks/sec）
// static const int32_t ENC_COUNTS_PER_REV = 1024;

// // 是否使用正交編碼器
// #define USE_QUADRATURE 1
// // ====================================================

PacketSerial myPacketSerial;

// Stores the most recently commanded motor speeds (as received over serial)
static Speeds g_lastCommandedSpeeds = {0, 0, 0, 0};

// // 4 輪順序：FL, FR, RL, RR
// enum { FL = 0, FR = 1, RL = 2, RR = 3 };

// // -------------------- 編碼器相關 --------------------
// volatile int32_t enc_counts[4] = {0, 0, 0, 0};   // 累積計數
// volatile int8_t  last_state[4] = {0, 0, 0, 0};   // quadrature 上次狀態（00/01/10/11）

// // TODO: 設定你的編碼器腳位
// // A 相腳位：
// const uint8_t ENC_A_PIN[4] = {
//   2,  // FL A
//   3,  // FR A
//   18, // RL A
//   19  // RR A
// };
// // B 相腳位（若單通道則可忽略，但腳位仍需有值）
// const uint8_t ENC_B_PIN[4] = {
//   4,   // FL B
//   5,   // FR B
//   20,  // RL B
//   21   // RR B
// };

// // 每個輪子的 ISR — 為了少寫四份，用 lambda 綁 index
// void IRAM_ATTR isr_generic(uint8_t idx) {
// #if USE_QUADRATURE
//   // 讀取 A/B，組成 2bit 狀態
//   int a = digitalRead(ENC_A_PIN[idx]);
//   int b = digitalRead(ENC_B_PIN[idx]);
//   int state = (a << 1) | b;           // 00/01/10/11

//   // 依照 quadrature 轉移表計算 +1 或 -1
//   // 合法相鄰轉移：00->01->11->10->00（順時針 +1），反向 -1
//   static const int8_t quad_table[4][4] = {
//     // prev -> 00  01  10  11 (curr)
//     /* 00 */ {  0, +1, -1,  0},
//     /* 01 */ { -1,  0,  0, +1},
//     /* 10 */ { +1,  0,  0, -1},
//     /* 11 */ {  0, -1, +1,  0}
//   };
//   int8_t d = quad_table[last_state[idx] & 0x3][state & 0x3];
//   enc_counts[idx] += d;
//   last_state[idx] = state;
// #else
//   // 單通道：A 的 RISING 計一個 tick，方向用 B 判斷
//   if (digitalRead(ENC_B_PIN[idx])) {
//     enc_counts[idx]++;   // 例如 B=1 表示正向
//   } else {
//     enc_counts[idx]--;
//   }
// #endif
// }

// // 針對四個輪子的 ISR 包裝
// void IRAM_ATTR isr_FL() { isr_generic(FL); }
// void IRAM_ATTR isr_FR() { isr_generic(FR); }
// void IRAM_ATTR isr_RL() { isr_generic(RL); }
// void IRAM_ATTR isr_RR() { isr_generic(RR); }

// -------------------- 轉速估測 --------------------


void update_speed_estimation() {
}

// -------------------- 封包處理 --------------------
// 協議：
// 1) 相容舊版：payload == 8 bytes => 直接視為 'm' 設定速度（4×int16）
// 2) 新版命令：buffer[0] = opcode
//    'm' (MOTOR_SPEEDS): 後續 8 bytes = 4×int16
//    'v' (READ_SPEEDS): 無 payload，回傳 4×int16（ticks/sec）
//    'e' (READ_ENCODERS): 無 payload，回傳 4×int32（raw counts）

static inline void send_speeds_tps() {
  // For now, return the last commanded speeds (echo of what was set)
  myPacketSerial.send(reinterpret_cast<const uint8_t*>(&g_lastCommandedSpeeds), sizeof(Speeds));
}

static inline void send_raw_counts() {
  int16_t c[4];
  // noInterrupts();
  // c[0] = enc_counts[FL];
  // c[1] = enc_counts[FR];
  // c[2] = enc_counts[RL];
  // c[3] = enc_counts[RR];
  // interrupts();
  c[0] = 50;
  c[1] = 50;
  c[2] = 50;
  c[3] = 50;
  myPacketSerial.send(reinterpret_cast<const uint8_t*>(c), sizeof(c));
}

void onPacketReceived(const uint8_t* buffer, size_t size) {
  // 相容舊版：只有 8 bytes => 視為設定速度
  // if (size == 8) {
  //   int16_t fl, fr, rl, rr;
  //   memcpy(&fl, buffer + 0, 2);
  //   memcpy(&fr, buffer + 2, 2);
  //   memcpy(&rl, buffer + 4, 2);
  //   memcpy(&rr, buffer + 6, 2);
  //   setMotorSpeeds(fl, fr, rl, rr);
  //   // 保留回 echo（方便上位機自測）
  //   myPacketSerial.send(buffer, size);
  //   return;
  // }

  if (size == 0) return;

  char op = static_cast<char>(buffer[0]);
  switch (op) {
    case SET_MOTOR_SPEEDS: // 'm'，來自 commands.h
    {
      if (size < 1 + 8) return;
      Speeds s;
      memcpy(&s.fl, buffer + 1, 2);
      memcpy(&s.fr, buffer + 3, 2);
      memcpy(&s.rl, buffer + 5, 2);
      memcpy(&s.rr, buffer + 7, 2);
      // Remember what was commanded so GET_MOTOR_SPEEDS can report it back
      g_lastCommandedSpeeds = s;
      setMotorSpeeds(s);
      // 回 ACK：回送同樣 payload（可改為單字節 OK 碼）
      myPacketSerial.send(buffer + 1, 8);
      break;
    }
    case GET_MOTOR_SPEEDS: // READ_SPEEDS：回傳 4×int16（ticks/sec）
    {
      send_speeds_tps();
      break;
    }
    case READ_ENCODERS: // 'e'：回傳 4×int16 raw counts（可選）
    {
      send_raw_counts();
      break;
    }
    default:
      // 未知指令：忽略或回錯誤碼
      break;
  }
}

// -------------------- 初始化 --------------------
void setupEncoders() {
//   // 設成輸入
//   for (int i = 0; i < 4; ++i) {
//     pinMode(ENC_A_PIN[i], INPUT_PULLUP);
//     pinMode(ENC_B_PIN[i], INPUT_PULLUP);
//   }

// #if USE_QUADRATURE
//   // 初始化 last_state
//   for (int i = 0; i < 4; ++i) {
//     int a = digitalRead(ENC_A_PIN[i]);
//     int b = digitalRead(ENC_B_PIN[i]);
//     last_state[i] = (a << 1) | b;
//   }
//   // A/B 都掛中斷（CHANGE）
//   attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[FL]), isr_FL, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENC_B_PIN[FL]), isr_FL, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[FR]), isr_FR, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENC_B_PIN[FR]), isr_FR, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[RL]), isr_RL, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENC_B_PIN[RL]), isr_RL, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[RR]), isr_RR, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENC_B_PIN[RR]), isr_RR, CHANGE);
// #else
//   // 單通道：只掛 A 的 RISING
//   attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[FL]), isr_FL, RISING);
//   attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[FR]), isr_FR, RISING);
//   attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[RL]), isr_RL, RISING);
//   attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[RR]), isr_RR, RISING);
// #endif
}

void setup() {
  initMotorController();
  pinMode(LED_BUILTIN, OUTPUT);

  // setupEncoders();

  myPacketSerial.begin(SERIAL_BAUD);
  myPacketSerial.setPacketHandler(&onPacketReceived);

  setMotorSpeeds(Speeds{0, 0, 0, 0});
}

void loop() {
  myPacketSerial.update();
  update_speed_estimation();

  // 心跳燈
  // static uint32_t lastBlink = 0;
  // if (millis() - lastBlink > 1000) {
  //   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //   lastBlink = millis();
  // }
}
