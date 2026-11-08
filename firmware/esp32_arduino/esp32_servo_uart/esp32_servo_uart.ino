// esp32_servo_uart.ino
// Tugas ESP32: HANYA menerima perintah sudut lewat UART dan menggerakkan 2 servo.
// Semua logika PID ada di mini PC (tracker_stb.py).
//
// Library: "ESP32Servo" (Arduino IDE -> Library Manager -> cari ESP32Servo)
// Board  : ESP32 Dev Module
//
// Protokol (teks, diakhiri newline), baud 115200:
//   S,<pan>,<tilt>\n   -> set sudut servo (0-180), contoh: S,90,75
//   PING\n             -> dibalas PONG
//
// Wiring UART (level 3.3V!):
//   STB TX  -> ESP32 GPIO16 (RX2)
//   STB RX  -> ESP32 GPIO17 (TX2)
//   GND STB -> GND ESP32 (wajib sama)
// Servo: pan (kiri-kanan) = GPIO18, tilt (atas-bawah) = GPIO19
//        daya servo dari 5V terpisah, GND servo disambung ke GND ESP32.

#include <ESP32Servo.h>

const int PIN_PAN  = 18;   // kiri - kanan
const int PIN_TILT = 19;   // atas - bawah
const int PIN_RX2  = 16;
const int PIN_TX2  = 17;
const long BAUD    = 115200;

const int US_MIN = 500;    // lebar pulsa 0 derajat (us)
const int US_MAX = 2500;   // lebar pulsa 180 derajat (us)
const float SUDUT_AWAL = 90.0;

Servo servoPan;
Servo servoTilt;
String buf = "";

void setSudut(Servo &s, float sudut) {
  sudut = constrain(sudut, 0.0, 180.0);
  int us = US_MIN + (int)((US_MAX - US_MIN) * sudut / 180.0);
  s.writeMicroseconds(us);
}

void prosesBaris(String t) {
  t.trim();
  if (t.length() == 0) return;

  if (t == "PING") {
    Serial2.println("PONG");
    return;
  }

  if (t.startsWith("S,")) {
    int koma = t.indexOf(',', 2);
    if (koma < 0) return;                       // data rusak, abaikan
    float pan  = t.substring(2, koma).toFloat();
    float tilt = t.substring(koma + 1).toFloat();
    setSudut(servoPan, pan);
    setSudut(servoTilt, tilt);
  }
}

void setup() {
  Serial.begin(115200);                          // untuk debug lewat USB
  Serial2.begin(BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);

  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);
  servoPan.attach(PIN_PAN, US_MIN, US_MAX);
  servoTilt.attach(PIN_TILT, US_MIN, US_MAX);

  setSudut(servoPan, SUDUT_AWAL);
  setSudut(servoTilt, SUDUT_AWAL);

  Serial.println("ESP32 siap. Servo di 90 derajat. Menunggu perintah UART...");
}

void loop() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n') {
      prosesBaris(buf);
      buf = "";
    } else if (c != '\r') {
      buf += c;
      if (buf.length() > 40) buf = "";           // buang sampah kalau tidak ada newline
    }
  }
}
