# esp32_main.py  ->  simpan di ESP32 dengan nama "main.py" (MicroPython)
# Tugas ESP32: HANYA menerima perintah sudut lewat UART dan menggerakkan 2 servo.
# Semua logika PID ada di mini PC.
#
# Protokol (teks, diakhiri newline):
#   S,<pan>,<tilt>\n   -> set sudut servo (0-180), contoh: S,90,75
#   PING\n             -> dibalas PONG
#
# Wiring UART (level 3.3V!):
#   STB TX  -> ESP32 GPIO16 (RX2)
#   STB RX  -> ESP32 GPIO17 (TX2)
#   GND STB -> GND ESP32 (wajib sama)
# Servo: sinyal pan = GPIO18, tilt = GPIO19, daya servo dari 5V terpisah,
#        GND servo disambung ke GND ESP32.

from machine import Pin, PWM, UART
import time

PIN_PAN = 18    # kiri - kanan
PIN_TILT = 19   # atas - bawah
UART_ID = 2
PIN_TX = 17
PIN_RX = 16
BAUD = 115200

US_MIN = 500    # lebar pulsa untuk 0 derajat (us)
US_MAX = 2500   # lebar pulsa untuk 180 derajat (us)
SUDUT_AWAL = 90

servo_pan = PWM(Pin(PIN_PAN), freq=50)
servo_tilt = PWM(Pin(PIN_TILT), freq=50)


def sudut_ke_duty(sudut):
    """Ubah sudut 0-180 menjadi duty_u16 untuk PWM 50 Hz (periode 20 ms)."""
    sudut = max(0.0, min(180.0, sudut))
    us = US_MIN + (US_MAX - US_MIN) * sudut / 180.0
    return int(us / 20000.0 * 65535)


def set_servo(pan, tilt):
    servo_pan.duty_u16(sudut_ke_duty(pan))
    servo_tilt.duty_u16(sudut_ke_duty(tilt))


def proses_baris(baris):
    try:
        teks = baris.decode().strip()
    except Exception:
        return
    if not teks:
        return
    if teks == "PING":
        uart.write(b"PONG\n")
        return
    bagian = teks.split(",")
    if len(bagian) == 3 and bagian[0] == "S":
        try:
            set_servo(float(bagian[1]), float(bagian[2]))
        except ValueError:
            pass  # data rusak, abaikan


uart = UART(UART_ID, baudrate=BAUD, tx=PIN_TX, rx=PIN_RX, timeout=10)
set_servo(SUDUT_AWAL, SUDUT_AWAL)
print("ESP32 siap. Servo di", SUDUT_AWAL, "derajat. Menunggu perintah UART...")

buf = b""
while True:
    if uart.any():
        data = uart.read()
        if data:
            buf += data
            if len(buf) > 256:      # buang sampah kalau tidak ada newline
                buf = buf[-64:]
            while b"\n" in buf:
                baris, buf = buf.split(b"\n", 1)
                proses_baris(baris)
    time.sleep_ms(2)
