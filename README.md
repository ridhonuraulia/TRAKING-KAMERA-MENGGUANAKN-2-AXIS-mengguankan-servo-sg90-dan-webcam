# Pan-Tilt Red Tracker (ESP32 + Mini PC)

Sistem pelacak objek berwarna **merah** dengan kamera di atas mekanik **pan-tilt** (2 servo).
Mini PC (STB) membaca kamera dan menghitung kontrol, sedangkan ESP32 hanya menggerakkan servo.
Semua pengaturan (PID, arah, batas sudut, mode) dilakukan lewat **webserver** di browser.

## Fitur

- Deteksi objek merah (OpenCV, HSV) dari webcam USB `/dev/video1`
- Kontrol **PID** terpisah untuk pan (kiri-kanan) dan tilt (atas-bawah)
- Mode **IF-ELSE** (langkah tetap) saat PID dimatikan, bisa diganti langsung dari web
- Grafik **error real-time** untuk melihat steady state
- Pengaturan **balik arah servo** dan **balik gambar** (kiri-kanan / atas-bawah)
- Pengaturan tersimpan otomatis (`config_tracking.json`)
- Program tes servo manual (tanpa PID) untuk mengecek kondisi servo

## Arsitektur

```
Webcam USB --> Mini PC (STB) --UART TTL--> ESP32 --PWM--> Servo Pan  (GPIO18)
                  |  (PID / IF-ELSE)                 \--> Servo Tilt (GPIO19)
                  +--> Webserver :5000 (video, PID, grafik)
```

Protokol UART (teks, 115200 baud, diakhiri newline):

| Perintah | Arti |
|---|---|
| `S,<pan>,<tilt>` | Set sudut servo 0-180, contoh `S,90,75` |
| `PING` | Dibalas `PONG` |

## Struktur folder

```
pan-tilt-red-tracker/
├── firmware/
│   ├── esp32_arduino/esp32_servo_uart/esp32_servo_uart.ino   # firmware ESP32 (Arduino)
│   └── esp32_micropython/main.py                              # firmware ESP32 (MicroPython)
├── mini_pc/
│   ├── tracker_stb.py        # program pengendali utama + webserver
│   └── requirements.txt
├── tools/
│   └── tes_servo_manual.py   # tes servo manual 0-180 (tanpa PID)
├── docs/images/              # gambar dokumentasi (wiring, screenshot)
├── assets/3d/                # file desain mekanik 3D (STL, STEP, dll.)
├── LICENSE
└── README.md
```

Pakai **salah satu** firmware ESP32 saja (Arduino atau MicroPython), keduanya memakai protokol yang sama.

## Wiring

| Dari | Ke |
|---|---|
| STB TX | ESP32 GPIO16 (RX2) |
| STB RX | ESP32 GPIO17 (TX2) |
| GND STB | GND ESP32 |
| Servo pan (sinyal) | ESP32 GPIO18 |
| Servo tilt (sinyal) | ESP32 GPIO19 |
| Daya servo | 5V terpisah, GND disambung ke GND ESP32 |

Level UART 3.3V. Jangan menyuplai daya servo dari pin ESP32.

## Instalasi

### ESP32 (pilih salah satu)

**Arduino IDE:** install library **ESP32Servo**, pilih board *ESP32 Dev Module*, buka
`firmware/esp32_arduino/esp32_servo_uart/esp32_servo_uart.ino`, lalu upload.

**MicroPython:** upload `firmware/esp32_micropython/main.py` ke ESP32 (Thonny atau `mpremote`).

### Mini PC (STB)

```bash
sudo apt install python3-opencv python3-flask python3-serial
# atau: pip install -r mini_pc/requirements.txt
sudo usermod -aG dialout $USER    # lalu login ulang
```

Sesuaikan konstanta di bagian atas `tracker_stb.py` bila perlu:

```python
KAMERA = "/dev/video1"
SERIAL_PORT = "/dev/ttyUSB0"   # atau /dev/ttyS1 untuk UART bawaan STB
```

## Menjalankan

```bash
python3 mini_pc/tracker_stb.py
```

Buka alamat yang tercetak di terminal, contoh `http://192.168.1.10:5000`.

### Tuning di webserver

1. Mulai dengan **Ki = 0**, naikkan **Kp** sampai servo responsif tanpa bergetar.
2. Tambah **Kd** sedikit untuk meredam osilasi.
3. Kalau servo bergerak menjauhi objek, centang **Balik arah servo** pada sumbu yang salah.
4. Matikan **Mode PID** untuk beralih ke mode IF-ELSE (atur *Langkah IF-ELSE* dan zona mati).
5. Pantau **grafik error** (skala ±0.1 atau ±0.05) untuk membandingkan steady state kedua mode.

### Tes servo manual

Matikan `tracker_stb.py` terlebih dahulu (port serial hanya boleh dipakai satu program), lalu:

```bash
python3 tools/tes_servo_manual.py
```

Gerakkan slider 0-180 atau tekan **Sapu 0-180** untuk mengecek apakah servo bergerak halus.

## Pemecahan masalah

- **Servo diam:** cek status serial di web, TX/RX tidak tertukar, GND bersama, dan daya servo 5V cukup.
- **Kamera tidak terbuka:** cek `ls /dev/video*` lalu ubah `KAMERA`.
- **Servo bergetar:** turunkan Kp/Kd, atau naikkan zona mati.
- **Arah terbalik:** gunakan *Balik arah servo* (arah gerak) atau *Balik gambar* (orientasi kamera).

## Lisensi

Proyek ini dilisensikan di bawah **Apache License 2.0**. Lihat berkas [LICENSE](LICENSE).
