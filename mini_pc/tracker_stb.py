#!/usr/bin/env python3
# tracker_stb.py - Program PENGENDALI di mini PC (STB)
# - Baca kamera /dev/video1, deteksi objek merah
# - PID untuk pan (kiri-kanan, GPIO18) dan tilt (atas-bawah, GPIO19)
# - Mode kontrol bisa diganti: PID  <->  IF-ELSE (langkah tetap), dari webserver
# - Grafik error real-time (untuk melihat steady state) di webserver
# - Grafik sudut servo: sudut REAL vs sudut TARGET (pan dan tilt)
# - Data Logger CSV: tombol Mulai Rekam / Stop & Simpan di web -> folder HASIL/Hasil_N.csv
# - Kirim sudut ke ESP32 lewat UART TTL
# - Webserver: lihat kamera + atur PID, arah, flip, batas sudut secara manual
#
# Install:
#   sudo apt install python3-opencv python3-flask python3-serial
#   (atau: pip install opencv-python-headless flask pyserial)
#   sudo usermod -aG dialout $USER   (lalu login ulang, agar boleh akses serial)
#
# Jalankan:  python3 tracker_stb.py
# Buka di browser:  http://<ip-stb>:5000

import atexit
import copy
import csv
import json
import os
import re
import signal
import socket
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np
import serial
from flask import Flask, Response, jsonify, render_template_string, request

# ===================== PENGATURAN DASAR =====================
KAMERA = "/dev/video1"
SERIAL_PORT = "/dev/ttyUSB0"   # ganti sesuai UART TTL di STB (mis. /dev/ttyS1)
SERIAL_BAUD = 115200
WEB_PORT = 5000
FILE_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "config_tracking.json")
LEBAR, TINGGI, FPS_KAMERA = 640, 480, 30
KIRIM_MAKS_HZ = 50
DIR_SKRIP = os.path.dirname(os.path.abspath(__file__))
DIR_HASIL = os.path.join(DIR_SKRIP, "HASIL")   # folder hasil rekaman CSV
HEADER_CSV = ["Time_ms", "SP_X", "PV_X", "Err_X", "P_X", "I_X", "D_X", "Out_PID_X", "Sudut_Pan",
              "SP_Y", "PV_Y", "Err_Y", "P_Y", "I_Y", "D_Y", "Out_PID_Y", "Sudut_Tilt"]

CONFIG_DEFAULT = {
    "tracking": True,            # True = otomatis, False = servo manual
    "mode_pid": True,            # True = PID, False = IF-ELSE (langkah tetap)
    "fov_h": 60.0,               # sudut pandang horizontal kamera (derajat)
    "fov_v": 45.0,               # sudut pandang vertikal kamera (derajat)
    "flip_h": False,             # balik gambar kiri-kanan
    "flip_v": False,             # balik gambar atas-bawah
    "tampil_mask": False,        # tampilkan mask deteksi (untuk tuning warna)
    "min_area": 400,             # luas minimum objek (piksel)
    "deadzone": 0.03,            # zona mati error (0-1)
    "s_min": 120,                # batas bawah saturasi HSV
    "v_min": 70,                 # batas bawah value HSV
    "timeout_hilang": 5.0,       # detik sebelum kembali ke tengah
    "pan": {"kp": 60.0, "ki": 0.0, "kd": 5.0, "invert": False,
            "min": 20, "max": 160, "manual": 90, "step": 40.0},
    "tilt": {"kp": 60.0, "ki": 0.0, "kd": 5.0, "invert": False,
             "min": 40, "max": 140, "manual": 90, "step": 40.0},
}

# ===================== KONFIGURASI =====================
lock = threading.Lock()
cfg = copy.deepcopy(CONFIG_DEFAULT)


def gabung(dasar, baru):
    """Gabungkan config tersimpan ke default (hanya key yang dikenal)."""
    for k, v in baru.items():
        if k in dasar:
            if isinstance(dasar[k], dict) and isinstance(v, dict):
                gabung(dasar[k], v)
            else:
                dasar[k] = type(dasar[k])(v) if not isinstance(dasar[k], bool) else bool(v)


def muat_config():
    if os.path.exists(FILE_CONFIG):
        try:
            with open(FILE_CONFIG) as f:
                gabung(cfg, json.load(f))
            print("[INFO] Config dimuat dari", FILE_CONFIG)
        except Exception as e:
            print("[PERINGATAN] Gagal baca config:", e)


def simpan_config():
    try:
        with open(FILE_CONFIG, "w") as f:
            json.dump(cfg, f, indent=2)
    except Exception as e:
        print("[PERINGATAN] Gagal simpan config:", e)


# ===================== PID =====================
class PID:
    def __init__(self):
        self.integral = 0.0
        self.err_lalu = 0.0

    def reset(self):
        self.integral = 0.0
        self.err_lalu = 0.0

    def hitung(self, err, kp, ki, kd, dt):
        """Hitung PID memakai error ternormalisasi (-1..1).
        Mengembalikan (total, P, I, D) semuanya dalam derajat/detik."""
        self.integral += err * dt
        self.integral = max(-1.0, min(1.0, self.integral))  # anti-windup
        turunan = (err - self.err_lalu) / dt if dt > 0 else 0.0
        self.err_lalu = err
        p_term = kp * err
        i_term = ki * self.integral
        d_term = kd * turunan
        return p_term + i_term + d_term, p_term, i_term, d_term


# ===================== SERIAL KE ESP32 =====================
class KomunikasiESP32:
    def __init__(self):
        self.ser = None
        self.terakhir_coba = 0.0

    def buka(self):
        if time.time() - self.terakhir_coba < 2.0:
            return
        self.terakhir_coba = time.time()
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
            print("[INFO] Serial terhubung:", SERIAL_PORT)
        except Exception as e:
            self.ser = None
            print("[PERINGATAN] Serial belum terhubung (%s): %s" % (SERIAL_PORT, e))

    @property
    def ok(self):
        return self.ser is not None

    def kirim(self, pan, tilt):
        if self.ser is None:
            self.buka()
            return
        try:
            self.ser.write(("S,%.1f,%.1f\n" % (pan, tilt)).encode())
        except Exception as e:
            print("[PERINGATAN] Gagal kirim serial:", e)
            self.ser = None


esp = KomunikasiESP32()

# ===================== STATUS BERSAMA =====================
status = {"jpeg": None, "pan": 90.0, "tilt": 90.0, "ditemukan": False, "fps": 0.0}

# riwayat untuk grafik: (waktu, error_pan, error_tilt, sudut_pan, sudut_tilt,
#                       target_pan, target_tilt)
riwayat = deque(maxlen=400)   # 20 Hz -> sekitar 20 detik
T0 = time.time()


# ===================== DATA LOGGER CSV =====================
# Semua akses ke dict `rekam` dilakukan di dalam `with lock:` (lock yang sama
# dengan config/status), jadi aman antar-thread (loop kontrol vs request web).
rekam = {"aktif": False, "file": None, "writer": None, "nama": "",
         "t_mulai_rekam": None, "baris": 0, "t_flush": 0.0,
         "terakhir_nama": "", "terakhir_baris": 0}


def nomor_hasil_tertinggi():
    """Pindai folder HASIL, kembalikan nomor urut tertinggi (0 jika kosong)."""
    tertinggi = 0
    for nama in os.listdir(DIR_HASIL):
        m = re.fullmatch(r"Hasil_(\d+)\.csv", nama)
        if m:
            tertinggi = max(tertinggi, int(m.group(1)))
    return tertinggi


def mulai_rekam():
    """Buka Hasil_<N+1>.csv baru dan tulis header. Mengembalikan nama file."""
    with lock:
        if rekam["aktif"]:
            return rekam["nama"]
        os.makedirs(DIR_HASIL, exist_ok=True)
        nomor = nomor_hasil_tertinggi() + 1
        while True:
            nama = "Hasil_%d.csv" % nomor
            try:
                # mode "x" = gagal kalau file sudah ada, jadi file lama tidak pernah tertimpa
                f = open(os.path.join(DIR_HASIL, nama), "x", newline="", encoding="utf-8")
                break
            except FileExistsError:
                nomor += 1
        writer = csv.writer(f)
        writer.writerow(HEADER_CSV)
        f.flush()
        rekam.update(aktif=True, file=f, writer=writer, nama=nama,
                     t_mulai_rekam=None,   # diisi di frame pertama -> Time_ms mulai dari 0
                     baris=0, t_flush=time.time())
        print("[INFO] Rekaman dimulai: %s" % os.path.join("HASIL", nama))
        return nama


def _tutup_rekam_locked():
    """Tutup file CSV dengan aman. HARUS dipanggil saat `lock` sudah dipegang."""
    if not rekam["aktif"]:
        return None
    nama, baris = rekam["nama"], rekam["baris"]
    try:
        rekam["file"].flush()
        os.fsync(rekam["file"].fileno())
    except Exception:
        pass
    try:
        rekam["file"].close()
    except Exception as e:
        print("[PERINGATAN] Gagal menutup file CSV:", e)
    rekam.update(aktif=False, file=None, writer=None, t_mulai_rekam=None,
                 terakhir_nama=nama, terakhir_baris=baris)
    print("[INFO] Rekaman disimpan: %s (%d baris)" % (os.path.join("HASIL", nama), baris))
    return nama, baris


def stop_rekam():
    with lock:
        return _tutup_rekam_locked()


def _status_rekam_locked():
    """Ringkasan status rekaman. HARUS dipanggil saat `lock` sudah dipegang."""
    durasi = 0.0
    if rekam["aktif"] and rekam["t_mulai_rekam"] is not None:
        durasi = time.time() - rekam["t_mulai_rekam"]
    return {"aktif": rekam["aktif"], "nama": rekam["nama"], "baris": rekam["baris"],
            "durasi": round(durasi, 1),
            "terakhir_nama": rekam["terakhir_nama"], "terakhir_baris": rekam["terakhir_baris"]}


def hitung_ifelse(err, langkah, deadzone):
    """Mode IF-ELSE: geser servo dengan kecepatan tetap (derajat/detik)
    menurut tanda error. Di dalam zona mati servo diam."""
    if err > deadzone:
        return langkah
    elif err < -deadzone:
        return -langkah
    else:
        return 0.0


def batas(nilai, kecil, besar):
    return max(kecil, min(besar, nilai))


# ===================== LOOP KAMERA + KONTROL =====================
def loop_kontrol():
    cap = cv2.VideoCapture(KAMERA, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, LEBAR)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TINGGI)
    cap.set(cv2.CAP_PROP_FPS, FPS_KAMERA)
    if not cap.isOpened():
        print("[ERROR] Kamera %s tidak bisa dibuka" % KAMERA)
        return
    print("[INFO] Kamera %s terbuka" % KAMERA)

    pid_pan, pid_tilt = PID(), PID()
    pan, tilt = 90.0, 90.0
    t_lalu = time.time()
    t_terlihat = 0.0
    t_kirim = 0.0
    t_rekam = 0.0
    kernel = np.ones((5, 5), np.uint8)
    hitung_fps, t_fps = 0, time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[PERINGATAN] Gagal baca frame, coba lagi...")
            time.sleep(0.1)
            continue

        sekarang = time.time()
        dt = max(1e-3, sekarang - t_lalu)
        t_lalu = sekarang

        with lock:
            c = copy.deepcopy(cfg)

        if c["flip_h"]:
            frame = cv2.flip(frame, 1)
        if c["flip_v"]:
            frame = cv2.flip(frame, 0)

        tinggi, lebar = frame.shape[:2]

        # ---- deteksi merah (HSV, dua rentang) ----
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, (0, c["s_min"], c["v_min"]), (10, 255, 255))
        m2 = cv2.inRange(hsv, (170, c["s_min"], c["v_min"]), (180, 255, 255))
        mask = cv2.morphologyEx(cv2.bitwise_or(m1, m2), cv2.MORPH_OPEN, kernel)
        mask = cv2.dilate(mask, kernel, iterations=1)

        kontur, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        target = None
        if kontur:
            terbesar = max(kontur, key=cv2.contourArea)
            if cv2.contourArea(terbesar) >= c["min_area"]:
                mom = cv2.moments(terbesar)
                if mom["m00"] > 0:
                    target = (int(mom["m10"] / mom["m00"]), int(mom["m01"] / mom["m00"]),
                              cv2.boundingRect(terbesar))

        tampilan = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if c["tampil_mask"] else frame

        # ---- kontrol ----
        err_x_g = err_y_g = None   # untuk grafik
        tgt_pan_g = tgt_tilt_g = None
        pid_x = pid_y = (0.0, 0.0, 0.0, 0.0)   # (total, P, I, D) untuk CSV
        if c["tracking"]:
            if target is not None:
                cx, cy, (bx, by, bw, bh) = target
                t_terlihat = sekarang
                # error ternormalisasi -1..1 (kanan/bawah = positif)
                ex = (cx - lebar / 2.0) / (lebar / 2.0)
                ey = (cy - tinggi / 2.0) / (tinggi / 2.0)
                err_x_g, err_y_g = ex, ey   # error asli (sebelum zona mati)
                p, t = c["pan"], c["tilt"]

                # sudut TARGET = sudut servo yang dibutuhkan agar objek tepat di tengah
                # gambar (dari posisi objek di kamera dan FOV kamera)
                tgt_pan_g = pan + (-1 if p["invert"] else 1) * ex * c["fov_h"] / 2.0
                tgt_tilt_g = tilt + (-1 if t["invert"] else 1) * ey * c["fov_v"] / 2.0

                if c["mode_pid"]:
                    # ---- mode PID ----
                    if abs(ex) < c["deadzone"]:
                        ex = 0.0
                        pid_pan.integral = 0.0
                    if abs(ey) < c["deadzone"]:
                        ey = 0.0
                        pid_tilt.integral = 0.0
                    pid_x = pid_pan.hitung(ex, p["kp"], p["ki"], p["kd"], dt)
                    pid_y = pid_tilt.hitung(ey, t["kp"], t["ki"], t["kd"], dt)
                    out_pan, out_tilt = pid_x[0], pid_y[0]
                else:
                    # ---- mode IF-ELSE (PID dimatikan) ----
                    pid_pan.reset()
                    pid_tilt.reset()
                    out_pan = hitung_ifelse(ex, p["step"], c["deadzone"])
                    out_tilt = hitung_ifelse(ey, t["step"], c["deadzone"])
                    pid_x = (out_pan, 0.0, 0.0, 0.0)    # tanpa komponen P/I/D
                    pid_y = (out_tilt, 0.0, 0.0, 0.0)
                pan += (-1 if p["invert"] else 1) * out_pan * dt
                tilt += (-1 if t["invert"] else 1) * out_tilt * dt

                cv2.rectangle(tampilan, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
                cv2.circle(tampilan, (cx, cy), 5, (0, 255, 0), -1)
            else:
                pid_pan.reset()
                pid_tilt.reset()
                # hilang lebih dari timeout -> pelan-pelan kembali ke tengah
                if sekarang - t_terlihat > c["timeout_hilang"]:
                    langkah = 60.0 * dt
                    pan += batas(90.0 - pan, -langkah, langkah)
                    tilt += batas(90.0 - tilt, -langkah, langkah)
        else:
            pan = float(c["pan"]["manual"])
            tilt = float(c["tilt"]["manual"])
            pid_pan.reset()
            pid_tilt.reset()

        pan = batas(pan, c["pan"]["min"], c["pan"]["max"])
        tilt = batas(tilt, c["tilt"]["min"], c["tilt"]["max"])

        # ---- kirim ke ESP32 ----
        if sekarang - t_kirim >= 1.0 / KIRIM_MAKS_HZ:
            esp.kirim(pan, tilt)
            t_kirim = sekarang

        # ---- data logger CSV (hanya jika tombol Mulai Rekam sudah ditekan) ----
        with lock:
            if rekam["aktif"]:
                try:
                    if rekam["t_mulai_rekam"] is None:
                        rekam["t_mulai_rekam"] = sekarang    # baris pertama: Time_ms = 0
                    t_ms = int((sekarang - rekam["t_mulai_rekam"]) * 1000)
                    sp_x, sp_y = lebar / 2.0, tinggi / 2.0
                    if target is not None:
                        pv_x, pv_y = target[0], target[1]
                        err_x_px, err_y_px = round(sp_x - pv_x, 2), round(sp_y - pv_y, 2)
                        nx, ny = pid_x, pid_y
                    else:
                        # objek hilang: PV/Err kosong, P/I/D/Out = 0.0, waktu & sudut tetap tercatat
                        pv_x = pv_y = err_x_px = err_y_px = ""
                        nx = ny = (0.0, 0.0, 0.0, 0.0)
                    def b(x, n=4):          # pembulatan; "+ 0.0" mengubah -0.0 menjadi 0.0
                        return round(x, n) + 0.0
                    rekam["writer"].writerow([
                        t_ms, sp_x, pv_x, err_x_px,
                        b(nx[1]), b(nx[2]), b(nx[3]), b(nx[0]), b(pan, 2),
                        sp_y, pv_y, err_y_px,
                        b(ny[1]), b(ny[2]), b(ny[3]), b(ny[0]), b(tilt, 2)])
                    rekam["baris"] += 1
                    if sekarang - rekam["t_flush"] >= 1.0:   # flush tiap 1 detik
                        rekam["file"].flush()
                        rekam["t_flush"] = sekarang
                except Exception as e:
                    print("[ERROR] Gagal menulis CSV, rekaman dihentikan:", e)
                    _tutup_rekam_locked()

        # ---- rekam untuk grafik (20 Hz) ----
        if sekarang - t_rekam >= 0.05:
            t_rekam = sekarang
            with lock:
                riwayat.append((round(sekarang - T0, 3),
                                None if err_x_g is None else round(err_x_g, 4),
                                None if err_y_g is None else round(err_y_g, 4),
                                round(pan, 1), round(tilt, 1),
                                None if tgt_pan_g is None else round(batas(tgt_pan_g, 0, 180), 1),
                                None if tgt_tilt_g is None else round(batas(tgt_tilt_g, 0, 180), 1)))

        # ---- gambar overlay ----
        cv2.line(tampilan, (lebar // 2, 0), (lebar // 2, tinggi), (255, 255, 0), 1)
        cv2.line(tampilan, (0, tinggi // 2), (lebar, tinggi // 2), (255, 255, 0), 1)
        cv2.putText(tampilan, "Pan %.1f  Tilt %.1f  %s" %
                    (pan, tilt, ("PID" if c["mode_pid"] else "IF-ELSE") if c["tracking"] else "MANUAL"),
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        hitung_fps += 1
        if sekarang - t_fps >= 1.0:
            fps = hitung_fps / (sekarang - t_fps)
            hitung_fps, t_fps = 0, sekarang
            with lock:
                status["fps"] = fps

        _, jpg = cv2.imencode(".jpg", tampilan, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with lock:
            status["jpeg"] = jpg.tobytes()
            status["pan"], status["tilt"] = pan, tilt
            status["ditemukan"] = target is not None


# ===================== WEBSERVER =====================
app = Flask(__name__)

HTML = """<!doctype html>
<html lang="id"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Tracker Merah PID</title>
<style>
 body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:10px}
 .wrap{display:flex;flex-wrap:wrap;gap:14px}
 img{max-width:100%;border:2px solid #444;border-radius:6px}
 .panel{background:#1c1c1c;border-radius:8px;padding:10px 14px;min-width:300px}
 h3{margin:6px 0;color:#7cf}
 .baris{display:flex;align-items:center;gap:8px;margin:5px 0;font-size:14px}
 .baris label{width:130px}
 .baris input[type=range]{flex:1}
 .baris span{width:52px;text-align:right;font-variant-numeric:tabular-nums}
 button{padding:7px 12px;margin:3px;border:0;border-radius:5px;background:#27a;color:#fff;cursor:pointer}
 #st{font-size:13px;color:#9f9}
 canvas{background:#000;border:1px solid #444;border-radius:6px;max-width:100%;display:block;margin-top:6px}
 select{background:#222;color:#eee;border:1px solid #555;padding:3px}
 button.merah{background:#a33}
 #rek{font-size:15px;font-weight:bold;margin:6px 0;font-variant-numeric:tabular-nums}
</style></head><body>
<h2>Tracker Merah - Pan/Tilt PID</h2>
<div class="wrap">
 <div style="max-width:660px"><img src="/video"><div id="st">...</div>
  <h3>Grafik Error (steady state)</h3>
  <div class="baris"><label>Skala grafik &plusmn;</label>
   <select id="skala"><option value="1">1.0</option><option value="0.5">0.5</option>
   <option value="0.2" selected>0.2</option><option value="0.1">0.1</option>
   <option value="0.05">0.05</option></select></div>
  <canvas id="gp" width="640" height="150"></canvas>
  <canvas id="gt" width="640" height="150"></canvas>
  <div id="ss" style="font-size:13px;color:#fc8;margin-top:6px"></div>
  <h3>Grafik Sudut Servo (real vs target)</h3>
  <canvas id="sa" width="640" height="170"></canvas>
  <canvas id="sb" width="640" height="170"></canvas>
  <div id="ssa" style="font-size:13px;color:#fc8;margin-top:6px"></div></div>
 <div>
  <div class="panel"><h3>Umum</h3><div id="umum"></div>
   <button onclick="tengah()">Servo ke tengah (90)</button></div>
  <div class="panel"><h3>Data Logger (CSV)</h3>
   <div id="rek" style="color:#9aa">IDLE</div>
   <button onclick="rekam('mulai')">Mulai Rekam</button>
   <button class="merah" onclick="rekam('stop')">Stop &amp; Simpan</button></div>
  <div class="panel"><h3>Pan (kiri-kanan, GPIO18)</h3><div id="pan"></div></div>
  <div class="panel"><h3>Tilt (atas-bawah, GPIO19)</h3><div id="tilt"></div></div>
 </div>
</div>
<script>
let cfg={};
const UMUM=[["tracking","Tracking AUTO","cek"],["mode_pid","Mode PID (off = IF-ELSE)","cek"],["flip_h","Balik gambar kiri-kanan","cek"],
 ["flip_v","Balik gambar atas-bawah","cek"],["tampil_mask","Tampil mask","cek"],
 ["min_area","Luas min","sl",50,5000,50],["deadzone","Zona mati","sl",0,0.3,0.005],
 ["s_min","S min (HSV)","sl",0,255,1],["v_min","V min (HSV)","sl",0,255,1],
 ["timeout_hilang","Timeout hilang (s)","sl",0,15,0.5],
 ["fov_h","FOV horizontal kamera","sl",20,120,1],["fov_v","FOV vertikal kamera","sl",15,100,1]];
const AXIS=[["kp","Kp","sl",0,200,0.5],["ki","Ki","sl",0,100,0.1],["kd","Kd","sl",0,50,0.1],
 ["step","Langkah IF-ELSE (deg/s)","sl",1,200,1],["invert","Balik arah servo","cek"],["min","Sudut min","sl",0,180,1],
 ["max","Sudut max","sl",0,180,1],["manual","Manual (AUTO off)","sl",0,180,1]];
let tunda={};
function kirim(path,val){
  const pp=path.split('.');let o=cfg;for(let i=0;i<pp.length-1;i++)o=o[pp[i]];o[pp[pp.length-1]]=val;
  clearTimeout(tunda[path]);
  tunda[path]=setTimeout(()=>fetch('/api/config',{method:'POST',
   headers:{'Content-Type':'application/json'},body:JSON.stringify({path:path,value:val})}),80);
}
function buat(el,awal,def){
  const [k,label,tipe,mn,mx,st]=def, path=awal?awal+'.'+k:k;
  const d=document.createElement('div');d.className='baris';
  const nilai=awal?cfg[awal][k]:cfg[k];
  if(tipe==='cek'){
    d.innerHTML=`<label>${label}</label><input type=checkbox ${nilai?'checked':''}>`;
    d.querySelector('input').onchange=e=>kirim(path,e.target.checked);
  }else{
    d.innerHTML=`<label>${label}</label><input type=range min=${mn} max=${mx} step=${st} value=${nilai}><span>${nilai}</span>`;
    const r=d.querySelector('input'),s=d.querySelector('span');
    r.oninput=()=>{s.textContent=r.value;kirim(path,parseFloat(r.value));};
  }
  document.getElementById(el).appendChild(d);
}
function rekam(aksi){fetch('/api/rekam/'+aksi,{method:'POST'});}
function tengah(){
  fetch('/api/center',{method:'POST'}).then(()=>location.reload());
}
fetch('/api/config').then(r=>r.json()).then(c=>{
  cfg=c;
  UMUM.forEach(d=>buat('umum','',d));
  AXIS.forEach(d=>buat('pan','pan',d));
  AXIS.forEach(d=>buat('tilt','tilt',d));
});
setInterval(()=>fetch('/api/status').then(r=>r.json()).then(s=>{
  const rk=s.rekam,el=document.getElementById('rek');
  if(rk.aktif){el.style.color='#f66';
    el.textContent='MEREKAM: '+rk.nama+'  |  '+rk.durasi.toFixed(1)+' dtk  |  '+rk.baris+' baris';}
  else{el.style.color='#9aa';
    el.textContent='IDLE'+(rk.terakhir_nama?'  (tersimpan terakhir: '+rk.terakhir_nama+', '+rk.terakhir_baris+' baris)':'');}
  document.getElementById('st').textContent=
   `Pan ${s.pan.toFixed(1)}  Tilt ${s.tilt.toFixed(1)}  | Objek: ${s.ditemukan?'ADA':'tidak ada'}  | Serial ESP32: ${s.serial?'OK':'TERPUTUS'}  | ${s.fps.toFixed(1)} FPS`;
}),500);
function gambar(id,data,idx,judul,dz,sk,warna){
  const cv=document.getElementById(id),g=cv.getContext('2d'),W=cv.width,H=cv.height;
  g.clearRect(0,0,W,H);
  const jendela=15, tn=data.length?data[data.length-1][0]:0;
  g.lineWidth=1;g.strokeStyle='#666';g.beginPath();g.moveTo(0,H/2);g.lineTo(W,H/2);g.stroke();
  g.strokeStyle='#3a3';[dz,-dz].forEach(v=>{const y=H/2-v/sk*H/2;g.beginPath();g.moveTo(0,y);g.lineTo(W,y);g.stroke();});
  g.strokeStyle=warna;g.lineWidth=2;g.beginPath();let buka=false;
  data.forEach(d=>{const v=d[idx];if(v===null){buka=false;return;}
    const x=W-(tn-d[0])/jendela*W,y=H/2-Math.max(-1,Math.min(1,v/sk))*H/2*0.97;
    if(!buka){g.moveTo(x,y);buka=true;}else g.lineTo(x,y);});
  g.stroke();g.lineWidth=1;
  g.fillStyle='#ccc';g.font='12px sans-serif';
  g.fillText(judul+'   (garis hijau = zona mati, skala \u00b1'+sk+', 15 detik)',6,14);
}
function gambarSudut(id,data,iR,iT,judul){
  const cv=document.getElementById(id),g=cv.getContext('2d'),W=cv.width,H=cv.height;
  g.clearRect(0,0,W,H);
  const jendela=15,tn=data.length?data[data.length-1][0]:0;
  const vis=data.filter(d=>tn-d[0]<=jendela);
  const nilai=[];vis.forEach(d=>{[iR,iT].forEach(i=>{if(d[i]!==null)nilai.push(d[i]);});});
  g.fillStyle='#ccc';g.font='12px sans-serif';
  g.fillText(judul+'   (biru = real, merah putus-putus = target, 15 detik)',6,14);
  if(!nilai.length)return;
  let mn=Math.min(...nilai),mx=Math.max(...nilai);
  if(mx-mn<10){const m=(mx+mn)/2;mn=m-5;mx=m+5;}
  const pad=(mx-mn)*0.12;mn-=pad;mx+=pad;
  const Y=v=>H-(v-mn)/(mx-mn)*(H-24)-4;
  g.strokeStyle='#333';g.lineWidth=1;
  for(let k=0;k<=4;k++){const v=mn+(mx-mn)*k/4,y=Y(v);
    g.beginPath();g.moveTo(0,y);g.lineTo(W,y);g.stroke();
    g.fillStyle='#888';g.fillText(v.toFixed(0)+'\u00b0',W-34,y-2);}
  const garis=(idx,warna,putus)=>{
    g.strokeStyle=warna;g.lineWidth=2;g.setLineDash(putus);g.beginPath();let buka=false;
    vis.forEach(d=>{const v=d[idx];if(v===null){buka=false;return;}
      const x=W-(tn-d[0])/jendela*W,y=Y(v);
      if(!buka){g.moveTo(x,y);buka=true;}else g.lineTo(x,y);});
    g.stroke();g.setLineDash([]);g.lineWidth=1;};
  garis(iT,'#f55',[6,4]);
  garis(iR,'#4af',[]);
}
setInterval(()=>fetch('/api/grafik').then(r=>r.json()).then(data=>{
  if(!cfg.deadzone&&cfg.deadzone!==0)return;
  const sk=parseFloat(document.getElementById('skala').value),dz=cfg.deadzone;
  gambar('gp',data,1,'Error Pan (kiri-kanan)',dz,sk,'#4af');
  gambar('gt',data,2,'Error Tilt (atas-bawah)',dz,sk,'#fa4');
  gambarSudut('sa',data,3,5,'Sudut Pan (kiri-kanan)');
  gambarSudut('sb',data,4,6,'Sudut Tilt (atas-bawah)');
  const tn=data.length?data[data.length-1][0]:0;
  const selisih=(iR,iT)=>{const a=data.filter(d=>d[iR]!==null&&d[iT]!==null&&tn-d[0]<=2)
    .map(d=>Math.abs(d[iT]-d[iR]));return a.length?(a.reduce((x,y)=>x+y,0)/a.length):null;};
  const fd=v=>v===null?'-':v.toFixed(2)+'\u00b0';
  document.getElementById('ssa').textContent=
    'Selisih rata-rata |target - real| 2 detik terakhir  ->  Pan: '+fd(selisih(3,5))+'  |  Tilt: '+fd(selisih(4,6));
  const rata=idx=>{const a=data.filter(d=>d[idx]!==null&&tn-d[0]<=2).map(d=>Math.abs(d[idx]));
    return a.length?(a.reduce((x,y)=>x+y,0)/a.length):null;};
  const f=v=>v===null?'-':v.toFixed(3);
  document.getElementById('ss').textContent=
    'Error rata-rata |e| 2 detik terakhir  ->  Pan: '+f(rata(1))+'  |  Tilt: '+f(rata(2))+
    '   (makin dekat 0 = steady state makin baik)';
}),250);
</script></body></html>"""


@app.route("/")
def index():
    return render_template_string(HTML)


def generator_video():
    while True:
        with lock:
            jpg = status["jpeg"]
        if jpg is None:
            time.sleep(0.05)
            continue
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
        time.sleep(0.033)


@app.route("/video")
def video():
    return Response(generator_video(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/config", methods=["GET", "POST"])
def api_config():
    if request.method == "POST":
        data = request.get_json(force=True)
        jalur = str(data.get("path", "")).split(".")
        nilai = data.get("value")
        with lock:
            node = cfg
            for k in jalur[:-1]:
                node = node.get(k) if isinstance(node, dict) else None
                if node is None:
                    return jsonify(ok=False), 400
            if jalur[-1] not in node or isinstance(node[jalur[-1]], dict):
                return jsonify(ok=False), 400
            asli = node[jalur[-1]]
            node[jalur[-1]] = bool(nilai) if isinstance(asli, bool) else \
                (float(nilai) if isinstance(asli, float) else int(float(nilai)))
            simpan_config()
        return jsonify(ok=True)
    with lock:
        return jsonify(cfg)


@app.route("/api/center", methods=["POST"])
def api_center():
    with lock:
        cfg["pan"]["manual"] = 90
        cfg["tilt"]["manual"] = 90
        cfg["tracking"] = False
        simpan_config()
    return jsonify(ok=True)


@app.route("/api/status")
def api_status():
    with lock:
        return jsonify(pan=status["pan"], tilt=status["tilt"],
                       ditemukan=status["ditemukan"], fps=status["fps"], serial=esp.ok,
                       rekam=_status_rekam_locked())


@app.route("/api/rekam/mulai", methods=["POST"])
def api_rekam_mulai():
    try:
        nama = mulai_rekam()
        return jsonify(ok=True, nama=nama)
    except Exception as e:
        print("[ERROR] Gagal memulai rekaman:", e)
        return jsonify(ok=False, pesan=str(e)), 500


@app.route("/api/rekam/stop", methods=["POST"])
def api_rekam_stop():
    hasil = stop_rekam()
    if hasil is None:
        return jsonify(ok=True, nama=None, baris=0)
    return jsonify(ok=True, nama=hasil[0], baris=hasil[1])


@app.route("/api/grafik")
def api_grafik():
    with lock:
        data = list(riwayat)
    return jsonify(data)


def ip_lokal():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "IP-STB"


def _keluar_aman(*_):
    """Dipanggil saat SIGTERM: keluar normal supaya blok finally/atexit menutup CSV."""
    sys.exit(0)


if __name__ == "__main__":
    os.makedirs(DIR_HASIL, exist_ok=True)
    atexit.register(stop_rekam)                 # jaga-jaga: CSV selalu ditutup saat keluar
    signal.signal(signal.SIGTERM, _keluar_aman)
    muat_config()
    esp.buka()
    threading.Thread(target=loop_kontrol, daemon=True).start()
    print("=" * 50)
    print("[INFO] Webserver aktif. Buka di browser:")
    print("       http://%s:%d" % (ip_lokal(), WEB_PORT))
    print("=" * 50)
    try:
        app.run(host="0.0.0.0", port=WEB_PORT, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C diterima, menutup program...")
    finally:
        stop_rekam()                            # flush + close CSV dengan aman
