#!/usr/bin/env python3
# tes_servo_manual.py - Program TES servo MANUAL di mini PC (STB)
# Tanpa PID, tanpa deteksi. Hanya: lihat kamera + gerakkan 2 servo 0-180 derajat
# dari webserver, untuk mengecek apakah servo rusak atau tidak.
#
# Pakai ESP32 yang sama (esp32_servo_uart.ino / esp32_main.py), tidak perlu diubah.
# PENTING: matikan dulu tracker_stb.py, karena port serial hanya bisa dipakai satu program.
#
# Jalankan:  python3 tes_servo_manual.py
# Buka di browser:  http://<ip-stb>:5000

import socket
import threading
import time

import cv2
import serial
from flask import Flask, Response, jsonify, render_template_string, request

KAMERA = "/dev/video1"
SERIAL_PORT = "/dev/ttyUSB0"   # ganti sesuai UART TTL di STB
SERIAL_BAUD = 115200
WEB_PORT = 5000
KECEPATAN_SAPU = 45.0          # derajat per detik untuk tombol "Sapu"

lock = threading.Lock()
state = {"pan": 90.0, "tilt": 90.0, "sapu": None, "arah": 1, "jpeg": None,
         "serial_ok": False}


# ===================== SERIAL + PENGIRIM =====================
def loop_pengirim():
    """Kirim sudut ke ESP32 tiap 50 ms (sekaligus menjalankan mode sapu)."""
    ser = None
    t_lalu = time.time()
    t_coba = 0.0
    while True:
        sekarang = time.time()
        dt = sekarang - t_lalu
        t_lalu = sekarang

        # mode sapu: 0 -> 180 -> 0 untuk satu sumbu
        with lock:
            if state["sapu"] in ("pan", "tilt"):
                sumbu = state["sapu"]
                state[sumbu] += state["arah"] * KECEPATAN_SAPU * dt
                if state[sumbu] >= 180:
                    state[sumbu], state["arah"] = 180.0, -1
                elif state[sumbu] <= 0:
                    state[sumbu], state["arah"] = 0.0, 1
            pan, tilt = state["pan"], state["tilt"]

        if ser is None and sekarang - t_coba > 2.0:
            t_coba = sekarang
            try:
                ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
                print("[INFO] Serial terhubung:", SERIAL_PORT)
            except Exception as e:
                print("[PERINGATAN] Serial belum terhubung (%s): %s" % (SERIAL_PORT, e))

        if ser is not None:
            try:
                ser.write(("S,%.1f,%.1f\n" % (pan, tilt)).encode())
            except Exception as e:
                print("[PERINGATAN] Gagal kirim serial:", e)
                ser = None
        with lock:
            state["serial_ok"] = ser is not None
        time.sleep(0.05)


# ===================== KAMERA =====================
def loop_kamera():
    cap = cv2.VideoCapture(KAMERA, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print("[ERROR] Kamera %s tidak bisa dibuka (servo tetap bisa dites)" % KAMERA)
        return
    print("[INFO] Kamera %s terbuka" % KAMERA)
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue
        h, w = frame.shape[:2]
        cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
        cv2.line(frame, (0, h // 2), (w, h // 2), (255, 255, 0), 1)
        with lock:
            teks = "Pan %.0f  Tilt %.0f" % (state["pan"], state["tilt"])
        cv2.putText(frame, teks, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with lock:
            state["jpeg"] = jpg.tobytes()


# ===================== WEBSERVER =====================
app = Flask(__name__)

HTML = """<!doctype html>
<html lang="id"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Tes Servo Manual</title>
<style>
 body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:10px}
 .wrap{display:flex;flex-wrap:wrap;gap:14px}
 img{max-width:100%;border:2px solid #444;border-radius:6px}
 .panel{background:#1c1c1c;border-radius:8px;padding:10px 14px;min-width:300px;margin-bottom:10px}
 h3{margin:6px 0;color:#7cf}
 input[type=range]{width:100%}
 .nilai{font-size:28px;font-variant-numeric:tabular-nums}
 button{padding:8px 12px;margin:3px;border:0;border-radius:5px;background:#27a;color:#fff;cursor:pointer}
 button.merah{background:#a33}
 #st{font-size:13px;color:#9f9;margin-top:4px}
</style></head><body>
<h2>Tes Servo Manual (0-180)</h2>
<div class="wrap">
 <div><img src="/video"><div id="st">...</div></div>
 <div>
  <div class="panel"><h3>Pan - kiri/kanan (GPIO18)</h3>
   <div class="nilai"><span id="vp">90</span>&deg;</div>
   <input id="sp" type="range" min="0" max="180" step="1" value="90">
   <button onclick="atur('pan',0)">0</button><button onclick="atur('pan',90)">90</button>
   <button onclick="atur('pan',180)">180</button>
   <button onclick="sapu('pan')">Sapu 0-180</button></div>
  <div class="panel"><h3>Tilt - atas/bawah (GPIO19)</h3>
   <div class="nilai"><span id="vt">90</span>&deg;</div>
   <input id="stt" type="range" min="0" max="180" step="1" value="90">
   <button onclick="atur('tilt',0)">0</button><button onclick="atur('tilt',90)">90</button>
   <button onclick="atur('tilt',180)">180</button>
   <button onclick="sapu('tilt')">Sapu 0-180</button></div>
  <div class="panel">
   <button onclick="tengah()">Kedua servo ke 90</button>
   <button class="merah" onclick="sapu(null)">Berhenti sapu</button></div>
 </div>
</div>
<script>
let tunda=null, sedangGeser=false;
function post(url,body){return fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});}
function atur(sumbu,nilai){
  const b={};b[sumbu]=nilai;
  post('/api/set',b);
  tampil(sumbu,nilai);
}
function tampil(sumbu,nilai){
  const id=sumbu==='pan'?['vp','sp']:['vt','stt'];
  document.getElementById(id[0]).textContent=Math.round(nilai);
  document.getElementById(id[1]).value=nilai;
}
function sapu(sumbu){post('/api/sapu',{sumbu:sumbu});}
function tengah(){post('/api/set',{pan:90,tilt:90});sapu(null);}
[['sp','pan'],['stt','tilt']].forEach(([id,sumbu])=>{
  const el=document.getElementById(id);
  el.addEventListener('pointerdown',()=>sedangGeser=true);
  el.addEventListener('pointerup',()=>sedangGeser=false);
  el.addEventListener('input',()=>{
    clearTimeout(tunda);
    tunda=setTimeout(()=>atur(sumbu,parseFloat(el.value)),30);
    document.getElementById(sumbu==='pan'?'vp':'vt').textContent=el.value;
  });
});
setInterval(()=>fetch('/api/status').then(r=>r.json()).then(s=>{
  if(!sedangGeser){tampil('pan',s.pan);tampil('tilt',s.tilt);}
  document.getElementById('st').textContent=
    'Serial ESP32: '+(s.serial_ok?'OK':'TERPUTUS')+'  |  Sapu: '+(s.sapu||'mati');
}),300);
</script></body></html>"""


@app.route("/")
def index():
    return render_template_string(HTML)


def generator_video():
    while True:
        with lock:
            jpg = state["jpeg"]
        if jpg is None:
            time.sleep(0.05)
            continue
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
        time.sleep(0.04)


@app.route("/video")
def video():
    return Response(generator_video(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/set", methods=["POST"])
def api_set():
    data = request.get_json(force=True)
    with lock:
        for k in ("pan", "tilt"):
            if k in data:
                state[k] = max(0.0, min(180.0, float(data[k])))
                if state["sapu"] == k:      # kalau digeser manual, sapu sumbu itu berhenti
                    state["sapu"] = None
    return jsonify(ok=True)


@app.route("/api/sapu", methods=["POST"])
def api_sapu():
    sumbu = request.get_json(force=True).get("sumbu")
    with lock:
        state["sapu"] = sumbu if sumbu in ("pan", "tilt") else None
        state["arah"] = 1
    return jsonify(ok=True)


@app.route("/api/status")
def api_status():
    with lock:
        return jsonify(pan=state["pan"], tilt=state["tilt"], sapu=state["sapu"],
                       serial_ok=state["serial_ok"])


def ip_lokal():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "IP-STB"


if __name__ == "__main__":
    threading.Thread(target=loop_pengirim, daemon=True).start()
    threading.Thread(target=loop_kamera, daemon=True).start()
    print("=" * 50)
    print("[INFO] Tes servo manual aktif. Buka di browser:")
    print("       http://%s:%d" % (ip_lokal(), WEB_PORT))
    print("=" * 50)
    app.run(host="0.0.0.0", port=WEB_PORT, threaded=True, use_reloader=False)
