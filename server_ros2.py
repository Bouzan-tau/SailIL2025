from flask import Flask, Response, render_template, request, jsonify
import threading
import subprocess
import pyautogui
import cv2
import time
import socket
import math

# TTS
from TTS.api import TTS
import simpleaudio as sa

app = Flask(__name__)

# Load the TTS model once
tts = TTS(model_name="tts_models/en/vctk/vits", progress_bar=False, gpu=False)

# Keep track of detected object history
object_history = []

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/update', methods=['POST'])
def update():
    data = request.json
    x = data.get("x", 0)
    y = data.get("y", 0)
    label = data.get("label", "None")

    # Validate input
    if not all(map(math.isfinite, [x, y])) or not label:
        print(f"❌ Invalid update received: x={x}, y={y}, label={label}")
        return jsonify(success=False)

    if label != "end":
        object_history.append({"x": x, "y": y, "label": label})
    return jsonify(success=True)

@app.route('/data')
def data():
    return jsonify(object_history)

@app.route('/say', methods=['POST'])
def say_text():
    data = request.json
    text = data.get("text", "")
    print(f"🔊 Speaking: {text}")
    threading.Thread(target=run_tts_runner, args=(text,)).start()
    return jsonify(success=True)

def run_tts_runner(text):
    try:
        safe_text = text.replace('"', '\\"')
        tts.tts_to_file(text=safe_text, file_path="output.wav", speaker="p360")
        wave_obj = sa.WaveObject.from_wave_file("output.wav")
        play_obj = wave_obj.play()
        play_obj.wait_done()
    except Exception as e:
        print(f"❌ Failed to run TTS: {e}")

def get_pyqt_window_box(title_keyword="Live Object Map"):
    try:
        win_id = subprocess.check_output(
            ["xdotool", "search", "--name", title_keyword]
        ).decode("utf-8").strip().split('\n')[0]

        win_info = subprocess.check_output(
            ["xdotool", "getwindowgeometry", "--shell", win_id]
        ).decode("utf-8").splitlines()

        geometry = {}
        for line in win_info:
            if '=' in line:
                key, val = line.split('=')
                geometry[key.strip()] = int(val.strip())

        left = geometry["X"]
        top = geometry["Y"]
        width = geometry["WIDTH"]
        height = geometry["HEIGHT"]

        return (left, top, width, height)

    except Exception as e:
        print("❌ PyQt window not found:", e)
        return None

def generate_frames():
    while True:
        try:
            with open("/tmp/live_gui.jpg", "rb") as f:
                frame_bytes = f.read()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        except Exception as e:
            print("❌ PyQt image not found", e)
        time.sleep(0.3)

def generate_frames_zed():
    while True:
        try:
            with open("/tmp/zed_view.jpg", "rb") as f:
                frame_bytes = f.read()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        except Exception as e:
            print("❌ ZED image not found:", e)
        time.sleep(0.3)

@app.route('/video')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/zed')
def zed_feed():
    return Response(generate_frames_zed(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/ping')
def ping():
    return jsonify(status="alive", server="ros2"), 200

def get_local_ip():
    """Return the actual local IP of the machine."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

if __name__ == '__main__':
    print("\033[96m🚀 SERVER IS STARTING...\033[0m")

    def announce_ready():
        ip = get_local_ip()
        print(f"\033[92m✅ SERVER IS READY ON http://{ip}:5000\033[0m")
        run_tts_runner(". Server is ready.")

    threading.Timer(1.0, announce_ready).start()
    app.run(host='0.0.0.0', port=5000)
