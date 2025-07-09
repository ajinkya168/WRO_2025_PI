from flask import Flask, Response
from picamera2 import Picamera2
import time
import cv2

app = Flask(__name__)
picam2 = Picamera2()

# Configure camera resolution
video_config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(video_config)
picam2.start()

time.sleep(1)  # Give camera time to warm up

def generate_frames():
    while True:
        frame = picam2.capture_array()
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video')
def video():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "<h2>PiCamera2 Stream</h2><img src='/video' width='640' height='480'>"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
