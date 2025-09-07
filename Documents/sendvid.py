import cv2
import numpy as np
from flask import Flask, Response
from pyngrok import ngrok

app = Flask(__name__)

# Define HSV range for RED 
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([179, 255, 255])

def generate_frames():
    cap = cv2.VideoCapture(0)  

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        contours, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 700: 
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    cap.release()

@app.route('/')
def index():
    return '<h2>Red Object Detection Stream</h2><a href="/video">Click here to watch</a>'

@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    public_url = ngrok.connect(5000)
    print(" * ngrok tunnel URL:", public_url)
    app.run(host="0.0.0.0", port=5000, threaded=True)
