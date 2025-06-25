import cv2
from ultralytics import YOLO

model = YOLO('yolov5n.pt')  

camera = cv2.VideoCapture(0)

if not camera.isOpened():
    print("Error")
    exit()

print("Starting object detection... ")

while True:
    ret, frame = camera.read()
    if not ret:
        print("Failed ")
        break

    # Perform object detection
    results = model(frame)

    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green box


    cv2.imshow('Live Object Detection', frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
