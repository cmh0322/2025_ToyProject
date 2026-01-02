import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter
from picamera2 import Picamera2
import time
import math
import serial

MODEL_PATH = "drone_v3_300.tflite"
LABEL_PATH = "drone_labels.txt"
MIN_CONFIDENCE = 0.6

ser = serial.Serial('/dev/serial0', 115200)

with open(LABEL_PATH, 'r') as f:
    labels = f.read().split('\n')
    LABELS = [label for label in labels if label.strip()]

interpreter = Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)
picam2.start()
time.sleep(2)

print("Start Object Detection")

frame_count = 0
fps_start = time.time()
fps = 0.0
detected = 0

try:
    while True:
        detected = 0
        frame = picam2.capture_array()
        frame = cv2.flip(frame, 0)
        imH, imW, _ = frame.shape

        image_resized = cv2.resize(frame, (width, height))

        input_data = np.expand_dims(image_resized, axis=0)
        input_data = (np.float32(input_data) - 127.5) / 127.5

        inf_start = time.time()
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        inf_time = (time.time() - inf_start) * 1000


        scores = interpreter.get_tensor(output_details[0]['index'])[0] 
        boxes = interpreter.get_tensor(output_details[1]['index'])[0]
        classes = interpreter.get_tensor(output_details[3]['index'])[0]
        

        detections = 0
        max_idx = 0
        max_acc = 0.0
        length = 0.0

        for i in range(len(scores)):
            if scores[i] > MIN_CONFIDENCE:
                if scores[i] > max_acc: 
                    max_acc = scores[i]
                    max_idx = i
                detections += 1

        sx = 0
        sy = 0
        bbox_size_pack = 0
        
        if max_acc > MIN_CONFIDENCE:
            detected = 1
            ymin = int(max(0, boxes[max_idx][0] * imH))
            xmin = int(max(0, boxes[max_idx][1] * imW))
            ymax = int(min(imH, boxes[max_idx][2] * imH))
            xmax = int(min(imW, boxes[max_idx][3] * imW))
            
            bbox_mid_y = (ymax + ymin) // 2
            bbox_mid_x = (xmax + xmin) // 2
            
            bbox_width = xmax - xmin
            bbox_height = ymax - ymin
            print(bbox_width, bbox_height)
            bbox_rect_size = bbox_width * bbox_height
            if bbox_rect_size < 96 * 96:
                bbox_size_pack = 1
            elif bbox_rect_size >= 96 * 96 and bbox_rect_size < 168 * 168:
                bbox_size_pack = 2
            elif bbox_rect_size >= 168 * 168  and bbox_rect_size < 196 * 196:
                bbox_size_pack = 3
            elif bbox_rect_size >= 196 * 196  and bbox_rect_size < 256 * 256:
                bbox_size_pack = 4
            else:
                bbox_size_pack = 5

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

            class_id = int(classes[max_idx])
            if class_id < len(LABELS):
                label = f'{LABELS[class_id]}: {int(scores[max_idx]*100)}%'
            else:
                label = f'ID{class_id} : {int(scores[max_idx]*100)}%'

            cv2.putText(frame, label, (xmin, ymin-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            err_x = (bbox_mid_x - 320) / 320.0
            err_y = (bbox_mid_y - 240) / 240.0
            sx = int(np.clip(err_x * 50.0, -50, 50))
            sy = int(np.clip(err_y * 30.0, -30, 30))
            print(sx, sy)
        packet = bytes([bbox_size_pack & 0xFF, sx & 0xFF, sy & 0xFF])
        
        #packet = bytes([0 & 0xFF, sx & 0xFF, sy & 0xFF])
        ser.write(packet)

        frame_count += 1
        if frame_count % 10 == 0:
            fps = 10 / (time.time() - fps_start)
            fps_start = time.time()

        info = f'FPS: {fps:.1f} | Inf: {inf_time:.0f}ms'
        cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('Drone Cam', frame)

        if cv2.waitKey(1) == ord('q'):
            break

except KeyboardInterrupt:
    print('Stop')
finally:
    picam2.stop()
    cv2.destroyAllWindows()
    ser.close()