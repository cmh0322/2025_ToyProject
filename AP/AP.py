import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter
from picamera2 import Picamera2
import time
import math
import serial

MODEL_PATH = "mobilenet_v1.tflite"
#MODEL_PATH = "/home/hsm/object_detection/efficientdet_lite1.tflite"
#MODEL_PATH = "/home/hsm/object_detection/saved_model.tflite"
LABEL_PATH = "coco_labels.txt"
MIN_CONFIDENCE = 0.5

ser = serial.Serial('/dev/serial0', 115200)
print(ser)

with open(LABEL_PATH, 'r') as f:
    labels = f.read().split('\n')
    LABELS = [label for label in labels if label.strip()]

interpreter = Interpreter(model_path = MODEL_PATH)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

picam2 = Picamera2()
config = picam2.create_preview_configuration(
        main = {"size": (640, 480), "format": "RGB888"}
        )

picam2.configure(config)
picam2.start()
time.sleep(2)

print("start")

frame_count = 0
fps_start = time.time()
fps = 0.0
tmp = 0
detected = 0

try:
    while True:
        detected = 0
        frame = picam2.capture_array()
        frame = cv2.flip(frame, 0)
        imH, imW, _ = frame.shape

        image_resized = cv2.resize(frame, (width, height))

        input_data = np.expand_dims(image_resized, axis = 0).astype(np.uint8)

        inf_start = time.time()
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        inf_time = (time.time() - inf_start) * 1000

        boxes = interpreter.get_tensor(output_details[0]['index'])[0]
        classes = interpreter.get_tensor(output_details[1]['index'])[0]
        scores = interpreter.get_tensor(output_details[2]['index'])[0]

        detections = 0

        max_idx = 0
        max_acc = 0.0
        length = 0.0

        for i in range(len(scores)):
            if scores[i] > MIN_CONFIDENCE:
                if scores[i] > max_acc and classes[i] == 0:
                    max_acc = scores[i]
                    max_idx = i
                detections += 1

        sx = 0
        sy = 0

        if max_acc != 0:
            detected = 1
            ymin = int(max(0, boxes[max_idx][0] * imH))
            xmin = int(max(0, boxes[max_idx][1] * imW))
            ymax = int(min(imH, boxes[max_idx][2] * imH))
            xmax = int(min(imW, boxes[max_idx][3] * imW))

            bbox_mid_y = (ymax + ymin) // 2
            bbox_mid_x = (xmax + xmin) // 2

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

            class_id = int(classes[max_idx])
            if class_id < len(LABELS):
                label = f'{LABELS[class_id]}: {int(scores[max_idx]*100)}%'
            else:
                label = f'ID{class_id} : {int(scores[max_idx]*100)}%'

            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (xmin, ymin - 20), (xmin + tw+5, ymin), (0, 255, 0), -1)
            cv2.putText(frame, label, (xmin+2, ymin-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

            cv2.circle(frame, (320, 240), 5, (255, 0, 0), -1, cv2.LINE_AA)
            cv2.circle(frame, (bbox_mid_x, bbox_mid_y), 5, (0, 255, 0), -1, cv2.LINE_AA)
            cv2.line(frame, (320, 240), (bbox_mid_x, bbox_mid_y), (255, 255, 255), 1, cv2.LINE_AA)

            length = math.sqrt((320 - bbox_mid_x)**2 + (240 - bbox_mid_y) ** 2)

            err_x = (bbox_mid_x - 320) / 320.0
            err_y = (bbox_mid_y - 240) / 240.0

            sx = int(np.clip(err_x * 70.0, -70, 70))
            sy = int(np.clip(err_y * 70.0, -70, 70))

        tmp += 1
        '''
        if tmp == 2:
            packet = bytes([
                0xAA,
                sx & 0xff,
                sy & 0xff
                ])
            ser.write(packet)
            tmp = 0
        '''
        #packet = bytes([0xAA, sx & 0xFF, sy & 0xFF])
        packet = bytes([detected & 0xFF, sx & 0xFF, sy & 0xFF])

        ser.write(packet)

        frame_count += 1
        if frame_count % 10 == 0:
            fps = 10 / (time.time() - fps_start)
            fps_start = time.time()

        info = f'FPS: {fps:.1f} | Inf: {inf_time:.0f}ms | Obj: {detections} | length: {length:.1f}'
        cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('mobilenet', frame)

        if cv2.waitKey(1) == ord('q'):
            break

except KeyboardInterrupt:
    print('end')
finally:
    picam2.stop()
    cv2.destroyAllWindows()
    total_time = time.time() - fps_start + 0.001
    print(f'average fps: {frame_count / total_time:.2f}')