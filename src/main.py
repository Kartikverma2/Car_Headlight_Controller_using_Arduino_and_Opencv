import cv2
import numpy as np
from pyfirmata2 import Arduino
import time

def initialize_arduino(port='COM9'):
    """
    Initialize Arduino board connection using pyfirmata2.
    port: Serial COM port where Arduino is connected (e.g., 'COM9' on Windows).
    Returns: Arduino board object.
    """
    board = Arduino(port)
    time.sleep(1)  # Wait a moment for connection to establish
    return board

def initialize_yolo(weights_path, config_path, names_path):
    """
    Load the YOLO object detection model.
    weights_path: path to .weights file
    config_path: path to .cfg config file
    names_path: path to .names file containing class names
    Returns: loaded YOLO net and list of class names.
    """
    net = cv2.dnn.readNet(weights_path, config_path)  # Load model
    with open(names_path, 'r') as f:
        classes = f.read().strip().split('\n')  # Read class labels
    return net, classes

def detect_cars(net, classes, frame, conf_threshold=0.5, nms_threshold=0.4):
    """
    Detect cars in the given video frame using YOLO model.
    net: YOLO net model
    classes: list of class labels
    frame: current video frame (numpy array)
    conf_threshold: minimum confidence to filter weak detections
    nms_threshold: threshold for Non-Maximum Suppression to reduce overlapping boxes
    Returns: 
        detected (bool): True if any car detected, else False
        boxes: detected bounding box coordinates
        confidences: confidence scores of detections
        class_ids: detected class indices
        indexes: filtered indices after NMS
    """
    height, width = frame.shape[:2]
    # Prepare input blob for the model
    blob = cv2.dnn.blobFromImage(frame, 1/255, (416,416), (0,0,0), swapRB=True, crop=False)
    net.setInput(blob)

    # Forward pass and get output layer names
    outputs = net.forward(net.getUnconnectedOutLayersNames())

    boxes, confidences, class_ids = [], [], []

    # Process all detections
    for output in outputs:
        for detection in output:
            scores = detection[5:]  # Class scores start at index 5
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            # Filter detections by confidence and class 'car'
            if confidence > conf_threshold and classes[class_id] == 'car':
                center_x = int(detection[0]*width)
                center_y = int(detection[1]*height)
                w = int(detection[2]*width)
                h = int(detection[3]*height)
                x = int(center_x - w//2)
                y = int(center_y - h//2)
                boxes.append([x,y,w,h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # Apply Non-Max Suppression to reduce overlapping boxes
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

    detected = len(indexes) > 0  # True if any boxes remain after NMS
    return detected, boxes, confidences, class_ids, indexes

def draw_boxes(frame, boxes, confidences, class_ids, indexes, classes):
    """
    Draw bounding boxes and labels on the frame for detected objects.
    frame: video frame image
    boxes: list of box coordinates
    confidences: list of detection confidences
    class_ids: list of detected class indices
    indexes: filtered indexes after Non-Maximum Suppression
    classes: list of class names
    """
    if len(indexes) > 0:
        for i in indexes.flatten():
            x,y,w,h = boxes[i]
            label = classes[class_ids[i]]
            conf = int(confidences[i]*100)  # Convert confidence to percentage
            # Draw rectangle around detected car
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0),2)
            # Put label text above the box
            cv2.putText(frame, f"{label} {conf}%", (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2)

def main():
    # Initialize Arduino board with correct COM port
    board = initialize_arduino('COM9')

    # Setup Arduino pins for high and low beam output controls
    high_beam = board.get_pin('d:6:o')  # Digital pin 6 as output for high beam
    low_beam = board.get_pin('d:8:o')   # Digital pin 8 as output for low beam

    # Load YOLO model files
    net, classes = initialize_yolo('yolov3.weights', 'yolov3.cfg', 'coco.names')

    # Open webcam (or change to file path to use a video file)
    cap = cv2.VideoCapture(0)  # 0 is default camera

    while True:
        ret, frame = cap.read()
        if not ret:
            break  # Exit on camera failure

        frame = cv2.resize(frame, (960,540))  # Resize frame for faster processing

        # Detect cars in the frame
        detected, boxes, confidences, class_ids, indexes = detect_cars(net, classes, frame)

        # Draw detection boxes on frame
        draw_boxes(frame, boxes, confidences, class_ids, indexes, classes)

        # Control Arduino pins based on detection result
        if detected:
            high_beam.write(1)  # Turn on high beam
            low_beam.write(0)   # Turn off low beam
        else:
            high_beam.write(0)  # Turn off high beam
            low_beam.write(1)   # Turn on low beam

        # Display the result frame
        cv2.imshow('YOLO Vehicle Detection with pyFirmata2', frame)

        # Break loop on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release camera and close windows
    cap.release()
    cv2.destroyAllWindows()
    # Close Arduino communication
    board.exit()

if __name__ == '__main__':
    main()
