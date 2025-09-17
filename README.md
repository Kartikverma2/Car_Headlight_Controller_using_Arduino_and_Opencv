
# Car Headlight Controller using Arduino and OpenCV
---

## Project Description
- An automatic car headlight control system using **Arduino** and **OpenCV**.  
The system uses YOLOv3-based object detection to identify vehicles in real-time from a video stream or webcam. Based on detection, it controls the car's headlights by communicating with Arduino using the `pyfirmata2` library. This ensures smart switching between high beam and low beam to improve driving safety.
- [Demonstration video](https://www.linkedin.com/posts/kartik-verma-2969a71ab_arduino-opencv-computervision-activity-7373708451782324224-Zl0z?utm_source=social_share_send&utm_medium=member_desktop_web&rcm=ACoAADEXNtIBY58IUKipPE6HyE3g9RW9TH1jyWc)
---

## Features
- Real-time vehicle detection using YOLOv3 model.
- Python-Arduino communication through pyfirmata2 for hardware control.
- Modular codebase with separate folders for models and Arduino sketches.
- Demo setup configured for webcam input or video files.
  
---

---

## Hardware Requirements
- Arduino board compatible with StandardFirmata (e.g., Arduino Leonardo)
- LEDs or Relay module to simulate high beam and low beam
- USB cable for Arduino-PC connection
- Optional external webcam for live video feed

---

## Software Requirements
- Python 3.x
- OpenCV (`opencv-python`)
- numpy
- pyfirmata2

---

## Installation and Setup

1. Clone this repository:
- git clone (https://github.com/Kartikverma2/Car_Headlight_Controller_using_Arduino_and_Opencv.git)


2. Install Python dependencies:
- *pip install -r requirements.txt*


3. Upload **StandardFirmata** to your Arduino:
- Open Arduino IDE
- Navigate to File > Examples > Firmata > StandardFirmata
- Select your board and port, then upload.

4. Ensure YOLO model files (`yolov3.weights`, `yolov3.cfg`, `coco.names`) are present in `models/`.

---

## Usage

-Run the main script: *python src/main.py*
- The system will launch video capture from your default camera.
- It detects cars in real-time, switching Arduino pins accordingly for high or low beam.
- Press `q` to quit the application.
---

## Contact

**Kartik Verma** – [rajputkartikverma2.com](mailto:rajputkartikverma2@gmail.com)  
GitHub: [https://github.com/Kartikverma2](https://github.com/Kartikverma2)

---

Thank you for your interest in this project!  
Feel free to reach out for collaborations or questions.

