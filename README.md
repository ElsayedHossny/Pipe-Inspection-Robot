# 🚀 **Pipe Inspection Robot** 🚀  
### *Graduation Project Team: DJNZ*  
**A revolution in pipeline inspection technology!**

---

## **Team Members:**
- **Elsayed Hossny Elsayed**  
- **Hamdy Ayman Hamdy**  
- **Sara Srour Ahmed**  
- **Hazem Mohamed Hamdy**  
- **Bassem Osama Sobhy**

## ⚙️ Architectural Diagram of the Project

![Arch Project](https://raw.githubusercontent.com/ElsayedHossny/Pipe-Inspection-Robot/main/Arch_Project.jpg)
This diagram illustrates the overall design of the project and how the system is planned.

## 🎯 Project Overview
This project presents an intelligent pipe inspection robot that combines embedded control with AI capabilities. 
It uses an **ATmega32** microcontroller for real-time motor and sensor handling, 
while a **Raspberry Pi 4** runs a **Graphical User Interface (GUI)** for control and hosts an **AI model** to detect cracks in the pipeline via a mounted camera.

---

## 🚀 Project Highlights
- **Real-time robot movement control via UART**
- **GUI-based user control panel on Raspberry Pi**
- **Speed control with PWM**
- **AI-based pipeline crack detection**
- **Modular, layered codebase (MCAL, HAL, GUI, AI)**

---

## **Technologies Used:**
- **Embedded Systems:**  ATmega32, Raspberry Pi.
- **Sensors:** Camera Modules.
- **Software:** Python, OpenCV, ROS (Robot Operating System).
- **Communication:** Ethernet Communication.
- **Mechanical Design:** 3D Modeling and Simulation, CAD.

---

## 🧠 System Components

| Component          | Description |
|------------------- |-------------|
| **ATmega32**       | Controls motors |
| **Raspberry Pi 4** | Runs GUI and AI crack detection |
| **UART Serial**    | Communication between Pi & ATmega32 |
| **L293D**          | Motor driver IC |
| **Camera**         | Captures live video of pipe interior |
| **AI Model**       | Detects cracks using CNN |

---

## 🔄 System Workflow

1. **User interacts** with the GUI on Raspberry Pi (via buttons like Forward, Stop, Speed Level 3, etc.).
2. GUI sends single-character **commands via UART** to ATmega32.
3. **ATmega32 receives** the commands and controls motors accordingly via L293D.
5. If a crack is detected, it’s shown on GUI or logged for inspection.

---

## 🐍 Atmega32 Features
- Wire control via Uart
- Motor control using L293D driver
- Adjustable speed control based on numeric values
- Modular code structure (MCAL, HAL layers)

## 🐍 Raspberry Pi Features

- Built-in **GUI (Tkinter/PyQt)** for easy control
- Uses `pyserial` to communicate with ATmega via UART
- Hosts lightweight **AI inference** (TensorFlow Lite or PyTorch Mobile)
- Optional: can upload detection results to the cloud

---

## 🤖 Crack Detection AI

- **Input**: Real-time camera frames
- **Preprocessing**: Resize, grayscale, normalize
- **Model Type**: CNN-based crack detector
- **Output**: Binary (crack / no crack) or bounding boxes
- **Format**: `.tflite`, `.pt`, or `.onnx`
- **Runtime**: Optimized for Raspberry Pi 4

---
