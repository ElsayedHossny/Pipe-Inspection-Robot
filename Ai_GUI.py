import tkinter as tk
from tkinter import font as tkfont
from PIL import Image, ImageTk
import serial
import threading
import time
import torch
import cv2
import numpy as np
from picamera2 import Picamera2
import os

class IndustrialRobotController:
    def _init_(self, root):
        self.root = root
        self.ser = None
        self.running = True
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.capture_count = 0
        self.frame_id = 0

        # Colors and fonts
        self.colors = {
            'bg': '#2D2D2D', 'panel': '#3A3A3A', 'text': '#FFFFFF',
            'primary': '#007ACC', 'secondary': '#6C757D', 'success': '#28A745',
            'danger': '#DC3545', 'warning': '#FFC107', 'active': '#0056B3'
        }

        # Setup
        self.setup_uart()
        self.setup_window()
        self.create_gui()
        self.bind_keys()
        self.setup_camera_and_model()

        # Start threads
        threading.Thread(target=self.capture_frames, daemon=True).start()
        threading.Thread(target=self.detect_objects, daemon=True).start()
        self.update_gui_frame()

    def setup_window(self):
        self.root.title("Industrial Robot Controller")
        #self.root.attributes('-fullscreen', True)
        self.root.configure(bg=self.colors['bg'])
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

    def setup_uart(self):
        try:
            self.ser = serial.Serial('/dev/serial0', 9600, timeout=1)
            print("UART connected successfully")
        except serial.SerialException:
            print("UART not available - simulation mode")

    def setup_camera_and_model(self):
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(main={"size": (640, 480)}))
        self.picam2.start()
        time.sleep(2)

        self.model = torch.hub.load('/home/pi/Ai_model_file/yolov5', 'custom',
                                    path='/home/pi/Ai_model_file/best.pt', source='local')
        self.model.conf = 0.25

        self.save_folder = "/home/pi/Desktop/Captured_Images"
        os.makedirs(self.save_folder, exist_ok=True)

    def capture_frames(self):
        last_time = time.time()
        while self.running:
            frame = self.picam2.capture_array()
            
            # Convert the frame to grayscale
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Store the grayscale frame
            with self.frame_lock:
                self.current_frame = gray_frame

            self.capture_count += 1

            if time.time() - last_time >= 1:
                print(f"?? Capturing FPS: {self.capture_count} frames/sec")
                self.capture_count = 0
                last_time = time.time()

    def detect_objects(self):
        while self.running:
            with self.frame_lock:
                if self.current_frame is None:
                    continue
                frame_copy = self.current_frame.copy()

            # Since YOLOv5 expects a 3-channel image, convert the grayscale image back to 3 channels
            frame_copy_rgb = cv2.cvtColor(frame_copy, cv2.COLOR_GRAY2RGB)

            # Run the YOLOv5 model on the grayscale (converted to RGB)
            results = self.model(frame_copy_rgb)
            detections = results.pandas().xyxy[0]

            if not detections.empty:
                print("? Object detected!")
                annotated = results.render()[0]
                filename = os.path.join(self.save_folder, f"detected_{self.frame_id}.jpg")
                cv2.imwrite(filename, annotated)
                print(f"?? Saved with boxes: {filename}")
            else:
                print("? No object detected.")

            self.frame_id += 1
            time.sleep(0.1)

    def update_gui_frame(self):
        with self.frame_lock:
            frame = self.current_frame.copy() if self.current_frame is not None else None

        if frame is not None:
            # Convert the grayscale frame to RGB for displaying in the Tkinter window
            rgb = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
            img = Image.fromarray(rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)

        self.root.after(30, self.update_gui_frame)

    def send_command(self, cmd):
        if self.ser:
            try:
                self.ser.write(cmd.encode('ascii'))
                print(f"UART Sent: {cmd}")
            except serial.SerialException as e:
                print(f"UART Error: {e}")
        else:
            print(f"Simulated send: {cmd}")

    def exit_and_send_u(self):
        self.send_command('U')
        self.cleanup()

    def on_key_press(self, event):
        key = event.keysym.lower()
        if key == 'u':
            self.exit_and_send_u()
            return

        if key in self.key_commands:
            if key in self.movement_keys:
                if not self.key_states[key]:
                    self.key_states[key] = True
                    self.send_command(self.key_commands[key])
            else:
                self.send_command(self.key_commands[key])

    def on_key_release(self, event):
        key = event.keysym.lower()
        if key in self.movement_keys and self.key_states[key]:
            self.key_states[key] = False
            self.send_command('Q')

    def create_gui(self):
        self.key_states = {'w': False, 'a': False, 's': False, 'd': False}
        self.key_commands = {'w': 'W', 's': 'S', 'a': 'A', 'd': 'D',
                             '0': '0', '1': '1', '2': '2', '3': '3',
                             '4': '4', '5': '5', '6': '6', '7': '7', '8': '8', '9': '9', 'u': 'U'}
        self.movement_keys = {'w', 'a', 's', 'd'}

        main_frame = tk.Frame(self.root, bg=self.colors['bg'], padx=20, pady=20)
        main_frame.grid(row=0, column=0, sticky="nsew")
        main_frame.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(1, weight=1)

        panel = tk.Frame(main_frame, bg=self.colors['panel'], padx=20, pady=20)
        panel.grid(row=0, column=0, sticky="ns")

        self.video_label = tk.Label(main_frame, bg='black')
        self.video_label.grid(row=0, column=1, sticky="nsew")

        title_font = tkfont.Font(family='Helvetica', size=18, weight='bold')
        button_font = tkfont.Font(family='Helvetica', size=14, weight='bold')
        speed_font = tkfont.Font(family='Helvetica', size=12, weight='bold')

        movement_frame = tk.Frame(panel, bg=self.colors['panel'])
        movement_frame.pack(pady=(0, 20))
        tk.Label(movement_frame, text="MOVEMENT CONTROL", font=title_font,
                 fg=self.colors['text'], bg=self.colors['panel']).pack(pady=(0, 10))

        self.make_button(movement_frame, "FORWARD", lambda: self.send_command('W')).pack()
        self.make_button(movement_frame, "LEFT", lambda: self.send_command('A')).pack()
        self.make_button(movement_frame, "STOP", lambda: self.send_command('Q'), self.colors['danger']).pack()
        self.make_button(movement_frame, "RIGHT", lambda: self.send_command('D')).pack()
        self.make_button(movement_frame, "BACKWARD", lambda: self.send_command('S')).pack()
        self.make_button(movement_frame, "EXIT", self.exit_and_send_u, self.colors['warning']).pack()

        speed_frame = tk.Frame(panel, bg=self.colors['panel'])
        speed_frame.pack()
        tk.Label(speed_frame, text="SPEED CONTROL", font=title_font,
                 fg=self.colors['text'], bg=self.colors['panel']).pack(pady=(0, 10))

        for percent, cmd in zip(range(10, 101, 10), list("1234567890")):
            self.make_button(speed_frame, f"{percent}%", lambda c=cmd: self.send_command(c), self.colors['success']).pack(pady=2)

    def make_button(self, parent, text, command, color=None):
        return tk.Button(parent, text=text, command=command,
                         bg=color or self.colors['primary'], fg=self.colors['text'],
                         font=('Helvetica', 12, 'bold'), width=15, height=2, bd=0,
                         activebackground=self.colors['active'])

    def darken_color(self, color, factor=0.8):
        rgb = tuple(int(color.lstrip('#')[i:i+2], 16) for i in (0, 2, 4))
        darkened = tuple(int(c * factor) for c in rgb)
        return '#%02x%02x%02x' % darkened

    def bind_keys(self):
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

    def cleanup(self):
        self.running = False
        self.picam2.stop()
        if self.ser:
            self.ser.close()

# Initialize the Tkinter window
root = tk.Tk()
robot_controller = IndustrialRobotController(root)
root.mainloop()