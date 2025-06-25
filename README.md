# 🤖Vision-based Obstacle-Avoiding Robot with Raspberry Pi 

This project demonstrates how to build a smart **obstacle-avoiding robot** using a **Raspberry Pi**, **OpenCV**, a **motor driver (L298N)**, and a **0.91-inch OLED screen**. The robot uses a webcam or Pi camera to detect obstacles in real-time and reacts accordingly while displaying its current status on the OLED display.

---

## 🧰 Materials Required

- 🧠 **Raspberry Pi** (e.g., Pi 4 or Pi 3 with GPIO & camera support)
- 📺 **0.91" OLED Display** (128x32, I2C)
- ⚙️ **L298N Motor Driver**
- 🔄 **2x DC Motors** with wheels
- 📷 **Camera** (Raspberry Pi Camera Module or USB Webcam)
- 🧱 **Robot Chassis**
- 🔌 **Power Supply** (for motors & Pi)
- 🔗 **Jumper Wires**, **Breadboard**
- ➕ (Optional) **Ultrasonic Sensor**

---

## 🔌 Hardware Connections

### 🔧 Motor Wiring

| Motor | L298N Pins |
|-------|------------|
| Left  | `Out1`, `Out2` |
| Right | `Out3`, `Out4` |

### ⚡ L298N to Pi GPIO

| L298N Pin | GPIO Pin |
|-----------|----------|
| IN1       | GPIO17   |
| IN2       | GPIO18   |
| IN3       | GPIO22   |
| IN4       | GPIO23   |
| ENA / ENB | 5V (Enable) |

### 📺 OLED Display (I2C)

| OLED Pin | Pi Pin     |
|----------|------------|
| VCC      | 5V         |
| GND      | GND        |
| SCL      | GPIO3 (SCL)|
| SDA      | GPIO2 (SDA)|

### 📷 Camera

- Pi Camera → **CSI Port**
- USB Webcam → **USB Port**

---

## 🛠️ Software Setup

### 1. Install Dependencies

```bash
sudo apt-get update && sudo apt-get upgrade
sudo apt-get install python3-opencv python3-pip
pip3 install RPi.GPIO numpy imutils
pip3 install adafruit-circuitpython-ssd1306
sudo apt-get install python3-pil libopencv-dev

