# 🦾 AL5D "Key Grabber" - Wireless Teleoperation System
### UFMFV8-15-3 Group Design and Integration Project
**Student Submission: Crossfire-13**

---

## 📋 Project Context
This repository contains the firmware and documentation for a custom robotic system designed for the **"Key Fetcher"** application scenario. The project goal was to modify a stock **Lynxmotion AL5D** arm to autonomously retrieve car keys for a rental desk.

This submission demonstrates a complete overhaul of the robot's control electronics and software to meet the **UFMFV8-15-3** module requirements.

## ✅ Requirements Verification
This system has been engineered to strictly adhere to the constraints outlined in the 2025/26 Project Brief:

| Requirement | Status | Implementation Details |
| :--- | :--- | :--- |
| **50cm Working Envelope** | **ACHIEVED** | [cite_start]Mechanics modified to extend reach beyond the stock AL5D limits to 50cm[cite: 56]. |
| **Custom Electronics** | **ACHIEVED** | Stock **SSC-32u removed**. [cite_start]Replaced with custom **ESP8266 + PCA9685** architecture[cite: 81]. |
| **Safety Management** | **ACHIEVED** | [cite_start]Implemented "Pre-Flight" drift checks, auto-tare, and watchdog timers to manage collision risks[cite: 50]. |
| **Teleoperation** | **ACHIEVED** | [cite_start]Wireless, low-latency control via ESP-NOW for safe remote operation[cite: 54]. |
| **Teach Mode** | **ACHIEVED** | [cite_start]System can record manual waypoints and replay them autonomously for non-specialist users[cite: 55]. |

## 🏗️ System Architecture

The control system is distributed across two custom firmware modules:

### 1. The Hub (Mission Control)
* **Hardware:** ESP32-S3 DevKitC-1 + LCD + Joysticks.
* **Role:** Primary computing and user interface.
* **Physics Engine:** Calculates **Inverse Kinematics** to keep the gripper parallel to the ground and applies **Anti-Topple** logic (shifting center of gravity) to support the extended 50cm reach.

### 2. The Robot (Actuator)
* **Hardware:** NodeMCU (ESP8266) + **PCA9685 Servo Driver**.
* **Role:** Motor driver and safety limiter.
* **Upgrade:** The stock SSC-32u was removed. This unit now handles servo signals directly via the PCA9685, applying **Slew-Rate Limiting** to smooth out motion and reduce mechanical stress.



## 🔌 Hardware Bill of Materials (BOM)

| Component | Role | Notes |
| :--- | :--- | :--- |
| **ESP32-S3 DevKitC-1** | Controller | Handles Physics & Logic |
| **NodeMCU V3 (ESP8266)** | Receiver | Replaces stock microcontroller |
| **PCA9685** | PWM Driver | **Replaces stock SSC-32u** |
| **Lynxmotion AL5D** | Frame | Modified for 50cm reach |
| **1602 LCD (I2C)** | Feedback | Displays Drift & Status |
| **5V 10A PSU** | Power | Dedicated servo power |

## 🚀 Installation

### Prerequisites
* **VS Code** with **PlatformIO**.
* **Libraries:** `LiquidCrystal_I2C`, `Adafruit PWM Servo Driver`, `ESP-NOW`.

### Setup
1.  **Clone the Repo:**
    ```bash
    git clone [https://github.com/Crossfire-13/Key_Grabber_Project.git](https://github.com/Crossfire-13/Key_Grabber_Project.git)
    ```
2.  **Flash Firmware:**
    * `src/Hub_Controller_v7.3.cpp` -> **ESP32-S3**
    * `src/Robot_Actuator_v4.2.cpp` -> **NodeMCU**
3.  **Run:**
    * Power the Robot first, then the Hub.
    * **Safety Check:** Wait for the LCD to show `Drift < 200`.
    * Press **Btn 1** to engage motors.

## ⚠️ Risk & Safety
* **Operational Reach:** The robot now extends to **50cm**. Ensure the workspace is clear before engagement to avoid property damage.
* **Software Interlock:** Motors are mathematically locked until the "Drift Check" passes.

---
*Note: This project is a student submission for academic assessment.*
