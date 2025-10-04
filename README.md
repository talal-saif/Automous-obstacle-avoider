# 🚗 Autonomous Obstacle Avoider

An advanced **autonomous robot** that detects and avoids obstacles in real time.  
Built in **C++17** with **CMake**, supporting both **Simulation** and **Raspberry Pi hardware mode**.

---

## 📑 Table of Contents
1. [Features](#-features)  
2. [Project Structure](#-project-structure)  
3. [Requirements](#-requirements)  
4. [Build & Run](#-build--run)  
5. [Configuration](#-configuration)  
6. [Roadmap](#-roadmap)  
7. [License](#-license)

---

## ✨ Features
- 🖥️ **Simulation mode** for testing on PC.  
- 🛠️ **Raspberry Pi mode** for real hardware.  
- ⚡ Modular design (motors, ultrasonic sensors, navigation).  
- 🔧 Easy to extend with real GPIO/PWM libraries.  
- 📂 Configurable parameters via YAML.

---

## 📂 Project Structure

| Folder / File        | Description                                |
|-----------------------|--------------------------------------------|
| `include/`           | Header files (motors, ultrasonic, navigator) |
| `pi/`                | Raspberry Pi target source code            |
| `sim/`               | Simulation target source code              |
| `config/params.yaml` | Adjustable runtime parameters              |
| `scripts/`           | Helper build & run scripts                 |
| `CMakeLists.txt`     | Root CMake build file                      |

---

## ⚙️ Requirements
- C++17 or later  
- CMake ≥ 3.16  
- Linux (Ubuntu / Raspberry Pi OS recommended)  
- (Optional) pigpio for GPIO on Raspberry Pi  

---

## 🚀 Build & Run

```bash
# Clone
git clone https://github.com/talal-saif/autonomous-obstacle-avoider.git
cd autonomous-obstacle-avoider

# Build
cmake -B build -S .
cmake --build build -j

# Run Simulation
./build/sim/sim_main

# Run on Pi
./build/pi/pi_main
