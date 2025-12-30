# ⚡ IoT-Based 3-Phase Energy Monitoring and Billing System (ESP32)

This project is a **complete IoT-enabled energy monitoring system** built using the **ESP32**, **ACS712 current sensors**, and a **ZMPT101B voltage sensor**.  
It measures **voltage, three-phase current, total power, energy consumption (kWh)**, and **automatically calculates the electricity bill** based on a per-unit rate (10 Tk/unit).  

All data are displayed locally on a **20x4 LCD** and uploaded to the **ThingSpeak IoT platform** for remote monitoring and visualization.

---

## 🔋 Features
✅ Measures **Voltage**, **Current (3 Channels)**, and **Total Power**  
✅ Calculates **Energy (kWh)** and **Electricity Bill** in real time  
✅ Displays data on **20x4 I²C LCD**  
✅ **IoT Integration:** Uploads live data to **ThingSpeak Cloud**  
✅ Automatically stores energy data in ESP32 flash (Preferences Library)  
✅ Built with **low-cost sensors** (ACS712 + ZMPT101B)  
✅ Suitable for **3-phase monitoring**, small industries, and energy R&D  

---

## 🧠 System Overview

### 🧩 Hardware Components
| Component | Description |
|------------|--------------|
| **ESP32-WROOM-32** | Main microcontroller with Wi-Fi |
| **3× ACS712 (Current Sensor)** | Measures current of each phase |
| **ZMPT101B (Voltage Sensor)** | Measures AC voltage |
| **20x4 I²C LCD** | Displays readings locally |
| **Power Supply** | 5V regulated DC |
| **ThingSpeak Server** | IoT cloud dashboard for visualization |

---

## ⚙️ Circuit Diagram Overview
     [ZMPT101B] → A0 (Voltage)
     [ACS712-1] → A1 (Phase-1 Current)
     [ACS712-2] → A2 (Phase-2 Current)
     [ACS712-3] → A3 (Phase-3 Current)
     [LCD I²C]  → SDA (GPIO21), SCL (GPIO22)
     [ESP32]    → Wi-Fi → ThingSpeak Server

---

## 💻 Software Setup

### 🧩 Required Libraries
Install from Arduino Library Manager:
WiFi
HTTPClient
LiquidCrystal_I2C
Preferences

### ⚙️ Configure Wi-Fi & ThingSpeak
- Open the `.ino` file  
- Set your Wi-Fi SSID and Password  
- Set your **ThingSpeak Channel ID** and **Write API Key**

---

## 🧮 Calculations
| Parameter | Formula |
|------------|----------|
| **Power (W)** | Voltage × (I1 + I2 + I3) |
| **Energy (kWh)** | Power × Time / 3600000 |
| **Bill (Tk)** | Energy × 10 (per unit price) |

---

## 🌐 IoT Visualization (ThingSpeak)
Each data field is mapped as:
| Field | Parameter |
|--------|------------|
| Field1 | Voltage (V) |
| Field2 | Current Phase 1 (A) |
| Field3 | Current Phase 2 (A) |
| Field4 | Current Phase 3 (A) |
| Field5 | Total Power (W) |
| Field6 | Energy (kWh) |
| Field7 | Bill (Tk) |

Visit [ThingSpeak.com](https://thingspeak.com) → Dashboard → Charts for live monitoring.

---

## 🧾 Display Example (20x4 LCD)
V:230.5V P:690.2W
I1:1.23A I2:1.18A
I3:1.12A
E:0.025kWh B:0.25Tk

---

## 📊 Data Flow
Sensors → ESP32 → LCD
↓
ThingSpeak IoT

---

## 🛠️ Project Folder Structure
IoT-3Phase-Energy-Monitor-ESP32/
│
├── /code/
│ └── IoT_3Phase_Energy_Monitor.ino
├── /images/
│ └── circuit_diagram.png
├── README.md
└── LICENSE

---

## 🧠 Learning Outcomes
- Understanding AC current and voltage measurement with sensors  
- Interfacing multiple analog sensors with ESP32  
- LCD display interfacing and I²C communication  
- Using ThingSpeak IoT for cloud visualization  
- Calculating power, energy, and dynamic billing in embedded firmware  
- Real-world embedded IoT data logging system  

---

## 📸 Demo Preview
*(You can add your image links here later)*  
![LCD Output](images/lcd_display.jpg)  
![ThingSpeak Dashboard](images/thingspeak_chart.jpg)

---

## 🧰 Future Improvements
- Add **relay control** for load cut-off when bill exceeds limit  
- Integrate **Blynk / MQTT Dashboard** for mobile app display  
- Include **NTP time sync** for daily/duration-based energy tracking  

---

## 📜 License
This project is released under the **MIT License**.  
You are free to use, modify, and share with attribution.

---

## 👨‍💻 Developed By
**Rajib Hasan**  
Embedded Systems & IoT Developer  
World University of Bangladesh  
[LinkedIn](https://linkedin.com) • [GitHub](https://github.com/rajib-hasan)  

