# ESP32 FANET-OGN Ground Station

> ⚠️ **Project Status: Active Development**
> Core features implemented, ongoing work for full OGN integration.

---

## Current Features

* ✅ FANET protocol decoder (working)
* ✅ FANET data display via Serial Monitor
* ✅ Built-in HTTP server
* ✅ OTA (Over-The-Air) updates
* ✅ WiFi Access Point + Station mode

---

## Roadmap

* 🔄 OGN data push to server (coming soon)
* 🔄 OGN tracker data reception (coming soon)
* 🔄 Web app data visualisation (coming soon)
---

## Getting Started

### Prerequisites

* **ESP-IDF 5.3** or newer
* ESP32 development board
* USB cable for flashing

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/yourusername/esp32-fanet-groundstation.git
cd esp32-fanet-groundstation
```

### 2. Configure WiFi (optional, for Station mode)

```bash
idf.py menuconfig
```

Navigate to:

```
Component config → WiFi Configuration
```

Set:

* WiFi SSID
* WiFi Password

---

### 3. Build the project

```bash
idf.py build
```

---

### 4. Flash to ESP32

```bash
idf.py -p PORT flash monitor
```

Replace `PORT` with your device (e.g., `/dev/ttyUSB0` on Linux or `COM3` on Windows).
