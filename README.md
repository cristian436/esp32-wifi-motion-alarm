# ESP32 WiFi Motion Alarm

Motion detection **without any motion sensor**: the ESP32 measures how the WiFi signal strength (RSSI) between itself and the router fluctuates. A person moving in the room disturbs the signal, and the system sends an alert on **Telegram**.

## Features
- **Dual-core architecture (FreeRTOS):** RSSI sampling runs in a high-priority task pinned to core 1 with a fixed 20 Hz cadence (`vTaskDelayUntil`), while the blocking network/Telegram code runs on core 0.
- **Robust signal processing:**
  - sample validation and **Hampel filter** (median / MAD) to reject glitches and outliers;
  - automatic **calibration** of the room's real noise level at startup;
  - detection based on a **normalized score** (signal spread / measured noise), so the threshold adapts to the environment;
  - **adaptive baseline** (EMA) updated only when the room is quiet, to follow slow drift.
- **Telegram bot:** `/attiva`, `/disattiva`, `/stato` (live diagnostics); commands accepted only from the authorized chat.
- Automatic WiFi reconnection and recalibration.

## Hardware
- ESP32 dev board
- A WiFi access point (the home router)

## Setup
1. Install the libraries `UniversalTelegramBot` and `ArduinoJson` from the Arduino Library Manager.
2. Flash the sketch and stay still for ~10 s during calibration.

## Tuning
| Parameter | Meaning |
|---|---|
| `K_ENTER` | alarm threshold (higher = less sensitive) |
| `CONFERME_NECESSARIE` | consecutive samples above threshold needed to trigger |
| `EMA_ALPHA` | how fast the baseline adapts |

## Photos
<!-- Add a photo of the setup and a screenshot of the Telegram chat here -->
