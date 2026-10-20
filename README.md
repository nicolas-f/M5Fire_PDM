# M5Core Fire - Real-Time Audio Spectrum Analyzer

<img width="320" alt="1792508008715" src="https://github.com/user-attachments/assets/768a3872-6204-4289-a867-a4cc628baafd" />

A real-time audio spectrum analyzer for the **M5Core Fire** (ESP32) platform.
This program visualizes the audio spectrum via a spectrogram, measures the sound level in dBA (A-weighting), and identifies the dominant frequency.

---

## Description

This code leverages the M5Core Fire hardware to:
- Capture audio from an **external PDM microphone** (Port A, pins 21/22) or the **internal ADC microphone** (GPIO 34).
- Display a real-time spectrogram on the 320x240 LCD screen with a logarithmic frequency scale.
- Calculate and display the A-weighted sound level (dBA) and the dominant frequency.
- Allow manual calibration of microphone sensitivity.

The system automatically falls back to the internal ADC microphone if no PDM microphone is detected at startup.

---

## Required Hardware

- **M5Core Fire** (ESP32)
- **External PDM microphone** (optional, e.g., INMP441 or SPH0645)
  - Wiring: PDM_CLK → GPIO 22, PDM_DATA → GPIO 21

---

## Dependencies

The following libraries are required:
- **M5Unified** (for M5Stack hardware management)
- **arduinoFFT** (for FFT computation)
- **ESP32 Core** (built into Arduino IDE, for ESP32-specific functions: `i2s_pdm`, `adc_continuous`, `Preferences`)

---

## Installation

### 1. Install the M5Stack Board Manager
To use the **M5Core Fire** with the Arduino IDE, follow these steps:

1. **Open Arduino IDE Preferences**:
   - Go to `File` -> `Preferences`.

2. **Add the M5Stack Board Manager URL**:
   - Copy the following URL into the **Additional Board Manager URLs** field:
     ```
     https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json
     ```
   - For users in China, use the mirror URL:
     ```
     https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index_cn.json
     ```
   - Click **OK** to save.

3. **Install the M5Stack Board Manager**:
   - Go to `Tools` -> `Board` -> `Boards Manager`.
   - Search for **M5Stack** and click **Install**.

4. **Select the Development Board**:
   - After installation, go to `Tools` -> `Board` -> `M5Stack` -> **`M5Fire`**.

> **Note**: If the **M5Fire** option is not available, select a board with the same chip type (e.g., **M5Stack-Core-ESP32**).

---

### 2. Manual Installation (Optional)
If you encounter network issues, you can manually install the **M5Stack Board Manager**:

1. **Download the Board Manager Archive**:
   - Download the package from:
     - [Google Drive - M5Stack Board Manager](https://drive.google.com/drive/folders/1p-mJ30os1rCQcKgcM2imAXZkGNUZWEzY)
     - [Baidu Netdisk - M5Stack Board Manager](https://pan.baidu.com/s/1KspFcwiw-C2YSdvZeiKmDA?pwd=f7rd)

2. **Extract the Archive**:
   - Extract the downloaded file and place the folder named **`m5stack`** in the following directory:
     - **Windows**:
       ```
       %USERNAME%\AppData\Local\Arduino15\packages
       ```
     - **macOS**:
       ```
       /Users/<Username>/Library/Arduino15/packages
       ```
       (Replace `<Username>` with your actual username.)

3. **Restart the Arduino IDE**:
   - The **M5Stack** boards will now appear in the `Tools` -> `Board` menu.

---

### 3. Install Libraries
- In `Sketch` -> `Include Library` -> `Manage Libraries`, install:
  - **M5Unified** (by M5Stack)
  - **arduinoFFT** (by Kosme)

### 4. Upload the Code
- Copy the source code into a new Arduino sketch.
- Connect the **M5Core Fire** via USB-C.
- Select the serial port (`Tools` -> `Port`).
- Compile and upload.

---

## Operation

### Audio Capture Modes
| Mode               | Sampling Rate | Buffer Size | Frequency Range |
|--------------------|----------------|-------------|-----------------|
| PDM Microphone     | 16 kHz         | 512         | 250 Hz – 6.3 kHz |
| ADC Microphone     | 44.1 kHz       | 1024        | 500 Hz – 22 kHz  |

### Display
- **Spectrogram**:
  - Usable area: 280x198 pixels.
  - Logarithmic frequency scale (Y-axis).
  - Gradient color palette (green to red) representing amplitude.

- **Header**:
  - A-weighted sound level in dBA.
  - Dominant frequency in Hz.
  - Active mode (PDM or ADC) and current sensitivity.

### Controls
| Button | Action |
|--------|--------|
| A      | Decrease calibration target (`target_dBA - 1`). |
| B      | Increase calibration target (`target_dBA + 1`). |
| C      | Save current microphone sensitivity. |

> Calibration is saved to flash memory and reloaded on startup.

### Signal Processing
- **Hamming windowing** before FFT to reduce spectral leakage.
- **A-weighting** (IEC 61672 standard) to simulate human ear response.
- **Temporal filtering** (125 ms time constant) for sound level.
- **Peak detection** for dominant frequency (quadratic interpolation).

---
---
## Customization

### Adjustable Parameters
- Frequency range:
  ```cpp
  activeFMin = 500.0f;   // Minimum frequency (Hz)
  activeFMax = 22050.0f; // Maximum frequency (Hz)
  ```
- Microphone sensitivity:
  ```cpp
  sensitivity_pdm = -26.0f;  // PDM sensitivity (dBFS)
  sensitivity_int = -28.0f;  // ADC sensitivity (dBFS)
  ```
- Calibration target:
  ```cpp
  target_dBA = 94.0f;  // Reference sound level (dBA)
  ```

### Adding a New Microphone
1. Define the pins in the `HARDWARE DEFINITIONS` section.
2. Add a new mode to the `MicSource` enum.
3. Implement detection in the `setup()` function.

---
## Troubleshooting

| Issue | Solution |
|-------|----------|
| No sound detected | Check PDM microphone wiring (CLK/DATA). |
| Empty spectrogram | Increase microphone gain or verify calibration. |
| Compilation error | Install missing libraries (`M5Unified`, `arduinoFFT`). |
| Crash on startup | Ensure **M5Fire** or **M5Stack-Core-ESP32** board is selected. |
| Incorrect dominant frequency | Reduce ambient noise or adjust `activeFMin`/`activeFMax`. |

---
## License
This project is licensed under the **MIT License**.

---
## Acknowledgments
- M5Stack for the hardware and `M5Unified` library.
- Kosme for the `arduinoFFT` library.

---
