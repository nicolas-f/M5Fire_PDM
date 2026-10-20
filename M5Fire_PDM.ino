#include <M5Unified.h>
#include <arduinoFFT.h>
#include <esp_adc/adc_continuous.h>
#include <Preferences.h>
#include <math.h>

// ============================================================================
// CONFIGURATION: 44.1 kHz Continuous ADC (Nyquist = 22.05 kHz)
// ============================================================================
#define SAMPLINGFREQUENCY     44100

// Internal analog microphone pin on M5Stack Fire
#define INTERNAL_MIC_PIN      34     // ADC1 Channel 6
#define INTERNAL_MIC_CHANNEL  ADC_CHANNEL_6

#define SIGNAL_LENGTH         1024

// Frequency display limits (Logarithmic 500 Hz to Nyquist 22.05 kHz)
#define F_MIN                 500.0f
#define F_MAX                 (SAMPLINGFREQUENCY / 2.0f) // 22050.0 Hz

// Display Dimensions
#define SCREEN_WIDTH          320
#define SCREEN_HEIGHT         240
#define HEADER_HEIGHT         42

#define AXIS_WIDTH            40
#define SPECTROGRAM_WIDTH     (SCREEN_WIDTH - AXIS_WIDTH)     // 280 pixels
#define SPECTROGRAM_HEIGHT    (SCREEN_HEIGHT - HEADER_HEIGHT) // 198 pixels

// Audio & FFT Buffers
int16_t adcRawSamples[SIGNAL_LENGTH];
double adcBuffer[SIGNAL_LENGTH];
double vImag[SIGNAL_LENGTH];

ArduinoFFT<double> FFT = ArduinoFFT<double>(adcBuffer, vImag, SIGNAL_LENGTH, SAMPLINGFREQUENCY);
static adc_continuous_handle_t mic_adc_handle = NULL;

M5Canvas spectrogram(&M5.Display);
uint16_t colorLUT[256];

// Precomputed tables
struct YMap {
    int b0;
    int b1;
    float frac;
};
YMap yMap[SPECTROGRAM_HEIGHT];
float aWeightPowerTable[SIGNAL_LENGTH / 2];

// ISO 266 1/3-Octave table up to Nyquist
struct ThirdOctave {
    float freq;
    const char* label;
    bool isFullOctave;
};

const ThirdOctave allThirdOctaves[] = {
    { 500.0f,   "500",   true  }, // Hidden bottom mark
    { 630.0f,   "630",   false },
    { 800.0f,   "800",   false },
    { 1000.0f,  "1k",    true  }, // Visible full octave
    { 1250.0f,  "1.2k",  false },
    { 1600.0f,  "1.6k",  false },
    { 2000.0f,  "2k",    true  },
    { 2500.0f,  "2.5k",  false },
    { 3150.0f,  "3.1k",  false },
    { 4000.0f,  "4k",    true  },
    { 5000.0f,  "5k",    false },
    { 6300.0f,  "6.3k",  false },
    { 8000.0f,  "8k",    true  },
    { 10000.0f, "10k",   false },
    { 12500.0f, "12.5k", false },
    { 16000.0f, "16k",   true  },
    { 20000.0f, "20k",   false },
    { 22050.0f, "22k",   true  }  // Hidden top mark
};
const int TOTAL_OCTAVES = sizeof(allThirdOctaves) / sizeof(allThirdOctaves[0]);

// ============================================================================
// CALIBRATION & LEVEL STATE
// ============================================================================
float target_dBA     = 94.0f;  // Expected calibration sound level (e.g. 94.0 dBA calibrator)
float sensitivity_94 = -28.0f; // Measured dBFS at 94 dB SPL
uint32_t calFlashTimer = 0;

double fastPowerFiltered = 0.0;
double currentLaf_dBA    = 0.0;

// NoiseCapture Color Palette (17 anchor colors)
struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

const RGB noiseCapturePalette[17] = {
    { 0x30, 0x30, 0x30 }, { 0x2D, 0x3C, 0x2D }, { 0x2A, 0x48, 0x2A },
    { 0x27, 0x54, 0x27 }, { 0x24, 0x60, 0x24 }, { 0x21, 0x6C, 0x21 },
    { 0x3F, 0x8E, 0x19 }, { 0x61, 0xA5, 0x14 }, { 0x82, 0xBB, 0x0F },
    { 0xA4, 0xD2, 0x0A }, { 0xC5, 0xE8, 0x05 }, { 0xE7, 0xFF, 0x00 },
    { 0xEB, 0xD4, 0x00 }, { 0xEF, 0xAA, 0x00 }, { 0xF3, 0x7F, 0x00 },
    { 0xF7, 0x55, 0x00 }, { 0xFB, 0x2A, 0x00 }
};

float computeAWeightPower(float freq) {
    if (freq < 10.0f) return 0.0f;
    float f2 = freq * freq;
    float f4 = f2 * f2;

    float num = 148698944.0f * f4; // 12194^2 * f^4
    float den = (f2 + 424.36f) * sqrtf((f2 + 11599.29f) * (f2 + 544496.41f)) * (f2 + 148698944.0f);
    if (den <= 0.0f) return 0.0f;

    float ra = num / den;
    float wa = ra * 1.25892541f; // 10^(2.00/20) normalization (0 dB @ 1 kHz)
    return wa * wa;
}

void initColorMap() {
    const int numColors = 17;
    for (int i = 0; i < 256; i++) {
        float pos = (i / 255.0f) * (numColors - 1);
        int idx = (int)pos;
        float frac = pos - idx;

        if (idx >= numColors - 1) {
            colorLUT[i] = M5.Display.color565(noiseCapturePalette[numColors - 1].r,
                                              noiseCapturePalette[numColors - 1].g,
                                              noiseCapturePalette[numColors - 1].b);
        } else {
            uint8_t r = (uint8_t)((1.0f - frac) * noiseCapturePalette[idx].r + frac * noiseCapturePalette[idx + 1].r);
            uint8_t g = (uint8_t)((1.0f - frac) * noiseCapturePalette[idx].g + frac * noiseCapturePalette[idx + 1].g);
            uint8_t b = (uint8_t)((1.0f - frac) * noiseCapturePalette[idx].b + frac * noiseCapturePalette[idx + 1].b);
            colorLUT[i] = M5.Display.color565(r, g, b);
        }
    }
}

void initLogFrequencyMap() {
    for (int y = 0; y < SPECTROGRAM_HEIGHT; y++) {
        float norm_y = (float)(SPECTROGRAM_HEIGHT - 1 - y) / (float)(SPECTROGRAM_HEIGHT - 1);
        float freq = F_MIN * powf(F_MAX / F_MIN, norm_y);

        float binFloat = (freq / (float)SAMPLINGFREQUENCY) * SIGNAL_LENGTH;
        binFloat = constrain(binFloat, 1.0f, (float)(SIGNAL_LENGTH / 2 - 2));

        yMap[y].b0 = (int)binFloat;
        yMap[y].b1 = yMap[y].b0 + 1;
        yMap[y].frac = binFloat - yMap[y].b0;
    }

    for (int k = 0; k < SIGNAL_LENGTH / 2; k++) {
        float freq = k * (float)SAMPLINGFREQUENCY / (float)SIGNAL_LENGTH;
        aWeightPowerTable[k] = computeAWeightPower(freq);
    }
}

int getScreenY(float freq) {
    if (freq < F_MIN) freq = F_MIN;
    if (freq > F_MAX) freq = F_MAX;
    float norm_y = logf(freq / F_MIN) / logf(F_MAX / F_MIN);
    int y = (SPECTROGRAM_HEIGHT - 1) - (int)roundf(norm_y * (SPECTROGRAM_HEIGHT - 1));
    return HEADER_HEIGHT + y;
}

void drawFrequencyAxis() {
    M5.Display.fillRect(SPECTROGRAM_WIDTH, HEADER_HEIGHT, AXIS_WIDTH, SPECTROGRAM_HEIGHT, colorLUT[0]);
    M5.Display.drawFastVLine(SPECTROGRAM_WIDTH, HEADER_HEIGHT, SPECTROGRAM_HEIGHT, 0x4208);
    M5.Display.setTextSize(1);

    for (int i = 1; i < TOTAL_OCTAVES - 1; i++) {
        int y_pos = getScreenY(allThirdOctaves[i].freq);

        if (allThirdOctaves[i].isFullOctave) {
            M5.Display.drawFastHLine(SPECTROGRAM_WIDTH, y_pos, 7, TFT_WHITE);
            M5.Display.setTextColor(TFT_WHITE, colorLUT[0]);
            M5.Display.setCursor(SPECTROGRAM_WIDTH + 9, y_pos - 3);
            M5.Display.print(allThirdOctaves[i].label);
        } else {
            M5.Display.drawFastHLine(SPECTROGRAM_WIDTH, y_pos, 4, TFT_WHITE);
            M5.Display.setTextColor(0xC618, colorLUT[0]);
            M5.Display.setCursor(SPECTROGRAM_WIDTH + 7, y_pos - 3);
            M5.Display.print(allThirdOctaves[i].label);
        }
    }
}

bool init_internal_adc_mic() {
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4096,
        .conv_frame_size = SIGNAL_LENGTH * 2,
    };
    esp_err_t err = adc_continuous_new_handle(&adc_config, &mic_adc_handle);
    if (err != ESP_OK) return false;

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLINGFREQUENCY,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = INTERNAL_MIC_CHANNEL, // ADC1_CH6 (GPIO 34)
        .unit = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };

    dig_cfg.pattern_num = 1;
    dig_cfg.adc_pattern = &adc_pattern;

    err = adc_continuous_config(mic_adc_handle, &dig_cfg);
    if (err != ESP_OK) return false;

    err = adc_continuous_start(mic_adc_handle);
    return (err == ESP_OK);
}

double getCleanPeakFrequency() {
    double maxY = 0;
    int maxBin = 0;
    int minBin = (int)((F_MIN * 0.95f / (float)SAMPLINGFREQUENCY) * SIGNAL_LENGTH);
    int maxBinLimit = (int)((F_MAX * 0.99f / (double)SAMPLINGFREQUENCY) * SIGNAL_LENGTH);

    for (int i = minBin; i <= maxBinLimit; i++) {
        if (adcBuffer[i] > maxY) {
            maxY = adcBuffer[i];
            maxBin = i;
        }
    }

    if (maxBin <= minBin || maxY < 5.0) return 0.0;

    double y1 = adcBuffer[maxBin - 1];
    double y2 = adcBuffer[maxBin];
    double y3 = adcBuffer[maxBin + 1];
    double delta = 0.5 * (y1 - y3) / (y1 - 2.0 * y2 + y3);

    return ((double)maxBin - delta) * ((double)SAMPLINGFREQUENCY / SIGNAL_LENGTH);
}

void loadCalibration() {
    Preferences prefs;
    prefs.begin("sound_cal", true);
    sensitivity_94 = prefs.getFloat("sens94", -28.0f);
    target_dBA     = prefs.getFloat("target", 94.0f);
    prefs.end();
}

void saveCalibration() {
    Preferences prefs;
    prefs.begin("sound_cal", false);
    prefs.putFloat("sens94", sensitivity_94);
    prefs.putFloat("target", target_dBA);
    prefs.end();
}

void setup() {
    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    cfg.internal_spk    = false;
    cfg.internal_mic    = false;
    M5.begin(cfg);

    loadCalibration();

    Serial.println("\n========================================");
    Serial.printf ("  M5Fire Fast Acoustic Sound Level Meter\n");
    Serial.printf ("  Sampling Rate: %d Hz (Nyquist: %.1f kHz)\n", SAMPLINGFREQUENCY, F_MAX / 1000.0f);
    Serial.printf ("  Response Time: FAST (tau = 125 ms, 34.7 dB/s decay)\n");
    Serial.printf ("  Loaded Sens94: %.2f dBFS (Target: %.1f dBA)\n", sensitivity_94, target_dBA);
    Serial.println("========================================");

    initColorMap();
    initLogFrequencyMap();
    
    bool ok = init_internal_adc_mic();
    Serial.printf("[INIT] Continuous ADC DMA: %s\n", ok ? "ONLINE" : "FAILED");

    spectrogram.setColorDepth(16);
    spectrogram.createSprite(SPECTROGRAM_WIDTH, SPECTROGRAM_HEIGHT);
    spectrogram.fillSprite(colorLUT[0]);

    M5.Display.fillScreen(colorLUT[0]);
    drawFrequencyAxis();
}

double smoothedFreq = 0.0;

void loop(void) {
    M5.update();

    // Calibration Buttons
    if (M5.BtnA.wasPressed()) {
        target_dBA -= 1.0f;
        if (target_dBA < 40.0f) target_dBA = 40.0f;
    }
    if (M5.BtnB.wasPressed()) {
        target_dBA += 1.0f;
        if (target_dBA > 120.0f) target_dBA = 120.0f;
    }
    if (M5.BtnC.wasPressed()) {
        double currentFast_dBFS = (fastPowerFiltered > 1e-12) ? 10.0 * log10(fastPowerFiltered) : -96.0;
        sensitivity_94 = (float)(currentFast_dBFS - target_dBA + 94.0f);
        
        saveCalibration();
        calFlashTimer = millis() + 2000;

        Serial.printf("[CAL] Saved New Sens94: %.2f dBFS @ 94dB (Cal Target: %.1f dBA)\n",
                      sensitivity_94, target_dBA);
    }

    if (!mic_adc_handle) return;

    // 1. Read DMA buffer
    uint8_t dmaRawBytes[SIGNAL_LENGTH * 2];
    uint32_t bytes_read = 0;
    esp_err_t err = adc_continuous_read(mic_adc_handle, dmaRawBytes, sizeof(dmaRawBytes), &bytes_read, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK || bytes_read == 0) return;

    // 2. Parse 12-bit ADC data & remove DC bias
    int32_t sum = 0;
    for (int i = 0; i < SIGNAL_LENGTH; i++) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)&dmaRawBytes[i * 2];
        int16_t val = (int16_t)(p->type1.data);
        adcRawSamples[i] = val;
        sum += val;
    }
    double mean = (double)sum / SIGNAL_LENGTH;

    for (int n = 0; n < SIGNAL_LENGTH; n++) {
        adcBuffer[n] = (double)adcRawSamples[n] - mean;
        vImag[n]     = 0.0;
    }

    // 3. Compute FFT
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    // 4. IEC 61672 A-weighted instantaneous power integration
    double frameAPower = 0.0;
    for (int k = 1; k < SIGNAL_LENGTH / 2; k++) {
        double ampNorm = (adcBuffer[k] / (SIGNAL_LENGTH / 2.0)) / 2048.0;
        double binPower = (ampNorm * ampNorm) * 0.5;
        frameAPower += binPower * aWeightPowerTable[k];
    }

    // ========================================================================
    // 5. IEC 61672-1 "FAST" TIME WEIGHTING (tau = 125 ms, 34.7 dB/s decay)
    // ========================================================================
    const double dt = (double)SIGNAL_LENGTH / SAMPLINGFREQUENCY; // ~23.2 ms per frame
    const double tau_fast = 0.125;                              // 125 ms standard Fast time constant
    const double alpha_fast = 1.0 - exp(-dt / tau_fast);        // ~0.169 filter weight

    if (fastPowerFiltered == 0.0) {
        fastPowerFiltered = frameAPower;
    } else {
        fastPowerFiltered = (1.0 - alpha_fast) * fastPowerFiltered + alpha_fast * frameAPower;
    }

    // Compute Fast A-weighted sound level (LAF) in dBA
    double fast_dBFS = (fastPowerFiltered > 1e-12) ? 10.0 * log10(fastPowerFiltered) : -96.0;
    currentLaf_dBA   = fast_dBFS - sensitivity_94 + 94.0;

    // 6. Dominant frequency tracking
    double rawPeak = getCleanPeakFrequency();
    if (fabs(rawPeak - smoothedFreq) > (F_MAX * 0.015f)) {
        smoothedFreq = rawPeak;
    } else {
        smoothedFreq = smoothedFreq * 0.75 + rawPeak * 0.25;
    }

    // 7. Slide spectrogram left by 1 pixel
    spectrogram.scroll(-1, 0);

    // 8. Render new column
    for (int y = 0; y < SPECTROGRAM_HEIGHT; y++) {
        double amp0 = adcBuffer[yMap[y].b0];
        double amp1 = adcBuffer[yMap[y].b1];
        double amp = (1.0f - yMap[y].frac) * amp0 + yMap[y].frac * amp1;
        amp /= (SIGNAL_LENGTH / 2.0);

        double bin_dBFS = (amp > 1.0) ? 20.0 * log10(amp / 2048.0) : -96.0;
        double bin_dBA  = bin_dBFS - sensitivity_94 + 94.0;

        int colorIdx = constrain(map((int)bin_dBA, 35, 95, 0, 255), 0, 255);
        spectrogram.drawPixel(SPECTROGRAM_WIDTH - 1, y, colorLUT[colorIdx]);
    }

    // 9. Push to screen
    spectrogram.pushSprite(0, HEADER_HEIGHT);

    // 10. Draw Header Bar
    M5.Display.fillRect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, 0x10A2);

    if (millis() < calFlashTimer) {
        M5.Display.fillRect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, 0x03E0);
        M5.Display.setTextColor(TFT_BLACK, 0x03E0);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(18, 12);
        M5.Display.print("CALIBRATED & SAVED!");
    } else {
        // Line 1: Fast Sound Level (LAF) in dBA + Dominant Tone
        M5.Display.setTextColor(TFT_WHITE, 0x10A2);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(6, 3);
        M5.Display.printf("%4.1f dBA [F]", currentLaf_dBA);

        M5.Display.setCursor(185, 3);
        if (smoothedFreq >= (F_MIN * 0.95f)) {
            M5.Display.printf("%5.0f Hz", smoothedFreq);
        } else {
            M5.Display.print("----- Hz");
        }

        // Line 2: Calibration target & Sens94 status
        M5.Display.setTextSize(1);
        M5.Display.setTextColor(0xCE79, 0x10A2);
        M5.Display.setCursor(6, 26);
        M5.Display.printf("Cal: %3.0fdB [A-/B+] | Sens94: %4.1fdBFS [C:CAL]", target_dBA, sensitivity_94);
    }
}
