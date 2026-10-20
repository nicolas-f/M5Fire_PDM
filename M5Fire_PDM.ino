#include <M5Unified.h>
#include <arduinoFFT.h>
#include <esp_adc/adc_continuous.h>
#include <math.h>

// ============================================================================
// CONFIGURATION: 44.1 kHz Continuous ADC (Nyquist = 22.05 kHz)
// ============================================================================
#define SAMPLINGFREQUENCY     44100

// Internal analog microphone pin on M5Stack Fire
#define INTERNAL_MIC_PIN      34     // ADC1 Channel 6
#define INTERNAL_MIC_CHANNEL  ADC_CHANNEL_6

#define SIGNAL_LENGTH         1024

// Frequency range extended to Nyquist (44.1 kHz / 2 = 22050 Hz)
#define F_MIN                 500.0f
#define F_MAX                 (SAMPLINGFREQUENCY / 2.0f) // 22050.0 Hz

// Display Dimensions
#define SCREEN_WIDTH          320
#define SCREEN_HEIGHT         240
#define HEADER_HEIGHT         34

#define AXIS_WIDTH            40
#define SPECTROGRAM_WIDTH     (SCREEN_WIDTH - AXIS_WIDTH)     // 280 pixels
#define SPECTROGRAM_HEIGHT    (SCREEN_HEIGHT - HEADER_HEIGHT) // 206 pixels

// ADC Raw Buffer & FFT Buffers
int16_t adcRawSamples[SIGNAL_LENGTH];
double adcBuffer[SIGNAL_LENGTH];
double vImag[SIGNAL_LENGTH];

ArduinoFFT<double> FFT = ArduinoFFT<double>(adcBuffer, vImag, SIGNAL_LENGTH, SAMPLINGFREQUENCY);

// Static handle avoids linker conflicts with Arduino core's internal adc_handle
static adc_continuous_handle_t mic_adc_handle = NULL;

M5Canvas spectrogram(&M5.Display);
uint16_t colorLUT[256];

// Precomputed log-Y to FFT bin interpolation table
struct YMap {
    int b0;
    int b1;
    float frac;
};
YMap yMap[SPECTROGRAM_HEIGHT];

// ISO 266 1/3-Octave table up to Nyquist
struct ThirdOctave {
    float freq;
    const char* label;
    bool isFullOctave;
};

const ThirdOctave allThirdOctaves[] = {
    { 500.0f,   "500",   true  }, // Hidden bottom boundary
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
    { 22050.0f, "22k",   true  }  // Hidden top boundary
};
const int TOTAL_OCTAVES = sizeof(allThirdOctaves) / sizeof(allThirdOctaves[0]);

int sensitivity_dB = 0;

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

    // Skip the first index (500) and last index (22k) to avoid text truncation
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
        .conv_frame_size = SIGNAL_LENGTH * 2, // 2 bytes per sample on ESP32
    };
    esp_err_t err = adc_continuous_new_handle(&adc_config, &mic_adc_handle);
    if (err != ESP_OK) {
        Serial.printf("[ERR] adc_continuous_new_handle: 0x%x\n", err);
        return false;
    }

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
    if (err != ESP_OK) {
        Serial.printf("[ERR] adc_continuous_config: 0x%x\n", err);
        return false;
    }

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

void setup() {
    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    cfg.internal_spk    = false;
    cfg.internal_mic    = false;
    M5.begin(cfg);

    Serial.println("\n========================================");
    Serial.printf ("  Internal Analog Mic (GPIO 34) @ %d Hz\n", SAMPLINGFREQUENCY);
    Serial.printf ("  Range: %.0f Hz - %.1f kHz (Nyquist)\n", F_MIN, F_MAX / 1000.0f);
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

    if (M5.BtnA.wasPressed()) {
        sensitivity_dB -= 3;
        if (sensitivity_dB < -24) sensitivity_dB = -24;
    }
    if (M5.BtnB.wasPressed()) {
        sensitivity_dB = 0;
    }
    if (M5.BtnC.wasPressed()) {
        sensitivity_dB += 3;
        if (sensitivity_dB > 36) sensitivity_dB = 36;
    }

    if (!mic_adc_handle) return;

    // 1. Read DMA buffer from Continuous ADC
    uint8_t dmaRawBytes[SIGNAL_LENGTH * 2];
    uint32_t bytes_read = 0;
    esp_err_t err = adc_continuous_read(mic_adc_handle, dmaRawBytes, sizeof(dmaRawBytes), &bytes_read, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK || bytes_read == 0) return;

    // 2. Parse 12-bit ADC data & remove DC bias (~2048)
    int32_t sum = 0;
    int16_t peakRawSample = 0;

    for (int i = 0; i < SIGNAL_LENGTH; i++) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)&dmaRawBytes[i * 2];
        int16_t val = (int16_t)(p->type1.data);
        adcRawSamples[i] = val;
        sum += val;
    }

    double mean = (double)sum / SIGNAL_LENGTH;

    for (int n = 0; n < SIGNAL_LENGTH; n++) {
        double centered = (double)adcRawSamples[n] - mean;
        adcBuffer[n] = centered;
        vImag[n]     = 0.0;
        if (abs((int)centered) > peakRawSample) peakRawSample = abs((int)centered);
    }

    double overall_dBFS = (peakRawSample > 0) ? 20.0 * log10((double)peakRawSample / 2048.0) : -96.0;

    static uint32_t lastDebug = 0;
    if (millis() - lastDebug > 1000) {
        lastDebug = millis();
        Serial.printf("[DEBUG] Raw Mean: %4.0f | Peak Dev: %4d | dBFS: %3.0f\n", 
                      mean, peakRawSample, overall_dBFS);
    }

    // 3. Compute FFT
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    // 4. Stable frequency tracking
    double rawPeak = getCleanPeakFrequency();
    if (fabs(rawPeak - smoothedFreq) > (F_MAX * 0.015f)) {
        smoothedFreq = rawPeak;
    } else {
        smoothedFreq = smoothedFreq * 0.75 + rawPeak * 0.25;
    }

    // 5. Tone dBFS calculation
    double tone_dBFS = -96.0;
    int peakBin = (int)roundf((smoothedFreq / (float)SAMPLINGFREQUENCY) * SIGNAL_LENGTH);
    if (peakBin >= 1 && peakBin < (SIGNAL_LENGTH / 2)) {
        double toneAmp = adcBuffer[peakBin] / (SIGNAL_LENGTH / 2.0);
        if (toneAmp > 1.0) {
            tone_dBFS = 20.0 * log10(toneAmp / 2048.0);
        }
    }

    // 6. Slide spectrogram left by 1 pixel
    spectrogram.scroll(-1, 0);

    // 7. Render new column with NoiseCapture color gradient
    for (int y = 0; y < SPECTROGRAM_HEIGHT; y++) {
        double amp0 = adcBuffer[yMap[y].b0];
        double amp1 = adcBuffer[yMap[y].b1];
        double amp = (1.0f - yMap[y].frac) * amp0 + yMap[y].frac * amp1;
        amp /= (SIGNAL_LENGTH / 2.0);

        double pix_dBFS = (amp > 1.0) ? 20.0 * log10(amp / 2048.0) : -96.0;
        double adjusted_dBFS = pix_dBFS + sensitivity_dB;

        int colorIdx = constrain(map((int)adjusted_dBFS, -60, -10, 0, 255), 0, 255);
        spectrogram.drawPixel(SPECTROGRAM_WIDTH - 1, y, colorLUT[colorIdx]);
    }

    // 8. Push to screen
    spectrogram.pushSprite(0, HEADER_HEIGHT);

    // 9. Update header bar
    M5.Display.fillRect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, 0x10A2);

    M5.Display.setTextColor(TFT_WHITE, 0x10A2);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(6, 4);
    M5.Display.printf("INTERNAL MIC (0.5k-%.1fk)", F_MAX / 1000.0f);
    M5.Display.setCursor(6, 18);
    M5.Display.printf("Peak: %3.0f dBFS | Sens: %+ddB", overall_dBFS, sensitivity_dB);

    M5.Display.setCursor(185, 2);
    M5.Display.setTextSize(2);
    if (smoothedFreq >= (F_MIN * 0.95f)) {
        M5.Display.printf("%5.0f Hz", smoothedFreq);
    } else {
        M5.Display.print("----- Hz");
    }
    M5.Display.setTextSize(1);
    M5.Display.setCursor(185, 20);
    M5.Display.printf("Tone: %3.0f dBFS", tone_dBFS);
}
