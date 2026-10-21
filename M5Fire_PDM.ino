#include <M5Unified.h>
#include <arduinoFFT.h>
#include <driver/i2s_pdm.h>
#include <esp_adc/adc_continuous.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <math.h>

// ============================================================================
// HARDWARE DEFINITIONS & PINOUTS
// ============================================================================
#define PIN_PDM_CLK           22  // Port A (Red) SCL
#define PIN_PDM_DATA          21  // Port A (Red) SDA
#define INTERNAL_MIC_CHANNEL  ADC_CHANNEL_6 // GPIO 34 on ADC1

#define SCREEN_WIDTH          320
#define SCREEN_HEIGHT         240
#define HEADER_HEIGHT         42
#define AXIS_WIDTH            40
#define SPECTROGRAM_WIDTH     (SCREEN_WIDTH - AXIS_WIDTH)     // 280 pixels
#define SPECTROGRAM_HEIGHT    (SCREEN_HEIGHT - HEADER_HEIGHT) // 198 pixels

#define MAX_BUFFER_SIZE       1024

enum MicSource {
    MIC_NONE,
    MIC_EXTERNAL_PDM,
    MIC_INTERNAL_ADC
};

MicSource activeMic = MIC_NONE;
uint32_t activeSamplingRate = 16000;
uint16_t activeSignalLength = 512;
float activeFMin = 250.0f;
float activeFMax = 6300.0f;

// Shared Audio Buffers
int16_t rawDmaBuffer[MAX_BUFFER_SIZE * 2];
int16_t adcRawSamples[MAX_BUFFER_SIZE];
double adcBuffer[MAX_BUFFER_SIZE];
double vImag[MAX_BUFFER_SIZE];

ArduinoFFT<double> FFT = ArduinoFFT<double>(adcBuffer, vImag, 512, 16000);

// Hardware Driver Handles
i2s_chan_handle_t rx_chan = NULL;
static adc_continuous_handle_t mic_adc_handle = NULL;

M5Canvas spectrogram(&M5.Display);
uint16_t colorLUT[256];

struct YMap {
    int b0;
    int b1;
    float frac;
};
YMap yMap[SPECTROGRAM_HEIGHT];
float aWeightPowerTable[MAX_BUFFER_SIZE / 2];

// ISO 266 1/3-Octave Table
struct ThirdOctave {
    float freq;
    const char* label;
    bool isFullOctave;
};

const ThirdOctave allThirdOctaves[] = {
    { 200.0f,   "200",   false },
    { 250.0f,   "250",   true  },
    { 315.0f,   "315",   false },
    { 400.0f,   "400",   false },
    { 500.0f,   "500",   true  },
    { 630.0f,   "630",   false },
    { 800.0f,   "800",   false },
    { 1000.0f,  "1k",    true  },
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
    { 22050.0f, "22k",   true  }
};
const int TOTAL_OCTAVES = sizeof(allThirdOctaves) / sizeof(allThirdOctaves[0]);

// Stored Sensitivities
float target_dBA = 94.0f;
float sensitivity_pdm = -26.0f;
float sensitivity_int = -28.0f;
uint32_t calFlashTimer = 0;

double fastPowerFiltered = 0.0;
double currentLaf_dBA = 0.0;
double smoothedFreq = 0.0;

// NoiseCapture Color Palette
struct RGB { uint8_t r, g, b; };
const RGB noiseCapturePalette[17] = {
    { 0x30, 0x30, 0x30 }, { 0x2D, 0x3C, 0x2D }, { 0x2A, 0x48, 0x2A },
    { 0x27, 0x54, 0x27 }, { 0x24, 0x60, 0x24 }, { 0x21, 0x6C, 0x21 },
    { 0x3F, 0x8E, 0x19 }, { 0x61, 0xA5, 0x14 }, { 0x82, 0xBB, 0x0F },
    { 0xA4, 0xD2, 0x0A }, { 0xC5, 0xE8, 0x05 }, { 0xE7, 0xFF, 0x00 },
    { 0xEB, 0xD4, 0x00 }, { 0xEF, 0xAA, 0x00 }, { 0xF3, 0x7F, 0x00 },
    { 0xF7, 0x55, 0x00 }, { 0xFB, 0x2A, 0x00 }
};

// Returns active sensitivity based on the selected microphone
inline float getCurrentSensitivity() {
    float sens = (activeMic == MIC_EXTERNAL_PDM) ? sensitivity_pdm : sensitivity_int;
    return (sens > 0.0f) ? 0.0f : sens;
}

// ============================================================================
// INITIALIZATION HELPERS
// ============================================================================
void initColorMap() {
    for (int i = 0; i < 256; i++) {
        float pos = (i / 255.0f) * 16.0f;
        int idx = (int)pos;
        float frac = pos - idx;

        if (idx >= 16) {
            colorLUT[i] = M5.Display.color565(noiseCapturePalette[16].r, noiseCapturePalette[16].g, noiseCapturePalette[16].b);
        } else {
            uint8_t r = (uint8_t)((1.0f - frac) * noiseCapturePalette[idx].r + frac * noiseCapturePalette[idx + 1].r);
            uint8_t g = (uint8_t)((1.0f - frac) * noiseCapturePalette[idx].g + frac * noiseCapturePalette[idx + 1].g);
            uint8_t b = (uint8_t)((1.0f - frac) * noiseCapturePalette[idx].b + frac * noiseCapturePalette[idx + 1].b);
            colorLUT[i] = M5.Display.color565(r, g, b);
        }
    }
}

float computeAWeightPower(float freq) {
    if (freq < 10.0f) return 0.0f;
    float f2 = freq * freq;
    float f4 = f2 * f2;
    float num = 148698944.0f * f4;
    float den = (f2 + 424.36f) * sqrtf((f2 + 11599.29f) * (f2 + 544496.41f)) * (f2 + 148698944.0f);
    if (den <= 0.0f) return 0.0f;
    float wa = (num / den) * 1.25892541f;
    return wa * wa;
}

void initLogFrequencyMap() {
    for (int y = 0; y < SPECTROGRAM_HEIGHT; y++) {
        float norm_y = (float)(SPECTROGRAM_HEIGHT - 1 - y) / (float)(SPECTROGRAM_HEIGHT - 1);
        float freq = activeFMin * powf(activeFMax / activeFMin, norm_y);
        float binFloat = (freq / (float)activeSamplingRate) * activeSignalLength;
        binFloat = constrain(binFloat, 1.0f, (float)(activeSignalLength / 2 - 2));

        yMap[y].b0 = (int)binFloat;
        yMap[y].b1 = yMap[y].b0 + 1;
        yMap[y].frac = binFloat - yMap[y].b0;
    }

    for (int k = 0; k < activeSignalLength / 2; k++) {
        float freq = k * (float)activeSamplingRate / (float)activeSignalLength;
        aWeightPowerTable[k] = computeAWeightPower(freq);
    }
}

int getScreenY(float freq) {
    if (freq < activeFMin) freq = activeFMin;
    if (freq > activeFMax) freq = activeFMax;
    float norm_y = logf(freq / activeFMin) / logf(activeFMax / activeFMin);
    int y = (SPECTROGRAM_HEIGHT - 1) - (int)roundf(norm_y * (SPECTROGRAM_HEIGHT - 1));
    return HEADER_HEIGHT + y;
}

void drawFrequencyAxis() {
    M5.Display.fillRect(SPECTROGRAM_WIDTH, HEADER_HEIGHT, AXIS_WIDTH, SPECTROGRAM_HEIGHT, colorLUT[0]);
    M5.Display.drawFastVLine(SPECTROGRAM_WIDTH, HEADER_HEIGHT, SPECTROGRAM_HEIGHT, 0x4208);
    M5.Display.setTextSize(1);

    int firstIdx = -1, lastIdx = -1;
    for (int i = 0; i < TOTAL_OCTAVES; i++) {
        if (allThirdOctaves[i].freq >= (activeFMin - 1.0f) && firstIdx == -1) firstIdx = i;
        if (allThirdOctaves[i].freq <= (activeFMax + 1.0f)) lastIdx = i;
    }

    for (int i = firstIdx + 1; i < lastIdx; i++) {
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

// ============================================================================
// HARDWARE DRIVER CONTROLLERS
// ============================================================================
bool startPDMDriver() {
    gpio_reset_pin((gpio_num_t)PIN_PDM_CLK);
    gpio_reset_pin((gpio_num_t)PIN_PDM_DATA);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    if (i2s_new_channel(&chan_cfg, NULL, &rx_chan) != ESP_OK) return false;

    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .clk = (gpio_num_t)PIN_PDM_CLK,
            .din = (gpio_num_t)PIN_PDM_DATA,
            .invert_flags = { .clk_inv = false },
        },
    };

    if (i2s_channel_init_pdm_rx_mode(rx_chan, &pdm_rx_cfg) != ESP_OK) return false;
    gpio_set_drive_capability((gpio_num_t)PIN_PDM_CLK, GPIO_DRIVE_CAP_3);
    return (i2s_channel_enable(rx_chan) == ESP_OK);
}

void stopPDMDriver() {
    if (rx_chan) {
        i2s_channel_disable(rx_chan);
        i2s_del_channel(rx_chan);
        rx_chan = NULL;
    }
    gpio_reset_pin((gpio_num_t)PIN_PDM_CLK);
    gpio_reset_pin((gpio_num_t)PIN_PDM_DATA);
}

bool startADCDriver() {
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4096,
        .conv_frame_size = 1024 * 2,
    };
    if (adc_continuous_new_handle(&adc_config, &mic_adc_handle) != ESP_OK) return false;

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 44100,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = INTERNAL_MIC_CHANNEL,
        .unit = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    dig_cfg.pattern_num = 1;
    dig_cfg.adc_pattern = &pattern;

    if (adc_continuous_config(mic_adc_handle, &dig_cfg) != ESP_OK) return false;
    return (adc_continuous_start(mic_adc_handle) == ESP_OK);
}

void stopADCDriver() {
    if (mic_adc_handle) {
        adc_continuous_stop(mic_adc_handle);
        adc_continuous_deinit(mic_adc_handle);
        mic_adc_handle = NULL;
    }
}

bool detectPDMMicOnPortA() {
    if (!startPDMDriver()) return false;

    delay(80);
    size_t bytes_read = 0;
    i2s_channel_read(rx_chan, (void *)rawDmaBuffer, 512 * 2 * sizeof(int16_t), &bytes_read, 200 / portTICK_PERIOD_MS);

    int16_t s_min = 32767, s_max = -32768;
    for (int i = 0; i < 512 * 2; i++) {
        if (rawDmaBuffer[i] < s_min) s_min = rawDmaBuffer[i];
        if (rawDmaBuffer[i] > s_max) s_max = rawDmaBuffer[i];
    }
    int span = s_max - s_min;

    bool isConnected = (span > 150 && abs(s_min) < 28000 && abs(s_max) < 28000);
    if (!isConnected) {
        stopPDMDriver();
    }
    return isConnected;
}

void configureMicrophoneMode(MicSource source) {
    if (source == activeMic) return;

    if (source == MIC_EXTERNAL_PDM) {
        stopADCDriver();
        if (!rx_chan) startPDMDriver();

        activeMic = MIC_EXTERNAL_PDM;
        activeSamplingRate = 16000;
        activeSignalLength = 512;
        activeFMin = 250.0f;
        activeFMax = 6300.0f;
        Serial.println("[MODE] Switched to EXTERNAL PDM MIC (Port A @ 16 kHz)");
    } else {
        stopPDMDriver();
        if (!mic_adc_handle) startADCDriver();

        activeMic = MIC_INTERNAL_ADC;
        activeSamplingRate = 44100;
        activeSignalLength = 1024;
        activeFMin = 500.0f;
        activeFMax = 22050.0f;
        Serial.println("[MODE] Switched to INTERNAL ANALOG MIC (GPIO 34 @ 44.1 kHz)");
    }

    FFT = ArduinoFFT<double>(adcBuffer, vImag, activeSignalLength, activeSamplingRate);
    initLogFrequencyMap();
    drawFrequencyAxis();
    fastPowerFiltered = 0.0;
    smoothedFreq = 0.0;
}

void loadCalibration() {
    Preferences prefs;
    prefs.begin("sound_cal", true);
    sensitivity_pdm = prefs.getFloat("sens_pdm", -26.0f);
    sensitivity_int = prefs.getFloat("sens_int", -28.0f);
    target_dBA = prefs.getFloat("target", 94.0f);
    prefs.end();

    if (sensitivity_pdm > 0.0f) sensitivity_pdm = 0.0f;
    if (sensitivity_int > 0.0f) sensitivity_int = 0.0f;
}

void saveCalibration() {
    Preferences prefs;
    prefs.begin("sound_cal", false);
    if (activeMic == MIC_EXTERNAL_PDM) {
        prefs.putFloat("sens_pdm", sensitivity_pdm);
    } else {
        prefs.putFloat("sens_int", sensitivity_int);
    }
    prefs.putFloat("target", target_dBA);
    prefs.end();
}

double getCleanPeakFrequency() {
    double maxY = 0;
    int maxBin = 0;
    int minBin = (int)((activeFMin * 0.95f / (float)activeSamplingRate) * activeSignalLength);
    int maxBinLimit = (int)((activeFMax * 0.99f / (double)activeSamplingRate) * activeSignalLength);

    for (int i = minBin; i <= maxBinLimit; i++) {
        if (adcBuffer[i] > maxY) {
            maxY = adcBuffer[i];
            maxBin = i;
        }
    }
    if (maxBin <= minBin || maxY < 5.0) return 0.0;

    double y1 = adcBuffer[maxBin - 1], y2 = adcBuffer[maxBin], y3 = adcBuffer[maxBin + 1];
    double delta = 0.5 * (y1 - y3) / (y1 - 2.0 * y2 + y3);
    return ((double)maxBin - delta) * ((double)activeSamplingRate / activeSignalLength);
}

// ============================================================================
// MODULAR LOOP HELPERS
// ============================================================================

// 1. Handles physical button interactions for calibration
void handleCalibrationControls() {
    if (M5.BtnA.wasPressed()) {
        target_dBA -= 1.0f;
        if (target_dBA < 25.0f) target_dBA = 25.0f;
    }
    if (M5.BtnB.wasPressed()) {
        target_dBA += 1.0f;
        if (target_dBA > 120.0f) target_dBA = 120.0f;
    }
    if (M5.BtnC.wasPressed()) {
        double currentFast_dBFS = (fastPowerFiltered > 1e-12) ? 10.0 * log10(fastPowerFiltered) : -96.0;
        float newSensitivity = (float)(currentFast_dBFS - target_dBA + 94.0f);
        if (newSensitivity > 0.0f) {
            newSensitivity = 0.0f;
        }
        if (activeMic == MIC_EXTERNAL_PDM) {
            sensitivity_pdm = newSensitivity;
        } else {
            sensitivity_int = newSensitivity;
        }
        saveCalibration();
        calFlashTimer = millis() + 2000;
    }
}

// 2. Acquires raw samples from active driver, removes DC bias, populates adcBuffer
bool acquireAudioSamples() {
    if (activeMic == MIC_EXTERNAL_PDM) {
        size_t pdm_bytes_read = 0;
        esp_err_t err = i2s_channel_read(rx_chan, (void *)rawDmaBuffer,
                                         activeSignalLength * 2 * sizeof(int16_t),
                                         &pdm_bytes_read, 1000 / portTICK_PERIOD_MS);
        if (err != ESP_OK || pdm_bytes_read == 0) return false;

        // Channel separation & disconnect check
        int16_t l_min = 32767, l_max = -32768, r_min = 32767, r_max = -32768;
        int64_t l_sum = 0, r_sum = 0;

        for (int n = 0; n < activeSignalLength; n++) {
            int16_t l = rawDmaBuffer[n * 2];
            int16_t r = rawDmaBuffer[n * 2 + 1];
            if (l < l_min) l_min = l; if (l > l_max) l_max = l;
            if (r < r_min) r_min = r; if (r > r_max) r_max = r;
            l_sum += l;
            r_sum += r;
        }

        // Unplug detection: saturation at ~30935 with span < 15
        static int disconnectCounter = 0;
        if ((l_max - l_min < 15 && abs(l_min) > 28000) || (r_max - r_min < 15 && abs(r_min) > 28000)) {
            if (++disconnectCounter > 6) {
                disconnectCounter = 0;
                configureMicrophoneMode(MIC_INTERNAL_ADC);
                return false;
            }
        } else {
            disconnectCounter = 0;
        }

        bool useRight = ((r_max - r_min) > (l_max - l_min));
        double mean = (double)(useRight ? r_sum : l_sum) / activeSignalLength;

        for (int n = 0; n < activeSignalLength; n++) {
            int16_t s = useRight ? rawDmaBuffer[n * 2 + 1] : rawDmaBuffer[n * 2];
            adcBuffer[n] = (double)s - mean;
            vImag[n] = 0.0;
        }
    }
    else { // MIC_INTERNAL_ADC
        uint32_t adc_bytes_read = 0;
        esp_err_t err = adc_continuous_read(mic_adc_handle, (uint8_t *)rawDmaBuffer,
                                           activeSignalLength * 2,
                                           &adc_bytes_read, 1000 / portTICK_PERIOD_MS);
        if (err != ESP_OK || adc_bytes_read == 0) return false;

        int32_t sum = 0;
        for (int i = 0; i < activeSignalLength; i++) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&((uint8_t *)rawDmaBuffer)[i * 2];
            int16_t val = (int16_t)(p->type1.data);
            adcRawSamples[i] = val;
            sum += val;
        }
        double mean = (double)sum / activeSignalLength;

        for (int n = 0; n < activeSignalLength; n++) {
            adcBuffer[n] = (double)adcRawSamples[n] - mean;
            vImag[n] = 0.0;
        }
    }

    return true;
}

// 3. Runs FFT, computes IEC A-weighting power, applies Fast decay, tracks peak frequency
void processAcousticsAndFFT() {
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    double fullScale = (activeMic == MIC_EXTERNAL_PDM) ? 32768.0 : 2048.0;

    double frameAPower = 0.0;
    for (int k = 1; k < activeSignalLength / 2; k++) {
        double ampNorm = (adcBuffer[k] / (activeSignalLength / 2.0)) / fullScale;
        frameAPower += (ampNorm * ampNorm * 0.5) * aWeightPowerTable[k];
    }

    // IEC Fast exponential integration (tau = 125 ms)
    const double dt = (double)activeSignalLength / activeSamplingRate;
    const double alpha_fast = 1.0 - exp(-dt / 0.125);
    if (fastPowerFiltered == 0.0) fastPowerFiltered = frameAPower;
    else fastPowerFiltered = (1.0 - alpha_fast) * fastPowerFiltered + alpha_fast * frameAPower;

    double fast_dBFS = (fastPowerFiltered > 1e-12) ? 10.0 * log10(fastPowerFiltered) : -96.0;
    currentLaf_dBA = fast_dBFS - getCurrentSensitivity() + 94.0;

    // Track dominant frequency
    double rawPeak = getCleanPeakFrequency();
    if (fabs(rawPeak - smoothedFreq) > (activeFMax * 0.015f)) {
        smoothedFreq = rawPeak;
    } else {
        smoothedFreq = smoothedFreq * 0.75 + rawPeak * 0.25;
    }
}

// 4. Scrolls waterfall left and renders the new column on the right
void renderSpectrogramColumn() {
    spectrogram.scroll(-1, 0);

    double fullScale = (activeMic == MIC_EXTERNAL_PDM) ? 32768.0 : 2048.0;
    float currentSens = getCurrentSensitivity();

    for (int y = 0; y < SPECTROGRAM_HEIGHT; y++) {
        double amp0 = adcBuffer[yMap[y].b0];
        double amp1 = adcBuffer[yMap[y].b1];
        double amp = (1.0f - yMap[y].frac) * amp0 + yMap[y].frac * amp1;
        amp /= (activeSignalLength / 2.0);

        double bin_dBFS = (amp > 1.0) ? 20.0 * log10(amp / fullScale) : -96.0;
        double bin_dBA = bin_dBFS - currentSens + 94.0;

        int colorIdx = constrain(map((int)bin_dBA, 35, 95, 0, 255), 0, 255);
        spectrogram.drawPixel(SPECTROGRAM_WIDTH - 1, y, colorLUT[colorIdx]);
    }

    spectrogram.pushSprite(0, HEADER_HEIGHT);
}

// 5. Renders the top header status and calibration information
void updateHeaderUI() {
    M5.Display.fillRect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, 0x10A2);

    if (millis() < calFlashTimer) {
        M5.Display.fillRect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, 0x03E0);
        M5.Display.setTextColor(TFT_BLACK, 0x03E0);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(18, 12);
        M5.Display.print("CALIBRATED & SAVED!");
    } else {
        M5.Display.setTextColor(TFT_WHITE, 0x10A2);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(6, 3);
        M5.Display.printf("%4.1f dBA", currentLaf_dBA);

        M5.Display.setCursor(180, 3);
        if (smoothedFreq >= (activeFMin * 0.95f)) {
            M5.Display.printf("%5.0f Hz", smoothedFreq);
        } else {
            M5.Display.print("----- Hz");
        }

        M5.Display.setTextSize(1);
        M5.Display.setTextColor(0xCE79, 0x10A2);
        M5.Display.setCursor(6, 26);
        M5.Display.printf("[%s] %3.0fdB [A/B] | S94:%4.1f [C]",
            (activeMic == MIC_EXTERNAL_PDM) ? "EXT PDM 16k" : "INT ADC 44k",
            target_dBA, getCurrentSensitivity());
    }
}

// ============================================================================
// MAIN SETUP & LOOP
// ============================================================================
void setup() {
    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    cfg.internal_spk = false;
    cfg.internal_mic = false;
    M5.begin(cfg);

    loadCalibration();
    initColorMap();

    spectrogram.setColorDepth(16);
    spectrogram.createSprite(SPECTROGRAM_WIDTH, SPECTROGRAM_HEIGHT);
    spectrogram.fillSprite(colorLUT[0]);
    M5.Display.fillScreen(colorLUT[0]);

    Serial.println("[BOOT] Probing Port A for PDM Microphone...");
    if (detectPDMMicOnPortA()) {
        configureMicrophoneMode(MIC_EXTERNAL_PDM);
    } else {
        configureMicrophoneMode(MIC_INTERNAL_ADC);
    }
}

void loop() {
    M5.update();

    handleCalibrationControls();

    if (!acquireAudioSamples()) {
        return;
    }

    processAcousticsAndFFT();
    renderSpectrogramColumn();
    updateHeaderUI();
}
