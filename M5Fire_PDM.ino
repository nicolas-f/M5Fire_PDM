#include <M5Unified.h>
#include <arduinoFFT.h> // Install the arduinoFFT library v2.0.4 from https://github.com/kosme/arduinoFFT 
#include <driver/i2s_pdm.h>
#include <math.h>

// Left side of the Fire with the M5 Grove PDM Mic plugged
#define GROVE_B_PIN_CLK   22 
#define GROVE_B_PIN_DATA  21 

#define SCREEN_WIDTH          320
#define SCREEN_HEIGHT         240
#define HEADER_HEIGHT         38
#define SPECTROGRAM_HEIGHT    (SCREEN_HEIGHT - HEADER_HEIGHT) // 202 pixels
#define SPECTROGRAM_WIDTH     SCREEN_WIDTH                    // 320 pixels

#define SIGNAL_LENGTH         512
#define SAMPLINGFREQUENCY     16000
#define MAX_DISPLAY_FREQ      6000  // 0 to 6000 Hz vertical range

// FFT buffers
int16_t stereoSamples[SIGNAL_LENGTH * 2];
double adcBuffer[SIGNAL_LENGTH];
double vImag[SIGNAL_LENGTH];

ArduinoFFT<double> FFT = ArduinoFFT<double>(adcBuffer, vImag, SIGNAL_LENGTH, SAMPLINGFREQUENCY);

// I2S Channel Handle
i2s_chan_handle_t rx_chan = NULL;

// In-memory Canvas for fast hardware scrolling
M5Canvas spectrogram(&M5.Display);

// 256-level Black -> Purple -> Magenta -> Pink color lookup table
uint16_t colorLUT[256];

void initColorMap() {
    for (int i = 0; i < 256; i++) {
        float t = i / 255.0f;
        uint8_t r = 0, g = 0, b = 0;

        if (t < 0.25f) {
            // Level 0..63: Black (0,0,0) -> Deep Purple (70,0,90)
            float f = t / 0.25f;
            r = (uint8_t)(70 * f);
            g = 0;
            b = (uint8_t)(90 * f);
        } else if (t < 0.50f) {
            // Level 64..127: Deep Purple -> Magenta (170,10,140)
            float f = (t - 0.25f) / 0.25f;
            r = (uint8_t)(70 + (170 - 70) * f);
            g = (uint8_t)(10 * f);
            b = (uint8_t)(90 + (140 - 90) * f);
        } else if (t < 0.75f) {
            // Level 128..191: Magenta -> Hot Pink (255,50,170)
            float f = (t - 0.50f) / 0.25f;
            r = (uint8_t)(170 + (255 - 170) * f);
            g = (uint8_t)(10 + (50 - 10) * f);
            b = (uint8_t)(140 + (170 - 140) * f);
        } else {
            // Level 192..255: Hot Pink -> Luminous Pastel Pink (255,180,225)
            float f = (t - 0.75f) / 0.25f;
            r = 255;
            g = (uint8_t)(50 + (180 - 50) * f);
            b = (uint8_t)(170 + (225 - 170) * f);
        }
        colorLUT[i] = M5.Display.color565(r, g, b);
    }
}

bool init_pdm() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    if (i2s_new_channel(&chan_cfg, NULL, &rx_chan) != ESP_OK) return false;

    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLINGFREQUENCY),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .clk = (gpio_num_t)GROVE_B_PIN_CLK,
            .din = (gpio_num_t)GROVE_B_PIN_DATA,
            .invert_flags = { .clk_inv = false },
        },
    };

    if (i2s_channel_init_pdm_rx_mode(rx_chan, &pdm_rx_cfg) != ESP_OK) return false;
    return (i2s_channel_enable(rx_chan) == ESP_OK);
}

// Bounded peak detector (ignores DC hum and PDM quantization noise)
double getCleanPeakFrequency() {
    double maxY = 0;
    int maxBin = 0;
    int minBin = 3; // ~94 Hz
    int maxBinLimit = (int)((MAX_DISPLAY_FREQ / (double)SAMPLINGFREQUENCY) * SIGNAL_LENGTH);

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

    initColorMap();
    init_pdm();

    // Create the off-screen canvas for the scrolling waterfall
    spectrogram.setColorDepth(16);
    spectrogram.createSprite(SPECTROGRAM_WIDTH, SPECTROGRAM_HEIGHT);
    spectrogram.fillSprite(TFT_BLACK);

    // Initial display frame
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.fillRect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, 0x18C3); // Dark slate header
    M5.Display.setTextColor(TFT_WHITE, 0x18C3);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(6, 4);
    M5.Display.print("SPECTROGRAM (0 - 6 kHz)");
    M5.Display.setCursor(6, 20);
    M5.Display.print("Time -> (Sliding Left)");
}

double smoothedFreq = 0.0;

void loop(void) {
    M5.update();
    if (!rx_chan) return;

    // 1. Read DMA stereo samples
    size_t bytes_read = 0;
    esp_err_t err = i2s_channel_read(rx_chan, (void *)stereoSamples, sizeof(stereoSamples), &bytes_read, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK || bytes_read == 0) return;

    // 2. Lock to Left channel & remove DC bias
    int64_t sum = 0;
    for (int n = 0; n < SIGNAL_LENGTH; n++) {
        sum += stereoSamples[n * 2];
    }
    double mean = (double)sum / SIGNAL_LENGTH;

    for (int n = 0; n < SIGNAL_LENGTH; n++) {
        adcBuffer[n] = (double)stereoSamples[n * 2] - mean;
        vImag[n]     = 0.0;
    }

    // 3. Compute FFT
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    // 4. Stable frequency tracking
    double rawPeak = getCleanPeakFrequency();
    if (fabs(rawPeak - smoothedFreq) > 80.0) {
        smoothedFreq = rawPeak;
    } else {
        smoothedFreq = smoothedFreq * 0.75 + rawPeak * 0.25;
    }

    // 5. Calculate max dB level for the header
    int maxAmplitudeDB = -100;
    for (int n = 1; n < SIGNAL_LENGTH / 2; n++) {
        double amp = adcBuffer[n] / (SIGNAL_LENGTH / 2.0);
        int a = (amp > 1.0) ? (int)(20.0 * log10(amp)) : -100;
        if (a > maxAmplitudeDB) maxAmplitudeDB = a;
    }

    // 6. SLIDE SPECTROGRAM LEFT BY 1 PIXEL
    spectrogram.scroll(-1, 0);

    // 7. Draw the new spectrum column at the far right edge (x = 319)
    // y = 0 is top (MAX_DISPLAY_FREQ), y = SPECTROGRAM_HEIGHT - 1 is bottom (0 Hz)
    for (int y = 0; y < SPECTROGRAM_HEIGHT; y++) {
        double freq = (SPECTROGRAM_HEIGHT - 1 - y) * ((double)MAX_DISPLAY_FREQ / SPECTROGRAM_HEIGHT);
        int bin = (int)((freq / SAMPLINGFREQUENCY) * SIGNAL_LENGTH);
        bin = constrain(bin, 1, (SIGNAL_LENGTH / 2) - 1);

        double amplitude = adcBuffer[bin] / (SIGNAL_LENGTH / 2.0);
        int dB = (amplitude > 1.0) ? (int)(20.0 * log10(amplitude)) : 0;

        // Map noise intensity: 10 dB (silence/Black) to 45 dB (loudest/Hot Pink)
        int colorIdx = map(dB, 10, 45, 0, 255);
        colorIdx = constrain(colorIdx, 0, 255);

        spectrogram.drawPixel(SPECTROGRAM_WIDTH - 1, y, colorLUT[colorIdx]);
    }

    // 8. Push the updated scrolling spectrogram to the display
    spectrogram.pushSprite(0, HEADER_HEIGHT);

    // 9. Update the top header info bar
    M5.Display.fillRect(170, 0, 150, HEADER_HEIGHT, 0x18C3);
    M5.Display.setTextColor(TFT_WHITE, 0x18C3);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(175, 4);
    if (smoothedFreq > 50.0) {
        M5.Display.printf("%4.0f Hz", smoothedFreq);
    } else {
        M5.Display.print("---- Hz");
    }
    M5.Display.setCursor(175, 20);
    M5.Display.setTextSize(1);
    M5.Display.printf("Peak Level: %2d dB", maxAmplitudeDB);
}
