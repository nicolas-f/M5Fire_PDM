/*
  Example animated analogue meters using a ILI9341 TFT LCD screen

  Needs Font 2 (also Font 4 if using large scale label)

  Make sure all the display driver and pin comnenctions are correct by
  editting the User_Setup.h file in the TFT_eSPI library folder.

  #########################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
  #########################################################################
*/

#include <M5Stack.h>
#include "driver/i2s.h"



#define PIN_CLK  22
#define PIN_DATA 21
const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = 512;
int16_t samples[BLOCK_SIZE];
#define SAMPLE_RATE 24000
long total_read = 0;



#define TFT_GREY 0x5AEB

#define LOOP_PERIOD 35  // Display updates every 35 ms
#define MIN_VALUE 30
#define MAX_VALUE 110

#define DEGTORAD 0.0174532925 // degree to radian

float ltx    = 0;               // Saved x coord of bottom of needle
uint16_t osx = 120, osy = 120;  // Saved x & y coords
uint32_t updateTime = 0;        // time for next update

setup_t tft_settings;
int old_analog  = -999;  // Value last displayed

int d            = 0;

int display_rms = 0;
double display_dbspl = 0.0f;


// http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=be&passmode=hp&order=2&usesr=usesr&sr=24000&frequencyLow=100&noteLow=&noteHigh=&pw=pw&calctype=float&run=Send
//High pass bessel filter order=2 alpha1=0.0041666666666667 
class  FilterBeHp2
{
  public:
    FilterBeHp2()
    {
      v[0]=0.0;
      v[1]=0.0;
    }
  private:
    float v[3];
  public:
    float step(float x) //class II 
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (9.823849154958753660e-1 * x)
         + (-0.96497792085018785357 * v[0])
         + (1.96456174113331383246 * v[1]);
      return 
         (v[0] + v[2])
        - 2 * v[1];
    }
};

FilterBeHp2 filter;

void init_pdm() {
  
    esp_err_t err;

    i2s_config_t audio_in_i2s_config = {
         .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
         .sample_rate = SAMPLE_RATE,
         .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
         .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // although the SEL config should be left, it seems to transmit on right
         .communication_format = I2S_COMM_FORMAT_STAND_I2S,
         .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
         .dma_buf_count = 2,
         .dma_buf_len = BLOCK_SIZE
        };
    
    // This function must be called before any I2S driver read/write operations.
    err = i2s_driver_install(I2S_PORT, &audio_in_i2s_config, 0, NULL);
    if (err != ESP_OK) {
      Serial.printf("Failed installing driver: %d\n", err);
      while (true);
    }
  
    // configure which pin on the ESP32 is connected to which pin on the mic:
    i2s_pin_config_t audio_in_pin_config = {
        .bck_io_num = I2S_PIN_NO_CHANGE, // not used
        .ws_io_num = PIN_CLK,  // IO 15 clock pin
        .data_out_num = I2S_PIN_NO_CHANGE, // Not used
        .data_in_num = PIN_DATA   // data pin
    };
    
    err = i2s_set_pin(I2S_PORT, &audio_in_pin_config);
    if (err != ESP_OK) {
      Serial.printf("Failed setting pin: %d\n", err);
      while (true);
    }
}

void print_number(double val, int precision, int x, int y) {
    char buf[16];
    dtostrf(val, 4, precision, buf);
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString(buf, x, y);  
}

void process_samples(void *pvParameters) {  
    while(1){
      size_t num_bytes_read;
      i2s_read(I2S_PORT, 
          (char *)samples, 
          BLOCK_SIZE * 2,
          &num_bytes_read,
          portMAX_DELAY); // no timeout
      if (num_bytes_read > 0) {            
        int samples_read = num_bytes_read / 2;
        total_read += samples_read;
        float sample;
        double rms = 0;
        for(int i=0; i < samples_read; i++) {
          sample = filter.step((float)(samples[i] / 32767));
          rms += samples[i];
        }            
        display_rms = rms;
        display_dbspl = 20 * log10(sqrt(rms / samples_read));
      }
    }
}

void setup(void) {
    M5.begin();
    M5.Power.begin();
    
    // Initialize the I2S peripheral
    init_pdm();
    // Create a task that will read the data
    xTaskCreatePinnedToCore(process_samples, "PDM_reader", 2048, NULL, 1, NULL, 1);

    M5.Lcd.fillScreen(TFT_BLACK);

    M5.Lcd.getSetup(tft_settings);

    analogMeter(tft_settings.tft_height, tft_settings.tft_width, 0, 0);  // Draw analogue meter
    updateTime = millis();  // Next update time
}

void loop() {
    if (updateTime <= millis()) {
        updateTime = millis() + LOOP_PERIOD;

        d += 4;
        if (d >= 360) d = 0;
        
        int val_db = 60.5 + 10.5 * sin((d + 0) * DEGTORAD);
        plotNeedle(tft_settings.tft_height, tft_settings.tft_width,val_db, 0);
    }
}

// #########################################################################
//  Draw the analogue meter on the screen
// #########################################################################
void analogMeter(int width, int height, int pos_x, int pos_y) {
    // Meter outline
    M5.Lcd.fillRect(pos_x, pos_y, width, height, TFT_GREY);
    int border = 3;
    int ticks_width = (width - border * 2) * 0.4;
    int ticks_height = (height - border * 2) / 2;
    int position_midle_analog = width * 0.5;
    int yposition_midle_analog = height * 0.8;
    int long_scale_length = 30;
    int short_scale_length = 16;
    M5.Lcd.fillRect(pos_x + border, pos_y + border, width - border * 2, height - border * 2, TFT_WHITE);

    M5.Lcd.setTextColor(TFT_BLACK);  // Text colour

    // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
    for (int i = -50; i <= 50; i += 5) {
        // Long scale tick length
        int tl = long_scale_length;

        // Coodinates of tick to draw
        float sx    = cos((i - 90) * DEGTORAD);
        float sy    = sin((i - 90) * DEGTORAD);
        uint16_t x0 = sx * (ticks_width + tl) + position_midle_analog;
        uint16_t y0 = sy * (ticks_height + tl) + yposition_midle_analog;
        uint16_t x1 = sx * ticks_width + position_midle_analog;
        uint16_t y1 = sy * ticks_height + yposition_midle_analog;

        // Coordinates of next tick for zone fill
        float sx2 = cos((i + 5 - 90) * DEGTORAD);
        float sy2 = sin((i + 5 - 90) * DEGTORAD);
        int x2    = sx2 * (ticks_width + tl) + position_midle_analog;
        int y2    = sy2 * (ticks_height + tl) + yposition_midle_analog;
        int x3    = sx2 * ticks_width + position_midle_analog;
        int y3    = sy2 * ticks_height + yposition_midle_analog;

        // Green zone limits
        if (i >= 0 && i < 25) {
            M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
            M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
        }

        // Orange zone limits
        if (i >= 25 && i < 50) {
            M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_ORANGE);
            M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_ORANGE);
        }

        // Short scale tick length
        if (i % 25 != 0) tl = short_scale_length;

        // Recalculate coords incase tick lenght changed
        x0 = sx * (ticks_width + tl) + position_midle_analog;
        y0 = sy * (ticks_height + tl) + yposition_midle_analog;
        x1 = sx * ticks_width + position_midle_analog;
        y1 = sy * ticks_height + yposition_midle_analog;

        // Draw tick
        M5.Lcd.drawLine(x0, y0, x1, y1, TFT_BLACK);

        // Check if labels should be drawn, with position tweaks
        if (i % 25 == 0) {
            // Calculate label positions
            x0 = sx * (ticks_width + tl + 10) + position_midle_analog;
            y0 = sy * (ticks_height + tl + 10) + yposition_midle_analog;
            switch (i / 25) {
                case -2:
                    M5.Lcd.drawCentreString("30", x0, y0 - 12, 2);
                    break;
                case -1:
                    M5.Lcd.drawCentreString("50", x0, y0 - 9, 2);
                    break;
                case 0:
                    M5.Lcd.drawCentreString("70", x0, y0 - 6, 2);
                    break;
                case 1:
                    M5.Lcd.drawCentreString("90", x0, y0 - 9, 2);
                    break;
                case 2:
                    M5.Lcd.drawCentreString("110", x0, y0 - 12, 2);
                    break;
            }
        }

        // Now draw the arc of the scale
        sx = cos((i + 5 - 90) * DEGTORAD);
        sy = sin((i + 5 - 90) * DEGTORAD);
        x0 = sx * ticks_width + position_midle_analog;
        y0 = sy * ticks_height + yposition_midle_analog;
        // Draw scale arc, don't draw the last part
        if (i < 50) M5.Lcd.drawLine(x0, y0, x1, y1, TFT_BLACK);
    }
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextDatum(BR_DATUM);
    M5.Lcd.drawString("dB(A)", width - border, height / 2);  
    
    M5.Lcd.setTextFont(4);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString("dB(A)", width / 2, height / 2);
    M5.Lcd.drawRect(pos_x + border, pos_y + border, width - border * 2, height - border * 2, TFT_BLACK);  // Draw bezel line

    plotNeedle(tft_settings.tft_height, tft_settings.tft_width, 0, 0);  // Put meter needle at 0
}

// #########################################################################
// Update needle position
// This function is blocking while needle moves, time depends on ms_delay
// 10ms minimises needle flicker if text is drawn within needle sweep area
// Smaller values OK if text not in sweep area, zero for instant movement but
// does not look realistic... (note: 100 increments for full scale deflection)
// #########################################################################
void plotNeedle(int width, int height, int value, byte ms_delay) {

    int border = 3;
    int ticks_width = (width - border * 2) * 0.4 - 2;
    int ticks_height = (height - border * 2) / 2 - 2;
    int position_midle_analog = width * 0.5;
    int yposition_midle_analog = height * 0.8;
    int long_scale_length = 30;
    int short_scale_length = 16;


    M5.Lcd.setTextColor(TFT_BLACK, TFT_WHITE);
    char buf[40];
    int l = sprintf(buf, "%d RMS", display_rms);
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextDatum(BL_DATUM);
    M5.Lcd.drawString(buf, border * 2, height * 0.6);

    if (value < MIN_VALUE - 10) value = MIN_VALUE - 10;  // Limit value to emulate needle end stops
    if (value > MAX_VALUE + 10) value = MAX_VALUE + 10;

    // Move the needle util new value reached
    while (!(value == old_analog)) {
        if (old_analog < value)
            old_analog++;
        else
            old_analog--;

        if (ms_delay == 0)
            old_analog = value;  // Update immediately id delay is 0

        float sdeg =
            map(old_analog, MIN_VALUE - 10, MAX_VALUE + 10, -150, -30);  // Map value to angle
        // Calcualte tip of needle coords
        float sx = cos(sdeg * DEGTORAD);
        float sy = sin(sdeg * DEGTORAD);

        // Calculate x delta of needle start (does not start at pivot point)
        float tx = tan((sdeg + 90) * DEGTORAD);

        // Erase old needle image
        M5.Lcd.drawLine(position_midle_analog - 1, yposition_midle_analog, osx - 1, osy, TFT_WHITE);
        M5.Lcd.drawLine(position_midle_analog, yposition_midle_analog, osx, osy, TFT_WHITE);
        M5.Lcd.drawLine(position_midle_analog + 1, yposition_midle_analog, osx + 1, osy, TFT_WHITE);

        // Re-plot text under needle
        M5.Lcd.setTextColor(TFT_BLACK);
        M5.Lcd.setTextFont(4);
        M5.Lcd.setTextDatum(MC_DATUM);
        M5.Lcd.drawString("dB(A)", width / 2, height / 2);

        // Store new needle end coords for next erase
        ltx = tx;        
        osx = sx * ticks_width + position_midle_analog;
        osy = sy * ticks_height + yposition_midle_analog;

        // Draw the needle in the new postion, magenta makes needle a bit bolder
        // draws 3 lines to thicken needle
        //M5.Lcd.drawLine(120 + 20 * ltx - 1, 140 - 20, osx - 1, osy, TFT_RED);
        M5.Lcd.drawLine(position_midle_analog - 1, yposition_midle_analog, osx - 1, osy, TFT_RED);
        M5.Lcd.drawLine(position_midle_analog, yposition_midle_analog, osx, osy, TFT_MAGENTA);
        M5.Lcd.drawLine(position_midle_analog + 1, yposition_midle_analog, osx + 1, osy, TFT_RED);

        // Slow needle down slightly as it approaches new postion
        if (abs(old_analog - value) < 10) ms_delay += ms_delay / 5;

        // Wait before next update
        delay(ms_delay);
    }

}
