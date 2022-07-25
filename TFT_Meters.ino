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

#define TFT_GREY 0x5AEB

#define LOOP_PERIOD 35  // Display updates every 35 ms
#define MIN_VALUE 30
#define MAX_VALUE 110

#define DEGTORAD 0.0174532925 // degree to radian

float ltx    = 0;               // Saved x coord of bottom of needle
uint16_t osx = 120, osy = 120;  // Saved x & y coords
uint32_t updateTime = 0;        // time for next update

int old_analog  = -999;  // Value last displayed

int d            = 0;

void print_number(double val, int precision, int x, int y) {
    char buf[16];
    dtostrf(val, 4, precision, buf);
    M5.Lcd.setTextFont(2);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString(buf, x, y);  
}

void setup(void) {
    M5.begin();
    M5.Power.begin();
    
    M5.Lcd.fillScreen(TFT_BLACK);

    setup_t tft_settings;
    M5.Lcd.getSetup(tft_settings);

    //print_number(tft_settings.tft_width, 0, 0, 150);
    //print_number(tft_settings.tft_height, 0, 60, 150);
    

    analogMeter(tft_settings.tft_height, tft_settings.tft_width, 0, 0);  // Draw analogue meter
    updateTime = millis();  // Next update time
}

void loop() {
    if (updateTime <= millis()) {
        updateTime = millis() + LOOP_PERIOD;

        d += 4;
        if (d >= 360) d = 0;
        
        int val_db = 40 + 8 * sin((d + 0) * DEGTORAD);
        plotNeedle(int, 0);
    }
}

// #########################################################################
//  Draw the analogue meter on the screen
// #########################################################################
void analogMeter(int width, int height, int pos_x, int pos_y) {
    // Meter outline
    M5.Lcd.fillRect(pos_x, pos_y, width, height, TFT_GREY);
    int border = 3;
    M5.Lcd.fillRect(pos_x + border, pos_y + border, width - border * 2, height - border * 2, TFT_WHITE);

    M5.Lcd.setTextColor(TFT_BLACK);  // Text colour

    // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
    for (int i = -50; i < 51; i += 5) {
        // Long scale tick length
        int tl = 15;

        // Coodinates of tick to draw
        float sx    = cos((i - 90) * DEGTORAD);
        float sy    = sin((i - 90) * DEGTORAD);
        uint16_t x0 = sx * (100 + tl) + 120;
        uint16_t y0 = sy * (100 + tl) + 140;
        uint16_t x1 = sx * 100 + 120;
        uint16_t y1 = sy * 100 + 140;

        // Coordinates of next tick for zone fill
        float sx2 = cos((i + 5 - 90) * DEGTORAD);
        float sy2 = sin((i + 5 - 90) * DEGTORAD);
        int x2    = sx2 * (100 + tl) + 120;
        int y2    = sy2 * (100 + tl) + 140;
        int x3    = sx2 * 100 + 120;
        int y3    = sy2 * 100 + 140;

        // Yellow zone limits
        // if (i >= -50 && i < 0) {
        //  M5.Lcd.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
        //  M5.Lcd.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
        //}

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
        if (i % 25 != 0) tl = 8;

        // Recalculate coords incase tick lenght changed
        x0 = sx * (100 + tl) + 120;
        y0 = sy * (100 + tl) + 140;
        x1 = sx * 100 + 120;
        y1 = sy * 100 + 140;

        // Draw tick
        M5.Lcd.drawLine(x0, y0, x1, y1, TFT_BLACK);

        // Check if labels should be drawn, with position tweaks
        if (i % 25 == 0) {
            // Calculate label positions
            x0 = sx * (100 + tl + 10) + 120;
            y0 = sy * (100 + tl + 10) + 140;
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
        x0 = sx * 100 + 120;
        y0 = sy * 100 + 140;
        // Draw scale arc, don't draw the last part
        if (i < 50) M5.Lcd.drawLine(x0, y0, x1, y1, TFT_BLACK);
    }

    M5.Lcd.drawString("dB(A)", 5 + 230 - 40, 119 - 20,
                      2);                        // Units at bottom right
    M5.Lcd.drawCentreString("dB(A)", 120, 70, 4);  // Comment out to avoid font 4
    M5.Lcd.drawRect(5, 3, 230, 119, TFT_BLACK);  // Draw bezel line

    plotNeedle(0, 0);  // Put meter needle at 0
}

// #########################################################################
// Update needle position
// This function is blocking while needle moves, time depends on ms_delay
// 10ms minimises needle flicker if text is drawn within needle sweep area
// Smaller values OK if text not in sweep area, zero for instant movement but
// does not look realistic... (note: 100 increments for full scale deflection)
// #########################################################################
void plotNeedle(int value, byte ms_delay) {
    M5.Lcd.setTextColor(TFT_BLACK, TFT_WHITE);
    char buf[8];
    dtostrf(value, 4, 0, buf);
    M5.Lcd.drawRightString(buf, 40, 119 - 20, 2);

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
        M5.Lcd.drawLine(120 + 20 * ltx - 1, 140 - 20, osx - 1, osy, TFT_WHITE);
        M5.Lcd.drawLine(120 + 20 * ltx, 140 - 20, osx, osy, TFT_WHITE);
        M5.Lcd.drawLine(120 + 20 * ltx + 1, 140 - 20, osx + 1, osy, TFT_WHITE);

        // Re-plot text under needle
        M5.Lcd.setTextColor(TFT_BLACK);
        M5.Lcd.drawCentreString("dB(A)", 120, 70,
                                4);  // // Comment out to avoid font 4

        // Store new needle end coords for next erase
        ltx = tx;
        osx = sx * 98 + 120;
        osy = sy * 98 + 140;

        // Draw the needle in the new postion, magenta makes needle a bit bolder
        // draws 3 lines to thicken needle
        M5.Lcd.drawLine(120 + 20 * ltx - 1, 140 - 20, osx - 1, osy, TFT_RED);
        M5.Lcd.drawLine(120 + 20 * ltx, 140 - 20, osx, osy, TFT_MAGENTA);
        M5.Lcd.drawLine(120 + 20 * ltx + 1, 140 - 20, osx + 1, osy, TFT_RED);

        // Slow needle down slightly as it approaches new postion
        if (abs(old_analog - value) < 10) ms_delay += ms_delay / 5;

        // Wait before next update
        delay(ms_delay);
    }

}