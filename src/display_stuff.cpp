#include <Arduino.h> 
#include <display_stuff.h>


void turnDisplayOff(Adafruit_SSD1306 &display){
    

    display.clearDisplay();
    display.ssd1306_command(SSD1306_DISPLAYOFF);
}
