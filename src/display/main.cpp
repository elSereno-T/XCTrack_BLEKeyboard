#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_GFX.h> 
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>

int discharge = 101;
struct {
    unsigned long now;
} timestamps;
bool blink;

// SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_32);
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
 

void battery(int x0, int soc, int h=32, int w = 16, bool warn=true){
    int y0 = (32-h)/2;
    int h_cap = max(1, h/10);
    int w_cap = (w*2)/3;
    int x0_cap = (w-w_cap)/2 + x0;
    int h_ext = h - h_cap;
    int h_int = h_ext - 2;
    int h_soc = (max(min(soc,100), 0) * h_int) / 100;
    display.drawRect(x0, y0+h_cap, w, h_ext,SSD1306_WHITE);
    display.fillRect(x0_cap,y0,w_cap,h_cap,SSD1306_WHITE);
    display.fillRect(x0+1,y0+h-1-h_soc,w-2,h_soc,SSD1306_WHITE);
    if ((soc < 20) && blink && warn){
        display.drawRect(x0+7, 20, 2,2,SSD1306_WHITE);
        display.drawRect(x0+7,8,2,10,SSD1306_WHITE);
    }

}
void power(int x0, int h = 20, int y0=16){
        int w = h/5;
        for (int sgn = -1; sgn<2; sgn+=2){
            display.fillTriangle(x0-(sgn*0),y0+(sgn*h/2),x0+(sgn*w),y0,x0,y0-(sgn*h/10),SSD1306_WHITE);
        }
}
void phone(bool charging, int x0, int w = 17){
    int r = 3;
    display.drawRoundRect(x0,0,w,32,r,SSD1306_WHITE);
    display.drawLine(x0+4, 29, x0+w-4-1,29,SSD1306_WHITE);
    display.fillCircle(x0+w/2,2,1,SSD1306_WHITE);
    // display.drawRect(x0,0,16,32);
    // display.clearPixel(x0,0);
    // display.clearPixel(x0+15,0);
    // display.clearPixel(x0,31);
    // display.clearPixel(x0+15,31);
    // battery(x0+4, 60, 20,w-8,false);
    if (charging){
        power(x0+w/2);
    } 
    // else {
    // }

    // if (!charging){
    //     int angle = (now/100) % 360;
    //     float local_cos = cos(angle * PI / 180);
    //     float local_sin = sin(angle * PI / 180);
    //     display.fillTriangle(
    //         int(local_cos*6)+x0+8,int(local_sin*6) + 15,
    //         int(local_sin*4)+x0+8,int(local_cos*4) + 15,
    //         -int(local_sin*4)+x0+8,-int(local_cos*4) + 15

    //     );
    // }

}

void camera(int xs, int n, bool connected, bool rec, int soc){
    int x0 = 26*(n/ 2) + xs;
    int y0 = 16*(n%2);

        display.drawRect(x0,y0+6,20,9,SSD1306_WHITE);
        display.drawCircle(x0+5,y0+3,3,SSD1306_WHITE);
        display.drawCircle(x0+13,y0+3,3,SSD1306_WHITE);
        display.drawTriangle(x0+19,y0+10,x0+23,y0+6,x0+23,y0+14,SSD1306_WHITE);
        if (!connected){
            display.drawLine(x0,y0,x0+23,y0+15,SSD1306_WHITE);
            display.drawLine(x0,y0+15,x0+23,y0,SSD1306_WHITE);
        } else{
             int w = max(min(soc,100), 0) * 18 / 100;
            display.fillRect(x0+1,y0+7,w,7,SSD1306_WHITE);if (rec){
            display.fillTriangle(x0+19,y0+10,x0+23,y0+6,x0+23,y0+14,SSD1306_WHITE);
            if (blink) display.fillCircle(x0+5,y0+3,3,SSD1306_WHITE);
            else display.fillCircle(x0+13,y0+3,3,SSD1306_WHITE);
        }
        } 

}

void GPS(int x0, bool powered){
    int r = 7;
    int w = 19;
    display.fillRoundRect(x0,0,w,32,r,SSD1306_WHITE);
    display.fillRect(x0+2,r-1,w-4,32-2*r-4 + 3,SSD1306_BLACK);
    display.fillRect(x0+5,3,w-10,2,SSD1306_BLACK);
    display.fillRect(x0+4,24,w-8,5,SSD1306_BLACK);
    display.fillRect(x0,0,2,r,SSD1306_WHITE);
    // display.setCursor(x0+9,14); display.println('z');
    // display.setCursor(x0+12,7); display.println('z');
    if (powered){
        power(x0+w/2,14,14);
        // display.fillTriangle(x0,5,x0-4,16,x0,16);
        // display.fillTriangle(x0,27,x0+4,16,x0,16);
    } else {
        int i_max = (timestamps.now%2000)/500;
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setFont();
        for (int i=0;i<i_max;i++){
            display.setCursor(x0+3+4*i,15-5*i); display.println('z');
        }
        // display.drawLine(x0-r+2,r-4,x0+r-2,36-r);
        // display.drawLine(x0-r+2,36-r,x0+r-2,r-4);
        // display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
        // display.drawString(x0,16,"z");
        // display.drawString(x0-4,20,"z");
        // display.drawString(x0+4,10,"Z");
    }
}


void setup(){
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    timestamps.now = 0;
}
void loop(){
    timestamps.now = millis();
    blink = (timestamps.now%1000)<500;
    discharge = (timestamps.now/500) % 101;
    
    display.clearDisplay();
    battery(112, 100-discharge);
    phone((timestamps.now%10000)<5000, 90);
    camera(0, 0,true,(timestamps.now%10000)<5000, 100-discharge);
    camera(0, 1,true,((timestamps.now+2000)%10000)<5000, discharge);
    camera(0, 2,false,((timestamps.now+2000)%10000)<5000, discharge);
    camera(0, 3,true,false, discharge);
    
    GPS(64,((timestamps.now+3000)%10000)<5000);
    
    display.display();


    delay(10);
}