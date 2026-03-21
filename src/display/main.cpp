#include <Arduino.h>

#include <Wire.h>
#include <SSD1306Wire.h>

int discharge = 101;
unsigned long now;
bool blink;

SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_32);

void battery(int x0, int soc){
    int h = max(min(soc,100), 0) * 28 / 100;
    display.drawRect(x0, 3, 16, 29);
    display.fillRect(x0+4,0,8,4);
    display.fillRect(x0+1,31-h,14,h);
    if ((soc < 20) && blink){
        display.drawRect(x0+7, 20, 2,2);
        display.drawRect(x0+7,8,2,10);
    }

}
void phone(bool charging, int x0){
    display.drawRect(x0,0,16,32);
    display.clearPixel(x0,0);
    display.clearPixel(x0+15,0);
    display.clearPixel(x0,31);
    display.clearPixel(x0+15,31);
    if (charging && blink){

        display.fillTriangle(x0+8,5,x0+4,16,x0+8,16);
        display.fillTriangle(x0+8,27,x0+12,16,x0+8,16);

    }
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

        display.drawRect(x0,y0+6,20,9);
        display.drawCircle(x0+5,y0+3,3);
        display.drawCircle(x0+13,y0+3,3);
        display.drawTriangle(x0+19,y0+10,x0+23,y0+6,x0+23,y0+14);
        if (!connected){
            display.drawLine(x0,y0,x0+23,y0+15);
            display.drawLine(x0,y0+15,x0+23,y0);
        } else{
             int w = max(min(soc,100), 0) * 18 / 100;
            display.fillRect(x0+1,y0+7,w,7);if (rec){
            display.fillTriangle(x0+19,y0+10,x0+23,y0+6,x0+23,y0+14);
            if (blink) display.fillCircle(x0+5,y0+3,3);
            else display.fillCircle(x0+13,y0+3,3);
        }
        } 

}

void GPS(int x0, bool powered){
    int r = 10;
    display.fillRect(x0,r-1,2*r,32-2*r);
    display.fillRect(x0,0,2,r);
    x0 += r;
    display.fillCircle(x0,r-1,r);
    display.fillCircle(x0,32-r,r);
    display.setColor(BLACK);
    display.fillRect(x0-r+2,r-4,2*r-4,32-2*r+7);
    display.setColor(WHITE);
    if (powered){
        // display.fillTriangle(x0,5,x0-4,16,x0,16);
        // display.fillTriangle(x0,27,x0+4,16,x0,16);
    } else {
        display.drawLine(x0-r+2,r-4,x0+r-2,36-r);
        display.drawLine(x0-r+2,36-r,x0+r-2,r-4);
    }
}


void setup(){
    display.init();
    now = 0;
}
void loop(){
    now = millis();
    blink = (now%1000)<500;
    discharge = (now/500) % 101;
    
    display.clear();
    battery(112, 100-discharge);
    phone((now%10000)<5000, 90);
    camera(0, 0,true,(now%10000)<5000, 100-discharge);
    camera(0, 1,true,((now+2000)%10000)<5000, discharge);
    camera(0, 2,false,((now+2000)%10000)<5000, discharge);
    camera(0, 3,true,false, discharge);
    
    GPS(64,((now+3000)%10000)<5000);
    
    display.display();


    delay(10);
}