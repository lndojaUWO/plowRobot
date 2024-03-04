#ifndef PlayArea_h
#define PlayArea_h
#include <Arduino.h>

class PlayArea {
    private:
        int xPos;
        int yPos;
        int angle;
        int xMax;
        int yMax;
    
    public:
        PlayArea(int xMax, int yMax);
        void setXPos(int x);
        void setYPos(int y);
        void setAngle(int a);
        void forward(int distance);
        void backward(int distance);
        int getXPos();
        int getYPos();
        int getAngle();
        int getXMax();
        int getYMax();
};

#endif