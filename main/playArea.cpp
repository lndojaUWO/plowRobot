#include "PlayArea.h"

PlayArea::PlayArea(int xMax, int yMax) {
    xPos = 0;
    yPos = 0;
    angle = 0;
    xMax = xMax;
    yMax = yMax;
}

void PlayArea::setXPos(int x) {
    xPos = x;
}

void PlayArea::setYPos(int y) {
    yPos = y;
}

void PlayArea::setAngle(int a) {
    a > 180 ? a -= 180 : a;
    angle = a;
}

void PlayArea::forward(int distance) {
    xPos += distance * cos(angle);
    yPos += distance * sin(angle);
}

void PlayArea::backward(int distance) {
    xPos -= distance * cos(angle);
    yPos -= distance * sin(angle);
}

int PlayArea::getXPos() {
    return xPos;
}

int PlayArea::getYPos() {
    return yPos;
}

int PlayArea::getAngle() {
    return angle;
}

int PlayArea::getXMax() {
    return xMax;
}

int PlayArea::getYMax() {
    return yMax;
}
