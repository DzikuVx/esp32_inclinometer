#include "oled_display.h"
#include "Arduino.h"
#include "math.h"

OledDisplay::OledDisplay(SSD1306 *display) {
    _display = display;
}

void OledDisplay::init() {
    _display->init();
    // _display->flipScreenVertically();
    _display->setFont(ArialMT_Plain_10);
}

void OledDisplay::loop() {
    page();
}

void OledDisplay::setPage(uint8_t page) {
    _page = page;
}

void OledDisplay::page() {

    static uint32_t lastUpdate = 0;

    _forceDisplay = false;
    switch (_page) {
        
        case OLED_PAGE_ANGLE:
            renderPageStatus();
            break;
    }

    lastUpdate = millis();
}

void OledDisplay::renderPageStatus() {

    _display->clear();

    String val;

    _display->setFont(ArialMT_Plain_10);
    
    //Gyro calibration
    if (imu.gyroCalibration.state == CALIBRATION_DONE) {
        val = "Gyro: OK";
    } else {
        val = "Gyro: Cal";
    }
    _display->drawString(0, 0, val);

    _display->setFont(ArialMT_Plain_16);
    _display->drawString(0, 20, "R: " + String(imu.angle.x, 1));
    _display->drawString(64, 20, "P: " + String(imu.angle.y, 1));

    _display->display();
}