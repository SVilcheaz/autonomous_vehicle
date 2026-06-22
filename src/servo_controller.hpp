#pragma once

class ServoController {
public:
    explicit ServoController(int gpio_pin);
    ~ServoController();

    void setAngle(double angle);

private:
    int handle_;
    int pin_;
};