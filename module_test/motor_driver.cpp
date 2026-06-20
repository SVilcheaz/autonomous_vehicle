#include <lgpio.h>
#include <cstdio>
#include <algorithm>
#include <stdexcept>

static constexpr int CHIP = 4;

struct MotorDriver {
    int ena, in1, in2;
    int enb, in3, in4;
    int handle;

    MotorDriver(int ena, int in1, int in2,
                int enb, int in3, int in4)
        : ena(ena), in1(in1), in2(in2),
          enb(enb), in3(in3), in4(in4)
    {
        handle = lgGpiochipOpen(CHIP);
        if (handle < 0) throw std::runtime_error("Cannot open gpiochip4");

        for (int pin : {ena, in1, in2, enb, in3, in4})
            lgGpioClaimOutput(handle, 0, pin, 0);
    }

    ~MotorDriver() {
        stop();
        lgGpiochipClose(handle);
    }

    void setMotorFront(int speed) { setMotor(in1, in2, ena, speed); }
    void setMotorRear(int speed) { setMotor(in3, in4, enb, speed); }

    void stop() {
        setMotorFront(0);
        setMotorRear(0);
    }

private:
    void setMotor(int pinA, int pinB, int pwmPin, int speed) {
        speed = std::clamp(speed, -100, 100);
        if (speed > 0) {
            std::printf("SPEED POSITIVE: Motor on pins %d/%d at speed %d\n", pinA, pinB, speed);
            lgGpioWrite(handle, pinA, 1);
            lgGpioWrite(handle, pinB, 0);
        } else if (speed < 0) {
            std::printf("SPEED NEGATIVE: Motor on pins %d/%d at speed %d\n", pinA, pinB, speed);
            lgGpioWrite(handle, pinA, 0);
            lgGpioWrite(handle, pinB, 1);
            speed = -speed;
        } else {
            std::printf("SPEED ZERO: Motor on pins %d/%d at speed %d\n", pinA, pinB, speed);    
            lgGpioWrite(handle, pinA, 0);
            lgGpioWrite(handle, pinB, 0);
        }
        std::printf("Setting PWM on pin %d to speed %d\n", pwmPin, speed);
        lgTxPwm(handle, pwmPin, 1000, speed, 0, 0);
    }
};

int main() {
    std::printf("Opening GPIO...\n");

    MotorDriver left (19, 6, 5,
                      13, 20, 16);
    MotorDriver right(18, 27, 22,
                      12, 23, 24);

    std::printf("Forward 10%% for 2s...\n");
    left.setMotorFront(-50); //moves back at +50
    left.setMotorRear(50); //moves back at -50
    right.setMotorFront(50); //moves front at +50
    right.setMotorRear(-50); //moves front at -50
    lguSleep(3.0);

    std::printf("Tank turn left 20%% for 1s...\n");
    // left.setMotorFront(-50);
    // left.setMotorRear(-50);
    // right.setMotorFront(50);
    // right.setMotorRear(50);
    // lguSleep(10.0);

    std::printf("Stopping all motors.\n");
    left.stop();
    right.stop();

    return 0;
}