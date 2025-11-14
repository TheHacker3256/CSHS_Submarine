#ifndef DIFFDRIVE_ARDUINO_PIGPIO_HPP
#define DIFFDRIVE_ARDUINO_PIGPIO_HPP

#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <time.h>

class PiGPIO{
  private:
    int front_left_drone_gpio = 0;
    int front_right_drone_gpio = 1;
    int rear_left_drone_gpio = 2;
    int rear_right_drone_gpio = 3;
    int forward_left_motor_gpio = 4;
    int forward_left_motor_gpio = 5;

  public:
    void initGPIO(){
      int cfg = gpioCfgGetInternals();
      cfg |= PI_CFG_NOSIGHANDLER;  // (1<<10)
      gpioCfgSetInternals(cfg);
      int status = gpioInitialise();
    }

    void deactivateGPIO(){
      gpioTerminate();
    }

    void initMotors(){
      gpioPWM(front_left_drone_gpio, 255);
      gpioPWM(front_right_drone_gpio, 255);
      gpioPWM(rear_left_drone_gpio, 255);
      gpioPWM(rear_right_drone_gpio, 255);
      gpioPWM(forward_left_drone_gpio, 255);
      gpioPWM(forward_right_motor_gpio, 255);

      sleep(1);

      gpioPWM(front_left_drone_gpio, 0);
      gpioPWM(front_right_drone_gpio, 0);
      gpioPWM(rear_left_drone_gpio, 0);
      gpioPWM(rear_right_drone_gpio, 0);
      gpioPWM(forward_left_drone_gpio, 0);
      gpioPWM(forward_right_motor_gpio, 0);

    }

    void setMotorSpeeds(double fldC, double frdC, double rldC, double rrdC, double flmC, double frmC){
      gpioPWM(front_left_drone_gpio, fldC*255);
      gpioPWM(front_right_drone_gpio, frdC*255);
      gpioPWM(rear_left_drone_gpio, rldC*255);
      gpioPWM(rear_right_drone_gpio, rrdC*255);
      gpioPWM(forward_left_drone_gpio, flmC*255);
      gpioPWM(forward_right_motor_gpio, frmC*255);
    }
}