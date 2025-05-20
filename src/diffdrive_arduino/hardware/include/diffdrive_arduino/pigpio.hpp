#ifndef DIFFDRIVE_ARDUINO_PIGPIO_HPP
#define DIFFDRIVE_ARDUINO_PIGPIO_HPP

#include <iostream>
#include <pigpio.h>
#include <unistd.h>


class PiGPIO{
  public:
    void initGPIO(){

    }

    void initMotors(){

    }

    void motorStartup(){

    }

    void setMotorSpeeds(double fldC, double frdC, double rldC, double rrdC, double flmC, double frmC){
      
    }

  private:
    int front_left_drone_gpio = 0;
    int front_right_drone_gpio = 0;
    int rear_left_drone_gpio = 0;
    int rear_right_drone_gpio = 0;
    int forward_left_motor_gpio = 0;
    int forward_left_motor_gpio = 0;


}