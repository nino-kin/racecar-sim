#!/usr/bin/env python3
# -*- coding:utf-8 -*-

class Motor:
    def __init__(self,vehicle):
        self.vehicle=vehicle
        #self.pwm = Adafruit_PCA9685.PCA9685(address=0x40)
        #self.pwm.set_pwm_freq(60)
        '''
        self.CHANNEL_STEERING = config.CHANNEL_STEERING
        self.CHANNEL_THROTTLE = config.CHANNEL_THROTTLE
        self.STEERING_CENTER_PWM = config.STEERING_CENTER_PWM
        self.STEERING_WIDTH_PWM = config.STEERING_WIDTH_PWM
        self.STEERING_RIGHT_PWM = config.STEERING_RIGHT_PWM
        self.STEERING_LEFT_PWM = config.STEERING_LEFT_PWM
        self.THROTTLE_STOPPED_PWM = config.THROTTLE_STOPPED_PWM
        self.THROTTLE_WIDTH_PWM = config.THROTTLE_WIDTH_PWM
        self.THROTTLE_FORWARD_PWM = config.THROTTLE_FORWARD_PWM
        self.THROTTLE_REVERSE_PWM = config.THROTTLE_REVERSE_PWM
        '''

    def set_throttle_pwm_duty(self, duty):
        self.vehicle.setAccel(duty)
        '''
        if duty >= 0:
            throttle_pwm = int(self.THROTTLE_STOPPED_PWM + (self.THROTTLE_FORWARD_PWM - self.THROTTLE_STOPPED_PWM) * duty / 100)
        else:
            throttle_pwm = int(self.THROTTLE_STOPPED_PWM + (self.THROTTLE_REVERSE_PWM - self.THROTTLE_STOPPED_PWM) * abs(duty) / 100)

        self.pwm.set_pwm(self.CHANNEL_THROTTLE, 0, throttle_pwm)
        '''
        #print(throttle_pwm)

    def set_steer_pwm_duty(self, duty):
        self.vehicle.setSteer(duty)
        '''
        if duty >= 0:
            steer_pwm = int(self.STEERING_CENTER_PWM + (self.STEERING_RIGHT_PWM - self.STEERING_CENTER_PWM) * duty / 100)
        else:
            steer_pwm = int(self.STEERING_CENTER_PWM + (self.STEERING_LEFT_PWM - self.STEERING_CENTER_PWM) * abs(duty) / 100)
        if steer_pwm > 450 or steer_pwm < 260:
            print ("Caution!, please set 260~450 not to break!\n")
        else: self.pwm.set_pwm(self.CHANNEL_STEERING, 0, steer_pwm)
        '''
        #print(steer_pwm)
