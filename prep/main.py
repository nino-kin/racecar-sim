#!/usr/bin/env python3
# -*- coding:utf-8 -*-

def main():
    #シミュレータ準備（変更しない）
    vehicle=Vehicle()
    sim=SimManager()
    sensor_simulation=sensor_simulation_cls()
    #togikai_drive=togikai_drive_cls(vehicle)
    #togikai_ultrasonic=togikai_ultrasonic_cls(vehicle)

    #ultrasonic_RrLH=togikai_ultrasonic_cls(vehicle,35,37)
    #ultrasonic_FrLH=togikai_ultrasonic_cls(vehicle,23,24)
    #ultrasonic_Fr=togikai_ultrasonic_cls(vehicle,15,26)
    #ultrasonic_FrRH=togikai_ultrasonic_cls(vehicle,32,31)
    #ultrasonic_RrRH=togikai_ultrasonic_cls(vehicle,36,38)

    ultrasonic_RrLH=Ultrasonic(vehicle,"RrLH")
    ultrasonic_FrLH=Ultrasonic(vehicle,"FrLH")
    ultrasonic_Fr=Ultrasonic(vehicle,"Fr")
    ultrasonic_FrRH=Ultrasonic(vehicle,"FrRH")
    ultrasonic_RrRH=Ultrasonic(vehicle,"RrRH")

    motor=Motor(vehicle)
    crs=Course()
    vehicle.setCourse(crs)
    sim.addObject(vehicle)
    sim.addObject(crs)
    PWM_PARAM=None
    pwm=None

if __name__ == "__main__":
    main()
