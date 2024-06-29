#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from config import config

class Ultrasonic:
    def __init__(self, vehicle, name):
        # 超音波発信/受信用のGPiOピン番号
        self.name = name
        self.trig = config.ultrasonics_dict_trig[name]
        self.echo = config.ultrasonics_dict_echo[name]
        self.records = np.zeros(config.ultrasonics_Nrecords)
        self.dis = 0
        self.vehicle = vehicle
        #self.ultrasonic=togikai_ultrasonic_cls(vehicle,self.trig,self.echo)
    # 障害物センサ測定関数
    def Mesure(self):
        self.dis = 0
        sigoff = 0
        sigon = 0
        '''
        GPIO.output(self.trig,GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig,GPIO.LOW)
        starttime=time.perf_counter()
        while(GPIO.input(self.echo)==GPIO.LOW):
            sigoff=time.perf_counter()
            if sigoff - starttime > 0.02:
            #     print("break1")
                break
        while(GPIO.input(self.echo)==GPIO.HIGH):
            sigon=time.perf_counter()
            if sigon - sigoff > 0.02:
            #     print("break2")
                break
        # time * sound speed / 2(round trip)
        d = (sigon - sigoff)*340000/2
        # 2m以上は無視
        '''
        d = self.vehicle.measureDist(self.trig,self.echo)*100
        #self.ultrasonic.Mesure()
        if d > 2000:
            self.dis = 2000
            #print("more than 2m!")
        # 負値のノイズの場合は一つ前のデータに置き換え
        elif d < 0:
            print("@",self.name,", a noise occureed, use the last value")
            self.dis = self.records[0]
            print(self.records)
        else:
            self.dis = d
        # 過去の超音波センサの値を記録の一番前に挿入し、最後を消す
        self.records = np.insert(self.records, 0, self.dis)
        self.records = np.delete(self.records,-1)
        return self.dis
