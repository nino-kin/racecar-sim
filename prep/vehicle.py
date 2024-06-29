#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
from operator import indexOf
import time
import matplotlib
matplotlib.use('nbagg')
import matplotlib.animation as anm
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib import rc
from IPython.display import HTML
%matplotlib inline

RAD_PER_DEGREE=3.14/180
eps=1E-5
epsilon_steer_angle=0.01

# 車両クラス
class Vehicle:
    def __init__(self,x=-3,y=0.75,angle=180):
        self.initialPos=np.array([x,y])

        self.pos=np.array([x,y]) #m
        self.vel=0#np.array([0,0]) #m/s
        self.angle_deg=angle
        self.steer_angle_deg=0 #deg

        #指示値
        self.accOrder=0 #%
        self.steerOrder=0 #%

        #センサ位置、角度
        self.sensor_local_pos={"F":np.array([0.14,0.0]),
                                "FR":np.array([0.13,-0.045]),
                                "FL":np.array([0.13,0.045]),
                                "RR":np.array([0.09,-0.045]),
                                "RL":np.array([0.09,0.045])}
        self.sensor_local_angle_deg={"F":0,
                                    "FR":-45,
                                    "FL": 45,
                                    "RR":-90,
                                    "RL": 90}
        self.sensor_measure_time=0.020


        #速度による舵角減少のマップ
        self.steer_map_vel=np.array([0.0,1.4])
        self.steer_map=np.array([1.0,0.5])

        #速度と指示値による加速度変化のマップ
        self.acceleration_map_duty=np.array([-100,-60,-20,0,20,40,60,80,100])
        self.acceleration_map_vel=np.array([-1.2,-1.0,-0.8,-0.6,-0.4,-0.2,-0.05,0.0,0.05,0.2,0.4,0.6,0.8,1.0,1.2,1.4])

        '''#measured
        self.acceleration_map=np.array([
                        [0.000,0.000,0.000,0.000,0.000,0.000,0.000],
                        [0.000,0.000,0.000,0.000,0.000,0.000,0.000],
                        [0.398,0.398,0.089,0.000,0.000,0.000,0.000],
                        [1.051,1.051,0.687,0.197,0.349,0.000,0.000],
                        [1.200,1.200,1.724,0.679,0.244,0.000,0.000],
                        [1.672,1.994,2.120,1.460,1.696,1.867,0.000]
                        ])
        '''

        self.acceleration_map=np.array([
                        [ 0.100,-0.758,-1.600,-1.884,-1.884,-0.758,-0.758,-0.758,-0.758,-0.758,-0.758,-0.758,-0.758,-0.800,-0.820,-0.840],
                        [ 0.100, 0.100, 0.750,-1.484,-1.484,-0.514,-0.514,-0.514,-0.514,-0.514,-0.580,-0.675,-0.750,-0.800,-0.820,-0.840],
                        [ 0.100, 0.100, 0.100, 0.100,-0.205,-0.333,-0.333,-0.333,-0.333,-0.514,-0.580,-0.675,-0.750,-0.800,-0.820,-0.840],
                        [ 0.820, 0.800, 0.750, 0.675, 0.580, 0.514, 0.000, 0.000, 0.000,-0.514,-0.580,-0.675,-0.750,-0.800,-0.820,-0.840],
                        [ 0.820, 0.800, 0.750, 0.675, 0.580, 0.514, 0.000, 0.000, 0.000,-0.514,-0.580,-0.675,-0.750,-0.800,-0.820,-0.840],
                        [ 0.398, 0.398, 0.398, 0.398, 0.398, 0.398, 0.398, 0.398, 0.398, 0.398, 0.089,-0.500,-0.100,-0.100,-0.100,-0.100],
                        [ 1.051, 1.051, 1.051, 1.051, 1.051, 1.051, 1.051, 1.051, 1.051, 1.051, 0.687, 0.197, 0.349,-0.100,-0.100,-0.100],
                        [ 1.200, 1.200, 1.200, 1.200, 1.200, 1.200, 1.200, 1.200, 1.200, 1.200, 1.524, 1.724, 1.524, 0.679,-0.100,-0.100],
                        [ 1.672, 1.672, 1.672, 1.672, 1.672, 1.672, 1.672, 1.672, 1.672, 1.672, 1.994, 2.120, 1.460, 1.696, 0.679,-0.100]
                        ])

        self.sensor_global_pos={}
        self.detected_pos={}
        self.dist={"F":0,
                    "FR":0,
                    "FL":0,
                    "RR":0,
                    "RL":0}
        self.course=None

        #車両のホイールベースや最大舵角
        self.wheel_base=0.170
        self.max_steer_angle=18.5#18.5

        #車両の外形（各辺の後輪軸中心からの距離）
        self.measurements_ext_back=0.06
        self.measurements_ext_front=0.23
        self.measurements_ext_left=0.07
        self.measurements_ext_right=0.07
        #タイヤの大きさ（描画用）
        self.measurements_tire_radius=0.0275
        self.measurements_tire_width=0.025

        self.shape_ext=np.array([[-self.measurements_ext_back,-self.measurements_ext_left],
                                [-self.measurements_ext_back,self.measurements_ext_right,],
                                [self.measurements_ext_front,self.measurements_ext_right],
                                [self.measurements_ext_front,-self.measurements_ext_left],
                                [-self.measurements_ext_back,-self.measurements_ext_left]])

        self.shape_tire=np.array([[-self.measurements_tire_radius,-self.measurements_tire_width/2],
                                [-self.measurements_tire_radius,self.measurements_tire_width/2],
                                [self.measurements_tire_radius,self.measurements_tire_width/2],
                                [self.measurements_tire_radius,-self.measurements_tire_width/2],
                                [-self.measurements_tire_radius,-self.measurements_tire_width/2]])

        self.local_tire_pos_fr=np.array([[self.wheel_base,-self.measurements_ext_right+self.measurements_tire_width/2]])
        self.local_tire_pos_fl=np.array([[self.wheel_base,self.measurements_ext_left-self.measurements_tire_width/2]])
        self.local_tire_pos_rr=np.array([[0,-self.measurements_ext_right+self.measurements_tire_width/2]])
        self.local_tire_pos_rl=np.array([[0,self.measurements_ext_left-self.measurements_tire_width/2]])

    def initializeState(self):
        self.pos[:]=self.initialPos
        self.vel=0#np.array([0,0]) #m/s

    def setSensorPose(self,name,pos,angle):
        self.sensor_local_pos[name]=np.array(pos)
        self.sensor_local_angle_deg[name]=angle
    def setPose(self,x,y,angle):
        self.initialPos=np.array([x,y]) #m
        self.pos[:]=self.initialPos #m
        self.angle_deg=angle
    def setCourse(self,course):
        self.course=course
    def start(self):
        pass
    def proceedTime(self,elapsed_time):

        #舵角の計算（速度で減衰）
        self.steer_angle_deg=self.max_steer_angle*(self.steerOrder/100.0)*common.fetch1DMap(abs(self.vel),self.steer_map_vel,self.steer_map)
        self.steer_angle_rad=self.steer_angle_deg*RAD_PER_DEGREE

        #速度計算
        curAcc=common.fetch2DMapSimple(np.array([self.accOrder,self.vel]),self.acceleration_map_duty,self.acceleration_map_vel,self.acceleration_map)
        move_dist=self.vel*elapsed_time+curAcc*(elapsed_time**2)/2.0

        #車両の進行を計算
        if abs(self.steer_angle_deg)>epsilon_steer_angle:
            vehicle_pos_turn_center_dist=self.wheel_base/abs(np.tan(self.steer_angle_rad))

            if self.steer_angle_deg<0:
                vehicle_to_turn_cneter_angle_rad=(self.angle_deg-90)*RAD_PER_DEGREE
                turn_cneter_to_vehicle_angle_deg=self.angle_deg+90
            else:
                vehicle_to_turn_cneter_angle_rad=(self.angle_deg+90)*RAD_PER_DEGREE
                turn_cneter_to_vehicle_angle_deg=self.angle_deg-90
            turn_center=self.pos + vehicle_pos_turn_center_dist * np.array([np.cos(vehicle_to_turn_cneter_angle_rad),np.sin(vehicle_to_turn_cneter_angle_rad)])


            circumference=2*vehicle_pos_turn_center_dist*np.pi
            move_angle_deg=(360*move_dist/circumference)*np.sign(self.steer_angle_deg)

            turn_center_to_next_vehicle_angle_rad=(turn_cneter_to_vehicle_angle_deg+move_angle_deg)*RAD_PER_DEGREE
            next_pos=turn_center + vehicle_pos_turn_center_dist * np.array([np.cos(turn_center_to_next_vehicle_angle_rad),np.sin(turn_center_to_next_vehicle_angle_rad)])
            next_angle_deg=self.angle_deg+move_angle_deg

        else:
            angle_rad=self.angle_deg*RAD_PER_DEGREE
            next_pos=self.pos + move_dist * np.array([np.cos(angle_rad),np.sin(angle_rad)])
            next_angle_deg=self.angle_deg

        #加速を計算
        next_vel=self.vel+curAcc*elapsed_time

        self.pos=next_pos
        self.vel=next_vel
        self.angle_deg=next_angle_deg

        #センサ距離を計算(表示用)
        self.measureDist(15,26)
        self.measureDist(13,24)
        self.measureDist(32,31)
        self.measureDist(35,37)
        self.measureDist(36,38)

    def setAccel(self,duty):
        self.accOrder=duty
        #self.acc=0.01*acc#np.array([0,0])
    def setSteer(self,steer):
        self.steerOrder=steer
        #self.steer_angle_deg=self.max_steer_angle*steer/100
        #self.steer_angle_rad=self.steer_angle_deg*RAD_PER_DEGREE
    def measureDist(self,trig,echo):
        if trig==15 and echo==26: #F
            sensor="F"
        elif trig==13 and echo==24: #FL
            sensor="FL"
        elif trig==32 and echo==31: #FR
            sensor="FR"
        elif trig==35 and echo==37: #RL
            sensor="RL"
        elif trig==36 and echo==38: #RR
            sensor="RR"
        else:
            print("no sensor in trig:%d echo:%d" % (trig,echo))
            return 0
        self.sensor_global_pos[sensor]=self.pos+common.rorate(self.sensor_local_pos[sensor],self.angle_deg)
        sensor_global_angle_deg=common.normalize_angle_deg(self.angle_deg+self.sensor_local_angle_deg[sensor])
        dist,pos_tmp=self.course.measureDist(self.sensor_global_pos[sensor],sensor_global_angle_deg)

        self.detected_pos[sensor]=pos_tmp
        self.dist[sensor]=dist

        return dist

    def getDrawInfo(self):
        return {"pos":self.pos,
                "angle_deg":self.angle_deg,
                "steer_angle_deg":self.steer_angle_deg,
                "dist_F":self.dist["F"],
                "dist_FL":self.dist["FL"],
                "dist_FR":self.dist["FR"],
                "dist_RL":self.dist["RL"],
                "dist_RR":self.dist["RR"],}
    def abort(self):
        result=self.checkCourseCrossing()
        if result:
            print("crash!!!")
        return result
    def checkCourseCrossing(self):
        shape_ext_global=common.rotatem(self.shape_ext,self.angle_deg)+np.reshape(self.pos,(-1,2))

        j1=self.course.judge_wall_crossing(shape_ext_global[0,0],shape_ext_global[0,1],shape_ext_global[1,0],shape_ext_global[1,1])
        j2=self.course.judge_wall_crossing(shape_ext_global[1,0],shape_ext_global[1,1],shape_ext_global[2,0],shape_ext_global[2,1])
        j3=self.course.judge_wall_crossing(shape_ext_global[2,0],shape_ext_global[2,1],shape_ext_global[3,0],shape_ext_global[3,1])
        j4=self.course.judge_wall_crossing(shape_ext_global[3,0],shape_ext_global[3,1],shape_ext_global[4,0],shape_ext_global[4,1])
        return j1 or j2 or j3 or j4
    def draw(self,ax,drawInfo=None):
        if drawInfo==None:
            drawInfo=self.getDrawInfo()
        pos=drawInfo["pos"]
        angle_deg=drawInfo["angle_deg"]
        steer_angle_deg=drawInfo["steer_angle_deg"]
        angle_rad=angle_deg*RAD_PER_DEGREE


        #センサー
        for sensor in ["F","FL","FR","RL","RR"]:
            dist=drawInfo["dist_"+sensor]
            sensor_global_pos=pos+common.rorate(self.sensor_local_pos[sensor],angle_deg)
            sensor_global_angle_deg=common.normalize_angle_deg(angle_deg+self.sensor_local_angle_deg[sensor])
            sensor_global_angle_deg_r=(sensor_global_angle_deg-20)%360.0
            sensor_global_angle_deg_l=(sensor_global_angle_deg+20)%360.0

            p=patches.Wedge([sensor_global_pos[0],sensor_global_pos[1]],dist,theta1=sensor_global_angle_deg_r,theta2=sensor_global_angle_deg_l,color="#ffaaaa")
            ax.add_patch(p)

        #車両
        ax.plot([pos[0]],[pos[1]],"bo")
        ax.plot([pos[0],pos[0]+0.2*np.cos(angle_rad)],[pos[1],pos[1]+0.2*np.sin(angle_rad)],"b-")

        shape_ext_global=common.rotatem(self.shape_ext,angle_deg)+np.reshape(pos,(-1,2))
        ax.plot(shape_ext_global[:,0],shape_ext_global[:,1],"m")

        #タイヤ
        shape_tire_rotate=common.rotatem(self.shape_tire,steer_angle_deg)
        pos_1x2=np.reshape(pos,(1,2))
        shape_tire_fl=common.rotatem(self.local_tire_pos_fl+shape_tire_rotate,angle_deg)+pos_1x2
        shape_tire_fr=common.rotatem(self.local_tire_pos_fr+shape_tire_rotate,angle_deg)+pos_1x2
        shape_tire_rl=common.rotatem(self.local_tire_pos_rl+self.shape_tire,angle_deg)+pos_1x2
        shape_tire_rr=common.rotatem(self.local_tire_pos_rr+self.shape_tire,angle_deg)+pos_1x2
        ax.plot(shape_tire_fl[:,0],shape_tire_fl[:,1],"g")
        ax.plot(shape_tire_fr[:,0],shape_tire_fr[:,1],"g")
        ax.plot(shape_tire_rl[:,0],shape_tire_rl[:,1],"g")
        ax.plot(shape_tire_rr[:,0],shape_tire_rr[:,1],"g")

        #衝突
        j1=self.course.judge_wall_crossing(shape_ext_global[0,0],shape_ext_global[0,1],shape_ext_global[1,0],shape_ext_global[1,1])
        j2=self.course.judge_wall_crossing(shape_ext_global[1,0],shape_ext_global[1,1],shape_ext_global[2,0],shape_ext_global[2,1])
        j3=self.course.judge_wall_crossing(shape_ext_global[2,0],shape_ext_global[2,1],shape_ext_global[3,0],shape_ext_global[3,1])
        j4=self.course.judge_wall_crossing(shape_ext_global[3,0],shape_ext_global[3,1],shape_ext_global[4,0],shape_ext_global[4,1])
        if j1 or j2 or j3 or j4:
            ax.plot(shape_ext_global[:,0],shape_ext_global[:,1],"r-o")
#togikai_drive_cls模擬
class togikai_drive_cls:
    def __init__(self,vehi):
        self.vehicle=vehi
    def ReadPWMPARAM(self,pwm):
        #path = 'alignment_parameter.txt'
        #with open(path) as f:
        #    l = f.readlines()
        #    PWM_PARAM = ([int(l[2]),int(l[4]),int(l[6])],[int(l[9]),int(l[11]),int(l[13])])
        l='''DO NOT CHANGE PARAMETER!!
    STEERING_RIGHT_PWM
    490
    STEERING_CENTER_PWM
    390
    STEERING_LEFT_PWM
    290

    THROTTLE_FORWARD_PWM
    470
    THROTTLE_STOPPED_PWM
    390
    THROTTLE_REVERSE_PWM
    310

    '''.split("\n")
        PWM_PARAM = ([int(l[2]),int(l[4]),int(l[6])],[int(l[9]),int(l[11]),int(l[13])])
        return PWM_PARAM


    def Accel(self,PWM_PARAM,pwm,time,Duty):
        self.vehicle.setAccel(Duty)


    def Steer(self,PWM_PARAM,pwm,time,Duty):
        self.vehicle.setSteer(Duty)
#togikai_ultrasonic_cls模擬
class togikai_ultrasonic_cls:
    def __init__(self,vehi,trig,echo):
        self.vehicle=vehi
        self.trig=trig
        self.echo=echo
    #障害物センサ測定関数
    #def Mesure(self,GPIO,time,trig,echo):
    #    return self.vehicle.measureDist(trig,echo)*100
    def Mesure(self):
        return self.vehicle.measureDist(self.trig,self.echo)*100

#共通関数
class common:
    def normalize_angle_deg(angle_deg):
        return (angle_deg+180+36000) % 360 -180
    def calc_line_abc(pos,angle_deg):
        line_A_angle_rad=(angle_deg-90)*RAD_PER_DEGREE
        a=np.sin(line_A_angle_rad)
        b=-np.cos(line_A_angle_rad)
        c=-a*pos[0]-b*pos[1]
        return [a,b,c]
    def rorate(pos,angle_deg):
        result=np.zeros(2)
        angle_rad=angle_deg*RAD_PER_DEGREE
        result[0]=pos[0]*np.cos(angle_rad)-pos[1]*np.sin(angle_rad)
        result[1]=pos[0]*np.sin(angle_rad)+pos[1]*np.cos(angle_rad)
        return result
    def rotatem(pos,angle_deg):
        angle_rad=angle_deg*RAD_PER_DEGREE
        result=np.zeros((pos.shape[0],2))
        result[:,0]=pos[:,0]*np.cos(angle_rad)-pos[:,1]*np.sin(angle_rad)
        result[:,1]=pos[:,0]*np.sin(angle_rad)+pos[:,1]*np.cos(angle_rad)
        return result

    def fetch1DMap(x,x_vals,y_vals):
        if x<=x_vals[0]:
            idxLow=0
            idxLHigh=0
        elif x>=x_vals[-1]:
            idxLow=len(x_vals)-1
            idxLHigh=len(x_vals)-1
        else:
            for j in range(len(x_vals)-1):
                if x>x_vals[j] and x<=x_vals[j+1]:
                    idxLow=j
                    idxLHigh=j+1
                    break
        high_x=x_vals[idxLHigh]
        low_x=x_vals[idxLow]
        high_y=y_vals[idxLHigh]
        low_y=y_vals[idxLow]
        if high_x-low_x<eps:
            result=low_y
        else:
            result=((high_x-x)*low_y+(x-low_x)*high_y)/(high_x-low_x)
        return result
    def fetch2DMapSimple(x,x1_vals,x2_vals,y_vals):
        if x[0]<=x1_vals[0]:
            idx1Low=0
            idx1High=0
        elif x[0]>=x1_vals[-1]:
            idx1Low=len(x1_vals)-1
            idx1High=len(x1_vals)-1
        else:
            for j in range(len(x1_vals)-1):
                if x[0]>x1_vals[j] and x[0]<=x1_vals[j+1]:
                    idx1Low=j
                    idx1High=j+1
                    break
        if x[1]<=x2_vals[0]:
            idx2Low=0
            idx2High=0
        elif x[1]>=x2_vals[-1]:
            idx2Low=len(x2_vals)-1
            idx2High=len(x2_vals)-1
        else:
            for j in range(len(x2_vals)-1):
                if x[1]>x2_vals[j] and x[1]<=x2_vals[j+1]:
                    idx2Low=j
                    idx2High=j+1
                    break

        x1_high=x1_vals[idx1High]
        x1_low=x1_vals[idx1Low]
        x2_high=x2_vals[idx2High]
        x2_low=x2_vals[idx2Low]
        y_high_x1_low_x2=y_vals[idx1High,idx2Low]
        y_low_x1_low_x2=y_vals[idx1Low,idx2Low]
        y_high_x1_high_x2=y_vals[idx1High,idx2High]
        y_low_x1_high_x2=y_vals[idx1Low,idx2High]



        if x1_high-x1_low>eps:
            y_merge_x1_low_x2=((x1_high-x[0])*y_low_x1_low_x2+(x[0]-x1_low)*y_high_x1_low_x2)/(x1_high-x1_low)
            y_merge_x1_high_x2=((x1_high-x[0])*y_low_x1_high_x2+(x[0]-x1_low)*y_high_x1_high_x2)/(x1_high-x1_low)
        else:
            y_merge_x1_low_x2=y_low_x1_low_x2
            y_merge_x1_high_x2=y_low_x1_high_x2


        if x2_high-x2_low>eps:
            y_merge_x1_merge_x2=((x2_high-x[1])*y_merge_x1_low_x2+(x[1]-x2_low)*y_merge_x1_high_x2)/(x2_high-x2_low)
        else:
            y_merge_x1_merge_x2=y_merge_x1_low_x2

        return y_merge_x1_merge_x2

#センサー模擬
class sensor_simulation_cls:
    def calc_course_angles(self,course_lines):
        x1=course_lines[:,0]
        y1=course_lines[:,1]
        x2=course_lines[:,2]
        y2=course_lines[:,3]
        angles_rad=np.arctan2(y2-y1,x2-x1)
        angles_deg=angles_rad/RAD_PER_DEGREE
        angles_deg=(angles_deg+360)%360-180
        return angles_deg,angles_rad
    def calc_course_line_dist(self,course_lines):
        x1=course_lines[:,0]
        y1=course_lines[:,1]
        x2=course_lines[:,2]
        y2=course_lines[:,3]
        dy=y2-y1
        dx=x2-x1
        return np.sqrt(dx**2+dy**2)

    def calc_course_line_abc(self,course_lines):
        x1=course_lines[:,0]
        y1=course_lines[:,1]
        x2=course_lines[:,2]
        y2=course_lines[:,3]
        abc=np.zeros((course_lines.shape[0],3))
        #y-y1=(x-x1)*(y2-y1)/(x2-x1)
        #(y-y1)(x2-x1)=(x-x1)(y2-y1)
        #y*(x2-x1)-y1*(x2-x1)= (y2-y1)*x-(y2-y1)*x1
        #y*(x2-x1)-(y2-y1)*x-y1*(x2-x1)+(y2-y1)*x1=0
        #a=-(y2-y1)
        #b=x2-x1
        #c=-y1*(x2-x1)+x1*(y2-y1)
        x2_minux_x1=x2-x1
        y2_minux_y1=y2-y1
        abc[:,0]=-y2_minux_y1#a
        abc[:,1]=x2_minux_x1#b
        abc[:,2]=-y1*x2_minux_x1+x1*y2_minux_y1#c
        return abc


    def sense_distance(self,x,y,angle_deg,course_lines,course_angles_deg,course_angles_rad,course_line_dist,course_line_abc):

        #壁絞り込み(壁の向き)
        angle_diff=course_angles_deg-angle_deg
        angle_diff=(angle_diff+360)%360-180
        angle_OK=np.logical_or(angle_diff<0,angle_diff>180)
        angle_rad=angle_deg*RAD_PER_DEGREE
        a=np.sin(angle_rad)
        b=-np.cos(angle_rad)
        c=-a*x-b*y
        angle_OK_idx=np.where(angle_OK)[0]

        #壁絞り込み(センサとの距離)
        dist=(a*course_lines[angle_OK_idx,0]+b*course_lines[angle_OK_idx,1]+c)/(a**2+b**2)
        dist_OK= dist<=course_line_dist[angle_OK_idx]
        dist_OK_idx=angle_OK_idx[dist_OK]

        #壁絞り込み(平行なもの除去)
        denomi=a*course_line_abc[dist_OK_idx,1]-b*course_line_abc[dist_OK_idx,0]
        non_parallel= abs(denomi)>=eps
        non_parallel_idx=dist_OK_idx[non_parallel]

        #壁絞り込み(交差)
        cross_x=(b*course_line_abc[non_parallel_idx,2]-c*course_line_abc[non_parallel_idx,1])/denomi[non_parallel]#(a*course_line_abc[dist_OK_idx,1]-b*course_line_abc[dist_OK_idx,0])
        cross_y=(c*course_line_abc[non_parallel_idx,0]-a*course_line_abc[non_parallel_idx,2])/denomi[non_parallel]#(a*course_line_abc[dist_OK_idx,1]-b*course_line_abc[dist_OK_idx,0])
        cross_1_dist=np.sqrt((cross_x-course_lines[non_parallel_idx,0])**2+(cross_y-course_lines[non_parallel_idx,1])**2)
        cross_2_dist=np.sqrt((cross_x-course_lines[non_parallel_idx,2])**2+(cross_y-course_lines[non_parallel_idx,3])**2)
        course_OK=np.logical_and((cross_1_dist/course_line_dist[non_parallel_idx])<=1.0,(cross_2_dist/course_line_dist[non_parallel_idx])<=1.0)
        course_OK_idx=non_parallel_idx[course_OK]

        #壁絞り込み(センサの向き)
        front_OK=np.logical_and(np.sign(np.sin(angle_rad))==np.sign(cross_y[course_OK]-y),np.sign(np.cos(angle_rad))==np.sign(cross_x[course_OK]-x))
        front_OK_idx=course_OK_idx[front_OK]

        #壁絞り込み(距離最小)
        front_dist=np.sqrt((x-cross_x[course_OK][front_OK])**2+(x-cross_x[course_OK][front_OK])**2)

        if front_dist.shape[0]!=0:
            min_idx=np.argmin(front_dist)
            sense_idx=front_OK_idx[min_idx]
            sense_x=cross_x[course_OK][front_OK][min_idx]
            sense_y=cross_y[course_OK][front_OK][min_idx]
            dist=np.sqrt((sense_x-x)**2+(sense_y-y)**2)
        else:
            dist=20.0
            sense_x=0
            sense_y=0
        return dist,np.array([sense_x,sense_y])


#コースクラス
class Course:
    def __init__(self):
        self.course_raw=np.array([
                [0,-2.421,0.227,-3.951,0.227,1],
                [1,-3.955822365,0.235004566,-4.055698936,0.238363996,1],
                [2,-4.055698936,0.238363996,-4.155170975,0.247951937,1],
                [3,-4.155170975,0.247951937,-4.25385044,0.263730987,1],
                [4,-4.25385044,0.263730987,-4.35135238,0.285639591,1],
                [5,-4.35135238,0.285639591,-4.447296438,0.313592284,1],
                [6,-4.447296438,0.313592284,-4.541308334,0.347480021,1],
                [7,-4.541308334,0.347480021,-4.633021325,0.387170606,1],
                [8,-4.633021325,0.387170606,-4.722077637,0.432509204,1],
                [9,-4.722077637,0.432509204,-4.80812986,0.483318949,1],
                [10,-4.80812986,0.483318949,-4.890842301,0.539401631,1],
                [11,-4.890842301,0.539401631,-4.969892299,0.60053847,1],
                [12,-4.969892299,0.60053847,-5.044971477,0.666490971,1],
                [13,-5.044971477,0.666490971,-5.115786949,0.73700185,1],
                [14,-5.115786949,0.73700185,-5.182062463,0.811796044,1],
                [15,-5.182062463,0.811796044,-5.243539478,0.890581778,1],
                [16,-5.243539478,0.890581778,-5.299978169,0.973051709,1],
                [17,-5.299978169,0.973051709,-5.351158367,1.058884118,1],
                [18,-5.351158367,1.058884118,-5.396880419,1.147744172,1],
                [19,-5.396880419,1.147744172,-5.436965962,1.239285226,1],
                [20,-5.436965962,1.239285226,-5.47125862,1.333150176,1],
                [21,-5.47125862,1.333150176,-5.499624617,1.428972853,1],
                [22,-5.499624617,1.428972853,-5.521953297,1.52637945,1],
                [23,-5.521953297,1.52637945,-5.538157556,1.624989983,1],
                [24,-5.538157556,1.624989983,-5.54817418,1.72441977,1],
                [25,-5.54817418,1.72441977,-5.551964094,1.824280932,1],
                [26,-5.551964094,1.824280932,-5.551997971,1.837548245,1],
                [27,-5.551964094,1.824280932,-5.551997971,1.837548245,1],
                [28,-5.562,1.836,-5.562,2.381,1],
                [29,-5.551998732,2.382592653,-5.546849044,2.482367552,1],
                [30,-5.546849044,2.482367552,-5.53176922,2.581130646,1],
                [31,-5.53176922,2.581130646,-5.506909782,2.677896125,1],
                [32,-5.506909782,2.677896125,-5.472518863,2.771698121,1],
                [33,-5.472518863,2.771698121,-5.428939741,2.861600345,1],
                [34,-5.428939741,2.861600345,-5.3766074,2.946705432,1],
                [35,-5.3766074,2.946705432,-5.3160442,3.026163902,1],
                [36,-5.3160442,3.026163902,-5.247854654,3.099182637,1],
                [37,-5.247854654,3.099182637,-5.172719401,3.165032796,1],
                [38,-5.172719401,3.165032796,-5.091388406,3.223057093,1],
                [39,-5.091388406,3.223057093,-5.004673479,3.272676355,1],
                [40,-5.004673479,3.272676355,-4.913440169,3.313395305,1],
                [41,-4.913440169,3.313395305,-4.818599126,3.344807505,1],
                [42,-4.818599126,3.344807505,-4.72109701,3.366599412,1],
                [43,-4.72109701,3.366599412,-4.621907044,3.37855351,1],
                [44,-4.621907044,3.37855351,-4.522019298,3.380550478,1],
                [45,-4.522019298,3.380550478,-4.422430806,3.372570383,1],
                [46,-4.422430806,3.372570383,-4.324135615,3.354692879,1],
                [47,-4.324135615,3.354692879,-4.228114863,3.327096411,1],
                [48,-4.228114863,3.327096411,-4.135326987,3.290056434,1],
                [49,-4.135326987,3.290056434,-4.046698154,3.243942666,1],
                [50,-4.046698154,3.243942666,-3.963113016,3.189215392,1],
                [51,-3.963113016,3.189215392,-3.885405883,3.126420877,1],
                [52,-3.885405883,3.126420877,-3.814352392,3.056185905,1],
                [53,-3.814352392,3.056185905,-3.750661768,2.979211532,1],
                [54,-3.750661768,2.979211532,-3.694969741,2.896266081,1],
                [55,-3.694969741,2.896266081,-3.678526913,2.867872434,1],
                [56,-3.694969741,2.896266081,-3.678526913,2.867872434,1],
                [57,-3.647397918,2.831615458,-3.593549188,2.747489422,1],
                [58,-3.593549188,2.747489422,-3.529637076,2.670729291,1],
                [59,-3.529637076,2.670729291,-3.456657901,2.602531669,1],
                [60,-3.456657901,2.602531669,-3.375749324,2.543959679,1],
                [61,-3.375749324,2.543959679,-3.28817262,2.495926393,1],
                [62,-3.28817262,2.495926393,-3.195293009,2.459180595,1],
                [63,-3.195293009,2.459180595,-3.098558381,2.43429511,1],
                [64,-3.098558381,2.43429511,-2.999476717,2.421657876,1],
                [65,-2.999476717,2.421657876,-2.89959259,2.421465891,1],
                [66,-2.89959259,2.421465891,-2.800463079,2.43372215,1],
                [67,-2.800463079,2.43372215,-2.703633503,2.458235591,1],
                [68,-2.703633503,2.458235591,-2.610613323,2.494624077,1],
                [69,-2.610613323,2.494624077,-2.52285262,2.542320352,1],
                [70,-2.52285262,2.542320352,-2.441719483,2.600580887,1],
                [71,-2.441719483,2.600580887,-2.368478687,2.668497464,1],
                [72,-2.368478687,2.668497464,-2.304271972,2.745011342,1],
                [73,-2.304271972,2.745011342,-2.295127772,2.757660455,1],
                [74,-2.304271972,2.745011342,-2.295127772,2.757660455,1],
                [75,-2.286009683,2.755623949,-2.224394919,2.834269805,1],
                [76,-2.224394919,2.834269805,-2.155240143,2.906375052,1],
                [77,-2.155240143,2.906375052,-2.079235627,2.971219969,1],
                [78,-2.079235627,2.971219969,-1.997140013,3.028157304,1],
                [79,-1.997140013,3.028157304,-1.909772744,3.076618732,1],
                [80,-1.909772744,3.076618732,-1.81800588,3.116120534,1],
                [81,-1.81800588,3.116120534,-1.722755396,3.146268421,1],
                [82,-1.722755396,3.146268421,-1.624972041,3.16676147,1],
                [83,-1.624972041,3.16676147,-1.525631842,3.177395129,1],
                [84,-1.525631842,3.177395129,-1.42572637,3.178063258,1],
                [85,-1.42572637,3.178063258,-1.326252836,3.168759186,1],
                [86,-1.326252836,3.168759186,-1.22820414,3.149575785,1],
                [87,-1.22820414,3.149575785,-1.132558959,3.120704532,1],
                [88,-1.132558959,3.120704532,-1.040271982,3.082433609,1],
                [89,-1.040271982,3.082433609,-0.952264374,3.035145019,1],
                [90,-0.952264374,3.035145019,-0.869414589,2.979310774,1],
                [91,-0.869414589,2.979310774,-0.792549596,2.915488188,1],
                [92,-0.792549596,2.915488188,-0.722436626,2.844314308,1],
                [93,-0.722436626,2.844314308,-0.659775515,2.76649956,1],
                [94,-0.659775515,2.76649956,-0.605191719,2.682820656,1],
                [95,-0.605191719,2.682820656,-0.559230068,2.594112842,1],
                [96,-0.559230068,2.594112842,-0.522349331,2.501261558,1],
                [97,-0.522349331,2.501261558,-0.494917635,2.405193604,1],
                [98,-0.494917635,2.405193604,-0.47720879,2.306867887,1],
                [99,-0.47720879,2.306867887,-0.469399559,2.20726585,1],
                [100,-0.469399559,2.20726585,-0.469,2.179,1],
                [101,-0.469399559,2.20726585,-0.469,2.179,1],
                [102,-0.469009903,2.172782291,-0.471886439,2.072885311,1],
                [103,-0.471886439,2.072885311,-0.47987205,1.973266483,1],
                [104,-0.47987205,1.973266483,-0.492945802,1.874186929,1],
                [105,-0.492945802,1.874186929,-0.511073426,1.77590636,1],
                [106,-0.511073426,1.77590636,-0.534207406,1.678682392,1],
                [107,-0.534207406,1.678682392,-0.562287103,1.582769871,1],
                [108,-0.562287103,1.582769871,-0.595238912,1.488420206,1],
                [109,-0.595238912,1.488420206,-0.632976461,1.39588071,1],
                [110,-0.632976461,1.39588071,-0.675400829,1.305393948,1],
                [111,-0.675400829,1.305393948,-0.722400813,1.217197109,1],
                [112,-0.722400813,1.217197109,-0.773853215,1.131521375,1],
                [113,-0.773853215,1.131521375,-0.829623167,1.048591324,1],
                [114,-0.829623167,1.048591324,-0.889564483,0.968624333,1],
                [115,-0.889564483,0.968624333,-0.953520042,0.891830014,1],
                [116,-0.953520042,0.891830014,-1.021322203,0.818409663,1],
                [117,-1.021322203,0.818409663,-1.092793241,0.748555731,1],
                [118,-1.092793241,0.748555731,-1.167745814,0.682451323,1],
                [119,-1.167745814,0.682451323,-1.245983453,0.620269711,1],
                [120,-1.245983453,0.620269711,-1.32730108,0.562173889,1],
                [121,-1.32730108,0.562173889,-1.411485543,0.50831614,1],
                [122,-1.411485543,0.50831614,-1.498316175,0.458837636,1],
                [123,-1.498316175,0.458837636,-1.587565373,0.413868072,1],
                [124,-1.587565373,0.413868072,-1.678999194,0.373525324,1],
                [125,-1.678999194,0.373525324,-1.77237797,0.33791514,1],
                [126,-1.77237797,0.33791514,-1.867456933,0.307130861,1],
                [127,-1.867456933,0.307130861,-1.963986859,0.281253181,1],
                [128,-1.963986859,0.281253181,-2.061714722,0.260349931,1],
                [129,-2.061714722,0.260349931,-2.160384353,0.244475902,1],
                [130,-2.160384353,0.244475902,-2.259737118,0.233672705,1],
                [131,-2.259737118,0.233672705,-2.35951259,0.227968658,1],
                [132,-2.35951259,0.227968658,-2.425663285,0.22700557,1],
                [133,-2.35951259,0.227968658,-2.425663285,0.22700557,1],
                [134,-4.240318121,2.255504235,-4.3326128,2.287975617,0],
                [135,-4.3326128,2.287975617,-4.42363138,2.25208249,0],
                [136,-4.42363138,2.25208249,-4.468920115,2.165355164,0],
                [137,-4.468920115,2.165355164,-4.446359851,2.070151525,0],
                [138,-4.446359851,2.070151525,-4.419852982,2.040638733,0],
                [139,-4.446359851,2.070151525,-4.419852982,2.040638733,0],
                [140,-4.426,2.033,-3.622,1.38842,0],
                [141,-3.622472105,1.388810143,-3.536647016,1.338492614,0],
                [142,-3.536647016,1.338492614,-3.462869813,1.320683869,0],
                [143,-3.536647016,1.338492614,-3.462869813,1.320683869,0],
                [144,-3.46217,1.32061,-2.49548,1.2161,0],
                [145,-2.496209813,1.216183869,-2.396881222,1.221807989,0],
                [146,-2.396881222,1.221807989,-2.304853774,1.259606778,0],
                [147,-2.304853774,1.259606778,-2.260240168,1.293510571,0],
                [148,-2.304853774,1.259606778,-2.260240168,1.293510571,0],
                [149,-2.2596,1.29407,-1.79972,1.68855,0],
                [150,-1.799375102,1.688867724,-1.718240964,1.746792203,0],
                [151,-1.718240964,1.746792203,-1.625302914,1.782854823,0],
                [152,-1.625302914,1.782854823,-1.55632757,1.793829824,0],
                [153,-1.625302914,1.782854823,-1.55632757,1.793829824,0],
                [154,-1.557380962,1.793709413,-1.460636258,1.816909884,0],
                [155,-1.460636258,1.816909884,-1.376798696,1.870473518,0],
                [156,-1.376798696,1.870473518,-1.315088358,1.948509624,0],
                [157,-1.315088358,1.948509624,-1.282291874,2.04243614,0],
                [158,-1.282291874,2.04243614,-1.282016053,2.141923443,0],
                [159,-1.282016053,2.141923443,-1.314291231,2.236030364,0],
                [160,-1.314291231,2.236030364,-1.375567926,2.314407442,0],
                [161,-1.375567926,2.314407442,-1.4591072,2.368435114,0],
                [162,-1.4591072,2.368435114,-1.555721775,2.392171658,0],
                [163,-1.555721775,2.392171658,-1.654786411,2.383006633,0],
                [164,-1.654786411,2.383006633,-1.745406418,2.341947969,0],
                [165,-1.745406418,2.341947969,-1.817615813,2.273511114,0],
                [166,-1.817615813,2.273511114,-1.863473326,2.185222451,0],
                [167,-1.863473326,2.185222451,-1.874091163,2.141270311,0],
                [168,-1.863473326,2.185222451,-1.874091163,2.141270311,0],
                [169,-1.876449779,2.139008428,-1.904473198,2.043338779,0],
                [170,-1.904473198,2.043338779,-1.955283616,1.957569932,0],
                [171,-1.955283616,1.957569932,-2.000633558,1.908260761,0],
                [172,-1.955283616,1.957569932,-2.000633558,1.908260761,0],
                [173,-1.99982,1.90901,-2.37567,1.56808,0],
                [174,-2.376076779,1.567700381,-2.463187607,1.520845509,0],
                [175,-2.463187607,1.520845509,-2.533389875,1.516149246,0],
                [176,-2.463187607,1.520845509,-2.533389875,1.516149246,0],
                [177,-2.53293,1.5161,-3.40565,1.59613,0],
                [178,-3.406109875,1.596179246,-3.498685412,1.631013788,0],
                [179,-3.498685412,1.631013788,-3.512511403,1.641596762,0],
                [180,-3.498685412,1.631013788,-3.512511403,1.641596762,0],
                [181,-3.5122,1.64134,-4.234,2.263,0]
        ])
        self.course_lines=self.course_raw[:,1:5]
        self.course_parts_idx=self.course_raw[:,5]
        self.course_angles_deg,self.course_angles_rad=sensor_simulation.calc_course_angles(self.course_lines)
        self.course_line_dist=sensor_simulation.calc_course_line_dist(self.course_lines)
        self.course_line_abc=sensor_simulation.calc_course_line_abc(self.course_lines)
        self.parts=list(set(self.course_parts_idx))
        self.parts_idx=[[idx for idx in range(self.course_lines.shape[0]) if self.course_parts_idx[idx]==part]for part in self.parts]
        self.plot_for_parts=[]
        for idxs in self.parts_idx:
            tmp=[]
            for i in idxs:
                tmp.append([self.course_lines[i,0],self.course_lines[i,1]])
                tmp.append([self.course_lines[i,2],self.course_lines[i,3]])
            tmp=np.array(tmp)
            self.plot_for_parts.append(tmp)

    def measureDist(self,pos,angle):
        angle_r=(angle-20.0)%360.0
        angle_l=(angle+20.0)%360.0
        angle_fr=(angle-10.0)%360.0
        angle_fl=(angle+10.0)%360.0
        dist_c,pos_c= sensor_simulation.sense_distance(pos[0],pos[1],angle,self.course_lines,
                                                self.course_angles_deg,self.course_angles_rad,
                                                self.course_line_dist,self.course_line_abc)
        dist_l,pos_l= sensor_simulation.sense_distance(pos[0],pos[1],angle_l,self.course_lines,
                                                self.course_angles_deg,self.course_angles_rad,
                                                self.course_line_dist,self.course_line_abc)
        dist_r,pos_r= sensor_simulation.sense_distance(pos[0],pos[1],angle_r,self.course_lines,
                                                self.course_angles_deg,self.course_angles_rad,
                                                self.course_line_dist,self.course_line_abc)
        dist_fl,pos_fl= sensor_simulation.sense_distance(pos[0],pos[1],angle_fl,self.course_lines,
                                                self.course_angles_deg,self.course_angles_rad,
                                                self.course_line_dist,self.course_line_abc)
        dist_fr,pos_fr= sensor_simulation.sense_distance(pos[0],pos[1],angle_fr,self.course_lines,
                                                self.course_angles_deg,self.course_angles_rad,
                                                self.course_line_dist,self.course_line_abc)

        dist=np.array([dist_c,dist_l,dist_r,dist_fl,dist_fr])
        pos=[pos_c,pos_l,pos_r,pos_fl,pos_fr]
        idx=np.argmin(dist)
        return dist[idx],pos[idx]

    def judge_wall_crossing(self,x1,y1,x2,y2):
        course_x1_minus_x2=self.course_lines[:,0]-self.course_lines[:,2]
        course_y1_minus_y2=self.course_lines[:,1]-self.course_lines[:,3]
        s1=course_x1_minus_x2*(y1-self.course_lines[:,1])-course_y1_minus_y2*(x1-self.course_lines[:,0])
        t1=course_x1_minus_x2*(y2-self.course_lines[:,1])-course_y1_minus_y2*(x2-self.course_lines[:,0])

        obj_x1_minus_x2=x1-x2
        obj_y1_minus_y2=y1-y2
        s2=obj_x1_minus_x2*(self.course_lines[:,1]-y1)-obj_y1_minus_y2*(self.course_lines[:,0]-x1)
        t2=obj_x1_minus_x2*(self.course_lines[:,3]-y1)-obj_y1_minus_y2*(self.course_lines[:,2]-x1)

        if np.any(np.logical_and(s1*t1<=0,s2*t2<=0)):
            return True
        else:
            return False

    def getDrawInfo(self):
        return None
    def draw(self,ax,drawInfo=None):
        for lines in self.plot_for_parts:
            ax.plot(lines[:,0],lines[:,1],'k-')
        ax.plot([4.5,4.5],[0.066,1.317],'g-')
    def start(self):
        pass
    def proceedTime(self,elapsed_time):
        pass
    def abort(self):
        return False



#PCA9685クラス模擬
class PCA9685:
    def __init__(self, address):
        pass
    def set_pwm_freq(self,p1):
        pass

#GPIOクラス模擬
class GPIO:
    def setmode(p1):
        pass
    def setup(p1,p2,initial=0):
        pass
    def cleanup():
        pass
    LOW=0
    BOARD=0
    IN=0
    OUT=0

#時間表示用オブジェクト
class timeView:
  def __init__(self,sim):
    self.sim=sim
  def start(self):
    pass
  def abort(self):
    return False
  def proceedTime(self,time):
    pass
  def getDrawInfo(self):
    return self.sim.getTime()
  def draw(self,ax,drawInfo):
    ax.text(1.5,-0.25,"time: % 3.3f (s)" % drawInfo)

#シミュレーション管理
class SimManager:
  def __init__(self):
    self.timeLength=0
    self.viewInterval=0
    self.objects=[]
    self.objects.append(timeView(self))
    self.animation=None
    self.drawInfoLog=[]
    self.timeLog=[]

    self.viewCount=0
    self.timeAfterView=0

  def getTime(self):
    return self.viewCount*self.viewInterval+self.timeAfterView#.prevTime
  def setControlFunc(self,con):
    self.control=con
  def simulate(self,timeLength,viewInterval,controlTimeInterval,figsize=(10,6.4)):
    self.drawInfoLog=[]
    self.timeLength=timeLength
    self.viewInterval=viewInterval
    self.figsize=figsize
    self.viewCount=0
    self.timeAfterView=0
    [obj.start() for obj in self.objects]
    while self.getTime()<self.timeLength:
      abort_control= self.control()==False
      abort_simulation=self.proceedSimulationTime(controlTimeInterval)
      if abort_control or abort_simulation:
        break

  def proceedSimulationTime(self,elapsed_time):
    abort=False
    leftTime=elapsed_time
    endTime=self.getTime()+elapsed_time
    while True:
        leftTimeNextView=self.viewInterval-self.timeAfterView
        leftTime=endTime-self.getTime()
        abort=any([obj.abort() for obj in self.objects])
        if abort:
            curDrawInfo=[obj.getDrawInfo() for obj in self.objects]
            self.drawInfoLog.append(curDrawInfo)
            break
        if leftTime<leftTimeNextView:
            self.timeAfterView+=leftTime
            [obj.proceedTime(leftTime) for obj in self.objects]
            break
        else:
            self.viewCount+=1
            self.timeAfterView=0
            [obj.proceedTime(leftTimeNextView) for obj in self.objects]
            curDrawInfo=[obj.getDrawInfo() for obj in self.objects]
            self.drawInfoLog.append(curDrawInfo)

    return abort

  def createNotebookAnimation(self):
    rc('animation', html='jshtml')
    matplotlib.rcParams['animation.embed_limit'] = 2**128
    fig = plt.figure(figsize=(self.figsize))
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    ax.set_xlim(-6,0)
    ax.set_ylim(0,4)

    self.animation = anm.FuncAnimation(fig, self.drawPicture, fargs=(ax,),
                                frames=int(self.getTime()/self.viewInterval),
                                interval = int(self.viewInterval * 1000),
                                repeat=False)

    return self.animation
  def drawPicture(self,i,ax):
    #draw objects
    ax.cla()
    drawInfoNow=self.drawInfoLog[i]
    for j in range(len(self.objects)):
      self.objects[j].draw(ax,drawInfo=drawInfoNow[j])
    ax.set_xlim(-6,0)
    ax.set_ylim(0,3.5)

  def addObject(self,obj):
    self.objects.append(obj)
