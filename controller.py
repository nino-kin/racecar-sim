#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import time

# データ記録用配列作成
d = np.zeros(6)

# 操舵、駆動モーターの初期化
#motor = motor.Motor()
motor.set_steer_pwm_duty(config.NUTRAL)
motor.set_throttle_pwm_duty(config.STOP)

# 操作判断プランナーの初期化
#plan = planner.Planner("NoName")
plan = Planner("NoName")

start_time = time.perf_counter()

#try:
#    while True:
def control_callback():
        global d

        # 認知（計測）
        ## デフォルトは左から測定していく感じ
        ## RrLHセンサ距離
        dis_RrLH = ultrasonic_RrLH.Mesure()
        ## FrLHセンサ距離
        dis_FrLH = ultrasonic_FrLH.Mesure()
        ## Frセンサ距離
        dis_Fr = ultrasonic_Fr.Mesure()
        ## FrRHセンサ距離
        dis_FrRH = ultrasonic_FrRH.Mesure()
        ## RrRHセンサ距離
        dis_RrRH = ultrasonic_RrRH.Mesure()
        ## センサーの値を滑らかにする


        # 判断（プランニング）＃
        ## 右左空いているほうに走る
        if config.mode_plan == "Right_Left_3":
            steer_pwm_duty,throttle_pwm_duty = plan.Right_Left_3(dis_FrLH, dis_Fr, dis_FrRH)
        ## 過去の値を使ってスムーズに走る
        elif config.mode_plan == "Right_Left_3_Records":
            steer_pwm_duty, throttle_pwm_duty  = plan.Right_Left_3_Records(dis_FrLH, dis_Fr, dis_FrRH)
        ## 右手法で走る
        elif config.mode_plan == "RightHand":
            steer_pwm_duty, throttle_pwm_duty  = plan.RightHand(dis_FrRH, dis_RrRH)
        ## 右手法にPID制御を使ってスムーズに走る
        elif config.mode_plan == "RightHand_PID":
            steer_pwm_duty, throttle_pwm_duty  = plan.RightHand_PID(ultrasonic_FrRH, ultrasonic_RrRH)
        ## ニューラルネットを使ってスムーズに走る
        #未実装
        #steer_pwm_duty, throttle_pwm_duty  = plan.NN(dis_FrRH, dis_RrRH)
        else:
            print("No plan mode selected, please change code to try your own one!")
            #break

        # 操作（ステアリング、アクセル）＃
        motor.set_steer_pwm_duty(steer_pwm_duty)
        motor.set_throttle_pwm_duty(throttle_pwm_duty)

        # 記録（距離データを配列に記録）#
        d = np.vstack([d,[time.perf_counter()-start_time, dis_RrLH, dis_FrLH, dis_Fr, dis_FrRH, dis_RrRH]])
        print('RrLH:{0:.1f} , FrLH:{1:.1f} , Fr:{2:.1f}, FrRH:{3:.1f} , RrRH:{4:.1f}'.format(dis_RrLH, dis_FrLH, dis_Fr, dis_FrRH, dis_RrRH))

        # 停止処理
        plan.Stop(ultrasonic_Fr)
        if plan.flag_stop ==True:
            ## 停止動作
            motor.set_steer_pwm_duty(config.NUTRAL)
            motor.set_throttle_pwm_duty(config.STOP)
            time.sleep(0.05)
            motor.set_throttle_pwm_duty(config.REVERSE)
            time.sleep(0.05)
            motor.set_throttle_pwm_duty(config.STOP)
            #break

        # 終了処理
        GPIO.cleanup()
        #google colabでは記録不可
        #np.savetxt(config.record_filename, d, fmt='%.3e')
        #print('記録停止')
