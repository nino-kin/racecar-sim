#!/usr/bin/env python3
# -*- coding:utf-8 -*-

class config:

    # モーター出力パラメータ （デューティー比：-100~100で設定）
    # アクセル用
    FORWARD_S = 80
    FORWARD_C = 60
    STOP = 0
    REVERSE = -60
    # 操舵用
    LEFT = 100 #<=100
    NUTRAL = 0
    RIGHT = -100 #<=100

    # 超音波センサの検知パラメータ
    ## 距離関連、単位はmm
    ### 前壁の検知距離
    DETECTION_DISTANCE_STOP = 60
    DETECTION_DISTANCE_Fr = 150
    ### 右左折判定基準
    DETECTION_DISTANCE_RL = 150
    ### 他
    DETECTION_DISTANCE_FrLH = 150
    DETECTION_DISTANCE_FrRH = 150
    DETECTION_DISTANCE_FrLH = 150
    DETECTION_DISTANCE_FrRH = 150
    DETECTION_DISTANCE_TARGET = 180 #目標距離
    DETECTION_DISTANCE_RANGE = 60/2 #修正認知半径距離

    # 判断モード
    #"Right_Left_3","Right_Left_3_Records","RightHand","RightHand_PID"
    mode_plan = "Right_Left_3"

    ## PIDパラメータ(PDまでを推奨)
    K_P = 0.7 #0.7
    K_I = 0.0 #0.0
    K_D = 0.3 #0.3

    #4/6向けパラメータはここまで↑↑↑～～～～～～～～～～～～～～～～～～～～～～～～～～～～～～

    # NNパラメータ
    Nnode = 3
    Nlayer = 3
    model = "linear" #"categorical"
    Ncategory = 5

    # 超音波センサ
    ## 使う超音波センサ位置の指示、計測ループが遅い場合は数を減らす
    ## 現在はultrasonic.pyのチェック用のみで使う,run.pyでの超音波センサの選択は別
    ### 前３つ使う場合はこちらをコメントアウト外す
    #ultrasonics_list = ["FrLH","Fr","FrRH"]
    ### ５つ使う場合はこちらをコメントアウト外す
    ultrasonics_list = ["RrLH", "FrLH", "Fr", "FrRH","RrRH"]

    ## 過去の超音波センサの値記録回数
    ultrasonics_Nrecords = 5

    ## 超音波センサ初期設定(配線を変えない限り触らない！)
    ## !!!超音波センサ初期設定、配線を変えない限り触らない
    ### GPIOピン番号の指示方法
    GPIO.setmode(GPIO.BOARD)
    ### Triger -- Fr:15, FrLH:13, RrLH:35, FrRH:32, RrRH:36
    t_list=[15,13,35,32,36]
    GPIO.setup(t_list,GPIO.OUT,initial=GPIO.LOW)
    ### Echo -- Fr:26, FrLH:24, RrLH:37, FrRH:31, RrRH:38
    e_list=[26,24,37,31,38]
    GPIO.setup(e_list,GPIO.IN)

    ## !!!超音波センサ初期設定、配線を変えない限り触らない
    ultrasonics_dict_trig = {"Fr":t_list[0], "FrLH":t_list[1], "RrLH":t_list[2], "FrRH":t_list[3], "RrRH":t_list[4]}
    ultrasonics_dict_echo = {"Fr":e_list[0], "FrLH":e_list[1], "RrLH":e_list[2], "FrRH":e_list[3], "RrRH":e_list[4]}
    N_ultrasonics = len(ultrasonics_list)
    print(" ", N_ultrasonics," ultrasonics are used")
    print(" ", ultrasonics_list)
    ## !!!

    # モーター用 パラメーター
    ## 過去の操作値記録回数
    motor_Nrecords = 5

    ## PWMピンのチャンネル
    ## !!!配線を変えない限り触らない
    CHANNEL_STEERING = 14
    CHANNEL_THROTTLE = 13

    ## 操舵のPWM値
    STEERING_CENTER_PWM = 350
    STEERING_WIDTH_PWM = 90
    ### DO NOT USE OVER 450, otherwise break your car!
    STEERING_RIGHT_PWM = STEERING_CENTER_PWM + STEERING_WIDTH_PWM
    ### DO NOT USE UNDER 260, otherwise break your car!
    STEERING_LEFT_PWM = STEERING_CENTER_PWM - STEERING_WIDTH_PWM

    ## アクセルのPWM値(motor.pyで調整した後値を入れる)
    THROTTLE_STOPPED_PWM = 390
    THROTTLE_WIDTH_PWM = 80
    THROTTLE_FORWARD_PWM = 430
    THROTTLE_REVERSE_PWM = 250

    # 記録(google colabで使用不可)
    #records = "records"
    #if not os.path.exists(records):
    #    # ディレクトリが存在しない場合、ディレクトリを作成する
    #    os.makedirs(records)
    #    print("make dir as ",records)

    ## 記録回数
    sampling_times = 100
    ## 目標サンプリング周期（何秒に１回）、複数センサ利用の場合は合計値、
    sampling_cycle = 0.05
    ## 記録したcsvファイル名(google colabで使用不可)
    #record_filename = './'+records+'/record_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'
