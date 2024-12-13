#!/usr/bin/env python3
# -*- coding:utf-8 -*-

#マシンの初期座標
initial_x=-3 #X座標(m)
initial_y=0.75 #Y座標(m)
initial_angle=180 #初期角度(°)

#シミュレーション時間(秒)
simulation_length=40.0

#センサー設定
#sensor_*_pos:センサー位置 [マシン前後(前が正),マシン左右(左が正)]  単位=m 　原点は後輪軸中心
#sensor_*_angle:センサー角度　単位=°　0が前向き、時計と反対周り

sensor_F_pos=[0.14,0.0]
sensor_F_angle=0

sensor_FR_pos=[0.13,-0.045]
sensor_FR_angle=-45

sensor_FL_pos=[0.13,0.045]
sensor_FL_angle=45

sensor_RR_pos=[0.09,-0.045]
sensor_RR_angle=-90

sensor_RL_pos=[0.09,0.045]
sensor_RL_angle=90

#制御周期(秒)
#関数controlが呼ばれる周期　※control関数の処理時間はこの数字に織り込んでください
control_interval=0.2

#表示サイズ
#シムレーション結果表示のなめらかさです。　※google colabの場合大きすぎると描画失敗します
figsize=(6*1.5,3.5*1.5)

#表示間隔(秒)　基本変えない！
#シムレーション結果表示のなめらかさ+加速の計算の周期です。　※google colabの場合小さいすぎると描画失敗します
view_interval=0.1
