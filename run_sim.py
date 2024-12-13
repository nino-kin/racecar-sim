#!/usr/bin/env python3
# -*- coding:utf-8 -*-

#シミュレーション開始(変更しない)
vehicle.setSensorPose("F",sensor_F_pos,sensor_F_angle)
vehicle.setSensorPose("FR",sensor_FR_pos,sensor_FR_angle)
vehicle.setSensorPose("FL",sensor_FL_pos,sensor_FL_angle)
vehicle.setSensorPose("RR",sensor_RR_pos,sensor_RR_angle)
vehicle.setSensorPose("RL",sensor_RL_pos,sensor_RL_angle)

sim.control=control_callback
vehicle.setPose(initial_x,initial_y,initial_angle)
vehicle.initializeState()
sim.simulate(simulation_length,view_interval,control_interval,figsize=figsize)

# シミュレーション結果描画
sim.createNotebookAnimation()
