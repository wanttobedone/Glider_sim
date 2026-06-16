#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把锁定 baseline bag 抽成无 ROS 依赖的 CSV，供 replay_core.cpp（host/STM32 原型）回放。
存的是原始 body-FLU 量（与真实传感器输出一致），FLU→FRD 由 C++ 端做。

输出:
  sensors.csv : t_sec,kind,v0..v5   kind=I(gyro_flu xyz, accel_flu xyz)/M(mag_flu xyz)/P(pressure)
  gt.csv      : t_sec,roll_ned,pitch_ned,yaw_ned,depth   (参考真值)

用法: python3 extract_sensors.py <bag> <out_dir>
"""
import sys, math
import numpy as np
import rosbag
import tf.transformations as tft

def main():
    bag_path, out_dir = sys.argv[1], sys.argv[2]
    Rwn = np.array([[0,1,0],[1,0,0],[0,0,-1.0]]); Tb = np.diag([1,-1,-1.0])

    rows = []  # (t, kind, [vals])
    gt = []
    b = rosbag.Bag(bag_path)
    for topic, m, _ in b.read_messages(topics=[
            '/ug_glider/imu','/ug_glider/pressure','/ug_glider/mag',
            '/ug_glider/ground_truth/pose']):
        t = m.header.stamp.to_sec()
        if topic == '/ug_glider/imu':
            rows.append((t,'I',[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,
                                 m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]))
        elif topic == '/ug_glider/pressure':
            rows.append((t,'P',[m.fluid_pressure]))
        elif topic == '/ug_glider/mag':
            rows.append((t,'M',[m.magnetic_field.x,m.magnetic_field.y,m.magnetic_field.z]))
        else:
            o = m.pose.pose.orientation
            R = Rwn @ tft.quaternion_matrix([o.x,o.y,o.z,o.w])[:3,:3] @ Tb
            pitch = math.asin(max(-1,min(1,-R[2,0])))
            roll = math.atan2(R[2,1],R[2,2]); yaw = math.atan2(R[1,0],R[0,0])
            gt.append((t, roll, pitch, yaw, -m.pose.pose.position.z))
    b.close()

    rows.sort(key=lambda x: x[0])
    with open(out_dir+'/sensors.csv','w') as f:
        f.write("t,kind,v0,v1,v2,v3,v4,v5\n")
        for t,k,v in rows:
            vv = v + [0.0]*(6-len(v))
            f.write("%.9f,%s,%.10e,%.10e,%.10e,%.10e,%.10e,%.10e\n"%(t,k,*vv))
    with open(out_dir+'/gt.csv','w') as f:
        f.write("t,roll,pitch,yaw,depth\n")
        for t,r,p,y,d in gt:
            f.write("%.9f,%.9f,%.9f,%.9f,%.9f\n"%(t,r,p,y,d))
    print("sensors=%d gt=%d -> %s"%(len(rows),len(gt),out_dir))

if __name__=='__main__':
    main()
