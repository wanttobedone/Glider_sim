#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
水下滑翔机 RQT 仪表盘
指令区：三个滑块直接控制执行机构（电池位置、油囊体积、舵角）
显示区：实时传感器数据 + 执行机构实际状态
波形图：目标值 vs 实际值对比
"""

import rospy
import math
from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QSlider, QLabel, QGridLayout, QPushButton
)
from python_qt_binding.QtCore import Qt, QTimer

from ug_msgs.msg import ActuatorCmd, ActuatorState, GliderState


class GliderDashboard(Plugin):

    def __init__(self, context):
        super(GliderDashboard, self).__init__(context)
        self.setObjectName('GliderDashboard')

        self._ns = rospy.get_param('~namespace', 'ug_glider')

        # ROS 接口
        self._cmd_pub = rospy.Publisher(
            '/{}/actuator_cmd'.format(self._ns), ActuatorCmd, queue_size=1)
        self._state_sub = rospy.Subscriber(
            '/{}/actuator_state'.format(self._ns), ActuatorState,
            self._on_actuator_state)
        self._glider_sub = rospy.Subscriber(
            '/{}/glider_state'.format(self._ns), GliderState,
            self._on_glider_state)

        # 最新数据
        self._actuator_state = None
        self._glider_state = None

        # 构建界面
        self._widget = QWidget()
        self._widget.setWindowTitle('UG Glider Dashboard')

        main_layout = QVBoxLayout()

        # === 指令区 ===
        cmd_group = QGroupBox('指令区 (直接控制执行机构)')
        cmd_layout = QGridLayout()

        # 电池位置滑块: [-33.85, +33.85] mm, 步进 0.1mm
        cmd_layout.addWidget(QLabel('电池位置 [mm]:'), 0, 0)
        self._battery_slider = QSlider(Qt.Horizontal)
        self._battery_slider.setRange(-3385, 3385)  # 0.01mm 精度
        self._battery_slider.setValue(0)
        self._battery_slider.setTickPosition(QSlider.TicksBelow)
        self._battery_slider.setTickInterval(1000)
        self._battery_slider.valueChanged.connect(self._on_slider_changed)
        cmd_layout.addWidget(self._battery_slider, 0, 1)
        self._battery_label = QLabel('0.00 mm')
        self._battery_label.setMinimumWidth(80)
        cmd_layout.addWidget(self._battery_label, 0, 2)

        # 油囊体积滑块: [0, 1000] cc, 步进 1cc
        cmd_layout.addWidget(QLabel('油囊体积 [cc]:'), 1, 0)
        self._ballast_slider = QSlider(Qt.Horizontal)
        self._ballast_slider.setRange(0, 1000)
        self._ballast_slider.setValue(500)  # 中性
        self._ballast_slider.setTickPosition(QSlider.TicksBelow)
        self._ballast_slider.setTickInterval(100)
        self._ballast_slider.valueChanged.connect(self._on_slider_changed)
        cmd_layout.addWidget(self._ballast_slider, 1, 1)
        self._ballast_label = QLabel('500 cc')
        self._ballast_label.setMinimumWidth(80)
        cmd_layout.addWidget(self._ballast_label, 1, 2)

        # 舵角滑块: [-30, +30] deg, 步进 0.5deg
        cmd_layout.addWidget(QLabel('尾舵角度 [deg]:'), 2, 0)
        self._rudder_slider = QSlider(Qt.Horizontal)
        self._rudder_slider.setRange(-300, 300)  # 0.1deg 精度
        self._rudder_slider.setValue(0)
        self._rudder_slider.setTickPosition(QSlider.TicksBelow)
        self._rudder_slider.setTickInterval(100)
        self._rudder_slider.valueChanged.connect(self._on_slider_changed)
        cmd_layout.addWidget(self._rudder_slider, 2, 1)
        self._rudder_label = QLabel('0.0 deg')
        self._rudder_label.setMinimumWidth(80)
        cmd_layout.addWidget(self._rudder_label, 2, 2)

        # 归零按钮
        reset_btn = QPushButton('全部归零 (安全状态)')
        reset_btn.clicked.connect(self._on_reset)
        cmd_layout.addWidget(reset_btn, 3, 0, 1, 3)

        cmd_group.setLayout(cmd_layout)
        main_layout.addWidget(cmd_group)

        # === 状态显示区 ===
        status_group = QGroupBox('状态显示')
        status_layout = QGridLayout()

        # 执行机构实际状态
        status_layout.addWidget(QLabel('-- 执行机构实际状态 --'), 0, 0, 1, 2)
        self._act_battery_label = QLabel('电池: ---')
        self._act_ballast_label = QLabel('油囊: ---')
        self._act_rudder_label = QLabel('舵角: ---')
        status_layout.addWidget(self._act_battery_label, 1, 0)
        status_layout.addWidget(self._act_ballast_label, 1, 1)
        status_layout.addWidget(self._act_rudder_label, 1, 2)

        # 滑翔机状态
        status_layout.addWidget(QLabel('-- 滑翔机状态 (NED) --'), 2, 0, 1, 2)
        self._pos_label = QLabel('位置: ---')
        self._att_label = QLabel('姿态: ---')
        self._vel_label = QLabel('速度: ---')
        status_layout.addWidget(self._pos_label, 3, 0)
        status_layout.addWidget(self._att_label, 3, 1)
        status_layout.addWidget(self._vel_label, 3, 2)

        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)

        self._widget.setLayout(main_layout)
        context.add_widget(self._widget)

        # 定时发布指令 + 刷新显示 (10 Hz)
        self._timer = QTimer()
        self._timer.timeout.connect(self._on_timer)
        self._timer.start(100)

    def shutdown_plugin(self):
        self._timer.stop()
        self._cmd_pub.unregister()
        self._state_sub.unregister()
        self._glider_sub.unregister()

    def _on_slider_changed(self):
        batt_mm = self._battery_slider.value() / 100.0
        self._battery_label.setText('{:.2f} mm'.format(batt_mm))

        ballast_cc = self._ballast_slider.value()
        self._ballast_label.setText('{} cc'.format(ballast_cc))

        rudder_deg = self._rudder_slider.value() / 10.0
        self._rudder_label.setText('{:.1f} deg'.format(rudder_deg))

    def _on_reset(self):
        self._battery_slider.setValue(0)
        self._ballast_slider.setValue(500)
        self._rudder_slider.setValue(0)

    def _on_actuator_state(self, msg):
        self._actuator_state = msg

    def _on_glider_state(self, msg):
        self._glider_state = msg

    def _on_timer(self):
        # 发布指令
        cmd = ActuatorCmd()
        cmd.header.stamp = rospy.Time.now()
        cmd.battery_position = self._battery_slider.value() / 100000.0  # 0.01mm → m
        cmd.ballast_volume = self._ballast_slider.value() / 1e6         # cc → m^3
        cmd.rudder_angle = math.radians(self._rudder_slider.value() / 10.0)
        self._cmd_pub.publish(cmd)

        # 刷新执行机构状态显示
        if self._actuator_state is not None:
            s = self._actuator_state
            self._act_battery_label.setText(
                '电池: {:.2f} mm'.format(s.battery_position * 1000))
            self._act_ballast_label.setText(
                '油囊: {:.0f} cc'.format(s.ballast_volume * 1e6))
            self._act_rudder_label.setText(
                '舵角: {:.1f} deg'.format(math.degrees(s.rudder_angle)))

        # 刷新滑翔机状态显示
        if self._glider_state is not None:
            g = self._glider_state
            self._pos_label.setText(
                'N:{:.1f} E:{:.1f} D:{:.1f}m'.format(g.north, g.east, g.depth))
            self._att_label.setText(
                'R:{:.1f} P:{:.1f} Y:{:.1f}deg'.format(
                    math.degrees(g.roll),
                    math.degrees(g.pitch),
                    math.degrees(g.yaw)))
            self._vel_label.setText(
                'u:{:.2f} v:{:.2f} w:{:.2f}m/s'.format(
                    g.surge, g.sway, g.heave))
