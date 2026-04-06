#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3Stamped
import threading

class DebugOffsetPublisher:
    def __init__(self):
        rospy.init_node('debug_offset_publisher')

        # 配信トピック
        self.pub_topic = rospy.get_param('~offset_topic', '/youbot/offset')
        self.pub = rospy.Publisher(self.pub_topic, Vector3Stamped, queue_size=1)

        # デフォルト値
        self.dx = rospy.get_param('~dx', 0.0)
        self.dy = rospy.get_param('~dy', 0.0)
        self.dz = rospy.get_param('~dz', 0.0)

        # 自動更新フラグと周期
        self.auto = rospy.get_param('~auto', False)
        self.rate_hz = rospy.get_param('~rate', 1.0)

        # ユーザー入力スレッド
        if not self.auto:
            threading.Thread(target=self._input_loop, daemon=True).start()

        self.run()

    def _input_loop(self):
        print("デバッグオフセット入力モード: dx dy dz をスペース区切りで入力してください")
        while not rospy.is_shutdown():
            try:
                line = input("> ")
                parts = line.strip().split()
                if len(parts) == 3:
                    self.dx, self.dy, self.dz = map(float, parts)
                    print(f"設定: dx={self.dx}, dy={self.dy}, dz={self.dz}")
                else:
                    print("3つの数値を入力してください (例: 0.1 0.0 -0.2)")
            except Exception as e:
                print(f"入力エラー: {e}")

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            msg = Vector3Stamped()
            msg.header.stamp = rospy.Time.now()
            msg.vector.x = self.dx
            msg.vector.y = self.dy
            msg.vector.z = self.dz
            self.pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        DebugOffsetPublisher()
    except rospy.ROSInterruptException:
        pass
