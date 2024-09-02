#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import math

def publish_rotate_transform(angular_velocity):
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(30.0)  # 10Hz

    while not rospy.is_shutdown():
        now = rospy.Time.now()  # 获取当前时间

        # 定义变换
        broadcaster.sendTransform(
            (0, 0, 0),             # translation: 在原点 (x, y, z)
            (0, 0, math.sin(angular_velocity * now.to_sec()), math.cos(angular_velocity * now.to_sec())),  # rotation: 绕z轴旋转的四元数
            now,                   # time
            "rotate",              # child frame
            "world"                # parent frame
        )

        rate.sleep()


def main():
    rospy.init_node('rotate_tf_for_vis')
    angular_velocity = 0.01  # 给定角速度，单位：弧度/秒
    publish_rotate_transform(angular_velocity)

if __name__ == '__main__':
    main()