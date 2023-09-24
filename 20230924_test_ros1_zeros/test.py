#!/usr/bin/env python

import rospy
import time
import numpy as np
from trajectory import *
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class Test:
    def __init__(self):
        rospy.init_node('test', anonymous=True)
        # rospy.Subscriber("/aaa_state", JointState, self.callback)
        js = rospy.wait_for_message("/aaa_state", JointState, timeout=None)
        act_pos = js.position[:]
        interpolation_method = "minjerk"
        duration = max(np.floor(np.abs(np.max(act_pos)) * 2.5), 2.0)
        sampling = 0.01
        print(f"{duration} sec")
        ts = np.arange(0.0, duration, sampling)
        waypg = WayPointGenerator(
            sampling,
            start_point=act_pos,
            dimensions=2,
            interpolation_method=interpolation_method,
        )
        waypg.append_way_point([0.0, 0.0], duration * 0.9)
        waypg.append_way_point([0.0, 0.0], duration * 0.1)
        waypg.print_settings()

        ys_waypg = []
        for _ in enumerate(ts):
            ys_waypg.append(waypg.generate_command())

        pub = rospy.Publisher('/aaa_command', Float64MultiArray, queue_size=10)
        loop_rate = int(1./sampling)  # hz
        rate = rospy.Rate(loop_rate)
        for y in ys_waypg:
            if rospy.is_shutdown():
                print("aaa")
                break
            msg = Float64MultiArray()
            msg.data = y[:]
            pub.publish(msg)
            print(f"{msg.data}")
            rate.sleep()

    def callback(self, data):
        self.act_pos = [data.position[0], data.position[1]]

    def get_act_pos(self):
        return self.act_pos


if __name__ == '__main__':
    try:
        test = Test()
    except rospy.ROSInterruptException:
        pass