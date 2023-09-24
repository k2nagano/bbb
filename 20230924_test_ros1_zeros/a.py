import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class Test:
    def __init__(self):
        rospy.init_node("a", anonymous=True)
        pub = rospy.Publisher("/aaa_state", JointState, queue_size=10)
        sub = rospy.Subscriber("/aaa_command", Float64MultiArray, self.sub_callback)

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            # msg.data = [self.i * 0.01, self.i * 0.01]
            msg.position = [1.23, 4.56]
            pub.publish(msg)
            rospy.loginfo(f"[PUB]{msg.position}")
            rate.sleep()

    def sub_callback(self, msg):
        rospy.loginfo(f"[SUB]{msg.data}")


def main(args=None):
    try:
        test = Test()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
