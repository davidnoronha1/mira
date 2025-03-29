#!/usr/bin/python3

import rospy
from std_msgs.msg import Char

if __name__ == "__main__":
    rospy.init_node("key_mapper")
    pub = rospy.Publisher("/keys", Char, queue_size=1)
    while not rospy.is_shutdown():
        c = Char()
        # print("Send input:")
        input_char = input()
        if len(input_char) == 1:
            c.data = ord(input_char)
            pub.publish(c)
        else:
            print("Please input only one character.")
