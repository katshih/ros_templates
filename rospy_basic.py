#! /usr/bin/env python
import rospy

# import numpy as np
# from rospy.numpy_msg import numpy_msg

# from tf2_msgs.msg import {TFMessage}
# from nav_msgs.msg import {Odometry}
# from sensor_msgs.msg import {PointCloud2, LaserScan, PointField}
# from geometry_msgs.msg import {Twist, TransformStamped, Polygon, PolygonStamped, Point32}
# from std_msgs.msg import {String, Float32}

class class_name:
    def __init__(self, args):
        self.args = args

        rospy.Subscriber("{topic_name}", {message_type}, self.callback_function)

        self.publisher =  rospy.Publisher("{topic_name}", {message_type}, queue_size = 10)

    def callback_function(self, message):
        return None

    def publishing_function(self):
        new_message = {message_type}
        new_message.header.stamp = rospy.Time.now()
        # fill in rest of values

        self.publisher.publish(new_message)

if __name__ == '__main__':
    rospy.init_node('{node_name}')

    args = None
    node = class_name(args)

    # if only triggered by callback functions
    # rospy.spin()

    # otherwise
    rate = rospy.Rate({Hz})
    while not rospy.is_shutdown():
        node.publishing_function()
        rate.sleep()