#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0
    radius = v / omega
    steering = math.atan(wheelbase / radius)
    rospy.logdebug(f"v={v}, omega={omega}, radius={radius}, steering={steering}")
    return steering

def cmd_callback(data):
    global wheelbase, pub, ackermann_cmd_topic, frame_id
    v = data.linear.x
    omega = data.angular.z
    steering = convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase)

    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.drive.steering_angle = steering
    msg.drive.speed = v
    pub.publish(msg)
    rospy.loginfo(f"Published: speed={v}, steering={steering}")

if __name__ == '__main__':
    rospy.init_node('twist_to_ackermann')
    
    wheelbase = rospy.get_param('~wheelbase', 0.36)
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
    frame_id = rospy.get_param('~frame_id', 'odom')

    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    
    rospy.loginfo(f"Starting conversion: wheelbase={wheelbase}, input={twist_cmd_topic}, output={ackermann_cmd_topic}")
    rospy.spin()
