#!/usr/bin/env python

from math import sin, cos, pi
import serial
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from parameters import *


def main():
    # Initializing ROS
    rospy.init_node('position_publisher', anonymous=False)  # Node Initialization
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)  # Publisher object define
    odom_broadcaster = tf.TransformBroadcaster()  # Broadcaster object

    ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)  # Arduino Serial port

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    # Initializing variables
    td1 = 0
    td2 = 0

    x = 0
    y = 0
    t = 0

    xp = 0
    yp = 0
    tp = 0

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        dt = (current_time - last_time).to_sec()  # Change in time calculation

        if dt == 0:
            dt = 0.001

        try:
            data = ser.readline().decode("utf-8", errors='ignore').strip().split(",")  # Serial read from arduino
        except:
            print("\nUnable to read Serial. Retrying..........")
            ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)  # Arduino Serial port
            continue

        try:
            dleft = float(data[0])  # Conversion of string data to float
            dright = float(data[1])
        except:
            continue

        # Robot states calculation fron individual wheel travel distance
        # dleft= left wheel travel distance
        # dright= right wheel travel distance

        dd1 = dleft - td1
        dd2 = dright - td2
        td1 = dleft
        td2 = dright

        dc = (dd1 + dd2) / 2
        x = x + dc * cos(t)
        y = y + dc * sin(t)
        t = t + (dd1 - dd2) / wheel_base

        # Velocity Calculation
        vx = (x - xp) / dt
        vy = (y - yp) / dt
        vt = (t - tp) / dt

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, t)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vt))

        # publish the message
        odom_pub.publish(odom)

        xp = x
        yp = y
        tp = t
        last_time = current_time


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
