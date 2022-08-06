from turtle import pos
import rospy
import numpy as np
from math import *
from obstacle_detector.msg import Obstacles
from robot_package.msg import Obstacle_pos
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def main():
    # Looking for nearest 5 obstacles
    obsx_array = []
    obsy_array = []
    obsr_array = []
    obsx_array = np.array(obsx_array)
    obsy_array = np.array(obsy_array)
    obsr_array = np.array(obsr_array)

    # Robot position subscriber callback
    def odom_callback(odom_data):
        global x_pos  # x_position
        global y_pos  # y_position
        global t_pos  # orientation
        x_pos = odom_data.pose.pose.position.x
        y_pos = odom_data.pose.pose.position.y
        [a, b, t_pos] = euler_from_quaternion(
            [odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z,
             odom_data.pose.pose.orientation.w])

    # Obstacle position callback function
    def callback(data):
        # Defining array of obstacles parameters
        x_pos_obstacle = []
        y_pos_obstacle = []
        r_pos_obstacle = []
        dist = []
        x_pos_obstacle = np.array(x_pos_obstacle)
        y_pos_obstacle = np.array(y_pos_obstacle)
        r_pos_obstacle = np.array(r_pos_obstacle)
        dist = np.array(dist)

        length = len(data.circles)
        for i in range(length):
            xloc = data.circles[i].center.x  # X location of obstacle from body frame
            yloc = data.circles[i].center.y  # Y location of obstacle from body frame
            radius = data.circles[i].radius  # Radius of obstacle
            d = sqrt(xloc * xloc + yloc * yloc).real  # Distance of obstacles from robot body

            if d <= 5:  # Taking obstacles whose distance is less than 5 meters
                dist = np.append(dist, d)
                x_pos_obstacle = np.append(x_pos_obstacle, xloc)
                y_pos_obstacle = np.append(y_pos_obstacle, yloc)
                r_pos_obstacle = np.append(r_pos_obstacle, radius)

        sort_dist = np.sort(dist)  # Sorting the distances

        for k in range(len(sort_dist)):
            location = np.where(dist == sort_dist[k])  # Location of nearest obstacles in ascending order
            obsx_array[k] = x_pos_obstacle[location]
            obsy_array[k] = y_pos_obstacle[location]
            obsr_array[k] = r_pos_obstacle[location]

            # if x_pos_obstacle[0] != x_pos_c and y_pos_obstacle[0] != y_pos_c:         # This part needs some amendation
            x_pos_c = x_pos
            y_pos_c = y_pos
            t_pos_c = t_pos

            # Converting the obstacle loocation from body frame to global frame to put it into the path planner
            obsx_array[k] = (x_pos_c + x_pos_obstacle[k] * cos(t_pos_c) - y_pos_obstacle[k] * sin(
                t_pos_c)).real  # X location on global coordinate system
            obsy_array[k] = (y_pos_c + y_pos_obstacle[k] * cos(t_pos_c) + x_pos_obstacle[k] * sin(
                t_pos_c)).real  # Y location on global coordinate system
            obsr_array[k] = (r_pos_obstacle[k]).real  # radius

        # Setting message to publisher object
        obs.locx = obsx_array
        obs.locy = obsy_array
        obs.radius = obsr_array
        pub.publish(obs)  # Obstacles location in global frame is publiishes in /obstacles_location topic

    rospy.init_node('obstacle_publisher', anonymous=False)  # Initializing the node
    rospy.Subscriber('/obstacles', Obstacles,
                     callback)  # /obstacles subscriber from obstacle detector package
    rospy.Subscriber('/odom', Odometry, odom_callback)  # /odom subscriber from position publisher node
    pub = rospy.Publisher("obstacles_location", Obstacle_pos, queue_size=1)  # Publisher Object
    obs = Obstacle_pos()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
