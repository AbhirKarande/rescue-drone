#!/usr/bin/env python
import rospy
import time
import numpy as np

from gui import GUI
from std_msgs.msg import Int32MultiArray, Bool
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math


class GUI_Controller():

    def __init__(self):
        # When this node shutsdown
        rospy.on_shutdown(self.shutdown_sequence)

        # Set the rate
        self.rate = 5.0
        self.dt = 1.0 / self.rate

        # Environment details
        self.obstacle_list = []
        self.door_list = []
        self.lidar = []
        self.map_size = (11, 11)
        self.path = np.array([[0, 0]])

        # Create the position
        self.position = np.zeros(3, dtype=np.float64)
        self.quaternion = np.zeros(4)
        self.yaw = 0

        self.crashed = False
        self.goal = None

        env_data = {'path': self.path, 'environment': self.obstacle_list}
        gui_data = {'quad1': {'position': [0, 0], 'orientation': [0, 0], 'L': 0.25}}

        # Create the subscribers and publishers
        self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)
        self.gps_sub = rospy.Subscriber("/uav/sensors/crash_detector", Bool, self.get_crash)
        self.path_sub = rospy.Subscriber('/uav/path', Int32MultiArray, self.get_path)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)
        self.laser_sub = rospy.Subscriber("/uav/sensors/lidar", LaserScan, self.get_laser)

        time.sleep(3)

        # Create the gui object
        self.gui_object = GUI(quads=gui_data, env=env_data, map_size=self.map_size)

        # Run the communication node
        self.UpdateLoop()

    def get_crash(self, msg):
        if msg.data:
            self.crashed = True

    # This is the main loop of this class
    def UpdateLoop(self):
        # Set the rate
        rate = rospy.Rate(self.rate)

        # While running
        while not rospy.is_shutdown():

            # Display the position
            self.gui_object.world["path"] = self.path
            self.gui_object.world["obstacles"] = self.obstacle_list
            self.gui_object.world["doors"] = self.door_list
            self.gui_object.quads['quad1']['position'] = list(self.position)
            self.gui_object.quads['quad1']['orientation'] = list(euler_from_quaternion(self.quaternion))
            self.gui_object.world['lidar'] = self.lidar
            self.gui_object.crashed = self.crashed
            if self.goal is not None:
                self.gui_object.world['goal'] = self.goal
            else:
                if 'goal' in self.gui_object.world:
                    del self.gui_object.world['goal']

            if not self.gui_object.crashed:
                self.gui_object.update()

            # Sleep any excess time
            rate.sleep()

    def get_path(self, msg):
        self.path = np.array(np.reshape(msg.data, (-1, 2)))
        self.path[:, 0] = self.path[:, 0]
        self.path[:, 1] = self.path[:, 1]

    # Call back to get the gps data
    def get_gps(self, msg):
        self.position[0] = msg.pose.position.x
        self.position[1] = msg.pose.position.y
        self.position[2] = msg.pose.position.z

        self.quaternion = (msg.pose.orientation.x,
                           msg.pose.orientation.y,
                           msg.pose.orientation.z,
                           msg.pose.orientation.w)
        _, _, self.yaw = euler_from_quaternion(self.quaternion)

    def get_laser(self, msg):
        self.lidar = []
        # print('Drone position', self.position[0], self.position[1])
        for index, range in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment  # - self.yaw
            position = (self.position[0] + range * math.cos(angle),
                        self.position[1] + range * math.sin(angle))
            # print('angle', angle)
            # print('range', range)
            # print('position', position)
            self.lidar.append(position)
        self.lidar = np.array(self.lidar)

    def get_map(self, msg):

        self.updateaxis = True

        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution

        self.start_x = msg.info.origin.position.x
        self.start_y = msg.info.origin.position.y

        self.end_x = self.start_x + (width * res)
        self.end_y = self.start_y + (height * res)

        map_data = np.reshape(msg.data, (width, height))
        self.map_size = (width / 2.0, height / 2.0)
        self.obstacle_list[:] = []  # clear the list
        self.door_list[:] = []  # clear the list
        self.goal = None
        for xi in range(0, width):
            for yi in range(0, height):
                if map_data[xi, yi] == -1:
                    self.door_list.append((xi + self.start_x, yi + self.start_y, 'closed'))
                elif map_data[xi, yi] == -2:
                    self.door_list.append((xi + self.start_x, yi + self.start_y, 'open'))
                elif map_data[xi, yi] == -3:
                    self.goal = (xi + self.start_x, yi + self.start_y)
                else:
                    self.obstacle_list.append((xi + self.start_x, yi + self.start_y, map_data[xi, yi]))

            # Called on ROS shutdown

    def shutdown_sequence(self):
        rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
    rospy.init_node('GUI_Controller_Node')
    try:
        v = GUI_Controller()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
