#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

class occupancy_grid:
    def __init__(self):
        self.lidar_reading = LaserScan()
        self.lidar_sub = rospy.Subscriber('/uav/sensors/lidar', LaserScan, self.lidar_callback)
        self.drone_pos = Vector3()
        self.drone_pos_sub = rospy.Subscriber('/uav/sensors/gps', Vector3, self.drone_pos_callback)
        self.occupancy_grid = OccupancyGrid()
        #initialize all cells of the occupancy grid to 50
        self.occupancy_grid.data = [50 for i in range(100)]
        self.occupancy_grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.mainloop()
    def lidar_callback(self, data):
        self.lidar_reading = data
    def drone_pos_callback(self, data):
        self.drone_pos = data
    def mainloop(self):
        #rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            #get the angle of the first ray
            angle = self.lidar_reading.angle_min
            #iterate over all the rays
            for i in range(len(self.lidar_reading.ranges)):
                #get the distance of the ray
                distance = self.lidar_reading.ranges[i]
                #calculate the x and y coordinates of the ray
                x = self.drone_pos.x + distance * math.cos(angle)
                y = self.drone_pos.y + distance * math.sin(angle)
                #calculate the index of the cell in the occupancy grid
                index = int(x) + int(y) * 10
                #set the cell to 100
                self.occupancy_grid.data[index] = 100
                #increment the angle
                angle += self.lidar_reading.angle_increment
            #publish the occupancy grid
            self.occupancy_grid_pub.publish(self.occupancy_grid)
            #rate.sleep()


            #publish the occupancy grid
            self.occupancy_grid_pub.publish(self.occupancy_grid)


