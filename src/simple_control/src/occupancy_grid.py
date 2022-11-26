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
            if self.lidar_reading and (self.lidar_reading.x != 0 or self.lidar_reading.y != 0):
                
                #update the current cell (drone's position)of the occupancy grid to 100
                
                self.occupancy_grid.data[int(self.drone_pos.x)] = 100
            elif self.lidar_reading and (self.lidar_reading.x == 0 or self.lidar_reading.y == 0):
                    
                    #update the current cell (drone's position)of the occupancy grid to 0
                self.occupancy_grid.data[int(self.drone_pos.x)] = 0

