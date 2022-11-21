#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid

class occupancy_grid:
    def __init__(self):
        self.lidar_reading = Vector3()
        self.lidar_sub = rospy.Subscriber('/lidar_reading', Vector3, self.lidar_callback)
        self.occupancy_grid = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.mainloop()
    def lidar_callback(self, data):
        self.lidar_reading = data

    def mainloop(self):
        #rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            #get each of the lidar readings around the drone and determine if there is an obstacle or not
            #if there is an obstacle, update the occupancy grid

            if self.lidar_reading and (self.lidar_reading.x != 0 or self.lidar_reading.y != 0):
                
                self.occupancy_grid.publish(self.lidar_reading)
            #if there is not an obstacle, update the occupancy grid
            #rate.sleep()
            pass

