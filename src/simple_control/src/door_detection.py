#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class door_detection:
    def __init__(self):
        self.lidar_reading = LaserScan()
        self.lidar_sub = rospy.Subscriber('/uav/sensors/lidar', LaserScan, self.lidar_callback)
        self.drone_pos = PoseStamped()
        self.drone_pos_sub = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.drone_pos_callback)
        #self.drone_pos_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)
        self.occupancy_grid = OccupancyGrid()
        self.dog = Vector3()
        self.dog_sub = rospy.Subscriber('/cell_tower/position', Vector3, self.dog_callback)
        #initialize all cells of the occupancy grid to 50
        
        self.occupancy_grid.info.map_load_time = rospy.Time.now()
        self.occupancy_grid.header.frame_id = 'world'
        self.occupancy_grid.header.stamp = rospy.Time.now()
        
        self.occupancy_grid.info.resolution = 1
        #get the map_width argument from fly.launch
        self.occupancy_grid.info.width = rospy.get_param('environment_controller/map_width')
        self.occupancy_grid.info.height = rospy.get_param('environment_controller/map_height')
        self.size = self.occupancy_grid.info.width * self.occupancy_grid.info.height
        self.occupancy_grid.data = [50 for i in range(self.size)]
        self.occupancy_grid.info.origin.position.x = float(-(self.occupancy_grid.info.width/float(2)))
        self.occupancy_grid.info.origin.position.y = float(-(self.occupancy_grid.info.height/float(2)))
        self.occupancy_grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.north = []
        self.west = []
        self.south = []
        self.east = []
        self.mainloop()
        
    def lidar_callback(self, data):
        self.lidar_reading = data
    def dog_callback(self, data):
        self.dog = data
    def drone_pos_callback(self, data):
        self.drone_pos = data

    def varianceOfLists(self, list1, list2, list3, list4):
        northVariance = np.var(list1)
        westVariance = np.var(list2)
        southVariance = np.var(list3)
        eastVariance = np.var(list4)
        variances = [northVariance, westVariance, southVariance, eastVariance]
        #return the list with the highest variance
        return max(variances), variances.index(max(variances))
    
    def mainloop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            North = 0
            West = 0
            South = 0
            East = 0
            for i in range(len(self.lidar_reading.ranges)):
                if i == 3:
                    North = self.lidar_reading.ranges[i]
                if i == 7:
                    West = self.lidar_reading.ranges[i]
                if i == 11:
                    South = self.lidar_reading.ranges[i]
                if i == 15:
                    East = self.lidar_reading.ranges[i]
            angl = self.lidar_reading.angle_min
            northAngle = angl + self.lidar_reading.angle_increment * 3
            westAngle = angl + self.lidar_reading.angle_increment * 7
            southAngle = angl + self.lidar_reading.angle_increment * 11
            eastAngle = angl + self.lidar_reading.angle_increment * 15
            cardinalAngles = [northAngle, westAngle, southAngle, eastAngle]
            cardinals = [North, West, South, East]
            print('ANGLANGL', angl)
            
            for i in range(len(cardinals)):
                #get the distance of the ray
                angle = cardinalAngles[i]
                distance = cardinals[i]
                
                #append the distance to the appropriate list
                if i == 0:
                    self.north.append(distance)
                if i == 1:
                    self.west.append(distance)
                if i == 2:
                    self.south.append(distance)
                if i == 3:
                    self.east.append(distance)
            
            maxVar, maxVarIndex = self.varianceOfLists(self.north, self.west, self.south, self.east)
            print('MAXVAR AND MAXINDEX', maxVar, maxVarIndex)
            distance1 = round(cardinals[maxVarIndex])
            x = self.drone_pos.pose.position.x + (distance1 * math.cos(cardinalAngles[maxVarIndex]))
            y = self.drone_pos.pose.position.y + (distance1 * math.sin(cardinalAngles[maxVarIndex]))
            index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
            self.occupancy_grid.data[int(index)] = -2


            self.occupancy_grid_pub.publish(self.occupancy_grid)

            
            
            


                
                

            #create a point using the x and y coordinates from the door set
            #call the use_key service with the point

            
        

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('door_detection_node')
    try:
        dd = door_detection()
    except rospy.ROSInterruptException:
        pass