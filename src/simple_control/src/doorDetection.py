#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import math


class DoorDetection:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
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
    def lidar_callback(self, data):
        self.lidar_reading = data
    def dog_callback(self, data):
        self.dog = data
    def drone_pos_callback(self, data):
        self.drone_pos = data
    
    def mainloop(self):
        rate = rospy.Rate(10)
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
            print('CARDINALS', cardinals)
            print('NORTH', North)
            
            for i in range(len(cardinals)):
                #get the distance of the ray
                angle = cardinalAngles[i]
                distance = cardinals[i]
                
                if distance == float('inf'):
                    if i == 0:
                        distance = self.lidar_reading.range_max
                        x = self.drone_pos.pose.position.x + (distance * math.cos(angle))
                        y = self.drone_pos.pose.position.y + (distance * math.sin(angle))
                        print('DISTANCE: ', distance,'ANGLE: ', angle, 'X: ', x,'Y: ', y)
                        index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
                        self.occupancy_grid.data[int(index)] = 50
                    if i == 1:
                        distance = self.lidar_reading.range_max
                        x = self.drone_pos.pose.position.x + (distance * math.cos(angle))
                        y = self.drone_pos.pose.position.y + (distance * math.sin(angle))
                        print('DISTANCE: ', distance,'ANGLE: ', angle, 'X: ', x,'Y: ', y)
                        index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
                        self.occupancy_grid.data[int(index)] = 50
                    if i == 2:
                        distance = self.lidar_reading.range_max
                        x = self.drone_pos.pose.position.x + (distance * math.cos(angle))
                        y = self.drone_pos.pose.position.y + (distance * math.sin(angle))
                        print('DISTANCE: ', distance,'ANGLE: ', angle, 'X: ', x,'Y: ', y)
                        index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
                        self.occupancy_grid.data[int(index)] = 50
                    if i == 3:
                        distance = self.lidar_reading.range_max
                        x = self.drone_pos.pose.position.x + (distance * math.cos(angle))
                        y = self.drone_pos.pose.position.y + (distance * math.sin(angle))
                        print('DISTANCE: ', distance,'ANGLE: ', angle, 'X: ', x,'Y: ', y)
                        index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
                        self.occupancy_grid.data[int(index)] = 50


                print(distance)
                distance += 0.15
                distance = round(distance)
                print('ROUNDED DISTANCE', distance, i)
                print('ANGLE', angle)
                #everything between the drone position and the distance of the ray is empty space
                x = self.drone_pos.pose.position.x + (distance * math.cos(angle))
                y = self.drone_pos.pose.position.y + (distance * math.sin(angle))
                print('DISTANCE: ', distance,'ANGLE: ', angle, 'X: ', x,'Y: ', y)
                index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
                self.occupancy_grid.data[int(index)] = 100
                    door.append((int(x), int(y)))
                    print('EAST DOOR', door)

                if distance > 1:
                    for j in range((int(distance)+1)):
                        #get the x and y coordinates of the ray
                        xEmpty = (math.cos(angle) * j)
                        yEmpty = (math.sin(angle) * j)
                        #get the index of the ray in the occupancy grid
                        index = self.occupancy_grid.info.height * (int(xEmpty) + self.occupancy_grid.info.width//2) + int(yEmpty) + self.occupancy_grid.info.height//2
                        #if the index is not out of bounds
                        if index < self.size and index >= 0:
                            #set the cell to empty space
                            self.occupancy_grid.data[index] = 0
                else:
                    #get the x and y coordinates of the ray
                    xEmpty = (math.cos(angle) * distance)
                    yEmpty = (math.sin(angle) * distance)
                    #get the index of the ray in the occupancy grid
                    index = self.occupancy_grid.info.height * (int(xEmpty) + self.occupancy_grid.info.width//2) + int(yEmpty) + self.occupancy_grid.info.height//2
                    #if the index is not out of bounds
                    if index < self.size and index >= 0:
                        #set the cell to occupied space
                        self.occupancy_grid.data[index] = 0
                print('X', x)
                print('Y', y)
                print('INDEX: ',int(index))
            


                
                

            #create a point using the x and y coordinates from the door set
            #call the use_key service with the point

            
        

            self.occupancy_grid_pub.publish(self.occupancy_grid)