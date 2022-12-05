#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from environment_controller.srv import use_key
import math
import numpy as np
class occupancy_grid:
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
        rospy.wait_for_service('use_key')
        self.mainloop()
    def lidar_callback(self, data):
        self.lidar_reading = data
        print(self.lidar_reading.ranges)
        print(self.lidar_reading.range_min)
        print(self.lidar_reading.range_max)

    def dog_callback(self, data):
        self.dog = data
    def drone_pos_callback(self, data):
        self.drone_pos = data
    def detect_door(self):
        #get 10 readings from the lidar 
        rate = rospy.Rate(15)
        North=[]
        South=[]
        East=[]
        West=[]
        for i in range(15):
            x=self.lidar_reading.ranges
            #if x is empty continue
            if len(x)==0:
                continue
            #print identifier x
            print("LOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOL")
            North.append(x[3])
            South.append(x[11])
            East.append(x[15])
            West.append(x[7])
            print('-'*50)
            rate.sleep()
        #calculate the variance of North, South, East, West
        North_var=np.var(North)
        South_var=np.var(South)
        East_var=np.var(East)
        West_var=np.var(West)
        #print north, south, east and west variances with the following format
        print('North variance: ', North_var)
        print('South variance: ', South_var)
        print('East variance: ', East_var)
        print('West variance: ', West_var)
        #print a line to identify
        print('-'*50)
        angl = self.lidar_reading.angle_min
        northAngle = angl + self.lidar_reading.angle_increment * 3
        westAngle = angl + self.lidar_reading.angle_increment * 7
        southAngle = angl + self.lidar_reading.angle_increment * 11
        eastAngle = angl + self.lidar_reading.angle_increment * 15
        #the direction with the most variance is the direction of the door
        #if the variance of North is the highest, the door is in the North
        if North_var>South_var and North_var>East_var and North_var>West_var:
            print('Door is in the North')
            distance = (np.mean(North))
            distance += 0.15
            distance = round(distance)
            x = self.drone_pos.pose.position.x + (distance * math.cos(northAngle))
            y = self.drone_pos.pose.position.y + (distance * math.sin(northAngle))
            door_index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
            self.occupancy_grid.data[door_index] = -2
#            self.occupancy_grid_pub.publish(self.occupancy_grid)
            if distance > 1:
                    for j in range((int(distance)+1)):
                        #get the x and y coordinates of the ray
                        xEmpty = (math.cos(northAngle) * j)
                        yEmpty = (math.sin(northAngle) * j)
                        #get the index of the ray in the occupancy grid
                        index = self.occupancy_grid.info.height * (int(xEmpty) + self.occupancy_grid.info.width//2) + int(yEmpty) + self.occupancy_grid.info.height//2
                        #if the index is not out of bounds
                        if index < self.size and index >= 0:
                            #set the cell to empty space
                            self.occupancy_grid.data[index] = 0
            else:
                #get the x and y coordinates of the ray
                xEmpty = (math.cos(northAngle) * distance)
                yEmpty = (math.sin(northAngle) * distance)
                #get the index of the ray in the occupancy grid
                index = self.occupancy_grid.info.height * (int(xEmpty) + self.occupancy_grid.info.width//2) + int(yEmpty) + self.occupancy_grid.info.height//2
                #if the index is not out of bounds
                if index < self.size and index >= 0:
                    #set the cell to occupied space
                    self.occupancy_grid.data[index] = 0
        #if the variance of South is the highest, the door is in the South
        elif South_var>North_var and South_var>East_var and South_var>West_var:
            print('Door is in the South')
            distance = (np.mean(South))
            distance += 0.15
            distance = round(distance)
            x = self.drone_pos.pose.position.x + (distance * math.cos(southAngle))
            y = self.drone_pos.pose.position.y + (distance * math.sin(southAngle))
            door_index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
            self.occupancy_grid.data[door_index] = -2
 #           self.occupancy_grid_pub.publish(self.occupancy_grid)
            if distance > 1:
                    for j in range((int(distance)+1)):
                        #get the x and y coordinates of the ray
                        xEmpty = (math.cos(southAngle) * j)
                        yEmpty = (math.sin(southAngle) * j)
                        #get the index of the ray in the occupancy grid
                        index = self.occupancy_grid.info.height * (int(xEmpty) + self.occupancy_grid.info.width//2) + int(yEmpty) + self.occupancy_grid.info.height//2
                        #if the index is not out of bounds
                        if index < self.size and index >= 0:
                            #set the cell to empty space
                            self.occupancy_grid.data[index] = 0
            else:
                #get the x and y coordinates of the ray
                xEmpty = (math.cos(southAngle) * distance)
                yEmpty = (math.sin(southAngle) * distance)
                #get the index of the ray in the occupancy grid
                index = self.occupancy_grid.info.height * (int(xEmpty) + self.occupancy_grid.info.width//2) + int(yEmpty) + self.occupancy_grid.info.height//2
                #if the index is not out of bounds
                if index < self.size and index >= 0:
                    #set the cell to occupied space
                    self.occupancy_grid.data[index] = 0

        #if the variance of East is the highest, the door is in the East
        elif East_var>North_var and East_var>South_var and East_var>West_var:
            print('Door is in the East')
            distance = (np.mean(East))
            distance += 0.15
            distance = round(distance)
            x = self.drone_pos.pose.position.x + (distance * math.cos(eastAngle))
            y = self.drone_pos.pose.position.y + (distance * math.sin(eastAngle))
            door_index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
            self.occupancy_grid.data[door_index] = -2
#            self.occupancy_grid_pub.publish(self.occupancy_grid)
            if distance > 1:
                    for j in range((int(distance)+1)):
                        #get the x and y coordinates of the ray
                        xEmpty = (math.cos(eastAngle) * j)
                        yEmpty = (math.sin(eastAngle) * j)
                        #get the index of the ray in the occupancy grid
                        index = self.occupancy_grid.info.height * (int(xEmpty) + self.occupancy_grid.info.width//2) + int(yEmpty) + self.occupancy_grid.info.height//2
                        #if the index is not out of bounds
                        if index < self.size and index >= 0:
                            #set the cell to empty space
                            self.occupancy_grid.data[index] = 0
            else:
                #get the x and y coordinates of the ray
                xEmpty = (math.cos(eastAngle) * distance)
                yEmpty = (math.sin(eastAngle) * distance)
                #get the index of the ray in the occupancy grid
                index = self.occupancy_grid.info.height * (int(xEmpty) + self.occupancy_grid.info.width//2) + int(yEmpty) + self.occupancy_grid.info.height//2
                #if the index is not out of bounds
                if index < self.size and index >= 0:
                    #set the cell to occupied space
                    self.occupancy_grid.data[index] = 0

            
        #if the variance of West is the highest, the door is in the West
        elif West_var>North_var and West_var>South_var and West_var>East_var:
            print('Door is in the West')
            distance = (np.mean(West))
            distance += 0.15
            distance = round(distance)
            x = self.drone_pos.pose.position.x + (distance * math.cos(westAngle))
            y = self.drone_pos.pose.position.y + (distance * math.sin(westAngle))
            door_index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
            self.occupancy_grid.data[door_index] = -2
            if distance > 1:
                    for j in range((int(distance)+1)):
                        #get the x and y coordinates of the ray
                        xEmpty = (math.cos(westAngle) * j)
                        yEmpty = (math.sin(westAngle) * j)
                        #get the index of the ray in the occupancy grid
                        index = self.occupancy_grid.info.height * (int(xEmpty) + self.occupancy_grid.info.width//2) + int(yEmpty) + self.occupancy_grid.info.height//2
                        #if the index is not out of bounds
                        if index < self.size and index >= 0:
                            #set the cell to empty space
                            self.occupancy_grid.data[index] = 0
            else:
                #get the x and y coordinates of the ray
                xEmpty = (math.cos(westAngle) * distance)
                yEmpty = (math.sin(westAngle) * distance)
                #get the index of the ray in the occupancy grid
                index = self.occupancy_grid.info.height * (int(xEmpty) + self.occupancy_grid.info.width//2) + int(yEmpty) + self.occupancy_grid.info.height//2
                #if the index is not out of bounds
                if index < self.size and index >= 0:
                    #set the cell to occupied space
                    self.occupancy_grid.data[index] = 0
#            self.occupancy_grid_pub.publish(self.occupancy_grid)


        
    def mainloop(self):
        rate = rospy.Rate(5)
        print(self.occupancy_grid.info.origin.position)
        self.occupancy_grid.data[self.size/2-1] = 0
        if self.dog and (self.dog.x != 0 or self.dog.y != 0):
             
            lookup = self.tfBuffer.lookup_transform('cell_tower', 'world', rospy.Time.now())
            #print('LOOKUP', lookup)
            goal_point = PointStamped("cell_tower",self.dog)
            #print("GOAL_POINT TYPE", type(goal_point))
            #print("GOAL_POINT", goal_point)
            print('SELF.DOG BEFORE TRANSFORM', self.dog)
            goal_point1 = do_transform_point(goal_point, lookup)
            #print('TRANSFORMED DOG POINT',goal_point1)
            self.dog = Vector3(int(goal_point1.point.x), int(goal_point1.point.y), int(goal_point1.point.z))
            print('SELF.DOG', self.dog)
            print('SELF.Drone', self.drone_pos.pose.position)
        #add the position of the dog to the occupancy grid
        dog_index = self.occupancy_grid.info.height * (int(self.dog.x) + self.occupancy_grid.info.width//2) + (int(self.dog.y) + self.occupancy_grid.info.height//2)
        print('DOG_INDEX', int(dog_index))
        self.occupancy_grid.data[int(dog_index)] = -3
        while not rospy.is_shutdown():
            
            #get the angle of the first ray
            #iterate over all the rays
            
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
                        index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
                        self.occupancy_grid.data[int(index)] = 50
                    if i == 1:
                        distance = self.lidar_reading.range_max
                        x = self.drone_pos.pose.position.x + (distance * math.cos(angle))
                        y = self.drone_pos.pose.position.y + (distance * math.sin(angle))
                        index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
                        self.occupancy_grid.data[int(index)] = 50
                    if i == 2:
                        distance = self.lidar_reading.range_max
                        x = self.drone_pos.pose.position.x + (distance * math.cos(angle))
                        y = self.drone_pos.pose.position.y + (distance * math.sin(angle))
                        index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
                        self.occupancy_grid.data[int(index)] = 50
                    if i == 3:
                        distance = self.lidar_reading.range_max
                        x = self.drone_pos.pose.position.x + (distance * math.cos(angle))
                        y = self.drone_pos.pose.position.y + (distance * math.sin(angle))
                        index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
                        self.occupancy_grid.data[int(index)] = 50


                distance += 0.15
                distance = round(distance)
                #everything between the drone position and the distance of the ray is empty space
                x = self.drone_pos.pose.position.x + (distance * math.cos(angle))
                y = self.drone_pos.pose.position.y + (distance * math.sin(angle))
                index = self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)
                self.occupancy_grid.data[int(index)] = 100
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
                
            #create a point using the x and y coordinates from the door set
            #call the use_key service with the point
            #useKeyservice = rospy.ServiceProxy('/use_key', use_key)
            #testDoor = Point(1,0,0)
            #useKeyservice(testDoor)
            #doorIndex = self.occupancy_grid.info.height * (1 + self.occupancy_grid.info.width//2) + (0 + self.occupancy_grid.info.height//2)
            #self.occupancy_grid.data[int(doorIndex)] = -2
            self.detect_door()

            self.occupancy_grid_pub.publish(self.occupancy_grid)                
            #     #calculate the index of the cell in the occupancy grid
                
            #     #set the cell to 100
            #     #increment the angle
            # #TODO: move the drone to where there is empty space
            
            
            # #publish the occupancy grid
            # print(self.occupancy_grid.data)
            rate.sleep()
if __name__ == '__main__':
  rospy.init_node('occupancy_grid_node')
  try:
    t = occupancy_grid()
  except rospy.ROSInterruptException:
    pass