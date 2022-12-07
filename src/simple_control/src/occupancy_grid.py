#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from simple_control.src.aStar import AStarPlanner
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from environment_controller.srv import use_key
import math
import time
import numpy as np
# from enum import Enum

# class DroneState(Enum):
#     GOAL_TRANSFORM = 1
#     OCCUPANCY_EXPLORE = 2
#     DOOR_DETECT = 3
#     PATH_PLAN = 4
#     MOVE = 5


class occupancy_grid:
    def __init__(self):
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.lidar_reading = LaserScan()
        self.lidar_sub = rospy.Subscriber('/uav/sensors/lidar', LaserScan, self.lidar_callback)
        self.drone_pos = PoseStamped()
        self.drone_pos_sub = rospy.Subscriber('/uav/sensors/gps', PoseStamped, self.drone_pos_callback)
        self.drone_pos_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)
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
        self.dog_detected=False
        self.have_plan=False
        self.dogCoords=(0,0)
        self.path = []
        
        
        self.mainloop()
    def lidar_callback(self, data):
        self.lidar_reading = data
        print(self.lidar_reading.ranges)
        print(self.lidar_reading.range_min)
        print(self.lidar_reading.range_max)
    def euclidean_to_grid(self, x, y):
        return self.occupancy_grid.info.height * (int(x) + self.occupancy_grid.info.width//2) + int(y) + (self.occupancy_grid.info.height//2)


    def dog_callback(self, data):
        self.dog = data
    def drone_pos_callback(self, data):
        self.drone_pos = data
    def detect_dog(self):
        rate = rospy.Rate(10)
        #call waitForTransform to make sure the transform is available
        
        for i in range(3): 
            lookup = self.tfBuffer.lookup_transform('world', 'cell_tower', rospy.Time.now())
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
            
            rate.sleep()
        #set the dog cell to -3
        self.occupancy_grid.data[self.euclidean_to_grid(self.dog.x, self.dog.y)] = -3
    
    def path_plan(self):
        #convert self.drone_pos.pose.position to an array of x and y
        #convert self.dog to an array of x and y
        
        drone_pos = [self.drone_pos.pose.position.x, self.drone_pos.pose.position.y]
        dog_pos = [self.dog.x, self.dog.y]
        #create goal_pos, a visible cell closest to the dog
        
        if not self.have_plan:
            astar = AStarPlanner(safe_distance=1)
            path = astar.plan(self.map, drone_pos, )
            if path != None:
                path = np.array(path)
                self.have_plan = True
                path[:,0] = path[:,0] - self.occupancy_grid.info.width//2
                path[:,1] = path[:,1] - self.occupancy_grid.info.height//2
        else:
            rospy.loginfo(str(rospy.get_name()) + "No Path")  







    def move(self, x, y):
        #create a vector3 message
        #set the x and y values of the message to x and y
        #publish the message
        move_msg = Vector3(x, y, 5)
        self.drone_pos_pub.publish(move_msg)

    def detect_door(self):
        #get 10 readings from the lidar 
        rate = rospy.Rate(15)
        North=[]
        South=[]
        East=[]
        West=[]
        for i in range(10):
            x=self.lidar_reading.ranges
            #if x is empty continue
            if len(x)==0:
                continue
            #print identifier x
            if x[3]!='inf':
                North.append(x[3])
            if x[15]!='inf':
                East.append(x[15])
            if x[11]!='inf':
                South.append(x[11])
            if x[7]!='inf':
                West.append(x[7])
            print('-'*50)
            rate.sleep()
        #calculate the variance of North, South, East, West
        North_var=np.std(North)
        South_var=np.std(South)
        East_var=np.std(East)
        West_var=np.std(West)

        if North_var>.02 and min(North)<1:
            northAngle = self.lidar_reading.angle_min + self.lidar_reading.angle_increment * 3
            dist = round(np.mean(North)+.175)
            x = self.drone_pos.pose.position.x + (dist * math.cos(northAngle))
            y = self.drone_pos.pose.position.y + (dist * math.sin(northAngle))
            pos=self.euclidean_to_grid(round(x),round(y))
            self.occupancy_grid.data[int(pos)] = -1
            useKeyservice = rospy.ServiceProxy('/use_key', use_key)
            testDoor = Point(x,y,0)
            bool1 = useKeyservice(testDoor)
            rospy.sleep(2)
            if not bool1:
                self.occupancy_grid.data[int(pos)] = 100
            else:
                self.occupancy_grid.data[int(pos)] = -2

            self.occupancy_grid.data[int(pos)] = -2
            return
        elif South_var>.02 and min(South)<1:
            southAngle = self.lidar_reading.angle_min + self.lidar_reading.angle_increment * 11
            dist = int(np.mean(South)+.175)
            x = self.drone_pos.pose.position.x + (dist * math.cos(southAngle))
            y = self.drone_pos.pose.position.y + (dist * math.sin(southAngle))
            pos=self.euclidean_to_grid(x,y)
            self.occupancy_grid.data[int(pos)] = -1
            useKeyservice = rospy.ServiceProxy('/use_key', use_key)
            testDoor = Point(x,y,0)
            bool1=useKeyservice(testDoor)
            rospy.sleep(2)
            if not bool1:
                self.occupancy_grid.data[int(pos)] = 100
            else:
                self.occupancy_grid.data[int(pos)] = -2
            print('door opened')
            
            self.occupancy_grid.data[int(pos)] = -2
            return
        elif East_var>.02 and min(East)<1:
            eastAngle = self.lidar_reading.angle_min + self.lidar_reading.angle_increment * 15
            dist = round(np.mean(East)+.175)
            x = round(self.drone_pos.pose.position.x + (dist * math.cos(eastAngle)))
            y = round(self.drone_pos.pose.position.y + (dist * math.sin(eastAngle)))
            pos=self.euclidean_to_grid(x,y)
            self.occupancy_grid.data[int(pos)] = -1
            useKeyservice = rospy.ServiceProxy('/use_key', use_key)
            testDoor = Point(x,y,0)
            bool1=useKeyservice(testDoor)
            rospy.sleep(2)
            if not bool1:
                self.occupancy_grid.data[int(pos)] = 100
            else:
                self.occupancy_grid.data[int(pos)] = -2
            print(East)
            print(East_var)
            print("LSLGKDHGLSKDLHGKSLGNLKDSNGKLSDGNISLDGHLSIDHGLISDHGOIGHSIODGHOSIDGHOISDGHOISDGh")
            self.occupancy_grid.data[int(pos)] = -2
            return
        
        elif West_var>.02 and min(West)<1:
            westAngle = self.lidar_reading.angle_min + self.lidar_reading.angle_increment * 7
            dist = round(np.mean(West)+.175)
            x = self.drone_pos.pose.position.x + (dist * math.cos(westAngle))
            y = self.drone_pos.pose.position.y + (dist * math.sin(westAngle))
            pos=self.euclidean_to_grid(x,y)
            self.occupancy_grid.data[int(pos)] = -1
            useKeyservice = rospy.ServiceProxy('/use_key', use_key)
            testDoor = Point(x,y,0)
            bool1 = useKeyservice(testDoor)
            rospy.sleep(2)
            if not bool1:
                self.occupancy_grid.data[int(pos)] = 100
            else:
                self.occupancy_grid.data[int(pos)] = -2
            return
    def dog_detection(self):   
        if self.dog and (self.dog.x != 0 or self.dog.y != 0) and not self.dog_detected:
           
            lookup = self.tfBuffer.lookup_transform('cell_tower', 'world', rospy.Time.now())
            #print('LOOKUP', lookup)
            goal_point = PointStamped("cell_tower",self.dog)
            #print("GOAL_POINT TYPE", type(goal_point))
            #print("GOAL_POINT", goal_point)
            print('SELF.DOG BEFORE TRANSFORM', self.dog)
            goal_point1 = do_transform_point(goal_point, lookup)
            #print('TRANSFORMED DOG POINT',goal_point1)
            self.dog = Vector3((goal_point1.point.x), (goal_point1.point.y), (goal_point1.point.z))
            print((self.dog.x))
            print((self.dog.y))
            print('*'*100)
            dog_index = self.euclidean_to_grid(round(self.dog.x), round(self.dog.y+.01))
            self.occupancy_grid.data[int(dog_index)] = -3
            self.dog_detected = True
            print('SELF.DOG', self.dog)
            print('SELF.Drone', self.drone_pos.pose.position)
            
                

    def occupancygrid_explore(self):            
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
            #print cardinalAngles and cardinals
            print('CARDINAL ANGLE', cardinalAngles)
            print('CARDINAL', cardinals)
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
            if self.occupancy_grid.data[int(index)] == -1 or self.occupancy_grid.data[int(index)] == -2:
                pass
            else:
                self.occupancy_grid.data[int(index)] = 100

            if distance > 1:
                for j in range((int(distance)+1)):
                    #get the x and y coordinates of the ray
                    xEmpty = (math.cos(angle) * j)
                    yEmpty = (math.sin(angle) * j)
                    #get the index of the ray in the occupancy grid
                    index = self.occupancy_grid.info.height * (int(xEmpty) + self.occupancy_grid.info.width//2) + int(yEmpty) + self.occupancy_grid.info.height//2
                    if(self.occupancy_grid.data[index]==-1 or self.occupancy_grid.data[index]==-2):
                        continue
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
        
    def mainloop(self):
        rate = rospy.Rate(5)
        print(self.occupancy_grid.info.origin.position)
        self.occupancy_grid.data[self.size/2-1] = 0
        #add the position of the dog to the occupancy grid
        while not rospy.is_shutdown():

        



            
            #get the angle of the first ray
            #iterate over all the rays
            
            try:
                if not self.dog_detected:
                    if self.dog and (self.dog.x != 0 or self.dog.y != 0) and not self.dog_detected:
           
                        lookup = self.tfBuffer.lookup_transform('cell_tower', 'world', rospy.Time.now())
                        #print('LOOKUP', lookup)
                        goal_point = PointStamped("cell_tower",self.dog)
                        #print("GOAL_POINT TYPE", type(goal_point))
                        #print("GOAL_POINT", goal_point)
                        print('SELF.DOG BEFORE TRANSFORM', self.dog)
                        goal_point1 = do_transform_point(goal_point, lookup)
                        #print('TRANSFORMED DOG POINT',goal_point1)
                        self.dog = Vector3((goal_point1.point.x), (goal_point1.point.y), (goal_point1.point.z))
                        print((self.dog.x))
                        print((self.dog.y))
                        print('*'*100)
                        dog_index = self.euclidean_to_grid(round(self.dog.x), round(self.dog.y+.01))
                        self.occupancy_grid.data[int(dog_index)] = -3
                        self.dog_detected = True
                        print('SELF.DOG', self.dog)
                        print('SELF.Drone', self.drone_pos.pose.position)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            self.occupancygrid_explore()
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