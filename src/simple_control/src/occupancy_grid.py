#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_geometry_msgs import do_transform_point
class occupancy_grid:
    def __init__(self):
        self.lidar_reading = LaserScan()
        self.lidar_sub = rospy.Subscriber('/uav/sensors/lidar', LaserScan, self.lidar_callback)
        self.drone_pos = Vector3()
        self.drone_pos_sub = rospy.Subscriber('/uav/sensors/gps', Vector3, self.drone_pos_callback)
        self.drone_pos_pub = rospy.Publisher('/uav/control/gps', Vector3, queue_size=1)
        self.occupancy_grid = OccupancyGrid()
        self.dog = Vector3()
        self.dog_sub = rospy.Subscriber('/cell_tower/position', Vector3, self.dog_callback)
        #initialize all cells of the occupancy grid to 50
        self.occupancy_grid.data = [50 for i in range(100)]
        self.occupancy_grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.mainloop()
    def lidar_callback(self, data):
        self.lidar_reading = data
    def dog_callback(self, data):
        self.dog = data
    def drone_pos_callback(self, data):
        self.drone_pos = data
    def mainloop(self):
        #rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.dog and (self.dog.x != 0 or self.dog.y != 0):
                try: 
                    print('SELF.LIDAR', self.lidar)
                    lookup = self.tfBuffer.lookup_transform('world', 'cell_tower', rospy.Time.now())
                    #print('LOOKUP', lookup)
                    goal_point = PointStamped("cell_tower",self.dog)
                    #print("GOAL_POINT TYPE", type(goal_point))
                    #print("GOAL_POINT", goal_point)
                    goal_point1 = do_transform_point(goal_point, lookup)
                    #print('TRANSFORMED DOG POINT',goal_point1)
                    self.dog = Vector3(int(goal_point1.point.x), int(goal_point1.point.y), int(goal_point1.point.z))
                    #print('SELF.DOG', self.dog)

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue
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
            #TODO: move the drone to the dog's position according to the occupancy grid so far
            
            #publish the occupancy grid
            self.occupancy_grid_pub.publish(self.occupancy_grid)
            #rate.sleep()


            #publish the occupancy grid
            self.occupancy_grid_pub.publish(self.occupancy_grid)


