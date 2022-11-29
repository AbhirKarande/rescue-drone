#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import math
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
        
        self.occupancy_grid.info.resolution = 2
        #get the map_width argument from fly.launch
        self.occupancy_grid.info.width = rospy.get_param('environment_controller/map_width')
        self.occupancy_grid.info.height = rospy.get_param('environment_controller/map_height')
        self.size = self.occupancy_grid.info.width * self.occupancy_grid.info.height
        self.occupancy_grid.data = [50 for i in range(self.size)]
        self.occupancy_grid.info.origin.position.x = -(self.occupancy_grid.info.width/2)
        self.occupancy_grid.info.origin.position.y = -(self.occupancy_grid.info.height/2)
        self.occupancy_grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.mainloop()
    def lidar_callback(self, data):
        self.lidar_reading = data
    def dog_callback(self, data):
        self.dog = data
    def drone_pos_callback(self, data):
        self.drone_pos = data
        

    def mainloop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.dog and (self.dog.x != 0 or self.dog.y != 0):
                try: 
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

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue
            #add the position of the dog to the occupancy grid
            dog_index = self.dog.x + (self.dog.y * self.occupancy_grid.info.width)
            print('DOG_INDEX', int(dog_index))
            self.occupancy_grid.data[int(dog_index)] = -3
            #get the angle of the first ray
            angle = self.lidar_reading.angle_min
            #iterate over all the rays
            for i in range(len(self.lidar_reading.ranges)):
                #get the distance of the ray
                distance = self.lidar_reading.ranges[i]
                #calculate the x and y coordinates of the ray
                x = self.drone_pos.pose.position.x + (distance * math.cos(angle))
                y = self.drone_pos.pose.position.y + (distance * math.sin(angle))
                index = self.occupancy_grid.info.width * (int(y) + int(self.occupancy_grid.info.height//2)) + (int(x) + int(self.occupancy_grid.info.width//2))
                self.occupancy_grid.data[int(index)] = 100
                print('X', x)
                print('Y', y)
                print('INDEX: ',int(index))
                emptyDistance = distance

                #TODO: figure out how to calculate empty space between obstacles and the drone
                #TODO: figure out how to detect doors
                # while emptyDistance > 0:
                #     emptyDistance = distance - 0.5

                #     emptyX = self.drone_pos.pose.position.x + emptyDistance * math.cos(angle)
                #     emptyY = self.drone_pos.pose.position.y + emptyDistance * math.sin(angle)
                #     emptyIndex = emptyX + (emptyY * self.occupancy_grid.info.width)
                #     self.occupancy_grid.data[int(emptyIndex)] = 0
                





                #calculate the index of the cell in the occupancy grid
                
                #set the cell to 100
                #increment the angle
                angle += self.lidar_reading.angle_increment
            #TODO: move the drone to where there is empty space
            
            
            #publish the occupancy grid
            print(self.occupancy_grid.data)
            self.occupancy_grid_pub.publish(self.occupancy_grid)
            rate.sleep()
if __name__ == '__main__':
  rospy.init_node('occupancy_grid_node')
  try:
    t = occupancy_grid()
  except rospy.ROSInterruptException:
    pass

