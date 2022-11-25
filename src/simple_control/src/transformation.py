#!/usr/bin/env python

#look up the transform between the coordinate frames for the cell_tower and the world
#this is the transform from the cell_tower to the world

#take the dog's last known position from /cell_tower/position and transform it to the world frame


import rospy
import tf2_ros
import time
import copy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

class transformation:
    def __init__(self):
        time.sleep(10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.dog = Vector3()
        self.lidar = Vector3()
        self.dog_sub = rospy.Subscriber('/cell_tower/position', Vector3, self.dog_callback)
        self.lidar_sub = rospy.Subscriber('/uav/sensors/lidar', Vector3, self.lidar_callback)

        self.mainloop()
    def dog_callback(self, data):
        self.dog = data
    def lidar_callback(self, data):
        self.lidar = data
    def mainloop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.dog and (self.dog.x != 0 or self.dog.y != 0):
                print('SELF.DOG', self.dog)
                try: 
                    print(self.lidar)
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
            rate.sleep()
        
if __name__ == '__main__':
  rospy.init_node('transformation_node')
  try:
    t = transformation()
  except rospy.ROSInterruptException:
    pass