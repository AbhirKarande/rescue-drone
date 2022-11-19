#!/usr/bin/env python
import rospy
import time
import math
import numpy as np

from pid_class import PID
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3, PoseStamped

class PositionController():

  def __init__(self):

    # Allow the simulator to start
    time.sleep(5)
    
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 100.0
    self.dt = 1.0 / self.rate

    # Getting the PID parameters
    stable_gains = rospy.get_param('/position_controller_node/gains/stable/', {'p': 1, 'i': 0.0, 'd': 0.0})
    Kp, Ki, Kd = stable_gains['p'], stable_gains['i'], stable_gains['d']

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Launching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": p - " + str(Kp))
    rospy.loginfo(str(rospy.get_name()) + ": i - " + str(Ki))
    rospy.loginfo(str(rospy.get_name()) + ": d - " + str(Kd))
    rospy.loginfo(str(rospy.get_name()) + ": rate - " + str(self.rate))

    # Creating the PID's
    self.position_PIDs = [PID(Kp, Ki, Kd, self.rate) for _ in range(3)]

    # Get the setpoints
    self.setpoints = np.array([0.0, 0.0, 3.0], dtype=np.float64)

    # Create the current output readings
    self.current_position = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    # Create the subscribers
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)
    self.pos_set_sub = rospy.Subscriber("uav/input/position", Vector3, self.set_pos)

    # Create the publishers
    self.vel_set_sub = rospy.Publisher('/uav/input/velocity', Vector3, queue_size=1)
    self.at_goal_pub = rospy.Publisher("uav/sensors/at_waypoint", Bool, queue_size=1)

    # Run the control loop
    self.ControlLoop()

  # This is the main loop of this class
  def ControlLoop(self):
    # Set the rate
    rate = rospy.Rate(50)

    # Keep track how many loops have happend
    loop_counter = 0

    # While running
    while not rospy.is_shutdown():

        # Use a PID to calculate the velocity you want
        output = [pids.get_output(setp, pos) for pids, setp, pos in zip(self.position_PIDs, self.setpoints, self.current_position)]

        # Create and publish the data
        velocity = Vector3(output[0], output[1], output[2])
        self.vel_set_sub.publish(velocity)

        # If we are close to the goal
        dx = self.setpoints[0] - self.current_position[0]
        dy = self.setpoints[1] - self.current_position[1]
        dz = self.setpoints[2] - self.current_position[2]
        distance_to_waypoint = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        msg = Bool()
        if distance_to_waypoint < 0.5:
          msg.data = True
        else:
          msg.data = False
        self.at_goal_pub.publish(msg)

        # Sleep any excess time
        rate.sleep()

  # Call back to get the gps data
  def get_gps(self, msg):
    self.current_position[0] = msg.pose.position.x
    self.current_position[1] = msg.pose.position.y
    self.current_position[2] = msg.pose.position.z


  # Call back to get the position setpoints
  def set_pos(self, msg):
    # If our set point changes reset the PID build up
    check_x = abs(self.setpoints[0] - msg.x) > 1e-6
    check_y = abs(self.setpoints[1] - msg.y) > 1e-6
    check_z = abs(self.setpoints[2] - msg.z) > 1e-6
    # Remove the buildup
    if check_x or check_y or check_z:
      for pid in self.position_PIDs:
        pid.remove_buildup()
    # Set new setpoint
    self.setpoints[0] = msg.x
    self.setpoints[1] = msg.y
    self.setpoints[2] = msg.z

	# Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")

def main():
  rospy.init_node('position_controller_node')
  try:
    poscon = PositionController()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()
