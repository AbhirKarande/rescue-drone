#!/usr/bin/env python
import rospy
import time
import numpy as np

from pid_class import PID
from mav_msgs.msg import RateThrust
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64


class AngleController():

  def __init__(self):

    # Allow the simulator to start
    time.sleep(5)

    # Run the shutdown sequence on shutdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Getting the PID parameters
    gains = rospy.get_param('/angle_controller_node/gains', {'p': 0.1, 'i': 0, 'd': 0})
    Kp, Ki, Kd = gains['p'], gains['i'], gains['d']

    # Set the rate
    self.rate = 100.0
    self.dt = 1.0 / self.rate

    # Display incoming parameters
    rospy.loginfo(str(rospy.get_name()) + ": Launching with the following parameters:")
    rospy.loginfo(str(rospy.get_name()) + ": p - " + str(Kp))
    rospy.loginfo(str(rospy.get_name()) + ": i - " + str(Ki))
    rospy.loginfo(str(rospy.get_name()) + ": d - " + str(Kd))
    rospy.loginfo(str(rospy.get_name()) + ": rate - " + str(self.rate))

    # Creating the PID's
    self.rpy_PIDS = [PID(Kp, Ki, Kd, self.rate) for _ in range(3)]

    # Create the setpoints
    self.rpy_setpoints = np.zeros(3, dtype=np.float64)
    self.thrust_setpoint = 10

    # Create the current output readings
    self.rpy_readings = np.zeros(3, dtype=np.float64)

    # Check if the drone has been armed
    self.armed = False

    # Create the subscribers
    self.imu_sub = rospy.Subscriber("/uav/sensors/attitude", Vector3, self.euler_angle_callback)
    self.att_sub = rospy.Subscriber("/uav/input/attitude", Vector3, self.attitude_set_callback)
    self.thrust_sub = rospy.Subscriber("/uav/input/thrust", Float64, self.thrust_callback)
    self.armed_sub = rospy.Subscriber("/uav/armed", Bool, self.armed_callback)
    self.yaw_sub = rospy.Subscriber("/uav/input/yaw", Float64, self.set_yaw_output)

    # Create the publishers
    self.rate_pub = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size=10)
    
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

      # Create the message we are going to send
      msg = RateThrust()
      msg.header.stamp = rospy.get_rostime()

      # If the drone has not been armed
      if self.armed == False:
        # Arm the drone
        msg.thrust = Vector3(0, 0, 10)
        msg.angular_rates = Vector3(0, 0, 0)   

      # Behave normally (Drone is armed)
      else:
        # Use a PID to calculate the rates you want to publish to maintain an angle
        output = [pids.get_output(setp, read) for pids, setp, read in zip(self.rpy_PIDS, self.rpy_setpoints, self.rpy_readings)]

        # Create the message
        msg.thrust = Vector3(0, 0, self.thrust_setpoint)
        msg.angular_rates = Vector3(output[0], output[1], output[2])

      # Publish the message
      self.rate_pub.publish(msg)

      # Sleep any excess time
      rate.sleep()

  # This is the callback for the yaw subscriber
  def set_yaw_output(self, msg):
    #rospy.loginfo("Received yaw" + str(msg.data))
    self.yaw_output = msg.data

	# Save the new attitude readings
  def euler_angle_callback(self, msg):
    self.rpy_readings[0] = msg.x
    self.rpy_readings[1] = msg.y
    self.rpy_readings[2] = msg.z

	# Save the new attitude setpoints
  def attitude_set_callback(self, msg):
    # Dont allow angles greater than 0.5 for x and y
    self.rpy_setpoints[0] = max(min(msg.x, 0.5),-0.5)
    self.rpy_setpoints[1] = max(min(msg.y, 0.5),-0.5)
    self.rpy_setpoints[2] = msg.z

	# Save the new thrust setpoints
  def thrust_callback(self, msg):
    self.thrust_setpoint = msg.data

	# Save whether the drone is armed or not
  def armed_callback(self, msg):
    self.armed = msg.data

	# Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


def main():
  rospy.init_node('angle_controller')
  try:
    angcon = AngleController()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()
