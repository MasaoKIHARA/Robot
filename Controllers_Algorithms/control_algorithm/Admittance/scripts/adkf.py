'''
Author: Yuhao Zhou
Date: 2023-05-22 17:52:57
LastEditors: Yuhao Zhou
LastEditTime: 2023-05-23 14:23:13
Description: Robot-patient
FilePath: /real_ws/src/UR_Real/Controllers_Algorithms/control_algorithm/Admittance/scripts/adkf.py
'''
import rospy
from geometry_msgs.msg import WrenchStamped
from filterpy.kalman import KalmanFilter
import numpy as np

class KalmanFilterNode:
   def __init__(self):
       self.kf = KalmanFilter(dim_x=6, dim_z=6)
       self.initialize()
       # Publisher for filtered force/torque data
       self.filtered_pub = rospy.Publisher('/adkf_wrench', WrenchStamped, queue_size=1)
       # Sage-Husa fuzzy adaptive filter parameters
       self.beta = 0.95  # Fuzzy adaptive factor
       self.prev_innovation = np.zeros(6)  # Previous innovation
       self.prev_residual = np.zeros(6)  # Previous residual

   def initialize(self):
       # Define state transition matrix
       self.kf.F = np.eye(6)
       # Define measurement matrix
       self.kf.H = np.eye(6)
       # Define process noise covariance matrix
       self.kf.Q = np.eye(6) * 0.0001
       # Define measurement noise covariance matrix
       self.kf.R = np.eye(6) * 0.2
       # Initialize the state estimate and covariance matrix
       self.kf.x = np.zeros(6)
       self.kf.P = np.eye(6)

   def filter_callback(self, msg):
       # Extract force and torque measurements from the received message
       force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
       torque = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
       # Prediction step
       self.kf.predict()
       # Update step
       self.kf.update(np.concatenate((force, torque)))
       # Sage-Husa fuzzy adaptive filter
       z = np.concatenate((force, torque))  # Measurement vector
       innovation = z - np.dot(self.kf.H, self.kf.x)  # Innovation
       # Calculate adaptive factor using the previous innovation and residual
       adaptive_factor = np.exp(-self.beta * np.dot(innovation, self.prev_residual))
       adaptive_factor = np.clip(adaptive_factor, 0.0, 1.0)
       # Update the Kalman filter gain using the adaptive factor
       self.kf.K *= adaptive_factor
       # Update the state estimate and covariance matrix
       self.kf.x += np.dot(self.kf.K, innovation)
       self.kf.P = np.dot(np.eye(6) - np.dot(self.kf.K, self.kf.H), self.kf.P)
       # Store the current innovation and residual for the next iteration
       self.prev_innovation = innovation
       self.prev_residual = np.dot(self.kf.H, innovation)
       # Publish the filtered force/torque data
       filtered_msg = WrenchStamped()
       filtered_msg.header = msg.header
       filtered_msg.wrench.force.x = self.kf.x[0]
       filtered_msg.wrench.force.y = self.kf.x[1]
       filtered_msg.wrench.force.z = self.kf.x[2]
       filtered_msg.wrench.torque.x = self.kf.x[3]
       filtered_msg.wrench.torque.y = self.kf.x[4]
       filtered_msg.wrench.torque.z = self.kf.x[5]
       self.filtered_pub.publish(filtered_msg)

def main():
   rospy.init_node('adaptive_kalman_filter_node')
   filter_node = KalmanFilterNode()
   # Subscribe to the "/wrench" topic
   rospy.Subscriber('/wrench', WrenchStamped, filter_node.filter_callback)
   # Spin and process callbacks
   rospy.spin()

if __name__ == '__main__':
   main()