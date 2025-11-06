'''
Author: Yuhao Zhou
Date: 2023-05-23 13:32:49
LastEditors: Yuhao Zhou
LastEditTime: 2023-05-25 10:34:02
Description: Robot-patient
FilePath: /real_ws/src/UR_Real/Controllers_Algorithms/control_algorithm/Admittance/scripts/ekf.py
'''
import rospy
from geometry_msgs.msg import WrenchStamped
from filterpy.kalman import ExtendedKalmanFilter
import numpy as np
from scipy import signal

class AdaptiveKalmanFilterNode:
    def __init__(self):
        self.kf = ExtendedKalmanFilter(dim_x=6, dim_z=6)
        self.initialize()
        # Publisher for filtered force/torque data
        self.filtered_pub = rospy.Publisher('/ekf_wrench', WrenchStamped, queue_size=1)

    def initialize(self):
        # Define the state transition function
        self.kf.f = lambda x, dt: x
        # Define the measurement function
        self.kf.h = lambda x: x
        # Define the Jacobian matrix of the state transition function
        self.kf.F = np.eye(6)
        # Define the Jacobian matrix of the measurement function
        self.kf.H = np.eye(6)
        # Define the process noise covariance matrix
        self.kf.Q = np.eye(6) * 0.0001
        # Define the measurement noise covariance matrix
        self.kf.R = np.eye(6) * 0.1
        # Initialize the state estimate and covariance matrix
        self.kf.x = np.zeros(6)
        self.kf.P = np.eye(6)

        # Notch filter parameters for sin/cos noise elimination
        self.fs = 500  # Sampling frequency
        self.f0 = 50    # Frequency to be attenuated (sin)
        self.Q = 30     # Quality factor (controls the bandwidth of the notch filter)
        self.b, self.a = signal.iirnotch(self.f0, self.Q, self.fs)

    def filter_callback(self, msg):
        # Extract force and torque measurements from the received message
        force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        torque = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])

        # Apply notch filter to eliminate sin/cos noise
        filtered_force = signal.lfilter(self.b, self.a, force)
        filtered_torque = signal.lfilter(self.b, self.a, torque)

        # Prediction step
        self.kf.predict()

        # Update step
        self.kf.update(np.concatenate((filtered_force, filtered_torque)), HJacobian=self.jacobian_h, Hx=self.measurement_function)

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

    def jacobian_h(self, x):
        return np.eye(6)

    def measurement_function(self, x):
        return x

def main():
  rospy.init_node('extended_kalman_filter_node')
  filter_node = AdaptiveKalmanFilterNode()
  # Subscribe to the "/wrench" topic
  rospy.Subscriber('/wrench', WrenchStamped, filter_node.filter_callback)
  # Spin and process callbacks
  rospy.spin()

if __name__ == '__main__':
  main()