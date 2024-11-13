#!/usr/bin/env python2

import rospy
import sys
import numpy as np
import tf
import math
import traceback
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Vector3
from std_msgs.msg import String 
import csv
import os

class AdaptiveControl:

    def __init__(self):
        try:
            rospy.init_node('attitude_controller', anonymous=True)
            namespace = rospy.get_namespace().strip('/')
            # Load configuration from YAML files
            self.read_ros_params()
            # Calculate the allocation matrix
            self.calculate_allocation_matrix()
            # Subscribe to the odometry topic
            self.sub = rospy.Subscriber('/pelican/msf_core/odometry', Odometry, self.odometry_callback)
            # Subscribe to clock
            self.sub_clock = rospy.Subscriber("/clock", Clock, self.clock_callback)
            # Initialize the odometry to some default
            self.odometry = Odometry()
            # Publisher for motor speeds
            self.pub = rospy.Publisher('/pelican/command/motor_speed', Actuators, queue_size=100)
            # Publisher for angol error
            self.pub_angular_error = rospy.Publisher('/pelican/angular_err', Vector3, queue_size=100)
            # Subscribe to the desired pose topic
            self.pose_sub = rospy.Subscriber('/pelican/command/trajectory', MultiDOFJointTrajectory, self.trajectory_callback)
            self.pose_pub = rospy.Publisher('/{}/command/pos'.format(namespace), Vector3, queue_size=100)
            # Initialize the desired pose to some default
            self.command_trajectory = MultiDOFJointTrajectory()
            # switch parameters
            self.angular_velocity_history = []
            self.is_stabilized = False
            self.stabilization_threshold = 1.0  # Threshold for angular velocity (rad/s)
            self.history_size = 70
            # Publish on a topic to check if the process succeeded
            self.rate = rospy.Rate(200)
            
        except Exception as e:
            rospy.logerr("Error in init: {}\n{}".format(e, traceback.format_exc()))
            rospy.loginfo("Error in initialization")
            sys.exit(1)

    def read_ros_params(self):
        try:
            # Parameters from ROS parameter server
            self.mass = rospy.get_param('~mass')
            self.g = 9.8
            inertia_params = rospy.get_param('~inertia')
            self.Ix = inertia_params['xx']
            self.Iy = inertia_params['yy']
            self.Iz = inertia_params['zz']
            self.inertia_matrix = np.diag([self.Ix, self.Iy, self.Iz])
            self.inv_inertia_matrix = np.linalg.inv(self.inertia_matrix)
            # Rotor configuration
            self.rotor_configuration = rospy.get_param('~rotor_configuration')
            # PID gains
            k_pos = rospy.get_param('~position_gain_1')
            k_vel = rospy.get_param('~velocity_gain_1')
            k_att = rospy.get_param('~attitude_gain_1')
            k_ang_rate = rospy.get_param('~angular_rate_gain_1')
            k_pos_Lee = rospy.get_param('~position_gain_Lee')
            k_vel_Lee = rospy.get_param('~velocity_gain_Lee')
            k_att_Lee = rospy.get_param('~attitude_gain_Lee')
            k_ang_rate_Lee = rospy.get_param('~angular_rate_gain_Lee')

            self.k_pos = np.array([k_pos['x'], k_pos['y'], k_pos['z']])
            self.k_vel = np.array([k_vel['x'], k_vel['y'], k_vel['z']])
            #Quaternion Based
            self.k_att = np.array([k_att['x'], k_att['y'], k_att['z']])
            self.k_ang_rate = np.array([k_ang_rate['x'], k_ang_rate['y'], k_ang_rate['z']])
            self.k_p = np.array([1,1,1])
            self.k_d = np.array([1,1,1])
            
            self.normalized_k_att_Agg = np.dot(self.k_att, self.inv_inertia_matrix)
            self.normalized_k_ang_rate_Agg = np.dot(self.k_ang_rate, self.inv_inertia_matrix)
            #Lee position controller
            self.k_pos_Lee = np.array([k_pos_Lee['x'], k_pos_Lee['y'], k_pos_Lee['z']])
            self.k_vel_Lee = np.array([k_vel_Lee['x'], k_vel_Lee['y'], k_vel_Lee['z']])
            self.k_att_Lee = np.array([k_att_Lee['x'], k_att_Lee['y'], k_att_Lee['z']])
            self.k_ang_rate_Lee = np.array([k_ang_rate_Lee['x'], k_ang_rate_Lee['y'], k_ang_rate_Lee['z']])
            self.normalized_k_att_Lee = np.dot(self.k_att_Lee, self.inv_inertia_matrix)
            self.normalized_k_ang_rate_Lee = np.dot(self.k_ang_rate_Lee, self.inv_inertia_matrix)
        except KeyError as e:
            rospy.logerr("Missing parameter: {}".format(e))
            sys.exit(1)
        except Exception as e:
            rospy.logerr("Error reading ROS parameters: {}\n{}".format(e, traceback.format_exc()))
            sys.exit(1)

    def calculate_allocation_matrix(self):
        # Calculate the matrix that maps the rotor velocities to torques and thrust
        try:
            num_rotors = len(self.rotor_configuration)
            allocation_matrix = np.zeros((4, num_rotors))
    
            for i, rotor_key in enumerate(sorted(self.rotor_configuration.keys())):
                rotor = self.rotor_configuration[rotor_key]
                # Calculate and set values in the allocation matrix foav_hovering _er each rotor
                allocation_matrix[0, i] = np.sin(rotor['angle']) * rotor['arm_length'] * rotor['rotor_force_constant']
                allocation_matrix[1, i] = -np.cos(rotor['angle']) * rotor['arm_length'] * rotor['rotor_force_constant']
                allocation_matrix[2, i] = -rotor['direction'] * rotor['rotor_force_constant'] * rotor['rotor_moment_constant']
                allocation_matrix[3, i] = rotor['rotor_force_constant']
    
            # Check the rank of the allocation matrix to ensure system controllability
            rank = np.linalg.matrix_rank(allocation_matrix)
            if rank < 4:
                print("The rank of the allocation matrix is {}, it should have rank 4, to have a fully controllable system, check your configuration.".format(rank))
            I = np.zeros((4, 4))
            I[:3,:3] = self.inertia_matrix
            I[3,3] = 1

            # Calculate the pseudo-inverse of the allocation matrix (A^T*(A*A^T)^{-1}) and then multiply by the inertia matrix I
            A_AT_inv = np.linalg.inv(allocation_matrix.dot(allocation_matrix.T))
            left_pseudo_inverse = allocation_matrix.T.dot(A_AT_inv)
            angular_acc_to_rotor_velocities = left_pseudo_inverse.dot(I)
            return angular_acc_to_rotor_velocities
        except np.linalg.LinAlgError as e:
            rospy.logerr("Error in matrix inversion: {}".format(e))
            rospy.loginfo("Error matrix inversion")
            sys.exit(1)

    def clock_callback(self, msg):
        # Callback of the timestamp
        self.current_time = msg

    def trajectory_callback(self, msg):
        # Callback per la pose desiderata
        self.command_trajectory = msg

    def odometry_callback(self, msg):
        # Callback function for odometry data
        self.odometry = msg

    def position_controller(self, R, cos_theta_xy, position, command_position, linear_velocity, command_linear_velocity):
        try:
            current_position = position
            position_error = current_position - command_position
            linear_velocity = np.dot(R, linear_velocity)
            velocity_error = linear_velocity - command_linear_velocity
            
            z_unit_vector = np.array([0.0,0.0,1.0])
            
            if abs(cos_theta_xy) > 1e-6:
                acceleration = (((np.multiply(position_error,self.k_pos) + np.multiply(velocity_error,self.k_vel)) / self.mass)-self.g * z_unit_vector)
            else:
                acceleration = ((np.multiply(position_error,self.k_pos_Lee) + np.multiply(velocity_error,self.k_vel_Lee)) / self.mass)-self.g * z_unit_vector
            # print("acceleration:", acceleration)
            return acceleration, position_error
        except Exception as e:
            rospy.logerr("Error in position controller: {}\n{}".format(e, traceback.format_exc()))
            rospy.loginfo("Error in Pos Cont")

    def attitude_controller(self, R, command_orientation, command_angular_velocity, angular_velocity, acceleration, b1_des, k_att, k_ang_rate ):
        try:
            b3_des = -acceleration / np.linalg.norm(acceleration)
            b2_des = self.cross_product(b3_des, b1_des)
            b2_des = b2_des / np.linalg.norm(b2_des)
            R_des = np.eye(3)
            R_des[:,0] = self.cross_product(b2_des,b3_des)
            R_des[:,1] = b2_des
            R_des[:,2] = b3_des
            R_comm = R_des
            RR = np.dot(R_comm.T, R)
            trace_RR= RR[0, 0] + RR[1, 1] + RR[2, 2]
            denominator = np.sqrt(1 + trace_RR)
            angol_error_matrix = 0.5*(np.dot(R_comm.T, R) - np.dot(R.T, R_comm))
            angol_error_vector = np.array((0,0,0))
            angol_error_vector = self.vector_from_skew_matrix(angol_error_matrix)/denominator
            angular_rate_error = angular_velocity - np.dot(np.dot(R_comm.T, R),command_angular_velocity)
            angular_acceleration = -np.multiply(angol_error_vector, k_att) -np.multiply(angular_rate_error, k_ang_rate)
            
            return angular_acceleration, angol_error_vector
        except Exception as e:
            rospy.logerr("Error in attitude controller: {}\n{}".format(e, traceback.format_exc()))
            rospy.loginfo("Error in Attitude")
            return [0, 0, 0, 0]  # Return a safe default value

    def publish_actuator(self, R, angular_acc_to_rotor_velocities, acceleration, angular_acceleration):
        # Publish actuator messages (motor angular speeds) on the robot topic
        pub_object = Actuators()
        pub_object.angular_velocities = self.calculate_rotor_velocities(R, angular_acc_to_rotor_velocities, acceleration, angular_acceleration)
        self.pub.publish(pub_object)           

    def calculate_rotor_velocities(self, R, angular_acc_to_rotor_velocities, acceleration, angular_acceleration):
        try:
            thrust = - self.mass * np.dot(acceleration, R[:, 2])

            angular_acceleration_thrust = np.array([0.0,0.0,0.0,0.0])
            angular_acceleration_thrust[:3] = angular_acceleration
            angular_acceleration_thrust[3] = thrust
            # Convert angular acceleration to rotor velocities
            rotor_velocities = np.dot(angular_acc_to_rotor_velocities, angular_acceleration_thrust)
            # Ensure all values are non-negative by taking the maximum of each element and 0
            rotor_velocities = np.maximum(rotor_velocities, np.zeros(rotor_velocities.shape))
            # Take the square root of each element
            rotor_velocities = np.sqrt(rotor_velocities)
            #file_name_rot_vel = "rotor_vel"
            #self.append2_string_to_file(file_name_rot_vel, rotor_velocities[0], rotor_velocities[1], rotor_velocities[2], rotor_velocities[3] )
            # print("velocities:", rotor_velocities)
            return rotor_velocities
        except Exception as e:
            rospy.logerr("Error in rotor velocity: {}\n{}".format(e, traceback.format_exc()))
            rospy.loginfo("Error in rotor velocity")

    def vector_from_skew_matrix(self, skew_matrix):
        # Extracts a 3D vector from a skew-symmetric matrix
        vector = np.array([skew_matrix[2, 1], skew_matrix[0, 2], skew_matrix[1, 0]])
        return vector
    
    def cross_product(self, a, b):
        vector = np.array([a[1]*b[2] - a[2]*b[1],
                           a[2]*b[0] - a[0]*b[2],
                           a[0]*b[1] - a[1]*b[0]])
        return vector

    def quaternion_log(self, q):
        """Compute the logarithm of a quaternion."""
        q_0 = q[3]  # Scalar part
        q_vec = q[:3]  # Vector part
        norm_q_vec = np.linalg.norm(q_vec)
        
        if norm_q_vec > 0:
            angle = np.arccos(q_0)
            direction = q_vec / norm_q_vec
            log_q = direction * angle
        else:
            log_q = np.array([0.0, 0.0, 0.0])
        
        return log_q

    def quaternion_conjugate(self, q):
        """Returns the conjugate of a quaternion."""
        q_conj = q.copy()
        q_conj[:3] = -q_conj[:3]  # Negate the vector part
        return q_conj

    def quaternion_product(self, q1, q2):
        """Returns the quaternion product of two quaternions q1 and q2."""
        # Extract scalar and vector components
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        # Compute the product
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return np.array([x, y, z, w])

    def compute_error_quaternion(self, qd, q):
        """Compute the error quaternion from desired quaternion qd and current quaternion q."""
        qd_conjugate = self.quaternion_conjugate(qd)
        q_error = self.quaternion_product(qd_conjugate, q)
        return q_error
  
    def compute_cos_theta_xy(self, q):
        """Compute the cosine of the tilt angle theta_xy using the quaternion."""
        # Quaternion rotation of the 'up' vector
        up_vector = np.array([0, 0, 1])  # 'up' vector in homogeneous coordinates for quaternion calculation
        q_up = np.hstack(([0], up_vector))  # Quaternion representation of the 'up' vector
        q_conj = self.quaternion_conjugate(q)
        rotated_vector = self.quaternion_product(self.quaternion_product(q, q_up), q_conj)
        
        # Calculate the dot product with the 'up' vector
        cos_theta_xy = rotated_vector[2]  # The z-component of the rotated vector
        # print("cos_theta_xy:", cos_theta_xy)
        return cos_theta_xy

    def check_angular_stabilization(self, angular_velocity, dynamic_tolerance=False):
        angular_velocity_magnitude = np.linalg.norm(angular_velocity)
        self.angular_velocity_history.append(angular_velocity_magnitude)
        if len(self.angular_velocity_history) > self.history_size:
            self.angular_velocity_history.pop(0)

        if dynamic_tolerance:
            elapsed_time = rospy.Time.now().to_sec() - self.start_time  # Adjust tolerance over time
            allowed_outliers = int(min(0.1, 0.01 * elapsed_time) * self.history_size)  # Gradually increase tolerance
        else:
            allowed_outliers = int(0.1 * self.history_size)  # Fixed tolerance (10%)
        
        below_threshold = [vel < self.stabilization_threshold for vel in self.angular_velocity_history]
        return sum(below_threshold) >= (self.history_size - allowed_outliers)


    def compute_weighting_factor(self, t, transition_time):
        # Linear transition example
        return max(0, min(1, 1 - t / transition_time))

    def control_loop(self, allocation_matrix):
         while not rospy.is_shutdown():
            while not self.command_trajectory.points and not rospy.is_shutdown():
                #rospy.logwarn("Waiting for trajectory points...")
                self.rate.sleep()

            try:
                # command_position = np.array([self.command_trajectory.points[0].transforms[0].translation.x,
                #                                 self.command_trajectory.points[0].transforms[0].translation.y,
                #                                 self.command_trajectory.points[0].transforms[0].translation.z])
                command_orientation = np.array([self.command_trajectory.points[0].transforms[0].rotation.x,
                                                    self.command_trajectory.points[0].transforms[0].rotation.y,
                                                    self.command_trajectory.points[0].transforms[0].rotation.z,
                                                    self.command_trajectory.points[0].transforms[0].rotation.w])
                command_linear_velocity = np.array([self.command_trajectory.points[0].velocities[0].linear.x,
                                                    self.command_trajectory.points[0].velocities[0].linear.y,
                                                    self.command_trajectory.points[0].velocities[0].linear.z])
                command_angular_velocity = np.array([self.command_trajectory.points[0].velocities[0].angular.x,
                                                    self.command_trajectory.points[0].velocities[0].angular.y,
                                                    self.command_trajectory.points[0].velocities[0].angular.z])
                position = np.array([self.odometry.pose.pose.position.x,
                        self.odometry.pose.pose.position.y,
                        self.odometry.pose.pose.position.z])
                orientation = np.array([self.odometry.pose.pose.orientation.x,
                                        self.odometry.pose.pose.orientation.y,
                                        self.odometry.pose.pose.orientation.z,
                                        self.odometry.pose.pose.orientation.w])
                euler_orientation = tf.transformations.euler_from_quaternion(orientation, 'sxyz')
                euler_command = tf.transformations.euler_from_quaternion(command_orientation, 'sxyz')
                linear_velocity = np.array([self.odometry.twist.twist.linear.x,
                                            self.odometry.twist.twist.linear.y,
                                            self.odometry.twist.twist.linear.z])
                angular_velocity = np.array([self.odometry.twist.twist.angular.x,
                                            self.odometry.twist.twist.angular.y,
                                            self.odometry.twist.twist.angular.z])
                R = tf.transformations.quaternion_matrix(orientation)
                R = R[:3, :3]
                angular_velocity = np.array([self.odometry.twist.twist.angular.x,
                                            self.odometry.twist.twist.angular.y,
                                            self.odometry.twist.twist.angular.z])
                yaw = 1.0
                b1_des = np.array([np.cos(yaw),np.sin(yaw), 0.0])
                cos_theta_xy = self.compute_cos_theta_xy(orientation)
                angular_acc_to_rotor_velocities = allocation_matrix #self.calculate_allocation_matrix()
                
                if not self.is_stabilized and self.check_angular_stabilization(angular_velocity):
                    # The drone is considered to be stabilizing; use the computed acceleration
                    command_position = np.array([self.odometry.pose.pose.position.x,
                        self.odometry.pose.pose.position.y,
                        self.odometry.pose.pose.position.z])
                    self.pose_pub.publish(Vector3(command_position[0], command_position[1], command_position[2]))
                    self.is_stabilized = True
                    total_force, position_error = self.position_controller(R, cos_theta_xy, position, command_position, linear_velocity, command_linear_velocity)
                    torque, angol_error = self.attitude_controller(R, command_orientation, command_angular_velocity, angular_velocity, total_force, b1_des, self.normalized_k_att_Lee, self.normalized_k_ang_rate_Lee)
                    self.pub_angular_error.publish(Vector3(angol_error[0], angol_error[1], angol_error[2]))
                    self.start_time = rospy.Time.now().to_sec()
                    rospy.logerr('Control strategy switched')

                elif self.is_stabilized:
                    # Use the stabilized controller
                    current_time = rospy.Time.now().to_sec()
                    # alpha = self.compute_weighting_factor(current_time - self.start_time, 1.5)
                    total_force, position_error = self.position_controller(R, cos_theta_xy, position, command_position, linear_velocity, command_linear_velocity)
                    # torque_aggressive = self.attitude_controller(R, command_orientation, command_angular_velocity, angular_velocity, total_force, b1_des, self.normalized_k_att_Agg, self.normalized_k_ang_rate_Agg)
                    torque, angol_error = self.attitude_controller(R, command_orientation, command_angular_velocity, angular_velocity, total_force, b1_des, self.normalized_k_att_Lee, self.normalized_k_ang_rate_Lee)
                    # torque = alpha * torque_aggressive + (1 - alpha) * torque_stable
                    self.pub_angular_error.publish(Vector3(angol_error[0], angol_error[1], angol_error[2]))
                    rospy.loginfo('switched')
                else:
                    # Use the original controller
                    total_force = np.array([0.0, 0.0, -10.4581])
                    torque, angol_error = self.attitude_controller(R, command_orientation, command_angular_velocity, angular_velocity, total_force, b1_des, self.normalized_k_att_Agg, self.normalized_k_ang_rate_Agg)
                    self.pub_angular_error.publish(Vector3(angol_error[0], angol_error[1], angol_error[2]))
                    rospy.loginfo('Not stabilized')
                
                self.publish_actuator(R, angular_acc_to_rotor_velocities, total_force, torque)
                
            except Exception as e:
                rospy.logerr("Error in control loop: {}\n{}".format(e, traceback.format_exc()))


def main():
    try:
        # Creare un'istanza del controllore PID con i file di configurazione
        Adaptive_control = AdaptiveControl()
        allocation_matrix = Adaptive_control.calculate_allocation_matrix()
        Adaptive_control.control_loop(allocation_matrix)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Unhandled exception in main: {}".format(e))

if __name__ == '__main__':
    main()
