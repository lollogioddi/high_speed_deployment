#!/usr/bin/env python2

import rospy
import sys
import numpy as np
import tf
import traceback
from trajectory_msgs.msg import MultiDOFJointTrajectory
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Vector3

class LeeControl:

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
            self.pub = rospy.Publisher('/pelican/command/motor_speed', Actuators, queue_size=10)
            # Publisher for angol error
            self.pub_angular_error = rospy.Publisher('/pelican/angular_err', Vector3, queue_size=10)
            # Subscribe to the desired pose topic
            self.pose_sub = rospy.Subscriber('/pelican/command/trajectory', MultiDOFJointTrajectory, self.trajectory_callback)
            self.pose_pub = rospy.Publisher('/{}/command/pos'.format(namespace), Vector3, queue_size=10)
            # Initialize the desired pose to some default
            self.command_trajectory = MultiDOFJointTrajectory()
            # Publish on a topic to check if the process succeeded
            self.rate = rospy.Rate(200)            

            self.angular_velocity_history = []
            self.stabilization_threshold = 0.5 # Threshold for angular velocity (rad/s)
            self.history_size = 100
            self.is_stabilized = False

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
            k_pos = rospy.get_param('~position_gain')
            k_vel = rospy.get_param('~velocity_gain')
            k_att = rospy.get_param('~attitude_gain_Lee')
            k_ang_rate = rospy.get_param('~angular_rate_gain_Lee')
            self.k_pos = np.array([k_pos['x'], k_pos['y'], k_pos['z']])
            self.k_vel = np.array([k_vel['x'], k_vel['y'], k_vel['z']])
            self.k_att = np.array([k_att['x'], k_att['y'], k_att['z']])
            self.k_ang_rate = np.array([k_ang_rate['x'], k_ang_rate['y'], k_ang_rate['z']])


            self.normalized_k_att = np.dot(self.k_att, self.inv_inertia_matrix)
            self.normalized_k_ang_rate = np.dot(self.k_ang_rate, self.inv_inertia_matrix)


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

    def position_controller(self, R, position, command_position, orientation, linear_velocity, command_linear_velocity, command_linear_acceleration):
        try:
            current_position = position
            position_error = current_position - command_position
            linear_velocity = np.dot(R, linear_velocity)
            velocity_error = linear_velocity - command_linear_velocity
            
            z_unit_vector = np.array([0.0,0.0,1.0])

            acceleration = ((np.multiply(position_error,self.k_pos) + np.multiply(velocity_error,self.k_vel)) / self.mass)-self.g * z_unit_vector - command_linear_acceleration
    

            return acceleration, position_error
        except Exception as e:
            rospy.logerr("Error in position controller: {}\n{}".format(e, traceback.format_exc()))
            rospy.loginfo("Error in Pos Cont")

    def attitude_controller(self, R, command_orientation, command_angular_velocity, angular_velocity, acceleration, b1_des):
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
            angular_acceleration = -np.multiply(angol_error_vector, self.normalized_k_att) -np.multiply(angular_rate_error,self.normalized_k_ang_rate)
            
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
    
    def check_angular_stabilization(self, angular_velocity):
        # Calculate the current angular velocity magnitude
        angular_velocity_magnitude = np.linalg.norm(angular_velocity)
        # Update the angular velocity history
        self.angular_velocity_history.append(angular_velocity_magnitude)
        # Ensure the history does not exceed the specified size
        if len(self.angular_velocity_history) > self.history_size:
            self.angular_velocity_history.pop(0)
        # Check if all recorded angular velocities are below the threshold
        return all(vel < self.stabilization_threshold for vel in self.angular_velocity_history)

    def control_loop(self, allocation_matrix):
        while not rospy.is_shutdown():
            while not self.command_trajectory.points and not rospy.is_shutdown():
                #rospy.logwarn("Waiting for trajectory points...")
                self.rate.sleep()

            try:
                # command_position = np.array([self.command_trajectory.points[0].transforms[0].translation.x,
                #                                 self.command_trajectory.points[0].transforms[0].translation.y,
                #                                 self.command_trajectory.points[0].transforms[0].translation.z])
                command_orientation = [self.command_trajectory.points[0].transforms[0].rotation.x,
                                                    self.command_trajectory.points[0].transforms[0].rotation.y,
                                                    self.command_trajectory.points[0].transforms[0].rotation.z,
                                                    self.command_trajectory.points[0].transforms[0].rotation.w]
                command_linear_velocity = np.array([self.command_trajectory.points[0].velocities[0].linear.x,
                                                    self.command_trajectory.points[0].velocities[0].linear.y,
                                                    self.command_trajectory.points[0].velocities[0].linear.z])
                command_linear_acceleration = np.array([self.command_trajectory.points[0].accelerations[0].linear.x,
                                                        self.command_trajectory.points[0].accelerations[0].linear.y,
                                                        self.command_trajectory.points[0].accelerations[0].linear.z])
                command_angular_velocity = np.array([self.command_trajectory.points[0].velocities[0].angular.x,
                                                    self.command_trajectory.points[0].velocities[0].angular.y,
                                                    self.command_trajectory.points[0].velocities[0].angular.z])
                position = np.array([self.odometry.pose.pose.position.x,
                        self.odometry.pose.pose.position.y,
                        self.odometry.pose.pose.position.z])
                orientation = [self.odometry.pose.pose.orientation.x,
                                        self.odometry.pose.pose.orientation.y,
                                        self.odometry.pose.pose.orientation.z,
                                        self.odometry.pose.pose.orientation.w]
                linear_velocity = np.array([self.odometry.twist.twist.linear.x,
                                            self.odometry.twist.twist.linear.y,
                                            self.odometry.twist.twist.linear.z])
                angular_velocity = np.array([self.odometry.twist.twist.angular.x,
                                            self.odometry.twist.twist.angular.y,
                                            self.odometry.twist.twist.angular.z])
                angular_velocity_norm = np.linalg.norm(angular_velocity)
                R = tf.transformations.quaternion_matrix(orientation)
                R = R[:3, :3]
                yaw = 1.0
                b1_des = np.array([np.cos(yaw),np.sin(yaw), 0.0])
                # Decoupling of position and attitude
                if not self.is_stabilized and self.check_angular_stabilization(angular_velocity):
                    self.is_stabilized = True
                    command_position = np.array([self.odometry.pose.pose.position.x,
                        self.odometry.pose.pose.position.y,
                        self.odometry.pose.pose.position.z])
                    self.pose_pub.publish(Vector3(command_position[0], command_position[1], command_position[2]))
                    rospy.loginfo('attitude stabilized, activating controller')
                    # The drone is considered to be stabilizing; use the computed acceleration
                    acceleration, position_error = self.position_controller(R, position, command_position, orientation, linear_velocity, command_linear_velocity, command_linear_acceleration)

                elif self.is_stabilized:
                    acceleration, position_error = self.position_controller(R, position, command_position, orientation, linear_velocity, command_linear_velocity, command_linear_acceleration)
                else:
                    # The drone is not stabilizing yet; override acceleration for attitude control
                    acceleration = np.array([0.0, 0.0, -10.4581])
                    rospy.loginfo("NOT --> self.normalized_k_att: {}".format(self.normalized_k_att))
                    rospy.loginfo("NOT --> self.normalized_k_ang_rate: {}".format(self.normalized_k_ang_rate))
                angular_acceleration, angol_error = self.attitude_controller( R, command_orientation, command_angular_velocity, angular_velocity, acceleration, b1_des)
                angular_acc_to_rotor_velocities = allocation_matrix #self.calculate_allocation_matrix()
                self.publish_actuator(R, angular_acc_to_rotor_velocities, acceleration, angular_acceleration)
                self.pub_angular_error.publish(Vector3(angol_error[0], angol_error[1], angol_error[2]))

            except Exception as e:
                rospy.logerr("Error in control loop: {}\n{}".format(e, traceback.format_exc()))


def main():
    try:
        # Creare un'istanza del controllore PID con i file di configurazione
        Lee_control = LeeControl()
        allocation_matrix = Lee_control.calculate_allocation_matrix()
        Lee_control.control_loop(allocation_matrix)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Unhandled exception in main: {}".format(e))

if __name__ == '__main__':
    main()