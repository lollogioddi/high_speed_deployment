#!/usr/bin/env python2.7
import rospy
import tf
import numpy as np
from trajectory_msgs.msg import MultiDOFJointTrajectory
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Float64MultiArray, Bool
from collections import deque
import pandas as pd

class DataSaver:
    def __init__(self):
        rospy.init_node('data_saver_node_msf', anonymous=True)
        namespace = rospy.get_namespace().strip('/')
        self.nameDrone = namespace
        self.sub = rospy.Subscriber('/pelican/msf_core/odometry', Odometry, self.odometry_callback)
        self.sub_clock = rospy.Subscriber("/clock", Clock, self.clock_callback)
        self.pose_sub = rospy.Subscriber('/pelican/command/trajectory', MultiDOFJointTrajectory, self.trajectory_callback)
        self.sub_angular_error = rospy.Subscriber('/pelican/angular_err', Vector3, self.angular_error_callback)
        self.sub_shutdown = rospy.Subscriber('/shutdown_data_saver_msf', Bool, self.shutdown_callback)
        self.sub_stop_simulation = rospy.Subscriber('/stop_simulation_signal_msf', Bool, self.stop_simulation_callback)
        self.sub_pose = rospy.Subscriber('/pelican/command/pos', Vector3, self.command_pose_callback)
        self.pub_time = rospy.Publisher('/time_in_sec', Float64, queue_size=100)
        self.pub_position = rospy.Publisher('/pelican/position', Vector3, queue_size=100)
        self.pub_position_error = rospy.Publisher('/pelican/pos_err', Vector3, queue_size=100)
        self.pub_angular_position = rospy.Publisher('/pelican/angular_pos', Vector3, queue_size=100)
        self.pub_command_trajectory = rospy.Publisher('/pelican/comm_pos', Vector3, queue_size=100)
        self.pub_metrics = rospy.Publisher('/performance_metrics_msf', Float64MultiArray, queue_size=100)

        self.current_time = Clock()
        self.command_trajectory = MultiDOFJointTrajectory()
        self.odometry = Odometry()
        self.angular_error = np.array([0.0, 0.0, 0.0])
        self.command_pose = None

        self.position_errors = deque(maxlen=300000)
        self.orientation_errors = deque(maxlen=300000)
        self.times = deque(maxlen=300000)
        self.linear_pos = deque(maxlen=300000)

        self.rate = rospy.Rate(400)

    def clock_callback(self, msg):
        self.current_time = msg

    def trajectory_callback(self, msg):
        self.command_trajectory = msg

    def command_pose_callback(self, msg):
        self.command_pose = msg

    def odometry_callback(self, msg):
        self.odometry = msg

    def angular_error_callback(self, msg):
        self.angular_error = np.array([msg.x, msg.y, msg.z])

    def shutdown_callback(self, msg):
        if msg.data:
            rospy.signal_shutdown("Shutdown signal received")

    def stop_simulation_callback(self, msg):
        if msg.data:
            rospy.loginfo("Stop simulation signal received, computing metrics...")
            self.compute_metrics()

    def publish(self, position, pos_err, euler_orientation, time, command_position):
        self.pub_time.publish(time)
        self.pub_position.publish(Vector3(*position))
        self.pub_position_error.publish(Vector3(*pos_err))
        self.pub_angular_position.publish(Vector3(*euler_orientation))
        self.pub_command_trajectory.publish(Vector3(*command_position))

    def process_and_publish(self):
        if self.command_trajectory.points:
            if self.command_pose is not None:
                command_position = np.array([
                    self.command_pose.x,
                    self.command_pose.y,
                    self.command_pose.z
                ])
            else:
                command_position = np.array([
                    self.odometry.pose.pose.position.x,
                    self.odometry.pose.pose.position.y,
                    self.odometry.pose.pose.position.z
                ])

            position = np.array([
                self.odometry.pose.pose.position.x,
                self.odometry.pose.pose.position.y,
                self.odometry.pose.pose.position.z
            ])

            orientation = [
                self.odometry.pose.pose.orientation.x,
                self.odometry.pose.pose.orientation.y,
                self.odometry.pose.pose.orientation.z,
                self.odometry.pose.pose.orientation.w
            ]
            euler_orientation = tf.transformations.euler_from_quaternion(orientation)
            pos_err = command_position - position
            ang_err = self.angular_error
            time = self.current_time.clock.secs + self.current_time.clock.nsecs * 1e-9

            self.linear_pos.append(position)
            self.position_errors.append(pos_err)
            self.orientation_errors.append(ang_err)
            self.times.append(time)

            self.publish(position, pos_err, euler_orientation, time, command_position)

    def calculate_overshoots(self, errors):
        overshoots = {}
        for key, value in errors.items():
            overshoots[key] = np.max(np.abs(value), axis=0)
        return overshoots

    def calculate_settling_times(self, errors, timestamps, interpolated_position):
        initial_samples_size = 300
        window_size = 100
        percentage_margin = 15 / 100.0  # Convert to a fraction once
        abs_errors = {k: np.abs(np.array(v)) for k, v in errors.items()}
        max_initial_errors = {k: np.max(v[:initial_samples_size], axis=0) for k, v in abs_errors.items()}

        # Define specific thresholds for angular errors
        specific_thresholds = {
            'position_err': [1, 1, 1],
            'orientation_err': [0.36, 0.36, 0.36]
        }
        
        error_thresholds = {}
        for k, max_error in max_initial_errors.items():
            if k in specific_thresholds:
                # Calculate the percentage-based threshold
                percentage_threshold = max_error * percentage_margin
                # Choose the higher between the specific threshold and the percentage-based threshold
                error_thresholds[k] = np.maximum(percentage_threshold, specific_thresholds[k])
            else:
                # Position error thresholds
                percentage_threshold = max_error * percentage_margin
                error_thresholds[k] = percentage_threshold

        settling_times = {}
        for k, v in abs_errors.items():
            settling_times[k] = []
            z_rolling_max = None
            for i in range(v.shape[1]):
                if 'position_err' in k:
                    # Check if the z-position is not too close to zero (to detect crash)
                    z_position = interpolated_position[:, 2]
                    z_rolling_max = pd.Series(z_position).rolling(window=window_size).max()
                    z_rolling_max = z_rolling_max.dropna()
                    if np.any(z_rolling_max < 3):  # Consider crashed if z < 0.5 meters within the rolling window
                        rospy.loginfo('the Drone {} chrashed'.format(self.nameDrone))
                        settling_times[k].append(float('inf'))
                    else:
                        # Check if the position error is stable without considering the threshold
                        windowed = pd.Series(v[:, i]).rolling(window=window_size).std()
                        within_threshold = windowed < percentage_margin
                        settled_index = next((j for j, settled in enumerate(within_threshold) if settled and j > initial_samples_size), None)
                        settling_times[k].append(timestamps[settled_index] if settled_index is not None else float('inf'))
                else:
                    windowed = pd.Series(v[:, i]).rolling(window=window_size).max()
                    within_threshold = windowed <= error_thresholds[k][i]
                    settled_index = next((j for j, settled in enumerate(within_threshold) if settled and j > initial_samples_size), None)
                    settling_times[k].append(timestamps[settled_index] if settled_index is not None else float('inf'))
        return settling_times

    def calculate_rmse_after_settling(self, errors, settling_times, timestamps):
        rmse_values = {}
        for error_type, error_data in errors.items():
            settling_time = settling_times[error_type]
            settling_time = min(settling_time) if isinstance(settling_time, list) else settling_time
            if settling_time == float('inf'):
                rmse_values[error_type] = float('inf')
            else:
                settled_data = error_data[timestamps >= settling_time]
                rmse_values[error_type] = np.sqrt(np.mean(np.square(settled_data)))
        return rmse_values

    def compute_metrics(self):
        if not self.position_errors or not self.orientation_errors or not self.times:
            rospy.logerr("No data collected to compute metrics.")
            return

        position_errors_np = np.array(self.position_errors)
        orientation_errors_np = np.array(self.orientation_errors)
        times_np = np.array(self.times)
        position_np = np.array(self.linear_pos)


        # Ensure all arrays are of the same length
        min_length = min(len(position_errors_np), len(orientation_errors_np), len(times_np), len(position_np))
        position_errors_np = position_errors_np[:min_length]
        orientation_errors_np = orientation_errors_np[:min_length]
        times_np = times_np[:min_length]
        position_np = position_np[:min_length]

        # Create dictionary to pass to existing functions
        errors = {
            'position_err': position_errors_np,
            'orientation_err': orientation_errors_np
        }

        # Create a DataFrame from the collected data
        df = pd.DataFrame({
            'position': [list(pe) for pe in position_np],
            'time': times_np,
            'position_err': [list(pe) for pe in position_errors_np],
            'orientation_err': [list(oe) for oe in orientation_errors_np]
        }).set_index('time')

        # Ensure each list of errors is the correct length
        df['position_err'] = df['position_err'].apply(lambda x: x if len(x) == 3 else [np.nan]*3)
        df['orientation_err'] = df['orientation_err'].apply(lambda x: x if len(x) == 3 else [np.nan]*3)
        df['position'] = df['position'].apply(lambda x: x if len(x) == 3 else [np.nan]*3)

        # Convert the error lists to DataFrames to facilitate interpolation
        df_position_err = df['position_err'].apply(pd.Series)
        df_orientation_err = df['orientation_err'].apply(pd.Series)
        df_position = df['position'].apply(pd.Series)

        # Interpolate missing values
        df_position_err = df_position_err.interpolate(method='index').ffill().bfill()
        df_orientation_err = df_orientation_err.interpolate(method='index').ffill().bfill()
        df_position = df_position.interpolate(method='index').ffill().bfill()

        # Convert the interpolated DataFrames back to lists
        df['position_err'] = df_position_err.apply(lambda x: list(x), axis=1)
        df['orientation_err'] = df_orientation_err.apply(lambda x: list(x), axis=1)
        df['position'] = df_position.apply(lambda x: list(x), axis=1)

        # Convert the interpolated DataFrame back to numpy arrays
        if df['position_err'].values.size == 0 or df['orientation_err'].values.size == 0 :
            rospy.logerr("Interpolated errors are empty.")
            return
        if df['position'].values.size == 0 :
            rospy.logerr("Interpolated altitude is empty.")
            return
        interpolated_position_errors = np.stack(df['position_err'].values)
        interpolated_orientation_errors = np.stack(df['orientation_err'].values)
        interpolated_position = np.stack(df['position'].values)
        interpolated_times = df.index.values
        # Save the interpolated_position to a CSV file for checking
        interpolated_position_df = pd.DataFrame(interpolated_position, columns=['x', 'y', 'z'])
        interpolated_position_df.to_csv('/home/lollogioddi/Desktop/sett/interpolated_position_{}.csv'.format(self.nameDrone), index=False)
        # Convert interpolated_times to a pandas DataFrame and save to CSV
        interpolated_times_df = pd.DataFrame(interpolated_times, columns=['time'])
        interpolated_times_df.to_csv('/home/lollogioddi/Desktop/sett/interpolated_times.csv_{}'.format(self.nameDrone), index=False)      
        errors['position_err'] = interpolated_position_errors
        errors['orientation_err'] = interpolated_orientation_errors

        z_position = interpolated_position[:, 2]
        maxZ = z_position.max()
        minZ = z_position.min()
        rospy.loginfo("Zmax: {}".format(maxZ))
        rospy.loginfo("Zmin: {}".format(minZ))

        # Compute overshoot and settling time using the provided methods
        rospy.loginfo("Calculating overshoots")
        overshoots = self.calculate_overshoots(errors)
        rospy.loginfo("Calculating settling times")
        settling_times = self.calculate_settling_times(errors, interpolated_times, interpolated_position)
        rospy.loginfo("Calculating rmse")
        rmse = self.calculate_rmse_after_settling(errors, settling_times, interpolated_times)
        # Flatten the lists and convert to float
        flat_overshoots = [float(item) for sublist in overshoots.values() for item in sublist]
        rospy.loginfo("Overshoots evaluated: {}".format(flat_overshoots))
        flat_settling_times = [float(item) for sublist in settling_times.values() for item in sublist]
        rospy.loginfo("Settling times evaluated: {}".format(flat_settling_times))
        flat_rmse = [float(item) if isinstance(sublist, (list, np.ndarray)) else float(sublist) for sublist in rmse.values() for item in (sublist if isinstance(sublist, (list, np.ndarray)) else [sublist])]
        rospy.loginfo("Rmse evaluated: {}".format(flat_rmse))
        metrics_msg = Float64MultiArray()
        metrics_msg.data = flat_overshoots + flat_settling_times + flat_rmse
        self.pub_metrics.publish(metrics_msg)

if __name__ == '__main__':
    try:
        node = DataSaver()
        while not rospy.is_shutdown():
            node.process_and_publish()
            node.rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node terminated.")
