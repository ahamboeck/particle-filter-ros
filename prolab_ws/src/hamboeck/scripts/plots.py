#!/usr/bin/env python
import rospy
import message_filters
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseArray
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import numpy as np
import scipy.ndimage
import os

# Create a directory for plots if it doesn't already exist
plot_dir = "./plots"
if not os.path.exists(plot_dir):
    os.makedirs(plot_dir)

# Global variables to store ground truth and pose array data
ground_truth_x = []
ground_truth_y = []
ground_truth_theta = []
pose_x = []
pose_y = []
pose_theta = []

def callback(model_states_msg, pose_msg):
    try:
        index = model_states_msg.name.index('turtlebot3')
    except ValueError:
        rospy.loginfo("Robot not found in model states.")
        return

    # Extract ground truth information
    x = model_states_msg.pose[index].position.x
    y = model_states_msg.pose[index].position.y
    quat = model_states_msg.pose[index].orientation
    (_, _, theta) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    ground_truth_x.append(x)
    ground_truth_y.append(y)
    ground_truth_theta.append(theta)

    # Calculate average pose from PoseArray
    x_vals = [pose.position.x for pose in pose_msg.poses]
    y_vals = [pose.position.y for pose in pose_msg.poses]
    theta_vals = [euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2] for pose in pose_msg.poses]

    pose_x.append(np.mean(x_vals))
    pose_y.append(np.mean(y_vals))
    pose_theta.append(np.mean([t - 2*np.pi if t > np.pi else t + 2*np.pi if t < -np.pi else t for t in theta_vals]))

def plot_results(sigma=5):  # Increased sigma for more smoothing
    # Apply Gaussian smoothing to the data
    smooth_pose_x = scipy.ndimage.gaussian_filter(pose_x, sigma=sigma)
    smooth_pose_y = scipy.ndimage.gaussian_filter(pose_y, sigma=sigma)
    smooth_pose_theta = scipy.ndimage.gaussian_filter(pose_theta, sigma=sigma)

    # Calculate errors post-smoothing
    x_error = np.abs(np.array(smooth_pose_x) - np.array(ground_truth_x))
    y_error = np.abs(np.array(smooth_pose_y) - np.array(ground_truth_y))
    theta_error = np.abs(np.array(smooth_pose_theta) - np.array(ground_truth_theta))

    plt.rcParams.update({'font.size': 24})  # Increased font size for better readability

    # Path Plot
    plt.figure(figsize=(12, 14))
    plt.plot(ground_truth_x, ground_truth_y, label='True Path', color='blue')
    plt.plot(smooth_pose_x, smooth_pose_y, label='MCL Predicted Path (Smoothed)', color='red')
    plt.title('Path Traveled (X-Y Plane)')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.legend(loc='best')
    plt.axis('equal')
    plt.grid(True)
    plt.savefig(os.path.join(plot_dir, 'Path_Traveled.png'))
    plt.close()

    # X Position Plot
    plt.figure(figsize=(12, 14))
    plt.plot(ground_truth_x, label='True Pose X (meters)')
    plt.plot(smooth_pose_x, label='MCL Predicted Pose X (Smoothed)')
    plt.title('X Position Over Time')
    plt.xlabel('Time Step')
    plt.ylabel('X (meters)')
    plt.legend(loc='best')
    plt.grid(True)
    plt.savefig(os.path.join(plot_dir, 'X_Position.png'))
    plt.close()

    # Y Position Plot
    plt.figure(figsize=(12, 14))
    plt.plot(ground_truth_y, label='True Pose Y (meters)')
    plt.plot(smooth_pose_y, label='MCL Predicted Pose Y (Smoothed)')
    plt.title('Y Position Over Time')
    plt.xlabel('Time Step')
    plt.ylabel('Y (meters)')
    plt.legend(loc='best')
    plt.grid(True)
    plt.savefig(os.path.join(plot_dir, 'Y_Position.png'))
    plt.close()

    # Theta Plot
    plt.figure(figsize=(12, 14))
    plt.plot(ground_truth_theta, label='True Pose Theta (radians)')
    plt.plot(smooth_pose_theta, label='MCL Predicted Pose Theta (Smoothed)')
    plt.title('Theta Over Time')
    plt.xlabel('Time Step')
    plt.ylabel('Theta (radians)')
    plt.legend(loc='best')
    plt.grid(True)
    plt.savefig(os.path.join(plot_dir, 'Theta_Position.png'))
    plt.close()

    # Error Plots
    plt.figure(figsize=(12, 14))
    plt.plot(x_error, label='X Error (meters)', color='orange')
    plt.title('Error in X Position')
    plt.xlabel('Time Step')
    plt.ylabel('Error (meters)')
    plt.legend(loc='best')
    plt.grid(True)
    plt.savefig(os.path.join(plot_dir, 'X_Error.png'))
    plt.close()

    plt.figure(figsize=(12, 14))
    plt.plot(y_error, label='Y Error (meters)', color='green')
    plt.title('Error in Y Position')
    plt.xlabel('Time Step')
    plt.ylabel('Error (meters)')
    plt.legend(loc='best')
    plt.grid(True)
    plt.savefig(os.path.join(plot_dir, 'Y_Error.png'))
    plt.close()

    plt.figure(figsize=(12, 14))
    plt.plot(theta_error, label='Theta Error (radians)', color='purple')
    plt.title('Error in Theta')
    plt.xlabel('Time Step')
    plt.ylabel('Error (radians)')
    plt.legend(loc='best')
    plt.grid(True)
    plt.savefig(os.path.join(plot_dir, 'Theta_Error.png'))
    plt.close()

def listener():
    rospy.init_node('plot_true_and_estimated_pose')

    model_states_sub = message_filters.Subscriber('/gazebo/model_states', ModelStates)
    pose_array_sub = message_filters.Subscriber('pose_array', PoseArray)

    ats = message_filters.ApproximateTimeSynchronizer([model_states_sub, pose_array_sub], queue_size=10, slop=0.1, allow_headerless=True)
    ats.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        plot_results(sigma=5)  # Apply Gaussian smoothing with sigma=5 for more substantial smoothing
