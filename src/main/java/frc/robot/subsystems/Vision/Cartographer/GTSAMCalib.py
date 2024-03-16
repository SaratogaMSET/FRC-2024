import gtsam
import numpy as np
import math
import time
import json
import os

import AprilTagDefault

measurement_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]))

graph = gtsam.NonlinearFactorGraph()
initial_estimate = gtsam.Values()

robot_trajectory = {}
april_tag_initial = AprilTagDefault.april_tag_poses


def optimize_landmark_poses(data):
    sample_data = data[0]
    robot_trajectory = data[1]

    print("Sample Size of", len(sample_data))
    
    if(robot_trajectory is None):
        robot_trajectory = {}
        for step in range(len(sample_data)):
            robot_pose = gtsam.Pose3(gtsam.Rot3.Rz(0), gtsam.Point3(0, 0, 0))
            robot_trajectory[step] = robot_pose

    landmark_graph = gtsam.NonlinearFactorGraph()
    landmark_estimate = gtsam.Values()

    single_target_frames = 0
    # Add factors and initial estimates for landmark poses
    for robot_key, relative_poses in enumerate(sample_data):
        # print(len(relative_poses))
        if(len(relative_poses) == 1): single_target_frames += 1
        # if(robot_key < len(sample_data) - 1):
        #     landmark_graph.add(gtsam.BetweenFactorPose3(robot_key, robot_key+1, robot_trajectory[robot_key + 1].between(robot_trajectory[robot_key]), measurement_noise))
        if not landmark_estimate.exists(robot_key):
            landmark_estimate.insert(robot_key, robot_trajectory[robot_key])
            
        for tag_id, relative_pose in relative_poses.items():
            tag_key = len(sample_data) + tag_id  # Create a new key for each AprilTag pose, offset by sample size
            if not landmark_estimate.exists(tag_key):
                # Add initial estimate for each landmark pose
                landmark_estimate.insert(tag_key, april_tag_initial[tag_id])
            
            # https://gtsam-jlblanco-docs.readthedocs.io/en/latest/LandmarkBasedSLAM.html
            landmark_graph.add(gtsam.BetweenFactorPose3(robot_key, tag_key, relative_pose, measurement_noise))

    optimizer = gtsam.LevenbergMarquardtOptimizer(landmark_graph, landmark_estimate)

    start_time = time.time()
    result = optimizer.optimize()
    print("Optimization Time: ", time.time() - start_time)
    print("Single Target Frames: ", single_target_frames)

    # Retrieve optimized landmark poses
    optimized_landmark_poses = {}
    for key in landmark_estimate.keys():
        tag_id = key - len(sample_data)
        optimized_landmark_poses[tag_id] = result.atPose3(key)

    return optimized_landmark_poses


def calculate_error(estimated_poses, ground_truth_poses):
    errors = {}
    for tag_id, estimated_pose in estimated_poses.items():
        # Check if the tag ID exists in the ground truth poses
        if tag_id in ground_truth_poses:
            ground_truth_pose = ground_truth_poses[tag_id]

            # Calculate the translation error
            translation_error = estimated_pose.translation() - ground_truth_pose.translation()

            # Calculate the rotation error
            estimated_rot = estimated_pose.rotation().matrix()
            ground_truth_rot = ground_truth_pose.rotation().matrix()
            rotation_error = np.arccos((np.trace(np.dot(estimated_rot.T, ground_truth_rot)) - 1) / 2)

            errors[tag_id] = (translation_error, rotation_error)
    
    return errors