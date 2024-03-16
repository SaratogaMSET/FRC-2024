from functools import partial
from typing import List, Optional

import gtsam
import numpy as np
import math
import random
import json
import os

import AprilTagDefault

def simulateData(num_steps):
    robot_trajectory = {}
    april_tag_initial = AprilTagDefault.april_tag_poses

    sample_data = []
    for step in range(num_steps):
        robot_pose = gtsam.Pose3(gtsam.Rot3.Rz(np.random.normal(0, 0.03) + step * 0.1), gtsam.Point3(2 + step * 0.1 + np.random.normal(0, 0.03), -1 + step * 0.1, 0.0 + np.random.normal(0, 0.03)))
        # robot_pose = gtsam.Pose3(gtsam.Rot3.Rz(0), gtsam.Point3(0, 0, 0))
        robot_trajectory[step] = robot_pose

        num_tags = random.randint(1, 4)
        tags_to_observe = random.sample(list(april_tag_initial.keys()), k=num_tags)
        step_data = {}
        for tag_id in tags_to_observe:
            # Add noise and bias to the observed AprilTag poses
            true_pose = april_tag_initial[tag_id]
            noisy_pose = gtsam.Pose3(gtsam.Rot3.Rodrigues(0, 0, 0), gtsam.Point3(0, 0, 0))  # Initialize noisy pose
            
            # Simulate bias (translation and rotation)
            noise_translation = gtsam.Point3(np.random.normal(0, 0.005), np.random.normal(0, 0.005), 0)  # Noise in x and y directions
            noise_rotation = gtsam.Rot3.Rodrigues(np.random.normal(0, 0.0), np.random.normal(0, 0.0), 0)  # Noise in z-axis rotation
            noisy_pose = true_pose.compose(gtsam.Pose3(noise_rotation, noise_translation))
            
            # Add the noisy pose to the step data
            step_data[tag_id] = robot_pose.between(noisy_pose)  # Use the robot key here
        
        # Add the step data to the sample data
        sample_data.append(step_data)

    #     Print sample data
    for i, step_data in enumerate(sample_data):
        if(i < 10):
            print(f"Step {i+1}:")
            print(robot_trajectory[i])
            for tag_id, pose in step_data.items():
                print(f"Tag {tag_id}:")
                print(pose)

    return sample_data, robot_trajectory