import csv
import os

class DataSaver:
    def __init__(self, reward_filepath, angles_filepath, torques_filepath):
        self.reward_filepath = reward_filepath
        self.angles_filepath = angles_filepath
        self.torques_filepath = torques_filepath

        if not os.path.exists(reward_filepath):
            with open(reward_filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['episode', 'reward'])

        if not os.path.exists(angles_filepath):
            with open(angles_filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Assuming there are 10 joint angles, named 'angle_1', 'angle_2', ..., 'angle_10'
                writer.writerow(['episode', 'time', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'joint_8', 'joint_9', 'joint_10'])

        if not os.path.exists(torques_filepath):
            with open(angles_filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Assuming there are 10 joint angles, named 'angle_1', 'angle_2', ..., 'angle_10'
                writer.writerow(['episode', 'time', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'joint_8', 'joint_9', 'joint_10'])

    def save_reward(self, episode, reward):
        with open(self.reward_filepath, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([episode, reward])

    def save_joint_angles(self, episode, time, joint_angles):
        with open(self.angles_filepath, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([episode, time] + joint_angles)
    
    def save_torque_angles(self, episode, time, joint_torques):
        with open(self.torques_filepath, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([episode, time] + joint_torques)