import csv
import os
from datetime import datetime
import shutil

class DataSaver:
    def __init__(self, mode, algorithm):
        # Get today date
        date_str = datetime.now().strftime("%Y%m%d")
        # Create directory path
        self.directory = os.path.join('data', f'{date_str}_{algorithm}_{mode}')
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
            
        self.reward_filepath = os.path.join(self.directory, 'rewards.csv')
        self.angles_filepath = os.path.join(self.directory, 'angles.csv')
        self.torques_filepath = os.path.join(self.directory, 'torques.csv')

        if not os.path.exists(self.reward_filepath):
            with open(self.reward_filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['episode', 'reward'])

        if not os.path.exists(self.angles_filepath):
            with open(self.angles_filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Assuming there are 10 joint angles, named 'angle_1', 'angle_2', ..., 'angle_10'
                writer.writerow(['episode', 'time', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'joint_8', 'joint_9', 'joint_10'])

        if not os.path.exists(self.torques_filepath):
            with open(self.angles_filepath, mode='w', newline='') as file:
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
    
    def zip_directory(self):
        # Specify the path to the ZIP file
        zip_filepath = self.directory + '.zip'
        shutil.make_archive(self.directory, 'zip', self.directory)
        print(f'The folder has been compressed to {zip_filepath}.')