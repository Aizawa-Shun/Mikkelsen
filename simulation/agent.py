import torch
from torch import tensor, optim, device, cuda
import torch.nn.functional as F
import torch.nn as nn
import numpy as np
import random
import os

class NeuralNetwork(nn.Module):
    def __init__(self, input_dim, num_joints, angle_range_size, hidden_layers):
        super().__init__()
        self.num_joints = num_joints
        self.angle_range_size = angle_range_size

        layers = []
        input_size = input_dim
         
        for hidden_size in hidden_layers:
            layers.append(nn.Linear(input_size, hidden_size))
            layers.append(nn.ReLU())
            input_size = hidden_size

        layers.append(nn.Linear(input_size, self.num_joints * self.angle_range_size))
        self.network = nn.Sequential(*layers)
    
    def forward(self, x):
        x = self.network(x)
        return x.view(-1, self.num_joints, self.angle_range_size)
    
class DQNAgent:
    def __init__(self, input_dim, config):
        self.state_dim = input_dim

        self.num_joints  = 8
        self.angle_range = np.deg2rad(np.arange(-180, 180, 1))
        self.angle_range_size = len(self.angle_range)
        
        self.joints = {
            'left': {
                'hip_roll': 1,
                'hip_yaw': 4,
                'hip_pitch': 7,
                'knee': 10,
                'ankle': 15
            },
            'right': {
                'hip_roll': 26,
                'hip_yaw': 29,
                'hip_pitch': 32,
                'knee': 35,
                'ankle': 40
            }
        }
        self.targget_update = config['targget_update']
        self.weight_save_path = config['weight_save_path']
        self.target_reward = config['target_reward']
        print(f'[INFO] Target Reward: {self.target_reward}')
        self.step = 0
        self.set_model(config)

    def set_model(self, config):
        self.weight_file_name = config['weight_file_path']
        self.weight_file = os.path.isfile(self.weight_file_name)
        self.device = device('cuda' if cuda.is_available() else 'cpu')
        print(f'[INFO] Using Device: {self.device.type}')
        self.learning_rate = config['learning_rate']

        if config['make_file']:
            self.model = NeuralNetwork(self.state_dim, self.num_joints, self.angle_range_size, config['hidden_layers']).to(self.device)
        else:
            try:
                self.model = torch.load(self.weight_file).to(self.device)
            except FileNotFoundError:
                print("Weight file not found, initializing a new model.")
                self.model = NeuralNetwork(self.state_dim, self.num_joints, self.angle_range, config['hidden_layers']).to(self.device)

        self.opt = optim.Adam(self.model.parameters(), lr=self.learning_rate)
    
    def setup(self):
        self.one_hot = None
        self.rewards = tensor([], dtype=torch.float32, requires_grad=True)
        self.policies = tensor([], dtype=torch.float32, requires_grad=True)
        self.steps = tensor([], dtype=torch.float32, requires_grad=True)
        self.experiences = []
        self.policy_list = []
        self.reward_list = []
        self.mean_reward_list = np.zeros((0))
    
    def first_states(self, states):
        # Get joint states (angles and torques)
        joint_positions = np.array(states['joint_positions'])
        joint_torques = np.array(states['joint_torques'])        

        # Combine joint angles and torques into one input vector
        self.inputs = np.concatenate((joint_positions, joint_torques))

        # Processes the data for input into the neural network
        self.inputs = torch.tensor(self.inputs, requires_grad=True).float().unsqueeze(0).to(self.device)
        
    def act(self):
        '''Decides the action to be taken by the robot based on the neural network's output.'''
        with torch.no_grad():
            self.output = self.model(self.inputs)

        # Apply softmax to get probabilities
        probabilities = F.softmax(self.output, dim=2).detach().cpu().numpy()

        left_joint_angles = {}
        right_joint_angles = {}
        one_hot_list = []

        for side in ['left', 'right']:
            for i, joint in enumerate(self.joints[side].keys()):
                if joint != 'ankle':  # Skip ankle joints
                    prob = probabilities[0, i, :]

                    # Normalize to make the sum equal to 1
                    prob /= np.sum(prob)

                    angle_idx = np.random.choice(len(self.angle_range), p=prob)
                    angle = self.angle_range[angle_idx]
                    if side == 'left':
                        left_joint_angles[joint] = np.round(angle, decimals=2)
                    else:
                        right_joint_angles[joint] = np.round(angle, decimals=2)

                    one_hot = np.zeros(len(self.angle_range))
                    one_hot[angle_idx] = 1
                    one_hot_list.append(one_hot)

        one_hot = torch.tensor(one_hot_list, dtype=torch.float32)
        action = {
            'left': left_joint_angles,
            'right': right_joint_angles
        }
        
        return action, one_hot
    
    def learn(self, states, action, reward, one_hot, next_state, done):
        # Get joint states (angles and torques)
        joint_positions = np.array(states['joint_positions'])
        joint_torques = np.array(states['joint_torques'])        

        # Combine joint angles and torques into one input vector
        self.inputs = np.concatenate((joint_positions, joint_torques))

        # Processes the data for input into the neural network
        self.inputs = torch.tensor(self.inputs, requires_grad=True).float().unsqueeze(0).to(self.device)
        
        self.store(action, reward, one_hot)

    def store(self, action, reward, one_hot):
        '''Stores information from each step of the learning process.'''
        step_dict = {
            "output": self.output,
            "reward": reward,
            "action": action,
            "one_hot": one_hot
        }
        self.experiences.append(step_dict)

        output_flat = step_dict["output"].flatten()
        one_hot_flat = step_dict["one_hot"].flatten()

        self.policy_list.append(torch.dot(output_flat, one_hot_flat).item())
        self.reward_list.append(step_dict["reward"])

        self.step += 1
        self.interval = 0

    def update_store_data(self):
        max_length = len(self.policy_list) if self.policies.size(0) == 0 else self.policies.size(1)
        if len(self.policy_list) < max_length:
            padding_size = max_length - len(self.policy_list)
            self.policy_list.extend([0.0] * padding_size)
            self.reward_list.extend([0.0] * padding_size)
        elif len(self.policy_list) > max_length:
            max_length = len(self.policy_list)
            self.policies = torch.cat([self.policies, torch.zeros((self.policies.size(0), max_length - self.policies.size(1)))], dim=1)
            self.rewards = torch.cat([self.rewards, torch.zeros((self.rewards.size(0), max_length - self.rewards.size(1)))], dim=1)

        self.policies = torch.cat([self.policies, torch.tensor(self.policy_list, requires_grad=True).view(1, -1)])
        self.rewards = torch.cat([self.rewards, torch.tensor(self.reward_list, requires_grad=True).view(1, -1)])
        self.steps = torch.cat([self.steps, tensor([float(self.step)], requires_grad=True)])
    
    def calculate_average_reward(self):
        '''Calculates the average reward over all episodes.'''
        rewards = torch.cat([self.rewards, torch.tensor(self.reward_list, requires_grad=True).view(1, -1)])
        average_reward = rewards.sum(dim=0).mean().item()
        return average_reward

    def update_policy(self):
        '''Updates the policy based on the rewards and policies collected during the episodes.'''
        reward_ave = (self.rewards.nansum(dim=1) / self.steps).mean()
        clampped = torch.clamp(self.policies, 1e-10, 1)

        Jmt = clampped.log() * (self.rewards - reward_ave)

        J = (Jmt.nansum(dim=1) / self.steps).mean()

        J.backward()
        self.opt.step()
        self.opt.zero_grad()
    
    def save_weight_file(self):
        torch.save(self.model, self.weight_save_path)