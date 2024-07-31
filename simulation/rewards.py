import numpy as np

def walk_2d(states, falling):
    reward = 0

    # Reward based on forward distance
    if states['pos'][0] > 0: reward += states['pos'][0] * 500

    # Penalty based on falling down
    if falling: reward -= 100

    # Penalty based on total force
    reward -= states['total_force'] * 0.8     
    return  reward

def stationary(states):
        '''
        Calculate the reward for maintaining a stable upright posture.

        Args:
            robot_state (dict): The current state of the robot, including position, angle, total force, velocity, energy used, and foot contact status.
            stable_duration (float): The duration for which the robot has maintained a stable posture.
            zmp (list): The Zero Moment Point calculated from the calculate_zmp function.
            support_polygon_area (float): The support polygon area calculated from the calculate_contact_area function.

        Returns:
            float: The calculated reward for the current state.
        '''
        reward = 0

        # Reward based on Center of Mass (CoM) height
        target_height = 0.30
        height_diff = abs(states['pos'][2] - target_height)
        reward += max(0, 1 - height_diff) * 8

        # Reward based on lateral position
        # lateral_position = abs(states['pos'][0]) + abs(states['pos'][1])
        # reward += max(0, 1 - lateral_position)

        # Penalties based on overall power, speed, and energy consumption
        # reward -= states['total_force'] * 0.05     # Penalties based on total force
        reward -= states['total_velocity'] * 0.05  # Penalties based on total velocity

        # Reward based on ZMP and support polygon area
        if states['zmp'] and states['contact_area']:
            zmp_in_support_polygon = np.linalg.norm(states['zmp'][:2]) <= np.sqrt(states['contact_area'])
            reward += 3.0 if zmp_in_support_polygon else -2.0

        # Check if both feet are in contact with the ground
        if states['left_foot_contact'] and states['right_foot_contact']:
            reward += 8.0
        else:
            reward -= 1.0

        return reward


     