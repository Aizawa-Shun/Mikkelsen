def reward_stationary(states):
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
        reward += max(0, 1 - height_diff) * 5

        # Reward based on lateral position
        # lateral_position = abs(states['pos'][0]) + abs(states['pos'][1])
        # reward += max(0, 1 - lateral_position)

        # Penalties based on overall power, speed, and energy consumption
        # reward -= states['total_force'] * 0.05     # Penalties based on total force
        # reward -= self.states['total_velocity'] * 0.05  # Penalties based on total velocity

        # Check if both feet are in contact with the ground
        if states['left_foot_contact'] and states['right_foot_contact']:
            reward += 5.0
        else:
            reward -= 1.0

        return reward