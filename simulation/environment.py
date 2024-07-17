import pybullet as p
import pybullet_data
from time import sleep
import numpy as np
import torch 

class Environment:
    def __init__(self, config):
        self.robot_path = config['robot_urdf']
        self.frame_path = config['frame_urdf']
        self.gui = config['gui']
        self.initial_position = config['initial_position']
        self.initial_orientation = config['initial_orientation']
        self.dt = config['dt']
        self.step_dt = config['step_dt']

        self.joints = {
            'left': {
                'hip_roll': 1,
                'hip_yaw': 4,
                'hip_pitch': 7,
                'knee': 10,
                'ankle': 15,
                'foot': 23
            },
            'right': {
                'hip_roll': 26,
                'hip_yaw': 29,
                'hip_pitch': 32,
                'knee': 35,
                'ankle': 40,
                'foot': 48
            }
        }
        self.legs = list(joint_id for side in self.joints.values() for joint_name, joint_id in side.items() if joint_name != 'foot')

        # Connect to physics engine
        if self.gui: self.physicsClient = p.connect(p.GUI)
        else: self.physicsClient = p.connect(p.DIRECT)

        # Add path for pybullet data
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set gravity
        p.setGravity(0, 0, -9.8)

        # Load URDF
        self.plane = p.loadURDF('plane.urdf')  # Load floor
        self.flame = p.loadURDF(self.frame_path, useFixedBase=True)

        self.setup()
    
    def setup(self):
        ''' Set up the physics engine and environment '''
        # Load robot's URDF
        self.robot = self.load_robot(self.robot_path)

        # Make closed link
        self.make_closed_link()

        # Simulation time
        self.t = 0
        self.interval = 0

        # Dictionary to store the robot's state
        self.states = {'pos': 0}

    def load_robot(self, robot_path):
        initial_orientation = p.getQuaternionFromEuler(self.initial_orientation)   # Convert from Euler to Quaternion
        return p.loadURDF(robot_path, basePosition=self.initial_position, baseOrientation=initial_orientation)
    
    def make_closed_link(self, maxForce=10e5):
        constraint_id_19_24 = p.createConstraint(
            parentBodyUniqueId=self.robot,
            parentLinkIndex=24,
            childBodyUniqueId=self.robot,
            childLinkIndex=19,
            jointType=p.JOINT_POINT2POINT, 
            jointAxis=[0, 0, 1],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0]
        )
        p.changeConstraint(constraint_id_19_24, maxForce=maxForce)

        constraint_id_43_49 = p.createConstraint(
            parentBodyUniqueId=self.robot,
            parentLinkIndex=43,
            childBodyUniqueId=self.robot,
            childLinkIndex=49,
            jointType=p.JOINT_POINT2POINT, 
            jointAxis=[0, 0, 1],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0]
        )
        p.changeConstraint(constraint_id_43_49, maxForce=maxForce)
    

    def reset(self):
        ''' Reset the environment to the initial state '''
        p.removeBody(self.robot)
        self.setup()
    
    def step(self, action):
        p.stepSimulation()
        if self.gui: sleep(self.dt)
        self.t += self.dt
        self.interval += 0

        self.control_by_model(action)

        self.get_observation()
        reward = self.calculate_reward()
        done = self.check_done()

        return  reward, done
    
    def step_simulation(self):
        p.stepSimulation()
        if self.gui: sleep(self.dt)
        self.t += self.dt
        self.interval += self.dt
    
    def control_by_model(self, action, torque=1.4):
        # Indices to skip (for example, ankle joints)
        ankle_joint_indices = [self.joints['left']['ankle'], self.joints['right']['ankle']]  # Index of ankle joints of left and right leg

        # Control left leg
        for name, index in self.joints['left'].items():
            if index not in ankle_joint_indices:
                if name in action['left']:
                    self.control_leg('left', name, action['left'][name], torque)

        # Control right leg
        for name, index in self.joints['right'].items():
            if index not in ankle_joint_indices:
                if name in action['right']:
                    self.control_leg('right', name, action['right'][name], torque)

    
    def control_leg(self, side, joint, angle, torque):
        """
        Function to control the robot's legs based on the specified parameters.
        :param side: String indicating left or right leg ('left' or 'right').
        :param joint: Joint name to control ('hip_roll', 'hip_yaw', 'hip_pitch', 'knee', 'ankle').
        :param angle: Target angle to reach [rad].
        """
        # Convert joint name to index
        joint_index = self.joints[side][joint]

        # Basic position control without feedback
        p.setJointMotorControl2(bodyUniqueId=self.robot,
                                jointIndex=joint_index, 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=angle,
                                force=torque
                                )

    def get_observation(self):
        self.states['pre_pos'] = self.states['pos']
        pos, ori = p.getBasePositionAndOrientation(self.robot)
        euler_angles = p.getEulerFromQuaternion(ori)

        self.states['pos'] = pos
        self.states['angle'] = euler_angles
        
        self.states['left_foot_contact'] = p.getContactPoints(self.robot, -1, self.joints['left']['foot'])
        self.states['right_foot_contact'] = p.getContactPoints(self.robot, -1, self.joints['right']['foot'])

        joint_states = p.getJointStates(self.robot, self.legs)
        self.states['joint_positions'] = [state[0] for state in joint_states]  
        self.states['joint_velocities'] = [state[1] for state in joint_states]  
        self.states['joint_torques'] = [state[3] for state in joint_states]

        self.states['total_force'] = np.sum(np.abs(self.states['joint_torques']))
        self.states['total_velocity'] = np.sum(np.abs(self.states['joint_velocities']))
    
    def calculate_reward(self):
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
        print(self.states['pos'][2])
        height_diff = abs(self.states['pos'][2] - target_height)
        reward += max(0, 1 - height_diff)

        # Reward based on lateral position
        lateral_position = abs(self.states['pos'][0]) + abs(self.states['pos'][1])
        reward += max(0, 1 - lateral_position)

        # Penalties based on overall power, speed, and energy consumption
        reward -= self.states['total_force'] * 0.05     # Penalties based on total force
        # reward -= self.states['total_velocity'] * 0.05  # Penalties based on total velocity

        # Check if both feet are in contact with the ground
        if self.states['left_foot_contact'] and self.states['right_foot_contact']:
            reward += 3.5
        else:
            reward -= 1.0

        return reward
    
    def check_done(self):
        done = False
        contact = None
        for joint_id in range(0, 5):
            if p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=joint_id):
                contact = p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=joint_id)
        if contact == None:
            for joint_id in range(25, 30):
                if p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=joint_id):
                    contact = p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=joint_id)
        if contact == None:
            if p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=self.joints['left']['hip_yaw']):
                contact = p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=self.joints['right']['hip_yaw'])
        if contact == None:
            if p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=-1):
                contact = p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=-1)
        
        if contact:
            done = True
        return done

    def disconnect(self):
        p.disconnect()