import pybullet as p
import pybullet_data
from time import sleep
import numpy as np
from scipy.spatial import ConvexHull
from utils.keyboard import InputHandler
from utils.camera import Camera
from simulation import rewards

class Environment:
    def __init__(self, config, operation, mode, dimension, target_action):
        self.dimension = dimension
        if self.dimension == '3d':
            self.robot_path = config['robot_urdf']
        elif self.dimension == '2d':
            self.robot_path = config['robot2d_urdf']
        self.frame_path = config['frame_urdf']
        self.gui = config['gui']
        self.initial_position = config['initial_position']
        self.initial_orientation = p.getQuaternionFromEuler(config['initial_orientation'])  # Convert from Euler to Quaternion
        self.sim_time = config['sim_time']
        self.dt = config['dt']
        self.step_dt = config['step_dt']

        self.done = False

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
        if self.dimension == '2d':
            if dimension == '2d':
                for side in self.joints:
                    for joint in self.joints[side]:
                        self.joints[side][joint] += 3

        self.legs = list(joint_id for side in self.joints.values() for joint_name, joint_id in side.items() if joint_name != 'foot')

        # Indices to skip (for example, ankle joints)
        self.exclusion_joint_indices = [self.joints['left']['ankle'], self.joints['right']['ankle']]  # Index of ankle joints of left and right leg
        if self.dimension == '2d':
            self.unused_joints = [
                self.joints['left']['hip_roll'], self.joints['right']['hip_roll'],
                self.joints['left']['hip_yaw'], self.joints['right']['hip_yaw'],
            ]
            # Add hip_roll and hip_yaw joints to the exclusion list
            self.exclusion_joint_indices.extend(self.unused_joints)
            
        # Target of movement by learning
        self.target_action = target_action
        
        # Connect to physics engine
        if self.gui: self.physicsClient = p.connect(p.GUI)
        else: self.physicsClient = p.connect(p.DIRECT)

        # Add path for pybullet data
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set gravity
        GRAVITY = -9.8
        p.setGravity(0, 0, GRAVITY)

        # Load URDF
        self.plane = p.loadURDF('plane.urdf')  # Load floor
        self.flame = p.loadURDF(self.frame_path, useFixedBase=True)

        self.setup()

        self.is_check_down = True if operation == 'simulation' else False

        self.camera = Camera(self.dimension)
        self.input_handler = InputHandler()
        self.input_handler.register_key_action('c', self.toggle_camera)
        self.camera_on = True

        if mode=='test': self.test_simulation()

    def setup(self):
        ''' Set up the physics engine and environment '''
        # Load robot URDF
        self.robot = self.load_robot(self.robot_path)

        # Make closed link
        self.make_closed_link(self.dimension)

        # Set color
        self.set_color()

        # Simulation time
        self.t = 0
       
        # Dictionary to store the robot's state
        self.states = {'pos': 0}

     


    def load_robot(self, robot_path):
        return p.loadURDF(robot_path, basePosition=self.initial_position, baseOrientation=self.initial_orientation)
    
    def make_closed_link(self, dimension, maxForce=10e5):
        parent_index1, child_index1, parent_index2, child_index2 = 19, 24, 43, 49
        if dimension == '2d':
            parent_index1, child_index1, parent_index2, child_index2 = (
                parent_index1 + 3, child_index1 + 3, parent_index2 + 3, child_index2 + 3
            )
        constraint_id_19_24 = p.createConstraint(
            parentBodyUniqueId=self.robot,
            parentLinkIndex=parent_index1,
            childBodyUniqueId=self.robot,
            childLinkIndex=child_index1,
            jointType=p.JOINT_POINT2POINT, 
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0]
        )
        p.changeConstraint(constraint_id_19_24, maxForce=maxForce)
        
        constraint_id_43_49 = p.createConstraint(
            parentBodyUniqueId=self.robot,
            parentLinkIndex=parent_index2,
            childBodyUniqueId=self.robot,
            childLinkIndex=child_index2,
            jointType=p.JOINT_POINT2POINT, 
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0]
        )
        p.changeConstraint(constraint_id_43_49, maxForce=maxForce)
        
    
    def set_color(self):
        for joint_id in range(-1, p.getNumJoints(self.robot), 1):
            p.changeVisualShape(self.robot, joint_id, rgbaColor=[0.5,0.5,0.5,1.00])
    
    def reset(self):
        ''' Reset the environment to the initial state '''
        p.removeBody(self.robot)
        self.setup()
    
    def step(self, action=None):
        p.stepSimulation()
        if self.gui: 
            sleep(self.dt)
            self.input_handler.check_input()
            self.camera.set(self.robot, self.camera_on)
        self.t += self.dt

        if action != None:
            self.control_by_model(action)
            self.get_observation()
            self.check_done()
            if self.target_action == 'stationary':
                reward = rewards.stationary(self.states)
            elif self.target_action == 'walk':
                if self.dimension == '2d':
                    reward = rewards.walk_2d(self.states, self.done)
                elif self.dimension == '3d':
                    reward = rewards.walk_3d(self.states, self.done)
            return  reward, self.done
        else:
            if self.is_check_down:
                self.check_done()
                return self.done
            else:
                pass
    
    def initial_pose(self):
        angles = {
            'hip_roll': 0,
            'hip_yaw': 0,
            'hip_pitch': 0,
            'knee': 0,
            'ankle': 0
        }
        torque = {
            'hip_roll': 14,
            'hip_yaw': 14,
            'hip_pitch': 14,
            'knee': 14,
            'ankle': 14
        }
        initial_action = {
            'left': {'angles': angles, 'torques': torque},
            'right':{'angles': angles, 'torques': torque}
        }
        self.control_by_model(initial_action)
    
    def control_by_model(self, action):
        # Control left leg
        for name, index in self.joints['left'].items():
            if index not in self.exclusion_joint_indices:
                if name in action['left']['angles'] and name in action['left']['torques']:
                    angle = action['left']['angles'][name]
                    torque = action['left']['torques'][name]
                    self.control_leg('left', name, angle, torque)
        # Control right leg
        for name, index in self.joints['right'].items():
            if index not in self.exclusion_joint_indices:
                if name in action['left']['angles'] and name in action['left']['torques']:
                    angle = action['right']['angles'][name]
                    torque = action['right']['torques'][name]
                    self.control_leg('right', name, angle, torque)

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

        pos, ori = 0, 0 
        if self.dimension == '3d':
            pos, ori = p.getBasePositionAndOrientation(self.robot)
        elif self.dimension == '2d':
            body_id = 4
            pos, ori = p.getLinkState(self.robot, body_id)[4], p.getLinkState(self.robot, body_id)[5]
        euler_angles = p.getEulerFromQuaternion(ori)

        self.states['pos'] = pos
        self.states['angle'] = euler_angles

        self.states['base_velocity'] = (np.array(pos) - np.array(self.states['pre_pos'])) / self.dt
        
        self.states['left_foot_contact'] = p.getContactPoints(self.robot, -1, self.joints['left']['foot'])
        self.states['right_foot_contact'] = p.getContactPoints(self.robot, -1, self.joints['right']['foot'])

        joint_states = p.getJointStates(self.robot, self.legs)
        self.states['joint_positions'] = [state[0] for state in joint_states]  
        self.states['joint_velocities'] = [state[1] for state in joint_states]  
        self.states['joint_torques'] = [state[3] for state in joint_states]

        self.states['total_force'] = np.sum(np.abs(self.states['joint_torques']))
        self.states['total_velocity'] = np.sum(np.abs(self.states['joint_velocities']))

        self.states['zmp']  = self.calculate_zmp()
        self.states['contact_area'] = self.calculate_contact_area()
    
    def check_done(self):
        contact = None

        if self.dimension == '3d':
            for joint_id in range(0, 5):
                if p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=joint_id):
                    contact = p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=joint_id)
            if contact == None:
                for joint_id in range(25, 30):
                    if p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=joint_id):
                        contact = p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=joint_id)
            if contact == None:
                if p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=self.joints['left']['hip_yaw']):
                    contact = p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=self.joints['left']['hip_yaw'])
            if contact == None:
                if p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=-1):
                    contact = p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=-1)
        elif self.dimension == '2d':
            if contact == None:
                body_id = 4
                pos_z = p.getLinkState(self.robot, body_id)[4][2]
                if pos_z < 0.145:
                    contact = True
        if contact:
            self.done = True
    
    def get_joints_info(self):
        jointNameToId = {} 
        for i in range(p.getNumJoints(self.robot)):
            jointInfo = p.getJointInfo(self.robot, i)
            jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
        return jointNameToId
    
    def toggle_camera(self):
        self.camera_on = not self.camera_on
        print(f"[INFO] Camera {'ON' if self.camera_on else 'OFF'}")
            
    def test_simulation(self):
        while True:
            self.step()
            self.initial_pose()
    
    def calculate_zmp(self):
        total_force = np.zeros(3)
        total_moment = np.zeros(3)
        total_mass = 0

        for link_id in range(p.getNumJoints(self.robot)):
            link_state = p.getLinkState(self.robot, link_id)
            link_world_position = link_state[4]
            link_mass = p.getDynamicsInfo(self.robot, link_id)[0]
            link_forces = np.array(p.getJointState(self.robot, link_id)[2])
            link_force = link_forces[:3]

            # Calculation of total forces and moments at the center of gravity
            total_force += link_mass * link_force
            total_moment += np.cross(link_world_position, link_mass * link_force)
            total_mass += link_mass

        com = total_force / total_mass
        zmp_x = com[0] - (total_moment[1] / total_force[2])
        zmp_y = com[1] + (total_moment[0] / total_force[2])
        
        zmp = [zmp_x, zmp_y, 0]

        return zmp
  
    def calculate_contact_area(self):
        contact_areas = []
        foot_link_indices = [self.joints['left']['foot'], self.joints['right']['foot']]

        for foot_link_index in foot_link_indices:
            contact_points = p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=foot_link_index)
            if not contact_points:
                contact_areas.append(0.0)
                continue

            points = np.array([point[6] for point in contact_points])
            if len(points) < 3:
                contact_areas.append(0.0)
                continue

            hull = ConvexHull(points[:, :2])
            area = hull.volume
            contact_areas.append(area)

        if len(foot_link_indices) == 1:
            return contact_areas[0]
        elif len(foot_link_indices) == 2:
            if contact_areas[0] == 0.0 or contact_areas[1] == 0.0:
                return max(contact_areas)  # One of the foot links is not in contact, return the area of the other
            else:
                # Calculate the support polygon area
                contact_points_1 = p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=foot_link_indices[0])
                contact_points_2 = p.getContactPoints(bodyA=self.robot, bodyB=self.plane, linkIndexA=foot_link_indices[1])
                
                points_1 = np.array([point[6] for point in contact_points_1])
                points_2 = np.array([point[6] for point in contact_points_2])
                combined_points = np.vstack((points_1, points_2))
                
                if len(combined_points) < 3:
                    return 0.0
                
                hull = ConvexHull(combined_points[:, :2])
                support_polygon_area = hull.volume

                return support_polygon_area
        else:
            raise ValueError("foot_link_indices must contain 1 or 2 elements")


    def disconnect(self):
        p.disconnect()