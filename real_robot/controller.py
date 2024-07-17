from real_robot import xl430

class HardwareController():
    def __init__(self):
        self.setup()
    
    def setup(self):
        self.DXL_ID = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.BAUDRATE = 57600 
        self.TORQUE_ENABLE = 1

        self.servo_l_hip_roll = xl430.XL430(self.DXL_ID[1], self.BAUDRATE, self.TORQUE_ENABLE)
        self.servo_l_hip_yaw = xl430.XL430(self.DXL_ID[2], self.BAUDRATE, self.TORQUE_ENABLE)
        self.servo_l_hip_pitch = xl430.XL430(self.DXL_ID[3], self.BAUDRATE, self.TORQUE_ENABLE)
        self.servo_l_knee = xl430.XL430(self.DXL_ID[4], self.BAUDRATE, self.TORQUE_ENABLE)
        self.servo_l_ankle = xl430.XL430(self.DXL_ID[5], self.BAUDRATE, self.TORQUE_ENABLE)

        self.servo_r_hip_roll = xl430.XL430(self.DXL_ID[6], self.BAUDRATE, self.TORQUE_ENABLE)
        self.servo_r_hip_yaw = xl430.XL430(self.DXL_ID[7], self.BAUDRATE, self.TORQUE_ENABLE)
        self.servo_r_hip_pitch = xl430.XL430(self.DXL_ID[8], self.BAUDRATE, self.TORQUE_ENABLE)
        self.servo_r_knee = xl430.XL430(self.DXL_ID[9], self.BAUDRATE, self.TORQUE_ENABLE)
        self.servo_r_ankle = xl430.XL430(self.DXL_ID[10], self.BAUDRATE, self.TORQUE_ENABLE)

        self.joints = {
            'left': {
                'hip_roll': self.servo_l_hip_roll,
                'hip_yaw': self.servo_l_hip_yaw,
                'hip_pitch': self.servo_l_hip_pitch,
                'knee': self.servo_l_knee,
                'ankle': self.servo_l_ankle
            },
            'right': {
                'hip_roll': self.servo_r_hip_roll,
                'hip_yaw': self.servo_r_hip_yaw,
                'hip_pitch': self.servo_r_hip_pitch,
                'knee': self.servo_r_knee,
                'ankle': self.servo_r_ankle
            }
        }

    def get_angle(self, leg, joint):
        joint = self.joints[leg][joint]
        angle = joint.read_present_angle()
        return angle
    
    def write_angle(self, leg, joint, angle):
        joint = self.joints[leg][joint]
        joint.write_angle(angle)

    def close(self):
        for side in self.joints:
            for joint_name, servo in self.joints[side].items():
                joint = self.joints[side][joint_name]
                joint.close_port()
        print('All servo ports closed successfully.')