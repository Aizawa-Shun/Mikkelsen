from real_robot.dynamixel_sdk import * 

class XL430:
    def __init__(
        self,
        # set variable
        id,
        baudrate,
        TORQUE_ENABLE,
        # Control table address
        ADDR_TORQUE_ENABLE          = 64,
        ADDR_GOAL_POSITION          = 116,
        ADDR_PRESENT_POSITION       = 132,
        DXL_MINIMUM_POSITION_VALUE  = 0,         # Refer to the Minimum Position Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 4095,      # Refer to the Maximum Position Limit of product eManual
        # DYNAMIXEL Protocol Version (1.0 / 2.0)
        PROTOCOL_VERSION = 2.0,
        # Use the actual port assigned to the U2D2.
        DEVICENAME                  = '/dev/ttyUSB0',  # Ubuntu
        # DEVICENAME                  = '\\COM3',  # Windows
        # Others
        DXL_MOVING_STATUS_THRESHOLD = 20,    # Dynamixel moving status threshold
        ):
        self.DXL_ID = id
        self.ADDR_TORQUE_ENABLE = ADDR_TORQUE_ENABLE
        self.ADDR_GOAL_POSITION = ADDR_GOAL_POSITION
        self.ADDR_PRESENT_POSITION = ADDR_PRESENT_POSITION
        self.DXL_MINIMUM_POSITION_VALUE = DXL_MINIMUM_POSITION_VALUE
        self.DXL_MAXIMUM_POSITION_VALUE = DXL_MAXIMUM_POSITION_VALUE
        self.BAUDRATE = baudrate
        self.PROTOCOL_VERSION = PROTOCOL_VERSION
        self.DEVICENAME = DEVICENAME
        self.DXL_MOVING_STATUS_THRESHOLD = DXL_MOVING_STATUS_THRESHOLD
        # Initialize PortHandler instance
        self.portHandler = PortHandler(self.DEVICENAME)
        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.open_port()
        self.set_port_baudrate()
        self.enable_torque(TORQUE_ENABLE)

    def open_port(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")

    def set_port_baudrate(self):
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
    
    def enable_torque(self, TORQUE_ENABLE):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")

    def read_present_angle(self):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        present_angle = dxl_present_position/11.375
        return present_angle
    
    def write_angle(self, write_angle):
        write_position = int(write_angle * 11.375)
        self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, write_position)
    
    def close_port(self):
        self.portHandler.closePort()