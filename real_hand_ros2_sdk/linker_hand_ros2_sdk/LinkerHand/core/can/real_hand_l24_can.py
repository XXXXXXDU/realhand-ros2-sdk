#!/usr/bin/env python3
import can
import time,sys,os
import threading
import numpy as np
from enum import Enum
current_dir = os.path.dirname(os.path.abspath(__file__))
target_dir = os.path.abspath(os.path.join(current_dir, ".."))
sys.path.append(target_dir)
from utils.color_msg import ColorMsg

class FrameProperty(Enum):
    INVALID_FRAME_PROPERTY = 0x00  # Invalid CAN frame property | no return
    # Parallel command region
    ROLL_POS = 0x01  # Roll joint position | Coordinate frame at each finger base; angles defined with finger extended
    YAW_POS = 0x02  # Yaw joint position | Coordinate frame at each finger base; angles defined with finger extended
    ROOT1_POS = 0x03  # Root-1 joint position | Proximal (closest to palm) root joint
    ROOT2_POS = 0x04  # Root-2 joint position | Proximal (closest to palm) root joint
    ROOT3_POS = 0x05  # Root-3 joint position | Proximal (closest to palm) root joint
    TIP_POS = 0x06  # Fingertip joint position | Proximal (closest to palm) root joint

    ROLL_SPEED = 0x09  # Roll joint speed | Coordinate frame at each finger base; angles defined with finger extended
    YAW_SPEED = 0x0A  # Yaw joint speed | Coordinate frame at each finger base; angles defined with finger extended
    ROOT1_SPEED = 0x0B  # Root-1 joint speed | Proximal (closest to palm) root joint
    ROOT2_SPEED = 0x0C  # Root-2 joint speed | Proximal (closest to palm) root joint
    ROOT3_SPEED = 0x0D  # Root-3 joint speed | Proximal (closest to palm) root joint
    TIP_SPEED = 0x0E  # Fingertip joint speed | Proximal (closest to palm) root joint

    ROLL_TORQUE = 0x11  # Roll joint torque | Coordinate frame at each finger base; angles defined with finger extended
    YAW_TORQUE = 0x12  # Yaw joint torque | Coordinate frame at each finger base; angles defined with finger extended
    ROOT1_TORQUE = 0x13  # Root-1 joint torque | Proximal (closest to palm) root joint
    ROOT2_TORQUE = 0x14  # Root-2 joint torque | Proximal (closest to palm) root joint
    ROOT3_TORQUE = 0x15  # Root-3 joint torque | Proximal (closest to palm) root joint
    TIP_TORQUE = 0x16  # Fingertip joint torque | Proximal (closest to palm) root joint

    ROLL_FAULT = 0x19  # Roll joint fault code | Coordinate frame at each finger base; angles defined with finger extended
    YAW_FAULT = 0x1A  # Yaw joint fault code | Coordinate frame at each finger base; angles defined with finger extended
    ROOT1_FAULT = 0x1B  # Root-1 joint fault code | Proximal (closest to palm) root joint
    ROOT2_FAULT = 0x1C  # Root-2 joint fault code | Proximal (closest to palm) root joint
    ROOT3_FAULT = 0x1D  # Root-3 joint fault code | Proximal (closest to palm) root joint
    TIP_FAULT = 0x1E  # Fingertip joint fault code | Proximal (closest to palm) root joint

    ROLL_TEMPERATURE = 0x21  # Roll joint temperature | Coordinate frame at each finger base; angles defined with finger extended
    YAW_TEMPERATURE = 0x22  # Yaw joint temperature | Coordinate frame at each finger base; angles defined with finger extended
    ROOT1_TEMPERATURE = 0x23  # Root-1 joint temperature | Proximal (closest to palm) root joint
    ROOT2_TEMPERATURE = 0x24  # Root-2 joint temperature | Proximal (closest to palm) root joint
    ROOT3_TEMPERATURE = 0x25  # Root-3 joint temperature | Proximal (closest to palm) root joint
    TIP_TEMPERATURE = 0x26  # Fingertip joint temperature | Proximal (closest to palm) root joint
    # Parallel command region

    # Serial command region
    THUMB_POS = 0x41  # Thumb joint positions | returns this type of data
    INDEX_POS = 0x42  # Index finger joint positions | returns this type of data
    MIDDLE_POS = 0x43  # Middle finger joint positions | returns this type of data
    RING_POS = 0x44  # Ring finger joint positions | returns this type of data
    LITTLE_POS = 0x45  # Pinky finger joint positions | returns this type of data

    THUMB_SPEED = 0x49  # Thumb speed | returns this type of data
    INDEX_SPEED = 0x4A  # Index speed | returns this type of data
    MIDDLE_SPEED = 0x4B  # Middle speed | returns this type of data
    RING_SPEED = 0x4C  # Ring speed | returns this type of data
    LITTLE_SPEED = 0x4D  # Pinky speed | returns this type of data

    THUMB_TORQUE = 0x51  # Thumb torque | returns this type of data
    INDEX_TORQUE = 0x52  # Index torque | returns this type of data
    MIDDLE_TORQUE = 0x53  # Middle torque | returns this type of data
    RING_TORQUE = 0x54  # Ring torque | returns this type of data
    LITTLE_TORQUE = 0x55  # Pinky torque | returns this type of data

    THUMB_FAULT = 0x59  # Thumb fault code | returns this type of data
    INDEX_FAULT = 0x5A  # Index fault code | returns this type of data
    MIDDLE_FAULT = 0x5B  # Middle fault code | returns this type of data
    RING_FAULT = 0x5C  # Ring fault code | returns this type of data
    LITTLE_FAULT = 0x5D  # Pinky fault code | returns this type of data

    THUMB_TEMPERATURE = 0x61  # Thumb temperature | returns this type of data
    INDEX_TEMPERATURE = 0x62  # Index temperature | returns this type of data
    MIDDLE_TEMPERATURE = 0x63  # Middle temperature | returns this type of data
    RING_TEMPERATURE = 0x64  # Ring temperature | returns this type of data
    LITTLE_TEMPERATURE = 0x65  # Pinky temperature | returns this type of data
    # Serial command region

    # Merged command region (non-essential per-finger controls combined)
    FINGER_SPEED = 0x81  # Finger speed | returns this type of data
    FINGER_TORQUE = 0x82  # Torque | returns this type of data
    FINGER_FAULT = 0x83  # Finger fault code | returns this type of data

    # Fingertip sensor data group
    HAND_NORMAL_FORCE = 0x90  # Five-finger normal force
    HAND_TANGENTIAL_FORCE = 0x91  # Five-finger tangential force
    HAND_TANGENTIAL_FORCE_DIR = 0x92  # Five-finger tangential force direction
    HAND_APPROACH_INC = 0x93  # Five-finger proximity

    THUMB_ALL_DATA = 0x98  # All thumb data
    INDEX_ALL_DATA = 0x99  # All index data
    MIDDLE_ALL_DATA = 0x9A  # All middle data
    RING_ALL_DATA = 0x9B  # All ring data
    LITTLE_ALL_DATA = 0x9C  # All pinky data
    # Action commands · ACTION
    ACTION_PLAY = 0xA0  # Action

    # Configuration commands · CONFIG
    HAND_UID = 0xC0  # Device unique identifier
    HAND_HARDWARE_VERSION = 0xC1  # Hardware version
    HAND_SOFTWARE_VERSION = 0xC2  # Software version
    HAND_COMM_ID = 0xC3  # Device ID
    HAND_FACTORY_RESET = 0xCE  # Factory reset
    HAND_SAVE_PARAMETER = 0xCF  # Save parameters

    WHOLE_FRAME = 0xF0  # Whole-frame transfer | 1-byte frame property + full struct (for 485/network transport)

class RealHandL24Can:
    def __init__(self, config, can_channel='can0', baudrate=1000000, can_id=0x28):
        self.config = config
        self.can_id = can_id
        self.running = True
        self.x01, self.x02, self.x03, self.x04,self.x05,self.x06,self.x07, self.x08,self.x09,self.x0A,self.x0B,self.x0C,self.x0D,self.x0E,self.speed = [],[],[],[],[],[],[],[],[],[],[],[],[],[],[]
        # Speed
        self.x49, self.x4a, self.x4b, self.x4c, self.x4d = [],[],[],[],[]
        self.x41,self.x42,self.x43,self.x44,self.x45 = [],[],[],[],[]
        # Initialize CAN bus based on OS
        if sys.platform == "linux":
            self.bus = can.interface.Bus(
                channel=can_channel, interface="socketcan", bitrate=baudrate, 
                can_filters=[{"can_id": can_id, "can_mask": 0x7FF}]
            )
        elif sys.platform == "win32":
            self.bus = can.interface.Bus(
                channel=can_channel, interface='pcan', bitrate=baudrate, 
                can_filters=[{"can_id": can_id, "can_mask": 0x7FF}]
            )
        else:
            raise EnvironmentError("Unsupported platform for CAN interface")

        # Initialize parameters based on can_id
        if can_id == 0x28:  # Left hand
            self.hand_exists = config['LINKER_HAND']['LEFT_HAND']['EXISTS']
            self.hand_joint = config['LINKER_HAND']['LEFT_HAND']['JOINT']
            self.hand_names = config['LINKER_HAND']['LEFT_HAND']['NAME']
        elif can_id == 0x27:  # Right hand
            self.hand_exists = config['LINKER_HAND']['RIGHT_HAND']['EXISTS']
            self.hand_joint = config['LINKER_HAND']['RIGHT_HAND']['JOINT']
            self.hand_names = config['LINKER_HAND']['RIGHT_HAND']['NAME']

        # Start receive thread
        self.receive_thread = threading.Thread(target=self.receive_response)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    def send_command(self, frame_property, data_list):
        """
        Send a command to the CAN bus
        :param frame_property: Data frame property
        :param data_list: Data payload
        """
        frame_property_value = int(frame_property.value) if hasattr(frame_property, 'value') else frame_property
        data = [frame_property_value] + [int(val) for val in data_list]
        msg = can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            # print(f"Message sent: ID={hex(self.can_id)}, Data={data}")
        except can.CanError as e:
            print(f"Failed to send message: {e}")
        time.sleep(0.002)

    def receive_response(self):
        """
        Receive and process responses from the CAN bus
        """
        while self.running:
            try:
                msg = self.bus.recv(timeout=1.0)  # Blocking receive, 1s timeout
                if msg:
                    self.process_response(msg)
            except can.CanError as e:
                print(f"Error receiving message: {e}")
    

    def set_joint_positions(self, joint_ranges):
        if len(joint_ranges) == 25:
            l24_pose = self.joint_map(joint_ranges)
            # Slice into chunks of 6 elements
            chunks = [l24_pose[i:i+6] for i in range(0, 30, 6)]
            self.send_command(FrameProperty.THUMB_POS, chunks[0])
            self.send_command(FrameProperty.INDEX_POS, chunks[1])
            self.send_command(FrameProperty.MIDDLE_POS, chunks[2])
            self.send_command(FrameProperty.RING_POS, chunks[3])
            self.send_command(FrameProperty.LITTLE_POS, chunks[4])
        # self.set_tip_positions(joint_ranges[:5])
        # print(l24_pose)
    
    # Set roll for all fingers
    def set_roll_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROLL_POS, joint_ranges)
    # Set yaw for all fingers
    def set_yaw_positions(self, joint_ranges):
        self.send_command(FrameProperty.YAW_POS, joint_ranges)
    # Set root-1 for all fingers
    def set_root1_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT1_POS, joint_ranges)
    # Set root-2 for all fingers
    def set_root2_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT2_POS, joint_ranges)
    # Set root-3 for all fingers
    def set_root3_positions(self, joint_ranges):
        self.send_command(FrameProperty.ROOT3_POS, joint_ranges)
    # Set fingertip for all fingers
    def set_tip_positions(self, joint_ranges=[80]*5):
        self.send_command(FrameProperty.TIP_POS, joint_ranges)
    # Get thumb joint positions
    def get_thumb_positions(self,j=[0]):
        self.send_command(FrameProperty.THUMB_POS, j)
    # Get index joint positions
    def get_index_positions(self, j=[0]):
        self.send_command(FrameProperty.INDEX_POS,j)
    # Get middle joint positions
    def get_middle_positions(self, j=[0]):
        self.send_command(FrameProperty.MIDDLE_POS,j)
    # Get ring joint positions
    def get_ring_positions(self, j=[0]):
        self.send_command(FrameProperty.RING_POS,j)
    # Get pinky joint positions
    def get_little_positions(self, j=[0]):
        self.send_command(FrameProperty.LITTLE_POS, j)
    # Disable mode 01
    def set_disability_mode(self, j=[1,1,1,1,1]):
        self.send_command(0x85,j)
    # Enable mode 00
    def set_enable_mode(self, j=[00,00,00,00,00]):
        self.send_command(0x85,j)

    
    def set_speed(self, speed):
        self.speed = [speed]*6
        ColorMsg(msg=f"L24 set speed to: {self.speed}", color="yellow")
        self.send_command(FrameProperty.THUMB_SPEED, self.speed)
        self.send_command(FrameProperty.INDEX_SPEED, self.speed)
        self.send_command(FrameProperty.MIDDLE_SPEED, self.speed)
        self.send_command(FrameProperty.RING_SPEED, self.speed)
        self.send_command(FrameProperty.LITTLE_SPEED, self.speed)
        
    def set_finger_torque(self, torque):
        self.send_command(0x42, torque)

    def request_device_info(self):
        self.send_command(0xC0, [0])
        self.send_command(0xC1, [0])
        self.send_command(0xC2, [0])

    def save_parameters(self):
        self.send_command(0xCF, [])
    def process_response(self, msg):
        if msg.arbitration_id == self.can_id:
            frame_type = msg.data[0]
            response_data = msg.data[1:]
            if frame_type == 0x01:
                self.x01 = list(response_data)
            elif frame_type == 0x02:
                self.x02 = list(response_data)
            elif frame_type == 0x03:
                self.x03 = list(response_data)
            elif frame_type == 0x04:
                self.x04 = list(response_data)
            elif frame_type == 0x05:
                self.x05 = list(response_data)
            elif frame_type == 0x06:
                self.x06 = list(response_data)
                print("_-"*20)
                print(self.x06)
            elif frame_type == 0xC0:
                print(f"Device ID info: {response_data}")
                if self.can_id == 0x28:
                    self.right_hand_info = response_data
                elif self.can_id == 0x27:
                    self.left_hand_info = response_data
            elif frame_type == 0x08:
                self.x08 = list(response_data)
            elif frame_type == 0x09:
                self.x09 = list(response_data)
            elif frame_type == 0x0A:
                self.x0A = list(response_data)
            elif frame_type == 0x0B:
                self.x0B = list(response_data)
            elif frame_type == 0x0C:
                self.x0C = list(response_data)
            elif frame_type == 0x0D:
                self.x0D = list(response_data)
            elif frame_type == 0x22:
                # ColorMsg(msg=f"Tangential force direction (5 fingers): {list(response_data)}")
                d = list(response_data)
                self.tangential_force_dir = [float(i) for i in d]
            elif frame_type == 0x23:
                # ColorMsg(msg=f"Proximity (5 fingers): {list(response_data)}")
                d = list(response_data)
                self.approach_inc = [float(i) for i in d]
            elif frame_type == 0x41: # Thumb joint position return
                self.x41 = list(response_data)
            elif frame_type == 0x42: # Index joint position return
                self.x42 = list(response_data)
            elif frame_type == 0x43: # Middle joint position return
                self.x43 = list(response_data)
            elif frame_type == 0x44: # Ring joint position return
                self.x44 = list(response_data)
            elif frame_type == 0x45: # Pinky joint position return
                self.x45 = list(response_data)
            elif frame_type == 0x49: # Thumb speed return
                self.x49 = list(response_data)
            elif frame_type == 0x4a: # Index speed return
                self.x4a = list(response_data)
            elif frame_type == 0x4b: # Middle speed return
                self.x4b = list(response_data)
            elif frame_type == 0x4c: # Ring speed return
                self.x4c = list(response_data)
            elif frame_type == 0x4d: # Pinky speed return
                self.x4d = list(response_data)

    # Topic mapping for L24
    def joint_map(self, pose):
        # L24 CAN expects 30 values by default
        l24_pose = [0.0] * 30  # Initialize l24_pose with 30 zeros

        # Mapping table to simplify index relationships
        mapping = {
            0: 10,  1: 5,   2: 0,   3: 15,  4: None,  5: 20,
            6: None, 7: 6,   8: 1,   9: 16,  10: None, 11: 21,
            12: None, 13: None, 14: 2,  15: 17, 16: None, 17: 22,
            18: None, 19: 8,  20: 3,   21: 18, 22: None, 23: 23,
            24: None, 25: 9,  26: 4,   27: 19, 28: None, 29: 24
        }

        # Apply mapping
        for l24_idx, pose_idx in mapping.items():
            if pose_idx is not None:
                l24_pose[l24_idx] = pose[pose_idx]

        return l24_pose

    # Convert L24 state values to CMD-format state values
    def state_to_cmd(self, l24_state):
        # L24 CAN returns 30 values; initialize CMD-format pose (25 values)
        pose = [0.0] * 25  # Original control command for L24 uses 25 values

        # Mapping from l24_state indices to CMD pose indices
        mapping = {
            0: 10,  1: 5,   2: 0,   3: 15,  5: 20,  7: 6,
            8: 1,   9: 16,  11: 21, 14: 2,  15: 17, 17: 22,
            19: 8,  20: 3,  21: 18, 23: 23, 25: 9,   26: 4,
            27: 19, 29: 24
        }
        for l24_idx, pose_idx in mapping.items():
            pose[pose_idx] = l24_state[l24_idx]
        return pose

    # Get all joint data
    def get_current_status(self, j=''):
        time.sleep(0.01)
        self.send_command(FrameProperty.THUMB_POS, j)
        self.send_command(FrameProperty.INDEX_POS,j)
        self.send_command(FrameProperty.MIDDLE_POS,j)
        self.send_command(FrameProperty.RING_POS,j)
        self.send_command(FrameProperty.LITTLE_POS, j)
        # return self.x41, self.x42, self.x43, self.x44, self.x45
        time.sleep(0.1)
        state= self.x41+ self.x42+ self.x43+ self.x44+ self.x45
        if len(state) == 30:
            l24_state = self.state_to_cmd(l24_state=state)
            return l24_state
    
    def get_speed(self,j=''):
        time.sleep(0.1)
        self.send_command(FrameProperty.THUMB_SPEED, j) # Thumb speed
        self.send_command(FrameProperty.INDEX_SPEED, j) # Index speed
        self.send_command(FrameProperty.MIDDLE_SPEED, j) # Middle speed
        self.send_command(FrameProperty.RING_SPEED, j) # Ring speed
        self.send_command(FrameProperty.LITTLE_SPEED, j) # Pinky speed
        speed = self.x49+ self.x4a+ self.x4b+ self.x4c+ self.x4d
        if len(speed) == 30:
            l24_speed = self.state_to_cmd(l24_state=speed)
            return l24_speed
    
    def get_finger_torque(self):
        return self.finger_torque
    # def get_current(self):
    #     return self.x06
    # def get_fault(self):
    #     return self.x07
    def close_can_interface(self):
        if self.bus:
            self.bus.shutdown()  # Close CAN bus

    '''
    This method is only to demonstrate the mapping relationships.
    For actual usage, prefer the methods above.
    '''
    def joint_map_2(self, pose):
        l24_pose = [0.0]*30  # L24 CAN expects 30 values; CMD control uses 25 values, mapped here
        '''
        Mapping needed
        # L24 CAN data format
        # ["Thumb yaw 0-10", "Thumb abduction 1-5", "Thumb base 2-0", "Thumb middle 3-15", "Reserved 4-", "Thumb tip 5-20",
        #  "Reserved 6-", "Index abduction 7-6", "Index base 8-1", "Index middle 9-16", "Reserved 10-", "Index tip 11-21",
        #  "Reserved 12-", "Reserved 13-", "Middle base 14-2", "Middle middle 15-17", "Reserved 16-", "Middle tip 17-22",
        #  "Reserved 18-", "Ring abduction 19-8", "Ring base 20-3", "Ring middle 21-18", "Reserved 22-", "Ring tip 23-23",
        #  "Reserved 24-", "Pinky abduction 25-9", "Pinky base 26-4", "Pinky middle 27-19", "Reserved 28-", "Pinky tip 29-24"]
        # CMD data format
        # ["Thumb base 0", "Index base 1", "Middle base 2", "Ring base 3","Pinky base 4",
        #  "Thumb abduction 5","Index abduction 6","Middle abduction 7","Ring abduction 8","Pinky abduction 9",
        #  "Thumb yaw 10","Reserved","Reserved","Reserved","Reserved",
        #  "Thumb middle 15","Index middle 16","Middle middle 17","Ring middle 18","Pinky middle 19",
        #  "Thumb tip 20","Index tip 21","Middle tip 22","Ring tip 23","Pinky tip 24"]
        '''
        l24_pose[0] = pose[10]
        l24_pose[1] = pose[5]
        l24_pose[2] = pose[0]
        l24_pose[3] = pose[15]
        l24_pose[4] = 0.0
        l24_pose[5] = pose[20]
        l24_pose[6] = 0.0
        l24_pose[7] = pose[6]
        l24_pose[8] = pose[1]
        l24_pose[9] = pose[16]
        l24_pose[10] = 0.0
        l24_pose[11] = pose[21]
        l24_pose[12] = 0.0
        l24_pose[13] = 0.0
        l24_pose[14] = pose[2]
        l24_pose[15] = pose[17]
        l24_pose[16] = 0.0
        l24_pose[17] = pose[22]
        l24_pose[18] = 0.0
        l24_pose[19] = pose[8]
        l24_pose[20] = pose[3]
        l24_pose[21] = pose[18]
        l24_pose[22] = 0.0
        l24_pose[23] = pose[23]
        l24_pose[24] = 0.0
        l24_pose[25] = pose[9]
        l24_pose[26] = pose[4]
        l24_pose[27] = pose[19]
        l24_pose[28] = 0.0
        l24_pose[29] = pose[24]
        return l24_pose
    

    def show_fun_table(self):
        pass
