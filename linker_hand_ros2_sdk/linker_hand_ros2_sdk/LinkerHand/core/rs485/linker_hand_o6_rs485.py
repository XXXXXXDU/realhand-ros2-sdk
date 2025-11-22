#!/usr/bin/env python3
"""
O6 Hand Modbus-RTU control class
Tested on Ubuntu 20.04 + Python3
author : hejianxin
"""

import minimalmodbus
import serial
import time
import logging
import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)-8s %(message)s",
    datefmt="%H:%M:%S"
)

# ------------------------------------------------------------------
# Input register addresses (Function code 04, read-only)
# ------------------------------------------------------------------
REG_RD_CURRENT_THUMB_PITCH      = 0   # Thumb flexion angle (0–255, small = flexed, large = extended)
REG_RD_CURRENT_THUMB_YAW        = 1   # Thumb abduction angle (0–255, small = toward palm, large = away)
REG_RD_CURRENT_INDEX_PITCH      = 2   # Index flexion angle
REG_RD_CURRENT_MIDDLE_PITCH     = 3   # Middle flexion angle
REG_RD_CURRENT_RING_PITCH       = 4   # Ring flexion angle
REG_RD_CURRENT_LITTLE_PITCH     = 5   # Little flexion angle
REG_RD_CURRENT_THUMB_TORQUE     = 6   # Thumb flexion torque (0–255)
REG_RD_CURRENT_THUMB_YAW_TORQUE = 7   # Thumb abduction torque
REG_RD_CURRENT_INDEX_TORQUE     = 8   # Index torque
REG_RD_CURRENT_MIDDLE_TORQUE    = 9   # Middle torque
REG_RD_CURRENT_RING_TORQUE      = 10  # Ring torque
REG_RD_CURRENT_LITTLE_TORQUE    = 11  # Little torque
REG_RD_CURRENT_THUMB_SPEED      = 12  # Thumb flexion speed (0–255)
REG_RD_CURRENT_THUMB_YAW_SPEED  = 13  # Thumb abduction speed
REG_RD_CURRENT_INDEX_SPEED      = 14  # Index speed
REG_RD_CURRENT_MIDDLE_SPEED     = 15  # Middle speed
REG_RD_CURRENT_RING_SPEED       = 16  # Ring speed
REG_RD_CURRENT_LITTLE_SPEED     = 17  # Little speed
REG_RD_THUMB_TEMP               = 18  # Thumb flexion temperature (0–70℃)
REG_RD_THUMB_YAW_TEMP           = 19  # Thumb abduction temperature
REG_RD_INDEX_TEMP               = 20  # Index temperature
REG_RD_MIDDLE_TEMP              = 21  # Middle temperature
REG_RD_RING_TEMP                = 22  # Ring temperature
REG_RD_LITTLE_TEMP              = 23  # Little temperature
REG_RD_THUMB_ERROR              = 24  # Thumb error code
REG_RD_THUMB_YAW_ERROR          = 25  # Thumb abduction error code
REG_RD_INDEX_ERROR              = 26  # Index error code
REG_RD_MIDDLE_ERROR             = 27  # Middle error code
REG_RD_RING_ERROR               = 28  # Ring error code
REG_RD_LITTLE_ERROR             = 29  # Little error code
REG_RD_HAND_FREEDOM             = 30  # Version (matches hand label)
REG_RD_HAND_VERSION             = 31  # Hand version
REG_RD_HAND_NUMBER              = 32  # Hand serial number
REG_RD_HAND_DIRECTION           = 33  # Hand orientation (left/right)
REG_RD_SOFTWARE_VERSION         = 34  # Software version
REG_RD_HARDWARE_VERSION         = 35  # Hardware version

# ------------------------------------------------------------------
# Holding register addresses (Function code 16, read/write)
# ------------------------------------------------------------------
REG_WR_THUMB_PITCH       = 0   # Thumb flexion angle (0–255)
REG_WR_THUMB_YAW         = 1   # Thumb abduction angle
REG_WR_INDEX_PITCH       = 2   # Index flexion angle
REG_WR_MIDDLE_PITCH      = 3   # Middle flexion angle
REG_WR_RING_PITCH        = 4   # Ring flexion angle
REG_WR_LITTLE_PITCH      = 5   # Little flexion angle
REG_WR_THUMB_TORQUE      = 6   # Thumb flexion torque
REG_WR_THUMB_YAW_TORQUE  = 7   # Thumb abduction torque
REG_WR_INDEX_TORQUE      = 8   # Index torque
REG_WR_MIDDLE_TORQUE     = 9   # Middle torque
REG_WR_RING_TORQUE       = 10  # Ring torque
REG_WR_LITTLE_TORQUE     = 11  # Little torque
REG_WR_THUMB_SPEED       = 12  # Thumb flexion speed
REG_WR_THUMB_YAW_SPEED   = 13  # Thumb abduction speed
REG_WR_INDEX_SPEED       = 14  # Index speed
REG_WR_MIDDLE_SPEED      = 15  # Middle speed
REG_WR_RING_SPEED        = 16  # Ring speed
REG_WR_LITTLE_SPEED      = 17  # Little speed


class LinkerHandO6RS485:
    """O6 Hand Modbus-RTU control class (supports left and right hands)"""

    # RIGHT_ID = 0x27          # Right hand ID
    # LEFT_ID  = 0x28          # Left hand ID
    # BAUD     = 115200        # Fixed baud rate
    TTL_TIMEOUT = 0.15         # Serial timeout
    FRAME_GAP = 0.030          # 30 ms inter-frame gap
    _last_ts  = 0              # Timestamp of last frame completion

    def __init__(self, hand_id=0x27,modbus_port="/dev/ttyUSB0",baudrate=115200):
        """
        modbus_port : Serial device, e.g. /dev/ttyUSB0
        hand_id     : 0x27 for right hand, 0x28 for left hand
        """
        self._id = hand_id

        self.joint_name = ["Thumb flexion", "Thumb abduction", "Index flexion",
                           "Middle flexion", "Ring flexion", "Little flexion"]
        try:
            self.instr = minimalmodbus.Instrument(modbus_port, self._id, mode='rtu')
            self.instr.serial.baudrate = baudrate
            self.instr.serial.bytesize = 8
            self.instr.serial.parity   = serial.PARITY_NONE
            self.instr.serial.stopbits = 1
            self.instr.serial.timeout  = self.TTL_TIMEOUT
            self.instr.close_port_after_each_call = True
            self.instr.clear_buffers_before_each_transaction = True
        except Exception as e:
            logging.error(f"Initialization failed: {e}")
            raise

    # ----------------------------------------------------------
    # Low-level read/write wrappers
    # ----------------------------------------------------------
    def _bus_free(self):
        """Ensure ≥ 30 ms since the previous frame"""
        elapse = time.perf_counter() - self._last_ts
        if elapse < self.FRAME_GAP:
            time.sleep(self.FRAME_GAP - elapse)

    def _read_reg(self, addr: int) -> int:
        """Read a single input register (FC 04) with a 30 ms frame gap"""
        self._bus_free()
        try:
            return self.instr.read_register(addr, functioncode=4)
        finally:
            self._last_ts = time.perf_counter()   # mark frame end

    def _write_reg(self, addr: int, value: int):
        """Write a single holding register (FC 16) with a 30 ms frame gap"""
        if not 0 <= value <= 255:
            raise ValueError("value must be 0–255")
        self._bus_free()
        try:
            self.instr.write_register(addr, value, functioncode=16)
        finally:
            self._last_ts = time.perf_counter()

    # ----------------------------------------------------------
    # Read-only properties (read live)
    # ----------------------------------------------------------
    def get_thumb_pitch(self) -> int:          return self._read_reg(REG_RD_CURRENT_THUMB_PITCH)      # Thumb flexion angle
    def get_thumb_yaw(self) -> int:            return self._read_reg(REG_RD_CURRENT_THUMB_YAW)        # Thumb abduction angle
    def get_index_pitch(self) -> int:          return self._read_reg(REG_RD_CURRENT_INDEX_PITCH)      # Index flexion angle
    def get_middle_pitch(self) -> int:         return self._read_reg(REG_RD_CURRENT_MIDDLE_PITCH)     # Middle flexion angle
    def get_ring_pitch(self) -> int:           return self._read_reg(REG_RD_CURRENT_RING_PITCH)       # Ring flexion angle
    def get_little_pitch(self) -> int:         return self._read_reg(REG_RD_CURRENT_LITTLE_PITCH)     # Little flexion angle

    def get_thumb_torque(self) -> int:         return self._read_reg(REG_RD_CURRENT_THUMB_TORQUE)     # Thumb flexion torque
    def get_thumb_yaw_torque(self) -> int:     return self._read_reg(REG_RD_CURRENT_THUMB_YAW_TORQUE) # Thumb abduction torque
    def get_index_torque(self) -> int:         return self._read_reg(REG_RD_CURRENT_INDEX_TORQUE)     # Index torque
    def get_middle_torque(self) -> int:        return self._read_reg(REG_RD_CURRENT_MIDDLE_TORQUE)    # Middle torque
    def get_ring_torque(self) -> int:          return self._read_reg(REG_RD_CURRENT_RING_TORQUE)      # Ring torque
    def get_little_torque(self) -> int:        return self._read_reg(REG_RD_CURRENT_LITTLE_TORQUE)    # Little torque

    def get_thumb_speed(self) -> int:          return self._read_reg(REG_RD_CURRENT_THUMB_SPEED)      # Thumb flexion speed
    def get_thumb_yaw_speed(self) -> int:      return self._read_reg(REG_RD_CURRENT_THUMB_YAW_SPEED)  # Thumb abduction speed
    def get_index_speed(self) -> int:          return self._read_reg(REG_RD_CURRENT_INDEX_SPEED)      # Index speed
    def get_middle_speed(self) -> int:         return self._read_reg(REG_RD_CURRENT_MIDDLE_SPEED)     # Middle speed
    def get_ring_speed(self) -> int:           return self._read_reg(REG_RD_CURRENT_RING_SPEED)       # Ring speed
    def get_little_speed(self) -> int:         return self._read_reg(REG_RD_CURRENT_LITTLE_SPEED)     # Little speed

    def get_thumb_temp(self) -> int:           return self._read_reg(REG_RD_THUMB_TEMP)               # Thumb temperature (℃)
    def get_thumb_yaw_temp(self) -> int:       return self._read_reg(REG_RD_THUMB_YAW_TEMP)           # Thumb abduction temperature
    def get_index_temp(self) -> int:           return self._read_reg(REG_RD_INDEX_TEMP)               # Index temperature
    def get_middle_temp(self) -> int:          return self._read_reg(REG_RD_MIDDLE_TEMP)              # Middle temperature
    def get_ring_temp(self) -> int:            return self._read_reg(REG_RD_RING_TEMP)                # Ring temperature
    def get_little_temp(self) -> int:          return self._read_reg(REG_RD_LITTLE_TEMP)              # Little temperature

    def get_thumb_error(self) -> int:          return self._read_reg(REG_RD_THUMB_ERROR)              # Thumb error code
    def get_thumb_yaw_error(self) -> int:      return self._read_reg(REG_RD_THUMB_YAW_ERROR)          # Thumb abduction error code
    def get_index_error(self) -> int:          return self._read_reg(REG_RD_INDEX_ERROR)              # Index error code
    def get_middle_error(self) -> int:         return self._read_reg(REG_RD_MIDDLE_ERROR)             # Middle error code
    def get_ring_error(self) -> int:           return self._read_reg(REG_RD_RING_ERROR)               # Ring error code
    def get_little_error(self) -> int:         return self._read_reg(REG_RD_LITTLE_ERROR)             # Little error code

    def get_hand_freedom(self) -> int:         return self._read_reg(REG_RD_HAND_FREEDOM)             # Version (label)
    def get_hand_version(self) -> int:         return self._read_reg(REG_RD_HAND_VERSION)             # Hand version
    def get_hand_number(self) -> int:          return self._read_reg(REG_RD_HAND_NUMBER)              # Hand number
    def get_hand_direction(self) -> int:       return self._read_reg(REG_RD_HAND_DIRECTION)           # Hand orientation
    def get_software_version(self) -> int:     return self._read_reg(REG_RD_SOFTWARE_VERSION)         # Software version
    def get_hardware_version(self) -> int:     return self._read_reg(REG_RD_HARDWARE_VERSION)         # Hardware version

    # ----------------------------------------------------------
    # Holding registers (write)
    # ----------------------------------------------------------
    def set_thumb_pitch(self, v: int):          self._write_reg(REG_WR_THUMB_PITCH, v)       # Set thumb flexion angle
    def set_thumb_yaw(self, v: int):            self._write_reg(REG_WR_THUMB_YAW, v)         # Set thumb abduction angle
    def set_index_pitch(self, v: int):          self._write_reg(REG_WR_INDEX_PITCH, v)       # Set index flexion angle
    def set_middle_pitch(self, v: int):         self._write_reg(REG_WR_MIDDLE_PITCH, v)      # Set middle flexion angle
    def set_ring_pitch(self, v: int):           self._write_reg(REG_WR_RING_PITCH, v)        # Set ring flexion angle
    def set_little_pitch(self, v: int):         self._write_reg(REG_WR_LITTLE_PITCH, v)      # Set little flexion angle

    def set_thumb_torque(self, v: int):         self._write_reg(REG_WR_THUMB_TORQUE, v)      # Set thumb flexion torque
    def set_thumb_yaw_torque(self, v: int):     self._write_reg(REG_WR_THUMB_YAW_TORQUE, v)  # Set thumb abduction torque
    def set_index_torque(self, v: int):         self._write_reg(REG_WR_INDEX_TORQUE, v)      # Set index torque
    def set_middle_torque(self, v: int):        self._write_reg(REG_WR_MIDDLE_TORQUE, v)     # Set middle torque
    def set_ring_torque(self, v: int):          self._write_reg(REG_WR_RING_TORQUE, v)       # Set ring torque
    def set_little_torque(self, v: int):        self._write_reg(REG_WR_LITTLE_TORQUE, v)     # Set little torque

    def set_thumb_speed(self, v: int):          self._write_reg(REG_WR_THUMB_SPEED, v)       # Set thumb flexion speed
    def set_thumb_yaw_speed(self, v: int):      self._write_reg(REG_WR_THUMB_YAW_SPEED, v)   # Set thumb abduction speed
    def set_index_speed(self, v: int):          self._write_reg(REG_WR_INDEX_SPEED, v)       # Set index speed
    def set_middle_speed(self, v: int):         self._write_reg(REG_WR_MIDDLE_SPEED, v)      # Set middle speed
    def set_ring_speed(self, v: int):           self._write_reg(REG_WR_RING_SPEED, v)        # Set ring speed
    def set_little_speed(self, v: int):         self._write_reg(REG_WR_LITTLE_SPEED, v)      # Set little speed

    # ----------------------------------------------------------
    # Fixed functions
    # ----------------------------------------------------------
    def is_valid_6xuint8(self, lst) -> bool:
        lst = [int(x) for x in lst]
        return (
            isinstance(lst, list) and
            len(lst) == 6 and
            all(type(x) is int and 0 <= x <= 255 for x in lst)
        )
    
    def set_joint_positions(self, joint_angles=[0] * 6):
        if self.is_valid_6xuint8(joint_angles):
            self.set_thumb_pitch(joint_angles[0])
            self.set_thumb_yaw(joint_angles[1])
            self.set_index_pitch(joint_angles[2])
            self.set_middle_pitch(joint_angles[3])
            self.set_ring_pitch(joint_angles[4])
            self.set_little_pitch(joint_angles[5])

    def set_speed(self, speed=[200] * 6):
        """Set speeds — params: list len=6"""
        if self.is_valid_6xuint8(speed):
            self.set_thumb_speed(speed[0])
            self.set_thumb_yaw_speed(speed[1])
            self.set_index_speed(speed[2])
            self.set_middle_speed(speed[3])
            self.set_ring_speed(speed[4])
            self.set_little_speed(speed[5])
    
    def set_torque(self, torque=[200] * 6):
        """Set torques — params: list len=6"""
        if self.is_valid_6xuint8(torque):
            self.set_thumb_torque(torque[0])
            self.set_thumb_yaw_torque(torque[1])
            self.set_index_torque(torque[2])
            self.set_middle_torque(torque[3])
            self.set_ring_torque(torque[4])
            self.set_little_torque(torque[5])

    def set_current(self, current=[200] * 6):
        """Set current — params: list len=6"""
        print("Setting current is not supported on O6", flush=True)
        pass

    def get_version(self) -> list:
        """Get current firmware version"""
        return [self.get_hand_freedom(), self.get_hand_version(), self.get_hand_number(),
                self.get_hand_direction(), self.get_software_version(), self.get_hardware_version()]
    
    def get_current(self):
        """Get current"""
        print("Getting current is not supported on O6", flush=True)
        pass

    def get_state(self) -> list:
        """Get finger motor states"""
        return [self.get_thumb_pitch(), self.get_thumb_yaw(), self.get_index_pitch(), self.get_middle_pitch(),
                self.get_ring_pitch(), self.get_little_pitch()]
    
    def get_state_for_pub(self) -> list:
        return self.get_state()

    def get_current_status(self) -> list:
        return self.get_state()
    
    def get_speed(self) -> list:
        """Get current speeds"""
        return [self.get_thumb_speed(), self.get_thumb_yaw_speed(), self.get_index_speed(),
                self.get_middle_speed(), self.get_ring_speed(), self.get_little_speed()]
    
    def get_joint_speed(self) -> list:
        return self.get_speed()
    
    def get_touch_type(self) -> list:
        """Get pressure-sensing type — O6 currently has no pressure sensing"""
        return -1
    
    def get_normal_force(self) -> list:
        """Get pressure data: point type"""
        return [-1] * 5
    
    def get_tangential_force(self) -> list:
        """Get pressure data: point type"""
        return [-1] * 5
    
    def get_approach_inc(self) -> list:
        """Get pressure data: point type"""
        return [-1] * 5
    
    def get_touch(self) -> list:
        return [-1] * 5
    
    def get_matrix_touch(self) -> list:
        """Get pressure data: matrix type"""
        thumb_matrix  = np.full((12, 6), -1)
        index_matrix  = np.full((12, 6), -1)
        middle_matrix = np.full((12, 6), -1)
        ring_matrix   = np.full((12, 6), -1)
        little_matrix = np.full((12, 6), -1)
        return thumb_matrix, index_matrix, middle_matrix, ring_matrix, little_matrix
    
    def get_matrix_touch_v2(self) -> list:
        """Get pressure data: matrix type"""
        return self.get_matrix_touch()
    
    def get_torque(self) -> list:
        """Get current torques"""
        return [self.get_thumb_torque(), self.get_thumb_yaw_torque(), self.get_index_torque(),
                self.get_middle_torque(), self.get_ring_torque(), self.get_little_torque()]
    
    def get_temperature(self) -> list:
        """Get current motor temperatures"""
        return [self.get_thumb_temp(), self.get_thumb_yaw_temp(), self.get_index_temp(),
                self.get_middle_temp(), self.get_ring_temp(), self.get_little_temp()]
    
    def get_fault(self) -> list:
        """Get current motor error codes"""
        return [self.get_thumb_error(), self.get_thumb_yaw_error(), self.get_index_error(),
                self.get_middle_error(), self.get_ring_error(), self.get_little_error()]

    # ----------------------------------------------------------
    # Convenience functions
    # ----------------------------------------------------------
    def set_all_fingers(self, pitch: int):
        """Set flexion angle for all five fingers at once (0–255)"""
        for fn in (self.set_thumb_pitch, self.set_thumb_yaw, self.set_index_pitch,
                   self.set_middle_pitch, self.set_ring_pitch, self.set_little_pitch):
            fn(pitch)

    def relax(self):
        """Fully extend all fingers (255)"""
        self.set_all_fingers(255)

    def fist(self):
        """Fully flex all fingers (0)"""
        self.set_all_fingers(0)

    def dump_status(self):
        """Print all readable statuses"""
        print("--------- O6 Hand Status ---------")
        print(f"hand_state  state={self.get_state()}")
        print(f"Temperature  {self.get_temperature()}℃")
        print(f"Error code   {self.get_fault()}")
        print(f"version version={self.get_embedded_version()}")  # NOTE: get_embedded_version() not defined
        print("----------------------------------")


# ------------------------------------------------------------------
# Quick CLI test
# ------------------------------------------------------------------
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="O6 Hand Modbus tester")
    parser.add_argument("-p", "--port", required=True, help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("-l", "--left", action="store_true", help="Left hand; default is right hand")
    args = parser.parse_args()

    hand = LinkerHandO6RS485(hand_id=0x28, modbus_port="/dev/ttyUSB0", baudrate=115200)
    hand.dump_status()
    print("Run relax → extend")
    hand.relax()
    time.sleep(1)
    print("Run fist → close")
    hand.fist()
    time.sleep(1)
    hand.relax()
    print("Demo complete")
