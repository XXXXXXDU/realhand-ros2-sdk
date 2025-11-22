# LinkerHand Dexterous Hand ROS2 SDK

## Overview
The LinkerHand ROS SDK is developed by Lingxin Qiaoshou (Beijing) Technology Co., Ltd. It provides drivers and example source code for LinkerHand dexterous hands such as O6, L6, L7, O7, L10, and L21. It can be used with real hardware and simulators.  
The LinkerHand ROS2 SDK currently supports Ubuntu 22.04, ROS Humble, and Python 3.10 or later.

## Installation
  Ensure your environment is Ubuntu 20.04, ROS 2 Foxy, Python 3.8.20 or later.
- Download

    $ mkdir -p linker_hand_ros2_sdk/src
    $ cd linker_hand_ros2_sdk/src
    $ git clone https://github.com/linker-bot/linkerhand-ros2-sdk.git

- Build

    $ sudo apt install python3-can
    $ cd linker_hand_ros2_sdk/src/
    $ pip install -r requirements.txt

## Usage for Ubuntu
  **Before using, edit the [setting.yaml](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/config/setting.yaml) configuration file according to your actual needs.**  
- Change the password in `setting.yaml`. Default PASSWORD: `"12345678"`  
The default password is the Ubuntu system password and is used by the SDK to automatically bring up the CAN interface.

  **Before using, edit the single-hand [linker_hand.launch.py](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/linker_hand_ros2_sdk/launch/linker_hand.launch.py) or dual-hand [linker_hand_double.launch.py](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/linker_hand_ros2_sdk/launch/linker_hand_double.launch.py) files according to your actual hand parameters.**

- Start SDK (single hand)   Plug the LinkerHand USB-to-CAN device into the Ubuntu machine. Supported models: O6/L6/L7/L10/L20/L21/L25  
- Start SDK (dual hands)   First plug the left-hand LinkerHand USB-to-CAN device into the Ubuntu machine (usually recognized as `can0`). Then plug the right-hand device (usually recognized as `can1`). Supported models: O6/L6/L7/L10/L20/L21/L25

    # Bring up the CAN interface
    $ sudo /usr/sbin/ip link set can0 up type can bitrate 1000000 # The blue LED on the USB-to-CAN device stays solid
    $ cd linker_hand_ros2_sdk/
    $ colcon build --symlink-install
    $ source ./install/setup.bash
    $ sudo chmod a+x src/linker_hand_ros2_sdk/linker_hand_ros2_sdk/linker_hand_ros2_sdk/linker_hand.py
    $ # Single hand
    $ ros2 launch linker_hand_ros2_sdk linker_hand.launch.py
    $ # Dual hands
    $ ros2 launch linker_hand_ros2_sdk linker_hand_double.launch.py
    $ [linker_hand_sdk-1] 2025-06-24 17:21:14  Current SDK version: 2.1.4
    $ [linker_hand_sdk-1] 2025-06-24 17:21:14  left L10 set speed to [200, 250, 250, 250, 250, 250, 250, 250, 250, 250]
    $ [linker_hand_sdk-1] 2025-06-24 17:21:14  left L10 set maximum torque to [200, 200, 200, 200, 200]

## Usage for WIN + ROS2

  **Before using, edit the [linker_hand.launch.py](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/linker_hand_ros2_sdk/launch/linker_hand.launch.py) file according to your actual hand parameters.**

- Start SDK   Plug the LinkerHand USB-to-CAN device into the Windows machine. Supported models: L7/L10/L20/L21/L25  
- Note: You must install the USB-to-CAN driver before use.

    $ mkdir -p linker_hand_ros2_sdk/src
    $ cd linker_hand_ros2_sdk/src
    $ git clone https://github.com/linker-bot/linkerhand-ros2-sdk.git
    $ cd linker_hand_ros2_sdk/
    $ set PYTHONUTF8=1 # Set environment variable to use UTF-8 encoding
    $ colcon build --symlink-install
    $ call ./install/local_setup.bat
    $ ros2 launch linker_hand_ros2_sdk linker_hand.launch.py # Edit the CAN port name in the launch file first
    $ [linker_hand_sdk-1] 2025-06-24 17:21:14  Current SDK version: 2.1.4
    $ [linker_hand_sdk-1] 2025-06-24 17:21:14  left L10 set speed to [200, 250, 250, 250, 250, 250, 250, 250, 250, 250]
    $ [linker_hand_sdk-1] 2025-06-24 17:21:14  left L10 set maximum torque to [200, 200, 200, 200, 200]

## RS485 Protocol Switching
Currently supports O6/L6/L10. For other hand models, please refer to the MODBUS RS485 protocol documentation.

Edit `config/setting.yaml` according to the comments in the file. Set `MODBUS: "/dev/ttyUSB0"`, and set the `"modbus"` parameter in the [linker_hand.launch.py](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/linker_hand_ros2_sdk/launch/linker_hand.launch.py) configuration file to `"/dev/ttyUSB0"`. The USB-RS485 adapter usually appears as `/dev/ttyUSB*` or `/dev/ttyACM*` on Ubuntu.  
modbus: `"None"` or `"/dev/ttyUSB0"`

    # Ensure requirements.txt dependencies are installed
    # Install system-level related drivers
    $ pip install minimalmodbus --break-system-packages
    $ pip install pyserial --break-system-packages
    $ pip install pymodbus==3.5.1 --break-system-packages
    # Check the USB-RS485 port number
    $ ls /dev
    # If you see something like ttyUSB0, then grant permissions to the port
    $ sudo chmod 777 /dev/ttyUSB0

- Mapping between `position` and finger joints

    $ ros2 topic echo /cb_left_hand_control_cmd --flow-style

    header: 
      seq: 256
      stamp: 
        secs: 1744343699
        nsecs: 232647418
      frame_id: ''
    name: []
    position: [155.0, 162.0, 176.0, 125.0, 255.0, 255.0, 180.0, 179.0, 181.0, 68.0]
    velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

- Mapping between `state` and finger joints

    $ ros2 topic echo /cb_left_hand_state --flow-style
    ---
    header:
      stamp:
        sec: 1760593389
        nanosec: 128827739
      frame_id: ''
    name: []
    position: [200.0, 255.0, 254.0, 254.0, 254.0, 180.0]
    velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ---

  O6:  ["Thumb flexion", "Thumb yaw", "Index flexion", "Middle flexion", "Ring flexion", "Little flexion"]

  L6:  ["Thumb flexion", "Thumb yaw", "Index flexion", "Middle flexion", "Ring flexion", "Little flexion"]

  L7:  ["Thumb flexion", "Thumb yaw", "Index flexion", "Middle flexion", "Ring flexion", "Little flexion", "Thumb rotation"]

  L10: ["Thumb base", "Thumb ab/adduction", "Index base", "Middle base", "Ring base", "Little base", "Index ab/adduction", "Ring ab/adduction", "Little ab/adduction", "Thumb rotation"]

  L20: ["Thumb base", "Index base", "Middle base", "Ring base", "Little base", "Thumb ab/adduction", "Index ab/adduction", "Middle ab/adduction", "Ring ab/adduction", "Little ab/adduction", "Thumb yaw", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb tip", "Index tip", "Middle tip", "Ring tip", "Little tip"]

  G20 (Industrial): ["Thumb base", "Index base", "Middle base", "Ring base", "Little base", "Thumb ab/adduction", "Index ab/adduction", "Middle ab/adduction", "Ring ab/adduction", "Little ab/adduction", "Thumb yaw", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb tip", "Index tip", "Middle tip", "Ring tip", "Little tip"]

  L21: ["Thumb base", "Index base", "Middle base", "Ring base", "Little base", "Thumb ab/adduction", "Index ab/adduction", "Middle ab/adduction", "Ring ab/adduction", "Little ab/adduction", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb middle", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb tip", "Index tip", "Middle tip", "Ring tip", "Little tip"]

  L25: ["Thumb base", "Index base", "Middle base", "Ring base", "Little base", "Thumb ab/adduction", "Index ab/adduction", "Middle ab/adduction", "Ring ab/adduction", "Little ab/adduction", "Thumb roll", "Reserved", "Reserved", "Reserved", "Reserved", "Thumb middle", "Index middle", "Middle middle", "Ring middle", "Little middle", "Thumb tip", "Index tip", "Middle tip", "Ring tip", "Little tip"]

## Changelog
- > ### release_2.2.4
  - 1. Added CAN communication support for the G20 industrial dexterous hand

- > ### release_2.2.3
  - 1. GUI control adds real-time control of speed and torque

- > ### release_2.2.1
  - 1. Added matrix pressure sensor dot-array heatmap
  - 2. Added O6 RS485 communication

- > ### release_2.1.9
  - 1. Support for O6/L6 dexterous hands

- > ### release_2.1.8
  - 1. Fixed occasional frame collision issues

- > ### release_2.1.7
  - 1. Fixed known issues
  - 2. Moved [Mujoco and PyBullet simulation](https://github.com/linker-bot/linkerhand-sim) to a separate repository to reduce the SDK size

- > ### release_2.1.6
  - 1. Support dual CAN to control dual dexterous hands
  - 2. Added Mujoco simulation
  - 3. Added PyBullet simulation

- > ### release_1.0.3
  - 1. Support for L20/L25 dexterous hands

- > ### release_1.0.2
  - 1. Support for L10/O10 dexterous hands
  - 2. Support GUI control for L10/O10 dexterous hands
  - 3. Added LinkerHand waveform display for hands with pressure sensors

- > ### release_1.0.1
  - 1. Support for L7/O7 dexterous hands
  - 2. Support GUI control for L7/O7 dexterous hands


## [Examples](examples/)

  **Before using, edit the [setting.yaml](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/config/setting.yaml) configuration file according to your actual needs.**


## [Examples] General
- [gui_control (GUI control & action examples)](GUI control & action examples)  
The GUI control can use sliders to control each joint of the LinkerHand L10 and L20 independently. You can also record the current values of all sliders with buttons, saving the current joint state of the LinkerHand, and replay actions via functional buttons.    

Using `gui_control` to control the LinkerHand:  
The `gui_control` interface requires `linker_hand_sdk_ros` to be running and controls the LinkerHand via topics.  
After starting the ROS2 SDK:

  **Before using, edit the [gui_control.launch.py](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/gui_control/launch/gui_control.launch.py) file according to your actual hand parameters.**


    # Open a new terminal
    $ cd linker_hand_ros2_sdk/
    $ source ./install/setup.bash
    $ ros2 launch gui_control gui_control.launch.py

After launching, a UI window will pop up. You can control the corresponding LinkerHand joints via the sliders.

- Add or modify action examples. You can add or modify actions in the [constants.py](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/gui_control/gui_control/config/constants.py) file.

    # For example, add action sequences for L6
    "L6": HandConfig(
            joint_names_en=["thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch", "middle_mcp_pitch", "pinky_mcp_pitch", "ring_mcp_pitch"],
            joint_names=["Thumb flexion", "Thumb yaw", "Index flexion", "Middle flexion", "Ring flexion", "Little flexion"],
            init_pos=[250] * 6,
            preset_actions={
                "Open": [250, 250, 250, 250, 250, 250],
                "One": [0, 31, 255, 0, 0, 0],
                "Two": [0, 31, 255, 255, 0, 0],
                "Three": [0, 30, 255, 255, 255, 0], 
                "Four": [0, 30, 255, 255, 255, 255],
                "Five": [250, 250, 250, 250, 250, 250],
                "OK": [54, 41, 164, 250, 250, 250],
                "Thumbs Up": [255, 31, 0, 0, 0, 0],
                "Fist": [49, 61, 0, 0, 0, 0],
                # Add custom actions......
            }
        )

## [Examples] [matrix_touch_gui (Matrix pressure heatmap)]
The matrix pressure heatmap displays fingertip matrix pressure sensor data for each joint of the LinkerHand and visualizes it as a heatmap. Make sure your hand is equipped with matrix pressure sensors before using.  
After starting the ROS2 SDK:  
  **Before using, edit the [matrix_touch_gui.launch.py](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/matrix_touch_gui/launch/matrix_touch_gui.launch.py) file according to your actual hand parameters.**


    # Open a new terminal
    $ cd linker_hand_ros2_sdk/
    $ source ./install/setup.bash
    $ ros2 launch matrix_touch_gui matrix_touch_gui.launch.py

## Using the GUI on WIN + ROS2
  **Before using, edit the [gui_control.launch.py](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/gui_control/launch/gui_control.launch.py) file according to your actual hand parameters.**

    # Open a new terminal
    $ cd linker_hand_ros2_sdk/
    $ call ./install/setup.bash
    $ ros2 launch gui_control gui_control.launch.py

## L7
- [7001-action-group-show-ti (Finger movements)](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/examples/L7/gesture/action-group-show-ti.py)

## L10
- [10001-action-group-show-normal (Finger movements)](https://github.com/linker-bot/linkerhand-ros2-sdk/blob/main/examples/L10/gesture/action-group-show-normal.py)

## Topic Document
[Linker Hand Topic Document](doc/Topic-Reference.md)

## Mujoco & PyBullet Simulation
 - [Mujoco and PyBullet repository](https://github.com/linker-bot/linkerhand-sim)
