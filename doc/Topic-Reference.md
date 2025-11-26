# Real Hand ROS SDK Topic Documentation

## Topic Overview

This document provides a detailed overview of the ROS Topic for the Real Hand, including functions for controlling the hand's movements, retrieving sensor data, and setting operational parameters.

## Topic List
```bash
/cb_hand_setting_cmd # RealHand command topic
/cb_left_hand_control_cmd # Control left-hand motion by range 0~255 (range)
/cb_left_hand_control_cmd_arc # Control left-hand motion by arc -3.14~3.14 (radians) 
/cb_left_hand_force # Left-hand pressure (force) data topic
/cb_left_hand_matrix_touch # Left-hand matrix pressure data topic list(6x12)
/cb_left_hand_info  # Left-hand configuration info topic
/cb_left_hand_state # Left-hand state topic (range)
/cb_left_hand_state_arc # Left-hand state topic (radians)
/cb_right_hand_control_cmd # Control right-hand motion by range 0~255 (range)
/cb_right_hand_control_cmd_arc # Control right-hand motion by arc -3.14~3.14 (radians)
/cb_right_hand_force # Right-hand pressure (force) data topic
/cb_right_hand_matrix_touch # Right-hand matrix pressure data topic list(6x12)
/cb_right_hand_info # Right-hand configuration info topic
/cb_right_hand_state # Right-hand state topic (range)
/cb_right_hand_state_arc # Right-hand state topic (radians)

```

### Get hand status topic /cb_left_hand_state or /cb_right_hand_state
```bash

header: 
  seq: 211345
  stamp: 
    secs: 1744703535
    nsecs: 722361087
  frame_id: ''
name: 
  - joint71
  - joint72
  - joint73
  - joint77
  - joint75
  - joint76
  - joint77
  - joint78
  - joint79
  - joint80
  - joint81
  - joint82
  - joint83
  - joint84
  - joint88
  - joint86
  - joint87
  - joint88
  - joint89
  - joint90
position: [255.0, 132.0, 255.0, 255.0, 255.0, 255.0, 131.0, 127.0, 129.0, 127.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```
**Description**: 
Finger movement at specified location. Data format: sensor_msgs/JointState 
**Parameters**:
- `position`: The current state of the finger joint. List(float) L7 length: 7 L10 length: 10 L20 length: 20 L25 length: 25 Each element ranges from 0 to 255.
---

### Acquire pressure sensitivity data Topic /cb_left_hand_force or /cb_right_hand_force
```bash
rostopic echo /cb_left_hand_touch
data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 255.0, 255.0, 255.0, 255.0, 255.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```
**Description**: 
Data format for acquiring finger pressure sensitivity data
**Parameters**:
- `data`:
```bash
Index 0:  Thumb normal pressure value 0~255
Index 1:  Index finger normal pressure value 0~255
Index 2:  Middle finger normal pressure value 0~255
Index 3:  Ring finger normal pressure value 0~255
Index 4:  Little finger normal pressure value 0~255

Index 5:  Thumb tangential pressure value 0~255
Index 6:  Index finger tangential pressure value 0~255
Index 7:  Middle finger tangential pressure value 0~255
Index 8:  Ring finger tangential pressure value 0~255
Index 9:  Little finger tangential pressure value 0~255

Index 10: Thumb tangential pressure direction value 0~255 # 255 indicates no pressure direction
Index 11: Index finger tangential pressure direction value 0~255
Index 12: Middle finger tangential pressure direction value 0~255
Index 13: Ring finger tangential pressure direction value 0~255
Index 14: Little finger tangential pressure direction value 0~255

Index 15: Thumb proximity value 0~255
Index 16: Index finger proximity value 0~255
Index 17: Middle finger proximity value 0~255
Index 18: Ring finger proximity value 0~255
Index 19: Little finger proximity value 0~255

```
---




### Obtain Matrix Pressure Sensing Data Topic /cb_left_hand_matrix_touch or /cb_right_hand_matrix_touch Note: Only second-generation pressure sensors are supported.
```bash
ros2 topic echo /cb_left_hand_matrix_touch
data: "{"thumb_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,0, 0, 0, 0, 0]], "index_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,  0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "middle_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "ring_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]], "little_matrix": [[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]}"
```
**Description**: 
Obtain finger matrix pressure sensitivity data. Data format: std_msgs/String JSON.
**Parameters**:
- `data`:
```bash
thumb_matrix：Thumb matrix pressure values 0~255
index_matrix：Index finger matrix pressure values 0~255
middle_matrix：Middle finger matrix pressure values 0~255
ring_matrix：Ring finger matrix pressure values 0~255
little_matrix：Little finger matrix pressure values 0~255

```
---
### Retrieve RealHand configuration information Topic /cb_left_hand_info or /cb_right_hand_info
```bash
ros2 topic echo /cb_right_hand_info
data: "{\"version\": [7, 0, 0, 0], \"hand_joint\": \"L21\", \"speed\": [1, 0, 0, 0, 0, 0,\
  \ 0, 0, 0, 0, 6, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0], \"current\"\
  : [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \"fault\": [[0,\
  \ 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0,\
  \ 0, 0, 0, 0, 0]], \"motor_temperature\": [71, 52, 62, 46, 0, 65, 0, 50, 40, 0,\
  \ 0, 39, 0, 52, 41, 0, 0, 38, 0, 53, 41, 0, 0, 39, 0, 50, 40, 0, 0, 38], \"torque\"\
  : [16, 8, 3, 0, 0, 9, 0, 2, 0, 0, 0, 9, 0, 2, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 8,\
  \ 0, 0, 0, 8], \"is_touch\": true, \"touch_type\": 2, \"touch\": [0, 0, 0, 0, 0,\
  \ 0], \"finger_order\": [\"thumb_root\", \"index_finger_root\", \"middle_finger_root\"\
  , \"ring_finger_root\", \"little_finger_root\", \"thumb_abduction\", \"index_finger_abduction\"\
  , \"middle_finger_abduction\", \"ring_finger_abduction\", \"little_finger_abduction\"\
  , \"thumb_roll\", \"reserved\", \"reserved\", \"reserved\", \"reserved\", \"thumb_middle_joint\"\
  , \"reserved\", \"reserved\", \"reserved\", \"reserved\", \"thumb_tip\", \"index_finger_tip\"\
  , \"middle_finger_tip\", \"ring_finger_tip\", \"little_finger_tip\"]}"
```
**Description**: 
Get RealHand configuration info Data format std_msgs/String for Json
**Parameters**:
- `version`: hand version number version[0]: indicates L10 version[1]: indicates version version[2]: indicates batch number version[3]: 76 for left hand 82 for right hand others are internal codes
- `hand_joint`: L10 or L20 or L25 etc.
- `speed`: finger speed
- `current`: current finger voltage (if supported)
- `torque`: finger torque (if supported)
- `is_touch`: whether a pressure sensor is present
- `touch_type`: sensor type (if supported)
- `touch`: sensor data (if supported)
- `max_press_rco`: maximum current
- `fault`: motor fault 0 is normal others indicate faults
- `motor_temperature`: current motor temperature
- `finger_order`: current dexterous hand finger motor order


---



## range_to_arc RadianAngleReferenceTable

GetAndSendL10、L20RadianValues

topic:/cb_left_hand_state_arc and /cb_right_hand_state_arc GetRealHandStatepositionAsRadianValues

topic:/cb_left_hand_control_cmd_arc and /cb_right_hand_control_cmd_arc PublishpositionRadianValuesToControlRealHandFingerMotion

## RadianAndRangeReferenceTable

#---------------------------------------------------------------------------------------------------

L7DexterousHandJointOrder = ["ThumbFlexion", "ThumbAbduction","IndexFlexion", "MiddleFlexion", "RingFlexion","LittleFlexion","ThumbRotation"]
### L7 L OK
l7_l_min = [0, 0, 0, 0, 0, 0, -0.52]
l7_l_max = [0.44, 1.43, 1.62, 1.62, 1.62, 1.62, 1.01]
l7_l_derict = [-1, -1, -1, -1, -1, -1, -1]
### L7 R OK (urdfWillBeModifiedLater！！！)
l7_r_min = [0, -1.43, 0, 0, 0, 0, 0]
l7_r_max = [0.75, 0, 1.62, 1.62, 1.62, 1.62, 1.54]
l7_r_derict = [-1, 0, -1, -1, -1, -1, -1]
#---------------------------------------------------------------------------------------------------

L10DexterousHandJointOrder = ["ThumbBase", "ThumbSideSwing","IndexBase", "MiddleBase", "RingBase","LittleBase","IndexSideSwing","RingSideSwing","LittleSideSwing","ThumbRotation"]
### L10 L OK
l10_l_min = [0, 0, 0, 0, 0, 0, 0, -0.26, -0.26, -0.52]
l10_l_max = [1.45, 1.43, 1.62, 1.62, 1.62, 1.62, 0.26, 0, 0, 1.01]
l10_l_derict = [-1, -1, -1, -1, -1, -1, 0, -1, -1, -1]
### L10 R OK
l10_r_min = [0, 0, 0, 0, 0, 0, -0.26, 0, 0, -0.52]
l10_r_max = [0.75, 1.43, 1.62, 1.62, 1.62, 1.62, 0, 0.13, 0.26, 1.01]
l10_r_derict = [-1, -1, -1, -1, -1, -1, -1, 0, 0, -1]
#---------------------------------------------------------------------------------------------------

L20DexterousHandJointOrder = ["ThumbBase", "IndexBase", "MiddleBase", "RingBase","LittleBase","ThumbSideSwing","IndexSideSwing","MiddleSideSwing","RingSideSwing","LittleSideSwing","ThumbHorizontalSwing","Reserved","Reserved","Reserved","Reserved","ThumbTip","IndexDistal","MiddleDistal","RingDistal","LittleDistal"]
### L20 L OK
l20_l_min = [0, 0, 0, 0, 0, -0.297, -0.26, -0.26, -0.26, -0.26, 0.122, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l20_l_max = [0.87, 1.4, 1.4, 1.4, 1.4, 0.683, 0.26, 0.26, 0.26, 0.26, 1.78, 0, 0, 0, 0, 1.29, 1.08, 1.08, 1.08, 1.08]
l20_l_derict = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]
### L20 R OK
l20_r_min = [0, 0, 0, 0, 0, -0.297, -0.26, -0.26, -0.26, -0.26, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l20_r_max = [0.87, 1.4, 1.4, 1.4, 1.4, 0.683, 0.26, 0.26, 0.26, 0.26, 1.78, 0, 0, 0, 0, 1.29, 1.08, 1.08, 1.08, 1.08]
l20_r_derict = [-1, -1, -1, -1, -1, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]
#---------------------------------------------------------------------------------------------------

L21DexterousHandJointOrder = ["ThumbBase","IndexBase","MiddleBase","RingBase","LittleBase","ThumbSideSwing","IndexSideSwing","MiddleSideSwing","RingSideSwing","LittleSideSwing","ThumbRoll","Reserved","Reserved","Reserved","Reserved","ThumbMiddle","Reserved","Reserved","Reserved","Reserved","ThumbFingertip","IndexFingertip","MiddleFingertip","RingFingertip","LittleFingertip"]
### L21 L OK
l21_l_min = [0, 0, 0, 0, 0, 0, 0, -0.18, -0.18, 0, -0.6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l21_l_max = [1, 1.57, 1.57, 1.57, 1.57, 1.6, 0.18, 0.18, 0.18, 0.18, 0.6, 0, 0, 0, 0, 1.57, 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 1.57]
l21_l_derict = [-1, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]
### L21 R OK
l21_r_min = [0, 0, 0, 0, 0, 0, -0.18, -0.18, -0.18, -0.18, -0.6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
l21_r_max = [1, 1.57, 1.57, 1.57, 1.57, 1.6, 0.18, 0.18, 0.18, 0.18, 0.6, 0, 0, 0, 0, 1.57, 0, 0, 0, 0, 1.57, 1.57, 1.57, 1.57, 1.57]
l21_r_derict = [-1, -1, -1, -1, -1, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1]
#---------------------------------------------------------------------------------------------------



# Topic Examples
# L7
- L7RightHandFist
```bash
ros2 topic pub /cb_right_hand_control_cmd sensor_msgs/msg/JointState "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name: []
position: [75,115,0,0,0,0,42]
velocity: [255,255,255,255,255,255,255]
effort: []
"
