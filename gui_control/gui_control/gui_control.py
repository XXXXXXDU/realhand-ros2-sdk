import sys
import time, json
import threading
from dataclasses import dataclass
from typing import List, Dict
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QObject, QEvent
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, 
    QSlider, QLabel, QPushButton, QGroupBox, QScrollArea, QTabWidget, 
    QFrame, QSplitter, QMessageBox, QTextEdit
)
from PyQt5.QtGui import QFont

from .utils.mapping import *

from .config.constants import _HAND_CONFIGS
LOOP_TIME = 1000 # Loop action interval time (ms)
class ROS2NodeManager(QObject):
    """ROS2 node manager, handles ROS communication"""
    status_updated = pyqtSignal(str, str)  # Status type, message content

    def __init__(self, node_name: str = "hand_control_node"):
        super().__init__()
        self.node = None
        self.publisher = None
        self.joint_state = JointState()
        self.joint_state.header = Header()
        
        # Initialize ROS2 node
        self.init_node(node_name)

    def init_node(self, node_name: str):
        """Initialize ROS2 node"""
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = Node(node_name)
            
            # Declare parameters
            self.node.declare_parameter('hand_type', 'left')
            self.node.declare_parameter('hand_joint', 'L10')
            self.node.declare_parameter('topic_hz', 30)
            self.node.declare_parameter('is_arc', False)
            
            # Get parameters
            self.hand_type = self.node.get_parameter('hand_type').value
            self.hand_joint = self.node.get_parameter('hand_joint').value
            self.hz = self.node.get_parameter('topic_hz').value
            self.is_arc = self.node.get_parameter('is_arc').value
            
            if self.is_arc == True:
                # Create publisher
                self.publisher_arc = self.node.create_publisher(
                    JointState, f'/cb_{self.hand_type}_hand_control_cmd_arc', 10
                )
            # Create publisher
            self.publisher = self.node.create_publisher(
                JointState, f'/cb_{self.hand_type}_hand_control_cmd', 10
            )
                    # New speed / torque publishers
            self.speed_pub = self.node.create_publisher(
                String, f'/cb_hand_setting_cmd', 10)
            self.torque_pub = self.node.create_publisher(
                String, f'/cb_hand_setting_cmd', 10)
            self.status_updated.emit("info", f"ROS2 node initialized successfully: {self.hand_type} {self.hand_joint}")
            
            # Start ROS2 spin thread
            self.spin_thread = threading.Thread(target=self.spin_node, daemon=True)
            self.spin_thread.start()
        except Exception as e:
            self.status_updated.emit("error", f"ROS2 initialization failed: {str(e)}")
            raise

    def spin_node(self):
        """Run ROS2 node spin loop"""
        while rclpy.ok() and self.node:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def publish_joint_state(self, positions: List[int]):
        """Publish joint state message"""
        if not self.publisher or not self.node:
            self.status_updated.emit("error", "ROS2 publisher not initialized")
            return
            
        try:
            self.joint_state.header.stamp = self.node.get_clock().now().to_msg()
            self.joint_state.position = [float(pos) for pos in positions]
            # self.joint_state.velocity = [0.1] * len(positions)
            # self.joint_state.effort = [0.01] * len(positions)
            # If there are joint names, add them to the message
            #hand_config = HandConfig.from_hand_type(self.hand_joint)
            hand_config = _HAND_CONFIGS[self.hand_joint]
            if len(hand_config.joint_names) == len(positions):
                if hand_config.joint_names_en != None:
                    self.joint_state.name = hand_config.joint_names_en
                else:
                    self.joint_state.name = hand_config.joint_names
                
            self.publisher.publish(self.joint_state)
            if self.is_arc == True:
                if self.hand_joint == "O6":
                    if self.hand_type == "left":
                        pose = range_to_arc_left(positions,self.hand_joint)
                    elif self.hand_type == "right":
                        pose = range_to_arc_right(positions,self.hand_joint)
                elif self.hand_joint == "L7" or self.hand_joint == "L21" or self.hand_joint == "L25":
                    if self.hand_type == "left":
                        pose = range_to_arc_left(positions,self.hand_joint)
                    elif self.hand_type == "right":
                        pose = range_to_arc_right(positions,self.hand_joint)
                elif self.hand_joint == "L10":
                    if self.hand_type == "left":
                        pose = range_to_arc_left_10(positions)
                    elif self.hand_type == "right":
                        pose = range_to_arc_right_10(positions)
                elif self.hand_joint == "L20":
                    if self.hand_type == "left":
                        pose = range_to_arc_left_l20(positions)
                    elif self.hand_type == "right":
                        pose = range_to_arc_right_l20(positions)
                else:
                    print(f"Current {self.hand_joint} {self.hand_type} does not support radian conversion", flush=True)
                self.joint_state.position = [float(pos) for pos in pose]
                self.publisher_arc.publish(self.joint_state)
            self.status_updated.emit("info", "Joint state published")
        except Exception as e:
            self.status_updated.emit("error", f"Publish failed: {str(e)}")

    def publish_speed(self, val: int):
        joint_len = 0
        if (self.hand_joint.upper() == "O6" or self.hand_joint.upper() == "L6"):
            joint_len = 6
        elif self.hand_joint == "L7":
            joint_len = 7
        elif self.hand_joint == "L10":
            joint_len = 10
        else:
            joint_len = 5
        msg = String()
        v = [val] * joint_len
        data = {
            "setting_cmd": "set_speed",
            "params": {"hand_type":self.hand_type,"speed": v},
        }
        msg.data = json.dumps(data)
        print(f"Speed value: {v}", flush=True)
        self.speed_pub.publish(msg)

    def publish_torque(self, val: int):
        joint_len = 0
        if (self.hand_joint.upper() == "O6" or self.hand_joint.upper() == "L6"):
            joint_len = 6
        elif self.hand_joint == "L7":
            joint_len = 7
        elif self.hand_joint == "L10":
            joint_len = 10
        else:
            joint_len = 5
        msg = String()
        v = [val] * joint_len
        data = {
            "setting_cmd": "set_max_torque_limits",
            "params": {"hand_type":self.hand_type,"torque": v},
        }
        
        msg.data = json.dumps(data)
        print(f"Torque value: {v}", flush=True)
        self.torque_pub.publish(msg)

    def shutdown(self):
        """Shut down ROS2 node"""
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

class HandControlGUI(QWidget):
    """Dexterous hand control GUI"""
    status_updated = pyqtSignal(str, str)  # Status type, message content

    def __init__(self, ros_manager: ROS2NodeManager):
        super().__init__()
        
        # Loop control variables
        self.cycle_timer = None  # Loop timer
        self.current_action_index = -1  # Current action index
        self.preset_buttons = []  # Store references to preset action buttons
        
        # Set ROS manager
        self.ros_manager = ros_manager
        self.ros_manager.status_updated.connect(self.update_status)
        
        # Get hand configuration
        self.hand_joint = self.ros_manager.hand_joint
        self.hand_type = self.ros_manager.hand_type
        self.hand_config = _HAND_CONFIGS[self.hand_joint]
        
        # Initialize UI
        self.init_ui()
        
        # Set timer to publish joint state
        self.publish_timer = QTimer(self)
        self.publish_timer.setInterval(int(1000 / self.ros_manager.hz))
        self.publish_timer.timeout.connect(self.publish_joint_state)
        self.publish_timer.start()

    def init_ui(self):
        """Initialize user interface"""
        # Set window properties
        self.setWindowTitle(f'Dexterous Hand Control - {self.hand_type} {self.hand_joint}')
        self.setMinimumSize(1200, 900)
        
        # Set styles
        self.setStyleSheet("""
            QWidget {
                font-family: 'Microsoft YaHei', 'SimHei', sans-serif;
                font-size: 12px;
            }
            QGroupBox {
                border: 1px solid #CCCCCC;
                border-radius: 6px;
                margin-top: 6px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: #165DFF;
                font-weight: bold;
            }
            QSlider::groove:horizontal {
                border: 1px solid #999999;
                height: 8px;
                border-radius: 4px;
                background: #CCCCCC;
                margin: 2px 0;
            }
            QSlider::handle:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #165DFF, stop:1 #0E42D2);
                border: 1px solid #5C8AFF;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QPushButton {
                background-color: #E0E0E0;
                border: 1px solid #CCCCCC;
                border-radius: 4px;
                padding: 5px 10px;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #F0F0F0;
            }
            QPushButton:pressed {
                background-color: #D0D0D0;
            }
            QPushButton[category="preset"] {
                background-color: #E6F7FF;
                color: #1890FF;
                border-color: #91D5FF;
            }
            QPushButton[category="preset"]:hover {
                background-color: #B3E0FF;
            }
            QPushButton[category="action"] {
                background-color: #FFF7E6;
                color: #FA8C16;
                border-color: #FFD591;
            }
            QPushButton[category="action"]:hover {
                background-color: #FFE6B3;
            }
            QPushButton[category="danger"] {
                background-color: #FFF1F0;
                color: #F5222D;
                border-color: #FFCCC7;
            }
            QPushButton[category="danger"]:hover {
                background-color: #FFE8E6;
            }
            QLabel#StatusLabel {
                padding: 5px;
                border-radius: 4px;
            }
            QLabel#StatusInfo {
                background-color: #F0F7FF;
                color: #0066CC;
            }
            QLabel#StatusError {
                background-color: #FFF0F0;
                color: #CC0000;
            }
            /* Value display panel styles */
            QTextEdit#ValueDisplay {
                background-color: #F8F8F8;
                border: 1px solid #CCCCCC;
                border-radius: 4px;
                padding: 10px;
                font-family: Consolas, monospace;
                font-size: 12px;
            }
        """)
        
        # Create main vertical layout
        main_layout = QVBoxLayout(self)
        
        # Create horizontal splitter (original three panels)
        splitter = QSplitter(Qt.Horizontal)
        
        # Create left joint control panel
        self.joint_control_panel = self.create_joint_control_panel()
        splitter.addWidget(self.joint_control_panel)
        
        # Create middle preset actions panel
        self.preset_actions_panel = self.create_preset_actions_panel()
        splitter.addWidget(self.preset_actions_panel)
        
        # Create right status monitor panel
        self.status_monitor_panel = self.create_status_monitor_panel()
        splitter.addWidget(self.status_monitor_panel)
        
        # Set splitter sizes
        splitter.setSizes([500, 300, 400])
        
        # Add splitter to main layout and set stretch factor to 1 (resizable)
        main_layout.addWidget(splitter, stretch=1)
        
        # Create and add value display panel, set stretch factor to 0 (not resizable)
        self.value_display_panel = self.create_value_display_panel()
        main_layout.addWidget(self.value_display_panel, stretch=0)
        
        # Initial update of value display
        self.update_value_display()

    def create_joint_control_panel(self):
        """Create joint control panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Create title
        title_label = QLabel(f"Joint Control - {self.hand_joint}")
        title_label.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        layout.addWidget(title_label)

        # Create slider scroll area
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)
        
        scroll_content = QWidget()
        self.sliders_layout = QGridLayout(scroll_content)
        self.sliders_layout.setSpacing(10)
        
        # Create sliders
        self.create_joint_sliders()
        
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)
        
        return panel

    

    def create_joint_sliders(self):
        """Create joint sliders"""
        # Clear existing sliders
        for i in reversed(range(self.sliders_layout.count())):
            item = self.sliders_layout.itemAt(i)
            if item.widget():
                item.widget().deleteLater()
        
        # Create new sliders
        self.sliders = []
        self.slider_labels = []
        
        for i, (name, value) in enumerate(zip(
            self.hand_config.joint_names, self.hand_config.init_pos
        )):
            # Create label
            label = QLabel(f"{name}: {value}")
            label.setMinimumWidth(120)
            
            # Create slider
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 255)
            slider.setValue(value)
            slider.valueChanged.connect(
                lambda val, idx=i: self.on_slider_value_changed(idx, val)
            )
            
            # Add to layout
            row, col = divmod(i, 1)
            self.sliders_layout.addWidget(label, row, 0)
            self.sliders_layout.addWidget(slider, row, 1)
            
            self.sliders.append(slider)
            self.slider_labels.append(label)

    def create_preset_actions_panel(self):
        """Create preset actions panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # System presets
        sys_preset_group = QGroupBox("System Presets")
        sys_preset_layout = QGridLayout(sys_preset_group)
        sys_preset_layout.setSpacing(8)
        
        # Add system preset action buttons
        self.create_system_preset_buttons(sys_preset_layout)
        layout.addWidget(sys_preset_group)
        
        # Add action buttons
        actions_layout = QHBoxLayout()
        
        # Add cycle run button
        self.cycle_button = QPushButton("Cycle Preset Actions")
        self.cycle_button.setProperty("category", "action")
        self.cycle_button.clicked.connect(self.on_cycle_clicked)
        actions_layout.addWidget(self.cycle_button)
        
        self.home_button = QPushButton("Return to Home")
        self.home_button.setProperty("category", "action")
        self.home_button.clicked.connect(self.on_home_clicked)
        actions_layout.addWidget(self.home_button)
        
        self.stop_button = QPushButton("Stop All Actions")
        self.stop_button.setProperty("category", "danger")
        self.stop_button.clicked.connect(self.on_stop_clicked)
        actions_layout.addWidget(self.stop_button)
        
        layout.addLayout(actions_layout)
        
        return panel

    def create_system_preset_buttons(self, parent_layout):
        """Create system preset action buttons"""
        self.preset_buttons = []  # Clear button list
        if self.hand_config.preset_actions:
            buttons = []
            for idx, (name, positions) in enumerate(self.hand_config.preset_actions.items()):
                button = QPushButton(name)
                button.setProperty("category", "preset")
                button.clicked.connect(
                    lambda checked, pos=positions: self.on_preset_action_clicked(pos)
                )
                buttons.append(button)
                self.preset_buttons.append(button)  # Save button reference
                
            # Add to grid layout
            cols = 2
            for i, button in enumerate(buttons):
                row, col = divmod(i, cols)
                parent_layout.addWidget(button, row, col)

    def create_status_monitor_panel(self):
        """Create status monitor panel (speed/torque each on one line, with real-time slider values)"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # — 1. Title —
        title_label = QLabel("Status Monitor")
        title_label.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        layout.addWidget(title_label)

        # — 2. New: speed and torque settings (one per row) —
        quick_set_gb = QGroupBox("Quick Settings")
        qv_layout = QVBoxLayout(quick_set_gb)

        # Speed row
        speed_hbox = QHBoxLayout()
        speed_hbox.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(0, 255)
        self.speed_slider.setValue(255)
        self.speed_slider.setMinimumWidth(150)
        speed_hbox.addWidget(self.speed_slider)
        self.speed_val_lbl = QLabel("255")          # Real-time value
        self.speed_val_lbl.setMinimumWidth(30)
        speed_hbox.addWidget(self.speed_val_lbl)
        self.speed_btn = QPushButton("Set Speed")
        self.speed_btn.clicked.connect(
            lambda: (
                self.ros_manager.publish_speed(self.speed_slider.value()),
                self.status_updated.emit(
                    "info", f"Speed set to {self.speed_slider.value()}")
            ))
        speed_hbox.addWidget(self.speed_btn)
        speed_hbox.addStretch()
        qv_layout.addLayout(speed_hbox)

        # Torque row
        torque_hbox = QHBoxLayout()
        torque_hbox.addWidget(QLabel("Torque:"))
        self.torque_slider = QSlider(Qt.Horizontal)
        self.torque_slider.setRange(0, 255)
        self.torque_slider.setValue(255)
        self.torque_slider.setMinimumWidth(150)
        torque_hbox.addWidget(self.torque_slider)
        self.torque_val_lbl = QLabel("255")
        self.torque_val_lbl.setMinimumWidth(30)
        torque_hbox.addWidget(self.torque_val_lbl)
        self.torque_btn = QPushButton("Set Torque")
        self.torque_btn.clicked.connect(
            lambda: (
                self.ros_manager.publish_torque(self.torque_slider.value()),
                self.status_updated.emit(
                    "info", f"Torque set to {self.torque_slider.value()}")
            ))
        torque_hbox.addWidget(self.torque_btn)
        torque_hbox.addStretch()
        qv_layout.addLayout(torque_hbox)

        layout.addWidget(quick_set_gb)

        # — 3. Existing tabs (unchanged) —
        tab_widget = QTabWidget()

        # System info tab
        sys_info_widget = QWidget()
        sys_info_layout = QVBoxLayout(sys_info_widget)

        conn_group = QGroupBox("Connection Status")
        conn_layout = QVBoxLayout(conn_group)
        if self.ros_manager.publisher.get_subscription_count() > 0:
            self.connection_status = QLabel("ROS2 node connected")
            self.connection_status.setObjectName("StatusLabel")
            self.connection_status.setObjectName("StatusInfo")
        else:
            self.connection_status = QLabel("ROS2 node not connected")
            self.connection_status.setObjectName("StatusLabel")
            self.connection_status.setObjectName("StatusError")
        conn_layout.addWidget(self.connection_status)

        hand_info_group = QGroupBox("Hand Info")
        hand_info_layout = QVBoxLayout(hand_info_group)
        info_text = f"""Hand type: {self.hand_type}
Joint model: {self.hand_joint}
Joint count: {len(self.hand_config.joint_names)}
Publish rate: {self.ros_manager.hz} Hz"""
        self.hand_info_label = QLabel(info_text)
        self.hand_info_label.setWordWrap(True)
        hand_info_layout.addWidget(self.hand_info_label)

        sys_info_layout.addWidget(conn_group)
        sys_info_layout.addWidget(hand_info_group)
        sys_info_layout.addStretch()
        tab_widget.addTab(sys_info_widget, "System Info")

        # Status log tab
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        self.status_log = QLabel("Waiting for system to start...")
        self.status_log.setObjectName("StatusLabel")
        self.status_log.setObjectName("StatusInfo")
        self.status_log.setWordWrap(True)
        self.status_log.setMinimumHeight(300)
        log_layout.addWidget(self.status_log)
        clear_log_btn = QPushButton("Clear Log")
        clear_log_btn.clicked.connect(self.clear_status_log)
        log_layout.addWidget(clear_log_btn)
        tab_widget.addTab(log_widget, "Status Log")

        layout.addWidget(tab_widget)

        # — 4. Real-time slider value updates —
        self.speed_slider.valueChanged.connect(
            lambda v: self.speed_val_lbl.setText(str(v)))
        self.torque_slider.valueChanged.connect(
            lambda v: self.torque_val_lbl.setText(str(v)))
        return panel

    def create_value_display_panel(self):
        """Create slider value display panel"""
        panel = QGroupBox("Joint Values")
        layout = QVBoxLayout(panel)
        
        # Set top/bottom margins to 20px
        layout.setContentsMargins(10, 20, 10, 20)
        
        self.value_display = QTextEdit()
        self.value_display.setObjectName("ValueDisplay")
        self.value_display.setReadOnly(True)  # Read-only, allows copy
        self.value_display.setMinimumHeight(60)  # Adjust minimum height
        self.value_display.setMaximumHeight(80)  # Limit maximum height
        self.value_display.setText("[]")
        
        layout.addWidget(self.value_display)
        
        return panel

    def on_slider_value_changed(self, index: int, value: int):
        """Slider value changed event handler"""
        if 0 <= index < len(self.slider_labels):
            joint_name = self.hand_config.joint_names[index]
            self.slider_labels[index].setText(f"{joint_name}: {value}")
            
        # Update value display
        self.update_value_display()

    def update_value_display(self):
        """Update value display panel contents"""
        # Get current values of all sliders
        values = [slider.value() for slider in self.sliders]
        
        # Format as list
        self.value_display.setText(f"{values}")

    def on_preset_action_clicked(self, positions: List[int]):
        """Preset action button click handler"""
        if len(positions) != len(self.sliders):
            QMessageBox.warning(
                self, "Action mismatch", 
                f"Preset action joint count ({len(positions)}) does not match current joint count ({len(self.sliders)})"
            )
            return
            
        # Update sliders
        for i, (slider, pos) in enumerate(zip(self.sliders, positions)):
            slider.setValue(pos)
            self.on_slider_value_changed(i, pos)
            
        # Publish joint state
        self.publish_joint_state()

    def on_home_clicked(self):
        """Return to home button click handler"""
        for slider, pos in zip(self.sliders, self.hand_config.init_pos):
            slider.setValue(pos)
            
        self.publish_joint_state()
        self.status_updated.emit("info", "Returned to home position")
        
        # Update value display
        self.update_value_display()

    def on_stop_clicked(self):
        """Stop all actions button click handler"""
        # Stop loop timer
        if self.cycle_timer and self.cycle_timer.isActive():
            self.cycle_timer.stop()
            self.cycle_timer = None
            self.cycle_button.setText("Cycle Run Preset Actions")
            self.reset_preset_buttons_color()
            
        self.status_updated.emit("warning", "Stopped all actions")

    def on_cycle_clicked(self):
        """Cycle run preset actions button click handler"""
        if not self.hand_config.preset_actions:
            QMessageBox.warning(self, "No Preset Actions", "Current hand model has no preset actions to cycle")
            return
            
        if self.cycle_timer and self.cycle_timer.isActive():
            # Stop cycling
            self.cycle_timer.stop()
            self.cycle_timer = None
            self.cycle_button.setText("Cycle Run Preset Actions")
            self.reset_preset_buttons_color()
            self.status_updated.emit("info", "Stopped cycling preset actions")
        else:
            # Start cycling
            self.current_action_index = -1  # Reset index
            self.cycle_timer = QTimer(self)
            self.cycle_timer.timeout.connect(self.run_next_action)
            self.cycle_timer.start(LOOP_TIME)  # 2-second interval
            self.cycle_button.setText("Stop Cycling")
            self.status_updated.emit("info", "Started cycling preset actions")
            self.run_next_action()  # Run the first action immediately

    def run_next_action(self):
        """Run the next preset action"""
        if not self.hand_config.preset_actions:
            return
            
        # Reset all button colors
        self.reset_preset_buttons_color()
        
        # Compute next action index
        self.current_action_index = (self.current_action_index + 1) % len(self.hand_config.preset_actions)
        
        # Get next action
        action_names = list(self.hand_config.preset_actions.keys())
        action_name = action_names[self.current_action_index]
        action_positions = self.hand_config.preset_actions[action_name]
        
        # Execute action
        self.on_preset_action_clicked(action_positions)
        
        # Highlight current action button
        if 0 <= self.current_action_index < len(self.preset_buttons):
            button = self.preset_buttons[self.current_action_index]
            button.setStyleSheet("background-color: green; color: white; border-color: #91D5FF;")
            
        self.status_updated.emit("info", f"Running preset action: {action_name}")

    def reset_preset_buttons_color(self):
        """Reset all preset button colors"""
        for button in self.preset_buttons:
            button.setStyleSheet("")  # Restore default style
            button.setProperty("category", "preset")  # Restore category property
            # Force style refresh
            button.style().unpolish(button)
            button.style().polish(button)

    def on_joint_type_changed(self, joint_type: str):
        """Joint type changed event handler"""
        self.hand_joint = joint_type
        self.hand_config = _HAND_CONFIGS[self.hand_joint]
        
        # Update hand info
        info_text = f"""Hand type: {self.hand_type}
Joint model: {self.hand_joint}
Joint count: {len(self.hand_config.joint_names)}
Publish rate: {self.ros_manager.hz} Hz"""
        self.hand_info_label.setText(info_text)
        
        # Recreate sliders and preset buttons
        self.create_joint_sliders()
        self.create_system_preset_buttons(self.sys_preset_layout)  # Assuming sys_preset_layout is a class variable
        
        # Update value display
        self.update_value_display()
        self.status_updated.emit("info", f"Switched to hand model: {joint_type}")

    def publish_joint_state(self):
        """Publish current joint state"""
        positions = [slider.value() for slider in self.sliders]
        self.ros_manager.publish_joint_state(positions)

    def update_status(self, status_type: str, message: str):
        """Update status display"""
        # Update connection status
        if status_type == "info" and "ROS2 node initialized successfully" in message:
            self.connection_status.setText("ROS2 node connected")
            self.connection_status.setObjectName("StatusLabel")
            self.connection_status.setObjectName("StatusInfo")
            
        # Update log
        current_time = time.strftime("%H:%M:%S")
        log_entry = f"[{current_time}] {message}\n"
        current_log = self.status_log.text()
        
        if len(current_log) > 10000:  # Limit log length
            current_log = current_log[-10000:]
            
        self.status_log.setText(log_entry + current_log)
        
        # Set log style
        self.status_log.setObjectName("StatusLabel")
        if status_type == "error":
            self.status_log.setObjectName("StatusError")
        else:
            self.status_log.setObjectName("StatusInfo")

    def clear_status_log(self):
        """Clear status log"""
        self.status_log.setText("Log cleared")
        self.status_log.setObjectName("StatusLabel")
        self.status_log.setObjectName("StatusInfo")

    def closeEvent(self, event):
        """Window close event handler"""
        if self.cycle_timer and self.cycle_timer.isActive():
            self.cycle_timer.stop()
        super().closeEvent(event)

def main(args=None):
    """Main function"""
    try:
        # Create ROS2 node manager
        ros_manager = ROS2NodeManager()
        
        # Create Qt application
        app = QApplication(sys.argv)
        
        # Create GUI
        window = HandControlGUI(ros_manager)
        
        # Connect status update signal
        ros_manager.status_updated.connect(window.update_status)
        window.status_updated = ros_manager.status_updated
        
        # Show window
        window.show()
        
        # Run application
        exit_code = app.exec_()
        
        # Clean up ROS2
        if rclpy.ok():
            ros_manager.node.destroy_node()
            rclpy.shutdown()
            
        sys.exit(exit_code)
    except Exception as e:
        print(f"Application failed to start: {str(e)}")
        sys.exit(1)

if __name__ == '__main__':
    main()
