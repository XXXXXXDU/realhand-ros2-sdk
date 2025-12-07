import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PyQt5 import QtWidgets, QtCore
import numpy as np
import json
import sys
from typing import Dict, List

from RealHand.utils.init_real_hand import InitRealHand

class ForceGroupWindow(QtWidgets.QMainWindow):
    """Visualization window for dedicated force sensor group"""
    def __init__(self, group_id: int):
        super().__init__()
        self.setWindowTitle(f"Force Sensor Group {group_id+1}")
        self.setGeometry(100 + group_id*50, 100 + group_id*50, 800, 400)
        
        # Graphic settings
        self.canvas = FigureCanvasQTAgg(Figure(figsize=(8, 4)))
        self.setCentralWidget(self.canvas)
        self.ax = self.canvas.figure.add_subplot(111)
        self.ax.set_title(f'Force Group {group_id+1} (5 channels)')
        self.ax.set_xlabel('Time Step')
        self.ax.set_ylabel('Force (N)')
        self.ax.grid(True)
        
        # data storage
        self.buffer_size = 200
        self.x_data = np.arange(self.buffer_size)
        #self.channels = [f'Channel {i+1}' for i in range(5)]
        self.channels = ["thumb","index finger","middle finger","ring finger","little finger"]
        self.data = {name: np.full(self.buffer_size, np.nan) for name in self.channels}
        self.lines = {}
        self.data_ptr = 0
        
        # Color settings
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
        
        # Create curve
        for i, name in enumerate(self.channels):
            self.lines[name], = self.ax.plot(
                self.x_data,
                self.data[name],
                color=colors[i],
                label=name,
                linewidth=1.5
            )
        
        self.ax.legend(loc='upper right')
        self.ax.set_xlim(0, self.buffer_size)
        self.ax.set_ylim(0, 300)  # Assuming the force sensor range is 0-300N
        
        # Regular refresh
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)  # 20fps
    
    def add_data(self, new_data: List[float]):
        """Add new data points"""
        self.data_ptr = (self.data_ptr + 1) % self.buffer_size
        for name, val in zip(self.channels, new_data):
            self.data[name][self.data_ptr] = float(val)
    
    def update_plot(self):
        """Update drawing"""
        # Update curve data
        for name, line in self.lines.items():
            line.set_ydata(np.roll(self.data[name], -self.data_ptr))
        
        self.canvas.draw()

class HandMonitor(Node):
    def __init__(self):
        super().__init__('graphic_display')
        
        # Initialize a Qt application
        self.app = QtWidgets.QApplication(sys.argv)
        
        # window management
        self.force_windows = {}  # Storage Force Group Window {group_id: window}
        self.temp_window = None  # temperature window
        self.torque_window = None  # Torque window
        self.hand_joint, self.hand_type = InitRealHand().current_hand()
        if self.hand_type == "left":
            self.topic = "/cb_left_hand_info"
        else:
            self.topic = "/cb_right_hand_info"
            # self.topic = "/cb_left_hand_info"
        # ROS2 subscription
        self.subscription = self.create_subscription(
            String,
            self.topic,
            self.data_callback,
            10)
        
        # Qt event handling timer
        self.timer = self.create_timer(0.1, self.process_qt_events)
        
        self.get_logger().info("Hand monitor initialized")
    
    def data_callback(self, msg: String):
        """Handling hand data callbacks"""
        try:
            data = json.loads(msg.data)
            if self.hand_type == "left":
                tmp = "left_hand"
            else:
                tmp = "right_hand"
            if isinstance(data, dict) and tmp in data:
                hand_data = data[tmp]
                
                # Process force data (each force group has its own window).
                if 'force' in hand_data:
                    force_data = hand_data['force']
                    for group_id, group_values in enumerate(force_data):
                        if len(group_values) == 5:  # Each group should have 5 values.
                            if group_id not in self.force_windows:
                                self.force_windows[group_id] = ForceGroupWindow(group_id)
                                self.force_windows[group_id].show()
                            
                            # Cross-thread safe update
                            if QtCore.QThread.currentThread() == self.app.thread():
                                self.force_windows[group_id].add_data(group_values)
                            else:
                                QtCore.QMetaObject.invokeMethod(
                                    self.force_windows[group_id],
                                    'add_data',
                                    QtCore.Qt.QueuedConnection,
                                    QtCore.Q_ARG(list, group_values)
                                )
                
                # Processing temperature data (single window)
                if 'motor_temperature' in hand_data:
                    temp_data = hand_data['motor_temperature']
                    if len(temp_data) == 10:  # There should be 10 temperature values.
                        if self.temp_window is None:
                            self.create_temp_window()
                        self.update_window_data(self.temp_window, temp_data)
                
                # Processing torque data (single window)
                # if 'torque' in hand_data:
                #     torque_data = hand_data['torque']
                #     if len(torque_data) == 5:  # There should be 5 torque values.
                #         if self.torque_window is None:
                #             self.create_torque_window()
                #         self.update_window_data(self.torque_window, torque_data)
        
        except Exception as e:
            self.get_logger().error(f"Data processing error: {str(e)}")
    
    def create_temp_window(self):
        """Create a temperature window"""
        self.temp_window = DataPlotWindow(
            title="Motor Temperatures",
            ylabel="Temperature (°C)",
            channel_count=10,
            y_range=(20, 50)
        )
        self.temp_window.show()
    
    def create_torque_window(self):
        """Create a torque window"""
        self.torque_window = DataPlotWindow(
            title="Joint Torque",
            ylabel="Torque (Nm)",
            channel_count=5,
            y_range=(-0.5, 0.5)
        )
        self.torque_window.show()
    
    def update_window_data(self, window, data):
        """General window data update"""
        if QtCore.QThread.currentThread() == self.app.thread():
            window.add_data(data)
        else:
            QtCore.QMetaObject.invokeMethod(
                window,
                'add_data',
                QtCore.Qt.QueuedConnection,
                QtCore.Q_ARG(list, data)
            )
    
    def process_qt_events(self):
        """Handling the Qt event loop"""
        self.app.processEvents()
        
        # Clear closed windows
        self.force_windows = {k: v for k, v in self.force_windows.items() if v.isVisible()}
        if self.temp_window and not self.temp_window.isVisible():
            self.temp_window = None
        if self.torque_window and not self.torque_window.isVisible():
            self.torque_window = None
    
    def run(self):
        """Launch the Qt application"""
        self.app.exec_()
    
    def destroy_node(self):
        """Clean up resources"""
        for window in self.force_windows.values():
            window.close()
        if self.temp_window:
            self.temp_window.close()
        if self.torque_window:
            self.torque_window.close()
        super().destroy_node()

class DataPlotWindow(QtWidgets.QMainWindow):
    """General Data Plotting Window"""
    def __init__(self, title: str, ylabel: str, channel_count: int, y_range: tuple):
        super().__init__()
        self.setWindowTitle(title)
        self.setGeometry(100, 100, 800, 400)
        
        # Graphic settings
        self.canvas = FigureCanvasQTAgg(Figure(figsize=(8, 4)))
        self.setCentralWidget(self.canvas)
        self.ax = self.canvas.figure.add_subplot(111)
        self.ax.set_title(title)
        self.ax.set_xlabel('Time Step')
        self.ax.set_ylabel(ylabel)
        self.ax.grid(True)
        
        # data storage
        self.buffer_size = 200
        self.x_data = np.arange(self.buffer_size)
        self.channels = [f'Channel {i+1}' for i in range(channel_count)]
        #self.channels = ["thumb","index finger","middle finger","ring finger","little finger"]
        self.data = {name: np.full(self.buffer_size, np.nan) for name in self.channels}
        self.lines = {}
        self.data_ptr = 0
        
        # Create curve
        colors = matplotlib.colormaps['tab20'].colors
        for i, name in enumerate(self.channels):
            self.lines[name], = self.ax.plot(
                self.x_data,
                self.data[name],
                color=colors[i % len(colors)],
                label=name,
                linewidth=1.5
            )
        
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        self.ax.set_xlim(0, self.buffer_size)
        self.ax.set_ylim(*y_range)
        
        # Regular refresh
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)
    
    def add_data(self, new_data: List[float]):
        """Add new data points"""
        self.data_ptr = (self.data_ptr + 1) % self.buffer_size
        for name, val in zip(self.channels, new_data):
            self.data[name][self.data_ptr] = float(val)
    
    def update_plot(self):
        """Update drawing"""
        for name, line in self.lines.items():
            line.set_ydata(np.roll(self.data[name], -self.data_ptr))
        self.canvas.draw()

def main(args=None):
    rclpy.init(args=args)
    
    # Nodes must be created on the main thread.
    monitor = HandMonitor()
    
    # Start Qt thread
    from threading import Thread
    qt_thread = Thread(target=monitor.run, daemon=True)
    qt_thread.start()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


