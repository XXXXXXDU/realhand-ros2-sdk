#!/usr/bin/env python3
import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk
import threading
import time

from .views.left_widget import LeftControlWidget
from .views.right_widget import RightMatrixWidget

class HandMatrixGui(Node):
    """Hand matrix interface with refresh-rate control - Tkinter version"""
    
    def __init__(self):
        Node.__init__(self, 'raw_data_gui_node')

        # Declare parameters (with defaults)
        self.declare_parameter('hand_type', 'left')
        self.declare_parameter('hand_joint', 'L10')
        
        # Get parameter values
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value

        # Refresh control parameters
        self.refresh_interval = 200  # Default refresh interval 200 ms
        self.data_cache = {}  # Cache for incoming data

        # Create Tkinter app and window
        self.root = tk.Tk()
        self.root.title("ROS2 Finger Dot-Matrix Interface (Red Gradient)")
        self.root.geometry('1000x950')

        # Create central widget
        central_widget = ttk.Frame(self.root)
        central_widget.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create horizontal layout
        main_frame = ttk.Frame(central_widget)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Load left control panel - weight 2
        self.left_control = LeftControlWidget(main_frame, self.hand_type)
        self.left_control.set_subscribe_callback(self.toggle_subscription)
        self.left_control.set_refresh_callback(self.set_refresh_interval)
        self.left_control.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Load right matrix display - weight 1
        self.right_matrix = RightMatrixWidget(main_frame)
        self.right_matrix.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Initialize variables
        self.subscription = None
        self.is_subscribed = False
        
        # Create refresh timer
        self.refresh_timer_id = None
        self.start_refresh_timer()

        # Add initial messages
        self.add_message("Red-gradient interface started")
        self.add_message(f"Current refresh interval: {self.refresh_interval} ms")
        self.add_message("Select a topic and click 'Subscribe/Unsubscribe'")

        # Initialize with default dot matrices
        self.initialize_default_matrices()

    def initialize_default_matrices(self):
        """Initialize with default dot matrices (all dots gray)"""
        # Create empty default data (all zeros) for each finger
        default_data = [[0] * 6 for _ in range(12)]  # 12x6 all-zero matrix
        flattened_data = [item for sublist in default_data for item in sublist]
        
        finger_names = ["thumb_matrix", "index_matrix", "middle_matrix", "ring_matrix", "little_matrix"]
        for finger_name in finger_names:
            self.right_matrix.update_matrix_data(finger_name, flattened_data)

    def toggle_subscription(self):
        if self.is_subscribed:
            self.unsubscribe()
        else:
            self.subscribe()

    def subscribe(self):
        topic = self.left_control.get_selected_topic()
        self.add_message(f"Subscribing to topic: {topic}")
        self.subscription = self.create_subscription(
            String,
            topic,
            self.topic_callback,
            10
        )
        self.is_subscribed = True
        self.left_control.update_status(f"Status: Subscribed to {topic} | Red gradient", True)
        self.add_message(f"Subscribed to topic: {topic}")

    def unsubscribe(self):
        if self.subscription:
            self.destroy_subscription(self.subscription)
            self.subscription = None
            self.is_subscribed = False
            self.left_control.update_status("Status: Unsubscribed | Red gradient", False)
            self.add_message("Unsubscribed")
            # Restore default gray matrices after unsubscribing
            self.initialize_default_matrices()

    def topic_callback(self, msg):
        try:
            data = json.loads(msg.data)
            for finger_name, matrix_data in data.items():
                # Cache data; UI updates are driven by the timer
                self.data_cache[finger_name] = matrix_data
        except json.JSONDecodeError as e:
            self.add_message(f"JSON parse error: {str(e)}")
        except Exception as e:
            self.add_message(f"Data processing error: {str(e)}")

    def process_cached_data(self):
        """Periodically process cached data and update the UI"""
        if self.data_cache and self.is_subscribed:
            for finger_name, matrix_data in self.data_cache.items():
                # Update dot-matrix display
                self.right_matrix.update_matrix_data(finger_name, matrix_data)
                
                # Update terminal output
                display_name = {
                    "thumb_matrix": "Thumb",
                    "index_matrix": "Index",
                    "middle_matrix": "Middle",
                    "ring_matrix": "Ring",
                    "little_matrix": "Little"
                }.get(finger_name, finger_name)
                self.add_message(f"Updated {display_name} matrix data: {matrix_data}")
            
            # Clear cache
            self.data_cache.clear()
            
        # Continue the timer
        self.start_refresh_timer()

    def start_refresh_timer(self):
        """Start the refresh timer"""
        if self.refresh_timer_id:
            self.root.after_cancel(self.refresh_timer_id)
        self.refresh_timer_id = self.root.after(self.refresh_interval, self.process_cached_data)

    def set_refresh_interval(self, interval=200):
        if interval < 200:
            self.add_message("Refresh interval cannot be less than 200 ms")
            return
        self.refresh_interval = interval
        self.add_message(f"Refresh interval updated to {interval} ms")
        self.start_refresh_timer()

    def add_message(self, text):
        """Add a message to the terminal"""
        self.left_control.add_terminal_message(text)

    def run(self):
        """Run the GUI"""
        try:
            # Start ROS2 spin thread
            spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
            spin_thread.start()
            
            self.root.mainloop()
        finally:
            # Clean up timer
            if self.refresh_timer_id:
                self.root.after_cancel(self.refresh_timer_id)
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    try:
        gui_node = HandMatrixGui()
        gui_node.run()
    except Exception as e:
        print(f"Application failed to start: {str(e)}")
        rclpy.shutdown()

if __name__ == "__main__":
    main()
