import tkinter as tk
from tkinter import ttk

class DotMatrixWidget(tk.Canvas):
    """Dot-matrix display widget - white to dark red gradient"""
    
    def __init__(self, parent, rows=12, cols=6):
        self.rows = rows
        self.cols = cols
        self.dot_size = 8
        self.spacing = 4
        
        # Calculate canvas size
        width = cols * (self.dot_size + self.spacing) + self.spacing
        height = rows * (self.dot_size + self.spacing) + self.spacing
        
        super().__init__(parent, width=width, height=height, 
                        bg='white', highlightthickness=1, 
                        highlightbackground='#cccccc')
        
        self.data = None
        # Show default matrix on initialization
        self.draw_default_matrix()
        
    def draw_default_matrix(self):
        """Draw default matrix (all dots in gray)"""
        self.delete("all")
        for row in range(self.rows):
            for col in range(self.cols):
                x = self.spacing + col * (self.dot_size + self.spacing)
                y = self.spacing + row * (self.dot_size + self.spacing)
                # Default gray dots
                self.create_oval(x, y, x + self.dot_size, y + self.dot_size, 
                                fill='#c8c8c8', outline='#666666', width=1)
        
    def set_data(self, data):
        """Set dot-matrix data"""
        self.data = data
        self.draw_matrix()
        
    def draw_matrix(self):
        """Draw matrix - white to dark red gradient"""
        self.delete("all")
        
        for row in range(self.rows):
            for col in range(self.cols):
                x = self.spacing + col * (self.dot_size + self.spacing)
                y = self.spacing + row * (self.dot_size + self.spacing)
                
                if self.data and isinstance(self.data, list) and len(self.data) > row * self.cols + col:
                    try:
                        value = self.data[row * self.cols + col]
                        if value > 0:
                            # White to dark red gradient logic
                            # Assume value range is 0-255; larger values = deeper red
                            intensity = min(255, max(0, int(value)))
                            
                            if intensity < 128:
                                # Gradient from white to light red
                                # White (255,255,255) -> Light red (255,200,200)
                                red = 255
                                green = 255 - (intensity * 55 // 128)
                                blue = 255 - (intensity * 55 // 128)
                                color = f'#{red:02x}{green:02x}{blue:02x}'
                            else:
                                # Gradient from light red to dark red
                                # Light red (255,200,200) -> Dark red (255,0,0)
                                red = 255
                                green = 200 - ((intensity - 128) * 200 // 127)
                                blue = 200 - ((intensity - 128) * 200 // 127)
                                color = f'#{red:02x}{green:02x}{blue:02x}'
                        else:
                            color = '#c8c8c8'  # Default gray
                    except:
                        color = '#c8c8c8'  # Default gray
                else:
                    color = '#c8c8c8'  # Default gray
                
                # Draw dot
                self.create_oval(x, y, x + self.dot_size, y + self.dot_size, 
                                fill=color, outline='#666666', width=1)

class RightMatrixWidget(ttk.Frame):
    """Right-side matrix display widget - Tkinter version"""
    
    def __init__(self, parent):
        super().__init__(parent)
        self.finger_matrices = {}
        self.init_ui()
        
    def init_ui(self):
        """Initialize right-side matrix UI"""
        main_layout = ttk.Frame(self)
        main_layout.pack(fill=tk.BOTH, expand=True)
        
        # Dot-matrices for five fingers
        finger_names = ["thumb_matrix", "index_matrix", "middle_matrix", "ring_matrix", "little_matrix"]
        display_names = ["Thumb", "Index", "Middle", "Ring", "Little"]
        
        for finger_name, display_name in zip(finger_names, display_names):
            # Create finger layout
            finger_frame = ttk.Frame(main_layout)
            finger_frame.pack(fill=tk.X, pady=2)
            
            # Finger label
            ttk.Label(finger_frame, text=display_name).pack()
            
            # Create dot-matrix widget - show default gray matrix on init
            matrix = DotMatrixWidget(finger_frame)
            matrix.pack(pady=2)
            
            self.finger_matrices[finger_name] = matrix
            
    def update_matrix_data(self, finger_name, data):
        """Update dot-matrix data for a specific finger"""
        if finger_name in self.finger_matrices:
            # Flatten data if it's a 2D list
            if data and isinstance(data[0], list):
                flattened = [item for sublist in data for item in sublist]
            else:
                flattened = data
                
            self.finger_matrices[finger_name].set_data(flattened)
