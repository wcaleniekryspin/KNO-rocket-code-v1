import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import math
from collections import deque
from datetime import datetime

# --- fields / parser (unchanged semantics) ---
FIELDS = [
    ("packetTime", 16, 22),
    ("error", None, 8),
    ("status", None, 3),
    ("gpsLat", None, 17),
    ("gpsLng", None, 17),
    ("gpsAlti", None, 14),
    ("gpsHour", None, 5),
    ("gpsMin", None, 6),
    ("gpsSec", None, 6),
    ("gpsCentisec", None, 7),
    ("gpsSpeed", None, 10),
    ("gpsCourse", None, 9),
    ("gpsSatNum", None, 3),
    ("gpsHdop", None, 5),
    ("lsmAccelX", None, 15),
    ("lsmAccelY", None, 15),
    ("lsmAccelZ", None, 15),
    ("lsmGyroX", None, 10),
    ("lsmGyroY", None, 10),
    ("lsmGyroZ", None, 10),
    ("lsmTemp", None, 8),
    ("lsmSpeed", None, 10),
    ("bmpTemp", None, 8),
    ("bmpPress", None, 14),
    ("bmpAlti", None, 14),
    ("bmpSpeed", None, 10),
    ("battery", None, 7),
]


def parse_bits(binary_string):
    if not binary_string.startswith("1111111101100110"):
        raise ValueError("Błędny nagłówek! Dane nie zaczynają się od 0xFF66")
    data = {}
    pos = 16
    for name, start, length in FIELDS:
        if start is None:
            start = pos
        bits = binary_string[start:start + length]
        if len(bits) < length:
            raise ValueError(f"Za mało bitów dla pola {name}")
        data[name] = int(bits, 2)
        pos = start + length
    return data


def signed(value, bits):
    if value & (1 << (bits - 1)):
        return value - (1 << bits)
    return value


class GraphCanvas:
    """Custom graph widget using Tkinter Canvas"""

    def __init__(self, parent, title="Graph", width=300, height=200,
                 y_min=0, y_max=100, line_color="blue", bg_color="white"):
        self.canvas = tk.Canvas(parent, width=width, height=height,
                                bg=bg_color, highlightthickness=1,
                                highlightbackground="gray")

        self.width = width
        self.height = height
        self.title = title
        self.y_min = y_min
        self.y_max = y_max
        self.line_color = line_color
        self.bg_color = bg_color

        self.data = deque(maxlen=50)  # Store last 50 points

        # Draw initial graph
        self.draw_graph()

    def draw_graph(self):
        """Draw the graph axes and title"""
        self.canvas.delete("all")

        # Draw border
        self.canvas.create_rectangle(2, 2, self.width - 2, self.height - 2,
                                     outline="black", width=1)

        # Draw title
        self.canvas.create_text(self.width // 2, 15, text=self.title,
                                font=('Arial', 10, 'bold'))

        # Draw Y-axis labels
        y_range = self.y_max - self.y_min
        if y_range > 0:
            for i in range(5):  # 5 horizontal grid lines
                y = self.height - 30 - (i * (self.height - 60) // 4)
                value = self.y_min + (i * y_range / 4)
                self.canvas.create_text(30, y, text=f"{value:.0f}",
                                        font=('Arial', 8), anchor=tk.E)

                # Grid line
                self.canvas.create_line(40, y, self.width - 10, y,
                                        fill="lightgray", dash=(2, 2))

        # Draw X-axis label
        self.canvas.create_text(self.width - 20, self.height - 10, text="→",
                                font=('Arial', 10))

    def add_point(self, value):
        """Add a data point and update the graph"""
        self.data.append(value)
        self.update_graph()

    def update_graph(self):
        """Update the graph with current data"""
        if len(self.data) < 2:
            return

        # Keep background elements
        self.draw_graph()

        # Calculate scaling
        y_range = self.y_max - self.y_min
        if y_range <= 0:
            y_range = 1

        # Plot data points
        points = []
        for i, value in enumerate(self.data):
            # Normalize value to graph coordinates
            x = 40 + (i * (self.width - 50) / max(1, len(self.data) - 1))
            y = self.height - 30 - ((value - self.y_min) / y_range * (self.height - 60))

            # Clamp to graph bounds
            y = max(30, min(self.height - 30, y))

            points.extend([x, y])

        # Draw line
        if len(points) >= 4:
            self.canvas.create_line(points, fill=self.line_color, width=2, smooth=True)

        # Draw data points as small circles
        for i in range(0, len(points), 2):
            x, y = points[i], points[i + 1]
            self.canvas.create_oval(x - 2, y - 2, x + 2, y + 2,
                                    fill=self.line_color, outline=self.line_color)

        # Show current value
        if self.data:
            current = self.data[-1]
            self.canvas.create_text(self.width // 2, self.height - 20,
                                    text=f"Current: {current:.1f}",
                                    font=('Arial', 9, 'bold'))


class RocketMonitorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RT Monitor - Tkinter Canvas")
        self.root.geometry("1400x1000")

        # Variables
        self.ser = None
        self.running = True
        self.simulation_mode = False
        self.data_counter = 0
        self.RAW_BATT_MAX = 2 ** 7 - 1

        # Data storage
        self.max_points = 50
        self.accel_x = deque(maxlen=self.max_points)
        self.accel_y = deque(maxlen=self.max_points)
        self.accel_z = deque(maxlen=self.max_points)
        self.lsm_temp = deque(maxlen=self.max_points)
        self.bmp_temp = deque(maxlen=self.max_points)
        self.bmp_press = deque(maxlen=self.max_points)
        self.bmp_alti = deque(maxlen=self.max_points)
        self.battery = deque(maxlen=self.max_points)

        # Setup GUI
        self.setup_gui()

        # Start serial connection
        self.connect_serial()

        # Start update loop
        self.update_data()

    def setup_gui(self):
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Top status bar
        self.setup_status_bar(main_frame)

        # Center area with plots
        self.setup_plots_area(main_frame)

        # Command section (BETWEEN PLOTS AND LOGS)
        self.setup_command_section(main_frame)

        # Bottom log area
        self.setup_log_area(main_frame)

        # Controls frame
        self.setup_controls(main_frame)

    def setup_status_bar(self, parent):
        status_frame = ttk.LabelFrame(parent, text="Status", padding=10)
        status_frame.pack(fill=tk.X, padx=5, pady=5)

        self.status_label = ttk.Label(
            status_frame,
            text="Status: - | Error: - | Battery: -% | Mode: Not Connected",
            font=('Arial', 10, 'bold')
        )
        self.status_label.pack(side=tk.LEFT)

        self.connection_label = ttk.Label(
            status_frame,
            text="Disconnected",
            foreground="red",
            font=('Arial', 10, 'bold')
        )
        self.connection_label.pack(side=tk.RIGHT)

    def setup_plots_area(self, parent):
        plots_frame = ttk.Frame(parent)
        plots_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Left plots (2x2 grid)
        left_frame = ttk.Frame(plots_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Create custom graph widgets
        self.setup_custom_graphs(left_frame)

        # Right rocket visualization and sensor values
        right_frame = ttk.LabelFrame(plots_frame, text="Rocket & Sensors", padding=10)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False, padx=5)

        # Rocket visualization
        self.setup_rocket_canvas(right_frame)

        # Current sensor values display
        self.setup_sensor_values(right_frame)

    def setup_custom_graphs(self, parent):
        """Setup custom graph widgets using Tkinter Canvas"""
        # Create a 2x2 grid for graphs
        grid_frame = ttk.Frame(parent)
        grid_frame.pack(fill=tk.BOTH, expand=True)

        # Row 1
        row1 = ttk.Frame(grid_frame)
        row1.pack(fill=tk.BOTH, expand=True, pady=5)

        # Acceleration graph
        self.accel_graph = GraphCanvas(
            row1,
            title="Acceleration (X, Y, Z)",
            width=350,
            height=200,
            y_min=-1000,
            y_max=1000,
            line_color="blue"
        )
        self.accel_graph.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        # Temperature graph
        self.temp_graph = GraphCanvas(
            row1,
            title="Temperature (LSM, BMP)",
            width=350,
            height=200,
            y_min=0,
            y_max=50,
            line_color="red"
        )
        self.temp_graph.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        # Row 2
        row2 = ttk.Frame(grid_frame)
        row2.pack(fill=tk.BOTH, expand=True, pady=5)

        # Pressure graph
        self.press_graph = GraphCanvas(
            row2,
            title="Pressure",
            width=350,
            height=200,
            y_min=90000,
            y_max=103000,
            line_color="green"
        )
        self.press_graph.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

        # Battery graph
        self.batt_graph = GraphCanvas(
            row2,
            title="Battery (%)",
            width=350,
            height=200,
            y_min=0,
            y_max=100,
            line_color="purple"
        )
        self.batt_graph.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)

    def setup_rocket_canvas(self, parent):
        """Setup rocket visualization canvas"""
        rocket_frame = ttk.LabelFrame(parent, text="Rocket Orientation", padding=5)
        rocket_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # Canvas for rocket
        self.rocket_canvas = tk.Canvas(rocket_frame, width=300, height=250,
                                       bg='white', highlightthickness=1,
                                       highlightbackground="black")
        self.rocket_canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Rocket orientation text
        self.rocket_text = ttk.Label(
            rocket_frame,
            text="Roll: 0.0°  Pitch: 0.0°",
            font=('Arial', 10, 'bold')
        )
        self.rocket_text.pack(pady=5)

        # Draw initial rocket
        self.draw_rocket(0, 0)

    def setup_sensor_values(self, parent):
        """Display current sensor values"""
        values_frame = ttk.LabelFrame(parent, text="Current Values", padding=10)
        values_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # Create grid for sensor values
        self.value_labels = {}

        sensors = [
            ("Accel X:", "accel_x", "red"),
            ("Accel Y:", "accel_y", "green"),
            ("Accel Z:", "accel_z", "blue"),
            ("LSM Temp:", "lsm_temp", "magenta"),
            ("BMP Temp:", "bmp_temp", "cyan"),
            ("BMP Press:", "bmp_press", "orange"),
            ("BMP Alti:", "bmp_alti", "black"),
            ("Battery:", "battery", "purple"),
            ("Status:", "status_value", "red"),
            ("Error:", "error_value", "red"),
        ]

        for i, (label_text, key, color) in enumerate(sensors):
            frame = ttk.Frame(values_frame)
            frame.grid(row=i // 2, column=i % 2, sticky=tk.W + tk.E, padx=10, pady=3)

            ttk.Label(frame, text=label_text, width=12, anchor=tk.W).pack(side=tk.LEFT)

            value_label = ttk.Label(
                frame,
                text="-",
                width=10,
                anchor=tk.E,
                font=('Arial', 9, 'bold'),
                foreground=color
            )
            value_label.pack(side=tk.RIGHT)

            self.value_labels[key] = value_label

    def setup_command_section(self, parent):
        """Setup command sending section (between plots and logs)"""
        command_frame = ttk.LabelFrame(parent, text="Send Commands to Device", padding=15)
        command_frame.pack(fill=tk.X, padx=5, pady=10)

        # Main horizontal layout
        main_horizontal = ttk.Frame(command_frame)
        main_horizontal.pack(fill=tk.X, expand=True)

        # Left side - Command input
        left_frame = ttk.Frame(main_horizontal)
        left_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 20))

        # Command entry
        entry_frame = ttk.Frame(left_frame)
        entry_frame.pack(fill=tk.X, pady=5)

        ttk.Label(entry_frame, text="Command:", font=('Arial', 10, 'bold')).pack(side=tk.LEFT, padx=5)

        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(
            entry_frame,
            textvariable=self.command_var,
            width=40,
            font=('Arial', 10),
            state=tk.DISABLED
        )
        self.command_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        self.send_btn = ttk.Button(
            entry_frame,
            text="Send Command",
            command=self.send_command,
            state=tk.DISABLED,
            width=15
        )
        self.send_btn.pack(side=tk.LEFT, padx=5)

        # Bind Enter key to send command
        self.command_entry.bind('<Return>', lambda e: self.send_command())

        # Command history or info
        info_frame = ttk.Frame(left_frame)
        info_frame.pack(fill=tk.X, pady=5)

        ttk.Label(
            info_frame,
            text="Enter command and press Send or Enter. Commands are sent with newline.",
            font=('Arial', 8),
            foreground="gray"
        ).pack(side=tk.LEFT)

        # Right side - Predefined commands
        right_frame = ttk.LabelFrame(main_horizontal, text="Quick Commands", padding=10)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(20, 0))

        # Create 2 columns of quick command buttons
        quick_commands = [
            ("LED ON", "LED_ON"),
            ("LED OFF", "LED_OFF"),
            ("BEEP", "BEEP"),
            ("STATUS", "STATUS"),
            ("CALIBRATE", "CALIBRATE"),
            ("RESET", "RESET"),
            ("GET DATA", "GET_DATA"),
            ("SAVE CFG", "SAVE_CONFIG"),
            ("READ CFG", "READ_CONFIG"),
            ("TEST MODE", "TEST_MODE"),
        ]

        # First column
        col1_frame = ttk.Frame(right_frame)
        col1_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)

        # Second column
        col2_frame = ttk.Frame(right_frame)
        col2_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5)

        # Distribute buttons between columns
        for i, (text, cmd) in enumerate(quick_commands):
            if i < len(quick_commands) // 2:
                parent_frame = col1_frame
            else:
                parent_frame = col2_frame

            btn = ttk.Button(
                parent_frame,
                text=text,
                command=lambda c=cmd: self.send_quick_command(c),
                width=12
            )
            btn.pack(pady=2, fill=tk.X)

    def send_quick_command(self, command):
        """Send a predefined quick command"""
        self.command_var.set(command)
        self.send_command()

    def draw_rocket(self, roll_deg, pitch_deg):
        """Draw rocket on canvas with orientation"""
        canvas = self.rocket_canvas
        canvas.delete("all")

        width = canvas.winfo_width()
        height = canvas.winfo_height()

        if width <= 1 or height <= 1:
            return

        # Center of canvas
        center_x = width // 2
        center_y = height // 2

        # Rocket dimensions
        rocket_height = min(width, height) * 0.4
        rocket_width = rocket_height * 0.2

        # Calculate rotation
        roll_rad = math.radians(roll_deg)

        # Rocket body (simplified)
        # Nose cone
        nose_x = center_x
        nose_y = center_y - rocket_height / 2

        # Body
        body_top_y = center_y - rocket_height / 4
        body_bottom_y = center_y + rocket_height / 4

        # Fins
        fin_length = rocket_width * 1.5
        fin_height = rocket_height * 0.15

        # Draw rocket body (main cylinder)
        canvas.create_oval(
            center_x - rocket_width / 2, body_top_y,
            center_x + rocket_width / 2, body_bottom_y,
            fill="lightgray", outline="black", width=2
        )

        # Draw nose cone
        canvas.create_polygon(
            nose_x, nose_y,
            center_x + rocket_width / 2, body_top_y,
            center_x - rocket_width / 2, body_top_y,
            fill="darkgray", outline="black", width=2
        )

        # Draw fins (4 fins)
        fin_angles = [0, 90, 180, 270]
        for angle in fin_angles:
            angle_rad = math.radians(angle + roll_deg)

            # Fin position at bottom of rocket
            fin_x = center_x + math.cos(angle_rad) * rocket_width / 2
            fin_y = center_y + rocket_height / 4

            # Fin tip
            tip_x = fin_x + math.cos(angle_rad) * fin_length
            tip_y = fin_y + fin_height

            # Fin base (wider at bottom)
            base_x1 = fin_x - math.sin(angle_rad) * rocket_width / 4
            base_y1 = fin_y
            base_x2 = fin_x + math.sin(angle_rad) * rocket_width / 4
            base_y2 = fin_y

            canvas.create_polygon(
                base_x1, base_y1,
                base_x2, base_y2,
                tip_x, tip_y,
                fill="gray", outline="black", width=1
            )

        # Draw orientation indicators
        # Roll indicator
        roll_x = center_x + math.cos(roll_rad) * rocket_width
        roll_y = center_y + math.sin(roll_rad) * rocket_width

        canvas.create_line(
            center_x, center_y,
            roll_x, roll_y,
            fill="red", width=2, arrow=tk.LAST
        )

        # Pitch indicator (simplified - vertical line)
        pitch_y = center_y - pitch_deg * 2  # Scale for visualization

        canvas.create_line(
            center_x - rocket_width, pitch_y,
            center_x + rocket_width, pitch_y,
            fill="blue", width=2, dash=(3, 3)
        )

        # Draw center crosshair
        canvas.create_line(
            center_x - 10, center_y,
            center_x + 10, center_y,
            fill="green", width=1
        )
        canvas.create_line(
            center_x, center_y - 10,
            center_x, center_y + 10,
            fill="green", width=1
        )

        # Update orientation text
        self.rocket_text.config(text=f"Roll: {roll_deg:.1f}°  Pitch: {pitch_deg:.1f}°")

    def setup_log_area(self, parent):
        log_frame = ttk.LabelFrame(parent, text="Log", padding=10)
        log_frame.pack(fill=tk.X, padx=5, pady=5)

        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=4,
            width=80,
            font=('Courier', 9)
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)

        # Configure tag for colored messages
        self.log_text.tag_config('error', foreground='red')
        self.log_text.tag_config('warning', foreground='orange')
        self.log_text.tag_config('success', foreground='green')
        self.log_text.tag_config('sent', foreground='blue')
        self.log_text.tag_config('received', foreground='purple')

    def setup_controls(self, parent):
        controls_frame = ttk.Frame(parent)
        controls_frame.pack(fill=tk.X, padx=5, pady=5)

        # Port selection
        ttk.Label(controls_frame, text="Port:").pack(side=tk.LEFT, padx=5)
        self.port_var = tk.StringVar(value="COM3")
        self.port_combo = ttk.Combobox(
            controls_frame,
            textvariable=self.port_var,
            values=["COM3", "COM4", "COM5", "COM6", "/dev/ttyUSB0", "/dev/ttyACM0"],
            width=15,
            state='readonly'
        )
        self.port_combo.pack(side=tk.LEFT, padx=5)

        # Baud rate selection
        ttk.Label(controls_frame, text="Baud:").pack(side=tk.LEFT, padx=5)
        self.baud_var = tk.StringVar(value="9600")
        self.baud_combo = ttk.Combobox(
            controls_frame,
            textvariable=self.baud_var,
            values=["9600", "19200", "38400", "57600", "115200"],
            width=10,
            state='readonly'
        )
        self.baud_combo.pack(side=tk.LEFT, padx=5)

        # Connect button
        self.connect_btn = ttk.Button(
            controls_frame,
            text="Connect",
            command=self.toggle_connection
        )
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        # Simulation mode checkbox
        self.sim_var = tk.BooleanVar(value=False)
        self.sim_check = ttk.Checkbutton(
            controls_frame,
            text="Simulation Mode",
            variable=self.sim_var,
            command=self.toggle_simulation
        )
        self.sim_check.pack(side=tk.LEFT, padx=20)

        # Clear graphs button
        ttk.Button(
            controls_frame,
            text="Clear Graphs",
            command=self.clear_graphs
        ).pack(side=tk.LEFT, padx=5)

        # Clear log button
        ttk.Button(
            controls_frame,
            text="Clear Log",
            command=self.clear_log
        ).pack(side=tk.RIGHT, padx=5)

        # Quit button
        ttk.Button(
            controls_frame,
            text="Quit",
            command=self.quit_app
        ).pack(side=tk.RIGHT, padx=5)

    def log(self, message, level="info"):
        """Add message to log with timestamp"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"

        self.log_text.insert(tk.END, log_entry)

        # Apply tag based on level
        if level == "error":
            self.log_text.tag_add("error", "end-2c linestart", "end-1c")
        elif level == "warning":
            self.log_text.tag_add("warning", "end-2c linestart", "end-1c")
        elif level == "success":
            self.log_text.tag_add("success", "end-2c linestart", "end-1c")
        elif level == "sent":
            self.log_text.tag_add("sent", "end-2c linestart", "end-1c")
        elif level == "received":
            self.log_text.tag_add("received", "end-2c linestart", "end-1c")

        # Auto-scroll
        self.log_text.see(tk.END)

        # Limit log size
        lines = self.log_text.get("1.0", tk.END).split('\n')
        if len(lines) > 100:
            self.log_text.delete("1.0", f"{len(lines) - 100}.0")

    def clear_log(self):
        """Clear the log window"""
        self.log_text.delete("1.0", tk.END)

    def clear_graphs(self):
        """Clear all graph data"""
        self.accel_x.clear()
        self.accel_y.clear()
        self.accel_z.clear()
        self.lsm_temp.clear()
        self.bmp_temp.clear()
        self.bmp_press.clear()
        self.bmp_alti.clear()
        self.battery.clear()

        # Clear graph displays
        if hasattr(self, 'accel_graph'):
            self.accel_graph.data.clear()
            self.accel_graph.draw_graph()

        if hasattr(self, 'temp_graph'):
            self.temp_graph.data.clear()
            self.temp_graph.draw_graph()

        if hasattr(self, 'press_graph'):
            self.press_graph.data.clear()
            self.press_graph.draw_graph()

        if hasattr(self, 'batt_graph'):
            self.batt_graph.data.clear()
            self.batt_graph.draw_graph()

        self.log("Graphs cleared", "warning")

    def connect_serial(self):
        """Try to connect to serial port"""
        if self.sim_var.get():
            self.simulation_mode = True
            self.log("Simulation mode enabled", "warning")
            self.connection_label.config(text="Simulation", foreground="orange")
            self.status_label.config(text="Status: - | Error: - | Battery: -% | Mode: Simulation")
            self.enable_command_sending(False)
            return

        port = self.port_var.get()
        baud = int(self.baud_var.get())

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()

            self.ser = serial.Serial(port, baud, timeout=1)
            self.simulation_mode = False
            self.log(f"Connected to {port} at {baud} baud", "success")
            self.connection_label.config(text=f"Connected: {port}", foreground="green")
            self.status_label.config(text="Status: - | Error: - | Battery: -% | Mode: Serial")
            self.enable_command_sending(True)

        except Exception as e:
            self.log(f"Failed to connect to {port}: {e}", "error")
            self.connection_label.config(text="Disconnected", foreground="red")
            self.simulation_mode = True
            self.enable_command_sending(False)

    def enable_command_sending(self, enabled=True):
        """Enable or disable command sending controls"""
        state = tk.NORMAL if enabled else tk.DISABLED
        self.command_entry.config(state=state)
        self.send_btn.config(state=state)

        # Update button text color
        if enabled:
            self.send_btn.config(style="Accent.TButton")
        else:
            self.send_btn.config(style="TButton")

    def toggle_connection(self):
        """Toggle serial connection"""
        if self.connect_btn.cget("text") == "Connect":
            self.connect_serial()
            self.connect_btn.config(text="Disconnect")
        else:
            if self.ser and self.ser.is_open:
                self.ser.close()
                self.log("Disconnected from serial port", "warning")
            self.connect_btn.config(text="Connect")
            self.connection_label.config(text="Disconnected", foreground="red")
            self.enable_command_sending(False)

    def toggle_simulation(self):
        """Toggle simulation mode"""
        if self.sim_var.get():
            self.simulation_mode = True
            self.log("Switched to simulation mode", "warning")
            self.connection_label.config(text="Simulation", foreground="orange")
            self.enable_command_sending(False)
        else:
            self.connect_serial()

    def send_command(self):
        """Send command to the device via serial"""
        command = self.command_var.get().strip()

        if not command:
            messagebox.showwarning("Empty Command", "Please enter a command to send.")
            return

        if not self.ser or not self.ser.is_open:
            messagebox.showerror("Not Connected", "Serial port is not connected!")
            return

        try:
            # Add newline if not present
            if not command.endswith('\n'):
                command += '\n'

            # Encode and send
            self.ser.write(command.encode())

            # Log the sent command
            self.log(f"CMD >>> {command.strip()}", "sent")

            # Clear the entry field
            self.command_var.set("")

            # Focus back to entry for next command
            self.command_entry.focus_set()

            # Read response if available
            self.root.after(100, self.check_for_response)

        except Exception as e:
            self.log(f"Failed to send command: {e}", "error")
            messagebox.showerror("Send Error", f"Failed to send command:\n{e}")

    def check_for_response(self):
        """Check for and read response from serial"""
        if self.ser and self.ser.is_open and self.ser.in_waiting:
            try:
                response = self.ser.readline().decode(errors='ignore').strip()
                if response:
                    self.log(f"RES <<< {response}", "received")
            except Exception as e:
                self.log(f"Error reading response: {e}", "error")

    def generate_simulation_data(self):
        """Generate simulated sensor data"""
        self.data_counter += 1

        t = self.data_counter * 0.1

        return {
            'lsmAccelX': int(100 + 50 * math.sin(t)),
            'lsmAccelY': int(50 + 30 * math.sin(t * 1.3 + 1)),
            'lsmAccelZ': int(500 + 100 * math.sin(t * 0.7 + 2)),
            'lsmTemp': int(25 + 5 * math.sin(t * 0.2)),
            'bmpTemp': int(24 + 4 * math.sin(t * 0.2 + 0.5)),
            'bmpPress': int(101000 + 1000 * math.sin(t * 0.1)),
            'bmpAlti': int(100 + 50 * math.sin(t * 0.15)),
            'battery': int(max(20, 100 - self.data_counter * 0.01) * self.RAW_BATT_MAX / 100),
            'status': self.data_counter % 8,
            'error': 0
        }

    def update_data(self):
        """Main data update loop"""
        if not self.running:
            return

        try:
            decoded = None

            if self.simulation_mode:
                # Generate simulated data
                decoded = self.generate_simulation_data()
            elif self.ser and self.ser.is_open:
                # Read from serial
                try:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        decoded = parse_bits(line)
                except Exception as e:
                    self.log(f"Serial read error: {e}", "error")

            if decoded:
                # Process data
                self.process_sensor_data(decoded)

            # Schedule next update
            self.root.after(100, self.update_data)

        except Exception as e:
            self.log(f"Update error: {e}", "error")
            self.root.after(1000, self.update_data)

    def process_sensor_data(self, decoded):
        """Process and display sensor data"""
        # Battery calculation
        raw_batt = decoded.get('battery', 0)
        batt_percent = max(0.0, min(100.0, (raw_batt / self.RAW_BATT_MAX) * 100.0))
        self.battery.append(batt_percent)

        # Store sensor data
        accel_x_val = signed(decoded.get('lsmAccelX', 0), 15)
        accel_y_val = signed(decoded.get('lsmAccelY', 0), 15)
        accel_z_val = signed(decoded.get('lsmAccelZ', 0), 15)

        self.accel_x.append(accel_x_val)
        self.accel_y.append(accel_y_val)
        self.accel_z.append(accel_z_val)
        self.lsm_temp.append(decoded.get('lsmTemp', 0))
        self.bmp_temp.append(decoded.get('bmpTemp', 0))
        self.bmp_press.append(decoded.get('bmpPress', 0))
        self.bmp_alti.append(decoded.get('bmpAlti', 0))

        # Update status
        status_val = decoded.get('status', 0)
        error_val = decoded.get('error', 0)

        mode_text = "Simulation" if self.simulation_mode else "Serial"
        self.status_label.config(
            text=f"Status: {status_val} | Error: {error_val} | Battery: {batt_percent:.1f}% | Mode: {mode_text}"
        )

        # Update graphs
        self.update_graphs(accel_x_val, accel_y_val, accel_z_val,
                           decoded.get('lsmTemp', 0), decoded.get('bmpTemp', 0),
                           decoded.get('bmpPress', 0), batt_percent)

        # Update value labels
        self.update_value_labels(accel_x_val, accel_y_val, accel_z_val,
                                 decoded.get('lsmTemp', 0), decoded.get('bmpTemp', 0),
                                 decoded.get('bmpPress', 0), decoded.get('bmpAlti', 0),
                                 batt_percent, status_val, error_val)

        # Update rocket visualization
        self.update_rocket_visualization(accel_x_val, accel_y_val, accel_z_val)

    def update_graphs(self, accel_x, accel_y, accel_z, lsm_temp, bmp_temp, bmp_press, batt_percent):
        """Update all graph displays"""
        # For acceleration, show magnitude or X component
        accel_magnitude = math.sqrt(accel_x ** 2 + accel_y ** 2 + accel_z ** 2)

        # Update each graph
        if hasattr(self, 'accel_graph'):
            self.accel_graph.add_point(accel_magnitude)

        if hasattr(self, 'temp_graph'):
            # Average of LSM and BMP temperatures
            avg_temp = (lsm_temp + bmp_temp) / 2
            self.temp_graph.add_point(avg_temp)

        if hasattr(self, 'press_graph'):
            self.press_graph.add_point(bmp_press)

        if hasattr(self, 'batt_graph'):
            self.batt_graph.add_point(batt_percent)

    def update_value_labels(self, accel_x, accel_y, accel_z, lsm_temp, bmp_temp,
                            bmp_press, bmp_alti, batt_percent, status_val, error_val):
        """Update sensor value labels"""
        if hasattr(self, 'value_labels'):
            self.value_labels['accel_x'].config(text=f"{accel_x}")
            self.value_labels['accel_y'].config(text=f"{accel_y}")
            self.value_labels['accel_z'].config(text=f"{accel_z}")
            self.value_labels['lsm_temp'].config(text=f"{lsm_temp}")
            self.value_labels['bmp_temp'].config(text=f"{bmp_temp}")
            self.value_labels['bmp_press'].config(text=f"{bmp_press}")
            self.value_labels['bmp_alti'].config(text=f"{bmp_alti}")
            self.value_labels['battery'].config(text=f"{batt_percent:.1f}%")

            if hasattr(self, 'status_value'):
                self.status_value.config(text=f"{status_val}")

            if hasattr(self, 'error_value'):
                self.error_value.config(text=f"{error_val}")

    def update_rocket_visualization(self, accel_x, accel_y, accel_z):
        """Update rocket orientation visualization"""
        try:
            # Calculate orientation from accelerometer
            roll_rad = math.atan2(accel_y, accel_z)
            pitch_rad = math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z))
            roll_deg = math.degrees(roll_rad)
            pitch_deg = math.degrees(pitch_rad)
        except:
            roll_deg = pitch_deg = 0.0

        # Update rocket visualization
        if hasattr(self, 'rocket_canvas'):
            self.draw_rocket(roll_deg, pitch_deg)

    def quit_app(self):
        """Clean up and quit application"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.quit()
        self.root.destroy()


def main():
    """Main application entry point"""
    root = tk.Tk()

    # Set window icon (optional)
    try:
        root.iconbitmap('rocket.ico')  # If you have an icon file
    except:
        pass

    # Create a custom style for accent button
    style = ttk.Style()
    style.configure("Accent.TButton", foreground="white", background="#0078D7")

    # Create application
    app = RocketMonitorApp(root)

    # Handle window close
    root.protocol("WM_DELETE_WINDOW", app.quit_app)

    # Start main loop
    root.mainloop()


if __name__ == "__main__":
    main()