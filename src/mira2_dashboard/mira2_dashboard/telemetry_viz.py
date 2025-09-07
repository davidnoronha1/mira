import sys
from collections import deque
import numpy as np

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, 
    QLabel, QGridLayout, QFrame, QSizePolicy
)
from PyQt6.QtCore import QTimer, pyqtSignal
from PyQt6.QtGui import QFont

import pyqtgraph as pg
import rclpy
from rclpy.node import Node
from custom_msgs.msg import Telemetry

from .pwm_viz import ThrusterWidget


class TelemetrySubscriber(Node):
    """ROS2 node to subscribe to telemetry data."""
    
    def __init__(self, callback):
        super().__init__('telemetry_subscriber')
        self.callback = callback
        self.subscription = self.create_subscription(
            Telemetry,
            '/master/telemetry',
            self.telemetry_callback,
            10
        )
        
    def telemetry_callback(self, msg):
        """Forward telemetry message to the callback."""
        self.callback(msg)


class TelemetryPlotWidget(QWidget):
    """Widget containing real-time plots for telemetry data."""
    
    telemetry_received = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.max_points = 1024
        self.init_data_buffers()
        self.init_ui()
        self.init_ros()
        
        # Connect signal to slot
        self.telemetry_received.connect(self.update_plots)
        
        # Timer for updating plots
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.refresh_plots)
        self.plot_timer.start(50)  # 20 Hz update rate
        
    def init_data_buffers(self):
        """Initialize data buffers for storing telemetry history."""
        self.timestamps = deque(maxlen=self.max_points)
        
        # Battery and status
        self.battery_voltage = deque(maxlen=self.max_points)
        self.arm_status = deque(maxlen=self.max_points)
        
        # IMU data
        self.gyro_x = deque(maxlen=self.max_points)
        self.gyro_y = deque(maxlen=self.max_points)
        self.gyro_z = deque(maxlen=self.max_points)
        
        self.accel_x = deque(maxlen=self.max_points)
        self.accel_y = deque(maxlen=self.max_points)
        self.accel_z = deque(maxlen=self.max_points)
        
        # Compass
        self.compass_x = deque(maxlen=self.max_points)
        self.compass_y = deque(maxlen=self.max_points)
        self.compass_z = deque(maxlen=self.max_points)
        
        # Orientation
        self.q1 = deque(maxlen=self.max_points)
        self.q2 = deque(maxlen=self.max_points)
        self.q3 = deque(maxlen=self.max_points)
        self.q4 = deque(maxlen=self.max_points)
        
        self.roll_speed = deque(maxlen=self.max_points)
        self.pitch_speed = deque(maxlen=self.max_points)
        self.yaw_speed = deque(maxlen=self.max_points)
        
        # Pressure and heading
        self.internal_pressure = deque(maxlen=self.max_points)
        self.external_pressure = deque(maxlen=self.max_points)
        self.heading = deque(maxlen=self.max_points)
        
        # Thrusters
        self.thruster_pwms = [deque(maxlen=self.max_points) for _ in range(8)]
        
        # Latest values for display
        self.latest_telemetry = None
        
    def init_ui(self):
        """Initialize the user interface."""
        layout = QVBoxLayout()
        
        # Status bar
        self.create_status_bar()
        layout.addWidget(self.status_frame)
        
        # Tab widget for different plot categories
        self.tab_widget = QTabWidget()
        
        # IMU tab
        self.create_imu_tab()
        
        # Orientation tab
        self.create_orientation_tab()
        
        # Pressure and Navigation tab
        self.create_navigation_tab()
        
        # Thrusters tab
        self.create_thrusters_tab()
        
        layout.addWidget(self.tab_widget)
        self.setLayout(layout)
        
    def create_status_bar(self):
        """Create status bar showing current values."""
        self.status_frame = QFrame()
        self.status_frame.setFrameStyle(QFrame.Shape.Box)
        self.status_frame.setMaximumHeight(80)
        
        layout = QGridLayout()
        
        # Battery voltage
        self.battery_label = QLabel("-- V")
        self.battery_label.setStyleSheet("font-weight: bold; color: green;")
        layout.addWidget(QLabel("Battery:"), 0, 0)
        layout.addWidget(self.battery_label, 0, 1)
        
        # Arm status
        self.arm_label = QLabel("DISARMED")
        self.arm_label.setStyleSheet("font-weight: bold; color: red;")
        layout.addWidget(QLabel("Status:"), 0, 2)
        layout.addWidget(self.arm_label, 0, 3)
        
        # Heading
        self.heading_label = QLabel("-- °")
        layout.addWidget(QLabel("Heading:"), 1, 0)
        layout.addWidget(self.heading_label, 1, 1)
        
        # Pressure difference
        self.pressure_diff_label = QLabel("-- Pa")
        layout.addWidget(QLabel("Depth:"), 1, 2)
        layout.addWidget(self.pressure_diff_label, 1, 3)
        
        self.status_frame.setLayout(layout)
        
    def create_imu_tab(self):
        """Create IMU data plots tab."""
        imu_widget = QWidget()
        layout = QVBoxLayout()
        
        # Accelerometer section
        accel_layout = QHBoxLayout()
        
        # X Acceleration plot
        self.accel_x_plot = pg.PlotWidget(title="X Acceleration (m/s²)")
        self.accel_x_plot.setLabel('left', 'X Accel', 'm/s²')
        self.accel_x_plot.setLabel('bottom', 'Time', 's')
        self.accel_x_curve = self.accel_x_plot.plot(pen='r', name='X Accel')
        accel_layout.addWidget(self.accel_x_plot)
        
        # Y Acceleration plot
        self.accel_y_plot = pg.PlotWidget(title="Y Acceleration (m/s²)")
        self.accel_y_plot.setLabel('left', 'Y Accel', 'm/s²')
        self.accel_y_plot.setLabel('bottom', 'Time', 's')
        self.accel_y_curve = self.accel_y_plot.plot(pen='g', name='Y Accel')
        accel_layout.addWidget(self.accel_y_plot)
        
        # Z Acceleration plot
        self.accel_z_plot = pg.PlotWidget(title="Z Acceleration (m/s²)")
        self.accel_z_plot.setLabel('left', 'Z Accel', 'm/s²')
        self.accel_z_plot.setLabel('bottom', 'Time', 's')
        self.accel_z_curve = self.accel_z_plot.plot(pen='b', name='Z Accel')
        accel_layout.addWidget(self.accel_z_plot)
        
        layout.addLayout(accel_layout)
        
        # Gyroscope section
        gyro_layout = QHBoxLayout()
        
        # X Gyroscope plot
        self.gyro_x_plot = pg.PlotWidget(title="X Gyroscope (deg/s)")
        self.gyro_x_plot.setLabel('left', 'X Gyro', 'deg/s')
        self.gyro_x_plot.setLabel('bottom', 'Time', 's')
        self.gyro_x_curve = self.gyro_x_plot.plot(pen='r', name='X Gyro')
        gyro_layout.addWidget(self.gyro_x_plot)
        
        # Y Gyroscope plot
        self.gyro_y_plot = pg.PlotWidget(title="Y Gyroscope (deg/s)")
        self.gyro_y_plot.setLabel('left', 'Y Gyro', 'deg/s')
        self.gyro_y_plot.setLabel('bottom', 'Time', 's')
        self.gyro_y_curve = self.gyro_y_plot.plot(pen='g', name='Y Gyro')
        gyro_layout.addWidget(self.gyro_y_plot)
        
        # Z Gyroscope plot
        self.gyro_z_plot = pg.PlotWidget(title="Z Gyroscope (deg/s)")
        self.gyro_z_plot.setLabel('left', 'Z Gyro', 'deg/s')
        self.gyro_z_plot.setLabel('bottom', 'Time', 's')
        self.gyro_z_curve = self.gyro_z_plot.plot(pen='b', name='Z Gyro')
        gyro_layout.addWidget(self.gyro_z_plot)
        
        layout.addLayout(gyro_layout)
        
        imu_widget.setLayout(layout)
        self.tab_widget.addTab(imu_widget, "IMU")
        
    def create_orientation_tab(self):
        """Create orientation plots tab."""
        orientation_widget = QWidget()
        layout = QVBoxLayout()
        
        # Angular rates section - Split into 3 individual plots
        rates_layout = QHBoxLayout()
        
        # Roll Rate plot
        self.roll_rate_plot = pg.PlotWidget(title="Roll Rate (deg/s)")
        self.roll_rate_plot.setLabel('left', 'Roll Rate', 'deg/s')
        self.roll_rate_plot.setLabel('bottom', 'Time', 's')
        self.roll_rate_curve = self.roll_rate_plot.plot(pen='r', name='Roll Rate')
        rates_layout.addWidget(self.roll_rate_plot)
        
        # Pitch Rate plot
        self.pitch_rate_plot = pg.PlotWidget(title="Pitch Rate (deg/s)")
        self.pitch_rate_plot.setLabel('left', 'Pitch Rate', 'deg/s')
        self.pitch_rate_plot.setLabel('bottom', 'Time', 's')
        self.pitch_rate_curve = self.pitch_rate_plot.plot(pen='g', name='Pitch Rate')
        rates_layout.addWidget(self.pitch_rate_plot)
        
        # Yaw Rate plot
        self.yaw_rate_plot = pg.PlotWidget(title="Yaw Rate (deg/s)")
        self.yaw_rate_plot.setLabel('left', 'Yaw Rate', 'deg/s')
        self.yaw_rate_plot.setLabel('bottom', 'Time', 's')
        self.yaw_rate_curve = self.yaw_rate_plot.plot(pen='b', name='Yaw Rate')
        rates_layout.addWidget(self.yaw_rate_plot)
        
        layout.addLayout(rates_layout)
        
        orientation_widget.setLayout(layout)
        self.tab_widget.addTab(orientation_widget, "Orientation")
        
    def create_navigation_tab(self):
        """Create navigation plots tab."""
        nav_widget = QWidget()
        layout = QVBoxLayout()
        
        # Pressure section - Split into separate plots
        pressure_layout = QHBoxLayout()
        
        # Internal Pressure plot
        self.internal_pressure_plot = pg.PlotWidget(title="Internal Pressure (Pa)")
        self.internal_pressure_plot.setLabel('left', 'Internal Pressure', 'Pa')
        self.internal_pressure_plot.setLabel('bottom', 'Time', 's')
        self.internal_pressure_curve = self.internal_pressure_plot.plot(pen='r', name='Internal')
        pressure_layout.addWidget(self.internal_pressure_plot)
        
        # External Pressure plot
        self.external_pressure_plot = pg.PlotWidget(title="External Pressure (Pa)")
        self.external_pressure_plot.setLabel('left', 'External Pressure', 'Pa')
        self.external_pressure_plot.setLabel('bottom', 'Time', 's')
        self.external_pressure_curve = self.external_pressure_plot.plot(pen='b', name='External')
        pressure_layout.addWidget(self.external_pressure_plot)
        
        layout.addLayout(pressure_layout)
        
        # Heading plot (full width)
        self.heading_plot = pg.PlotWidget(title="Heading (degrees)")
        self.heading_plot.setLabel('left', 'Heading', '°')
        self.heading_plot.setLabel('bottom', 'Time', 's')
        self.heading_curve = self.heading_plot.plot(pen='m', name='Heading')
        layout.addWidget(self.heading_plot)
        
        nav_widget.setLayout(layout)
        self.tab_widget.addTab(nav_widget, "Navigation")
        
    def create_thrusters_tab(self):
        """Create thruster PWM plots tab."""
        thrusters_widget = ThrusterWidget(self, self.thruster_pwms)
        self.tab_widget.addTab(thrusters_widget, "Thrusters")
        
    def init_ros(self):
        """Initialize ROS2 node and subscription."""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.ros_node = TelemetrySubscriber(self.on_telemetry_received)
            
            # Timer to spin ROS node
            self.ros_timer = QTimer()
            self.ros_timer.timeout.connect(self.spin_ros)
            self.ros_timer.start(10)  # 100 Hz
            
        except Exception as e:
            print(f"Failed to initialize ROS: {e}")
            
    def spin_ros(self):
        """Spin ROS node to process callbacks."""
        try:
            if rclpy.ok():
                rclpy.spin_once(self.ros_node, timeout_sec=0.001)
        except Exception as e:
            print(f"ROS spin error: {e}")
            
    def on_telemetry_received(self, msg):
        """Handle received telemetry message."""
        self.telemetry_received.emit(msg)
        
    def update_plots(self, msg):
        """Update plot data with new telemetry message."""
        self.latest_telemetry = msg
        
        # Add timestamp (convert to relative time)
        if len(self.timestamps) == 0:
            self.base_time = msg.timestamp
        
        rel_time = msg.timestamp - self.base_time
        self.timestamps.append(rel_time)
        
        # Battery and status
        self.battery_voltage.append(msg.battery_voltage)
        self.arm_status.append(msg.arm)
        
        # IMU data (convert to appropriate units)
        self.gyro_x.append(msg.imu_gyro_x / 1000.0)  # Assuming raw values need scaling
        self.gyro_y.append(msg.imu_gyro_y / 1000.0)
        self.gyro_z.append(msg.imu_gyro_z / 1000.0)
        
        self.accel_x.append(msg.imu_xacc / 1000.0)
        self.accel_y.append(msg.imu_yacc / 1000.0)
        self.accel_z.append(msg.imu_zacc / 1000.0)
        
        # Compass
        self.compass_x.append(msg.imu_gyro_compass_x)
        self.compass_y.append(msg.imu_gyro_compass_y)
        self.compass_z.append(msg.imu_gyro_compass_z)
        
        # Orientation
        self.q1.append(msg.q1)
        self.q2.append(msg.q2)
        self.q3.append(msg.q3)
        self.q4.append(msg.q4)
        
        self.roll_speed.append(msg.rollspeed)
        self.pitch_speed.append(msg.pitchspeed)
        self.yaw_speed.append(msg.yawspeed)
        
        # Pressure and heading
        self.internal_pressure.append(msg.internal_pressure)
        self.external_pressure.append(msg.external_pressure)
        self.heading.append(msg.heading)
        
        # Thrusters
        for i, pwm in enumerate(msg.thruster_pwms):
            self.thruster_pwms[i].append(pwm)
            
    def refresh_plots(self):
        """Refresh all plots with current data."""
        if len(self.timestamps) == 0:
            return
            
        times = np.array(self.timestamps)
        
        # Update status bar
        if self.latest_telemetry:
            msg = self.latest_telemetry
            
            # Battery
            self.battery_label.setText(f"{msg.battery_voltage:.2f} V")
            if msg.battery_voltage < 11.0:  # Low battery threshold
                self.battery_label.setStyleSheet("font-weight: bold; color: red;")
            elif msg.battery_voltage < 12.0:
                self.battery_label.setStyleSheet("font-weight: bold; color: orange;")
            else:
                self.battery_label.setStyleSheet("font-weight: bold; color: green;")
            
            # Arm status
            if msg.arm:
                self.arm_label.setText("ARMED")
                self.arm_label.setStyleSheet("font-weight: bold; color: green;")
            else:
                self.arm_label.setText("DISARMED")
                self.arm_label.setStyleSheet("font-weight: bold; color: red;")
            
            # Heading
            self.heading_label.setText(f"{msg.heading:.1f}°")
            
            # Pressure difference (depth approximation)
            pressure_diff = msg.external_pressure - msg.internal_pressure
            self.pressure_diff_label.setText(f"{pressure_diff:.1f} Pa")
        
        # Update IMU plots
        if len(times) > 1:
            self.gyro_x_curve.setData(times, np.array(self.gyro_x))
            self.gyro_y_curve.setData(times, np.array(self.gyro_y))
            self.gyro_z_curve.setData(times, np.array(self.gyro_z))
            
            self.accel_x_curve.setData(times, np.array(self.accel_x))
            self.accel_y_curve.setData(times, np.array(self.accel_y))
            self.accel_z_curve.setData(times, np.array(self.accel_z))
            
            # Update orientation plots (roll, pitch, yaw rates only)
            self.roll_rate_curve.setData(times, np.array(self.roll_speed))
            self.pitch_rate_curve.setData(times, np.array(self.pitch_speed))
            self.yaw_rate_curve.setData(times, np.array(self.yaw_speed))
            
            # Update navigation plots
            self.internal_pressure_curve.setData(times, np.array(self.internal_pressure))
            self.external_pressure_curve.setData(times, np.array(self.external_pressure))
            self.heading_curve.setData(times, np.array(self.heading))
            
            # Update thruster plots
            for i, curve in enumerate(self.thruster_curves):
                if len(self.thruster_pwms[i]) > 0:
                    curve.setData(times, np.array(self.thruster_pwms[i]))
                    
    def closeEvent(self, event):
        """Clean up when widget is closed."""
        try:
            if hasattr(self, 'ros_node'):
                self.ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Cleanup error: {e}")
        super().closeEvent(event)