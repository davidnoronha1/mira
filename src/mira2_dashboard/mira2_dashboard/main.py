from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QHBoxLayout,
    QVBoxLayout, QFrame, QSizePolicy
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
import sys
import subprocess
from PyQt6.QtCore import qDebug, qInfo, qWarning, qCritical, qFatal

from .ros_status import is_node_alive, is_topic_alive
from .connection_status import can_connect_to_device_ssh, is_joystick_connected
from .telemetry_viz import TelemetryPlotWidget

class Dashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mira2 Dashboard")
        self.setMinimumSize(800, 600)
        self.init_ui()

    def make_status_row(self, label_text, status_text, status_color, launch=True):
        row = QHBoxLayout()
        row.setSizeConstraint(QHBoxLayout.SizeConstraint.SetMinimumSize)

        label = QLabel(label_text)
        row.addWidget(label, alignment=Qt.AlignmentFlag.AlignLeft)

        status = QLabel(status_text)
        status.setStyleSheet(f"color: {status_color}; font-weight: bold;")
        status.setFont(QFont("", weight=QFont.Weight.Bold))
        row.addWidget(status, alignment=Qt.AlignmentFlag.AlignLeft)

        button = None
        if launch:
            row.addStretch()
            button = QPushButton("LAUNCH")
            row.addWidget(button, alignment=Qt.AlignmentFlag.AlignRight)

        return row, status, button

    def make_seperator(self):
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        return line

    def update_status_label(self, label, is_connected, active_text="ACTIVE", inactive_text="INACTIVE"):
        """Update a status label with appropriate text and color."""
        if is_connected:
            label.setText(active_text)
            label.setStyleSheet("color: green; font-weight: bold;")
        else:
            label.setText(inactive_text)
            label.setStyleSheet("color: red; font-weight: bold;" if active_text == "CONNECTED" else "color: grey; font-weight: bold;")

    def check_control_master_status(self):
        """Check if control master node is running."""
        return is_node_alive("mira2_control_master")

    def check_teleop_status(self):
        """Check if teleop node is running."""
        return is_node_alive("teleop_twist_keyboard") or is_node_alive("teleop")

    def check_rpi_connection(self):
        """Check if RPI is connected via SSH."""
        # Replace with your RPI's IP address
        rpi_ip = "192.168.2.3"  # Update this with actual RPI IP
        return can_connect_to_device_ssh(rpi_ip)

    def check_controller_connection(self):
        """Check if controller/joystick is connected."""
        return is_joystick_connected()

    def refresh_connections(self):
        qInfo("Refreshing connections...")
        """Refresh all connection statuses."""
        # Check ROS nodes
        control_active = self.check_control_master_status()
        self.update_status_label(self.control_status, control_active, "ACTIVE", "INACTIVE")

        teleop_active = self.check_teleop_status()
        self.update_status_label(self.teleop_status, teleop_active, "ACTIVE", "INACTIVE")

        # Check connections
        rpi_connected = self.check_rpi_connection()
        self.update_status_label(self.rpi_status, rpi_connected, "CONNECTED", "DISCONNECTED")

        controller_connected = self.check_controller_connection()
        self.update_status_label(self.controller_status, controller_connected, "CONNECTED", "DISCONNECTED")

    def launch_telemetry_viz(self):
        """Launch telemetry visualization in a separate window."""
        try:
            subprocess.Popen(['ros2', 'run', 'mira2_dashboard', 'telemetry_viz'])
            qInfo("Launched telemetry visualization")
        except Exception as e:
            qWarning(f"Failed to launch telemetry visualization: {e}")

    def init_ui(self):
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(10)

        # Section 1: Control Master
        row1, self.control_status, self.control_btn = self.make_status_row(
            "Control Master:", "ACTIVE", "green"
        )
        main_layout.addLayout(row1)

        # Section 2: Teleop
        row2, self.teleop_status, self.teleop_btn = self.make_status_row(
            "Teleop:", "INACTIVE", "grey"
        )
        main_layout.addLayout(row2)

        # Separator
        main_layout.addWidget(self.make_seperator())

        # Section 3: Connections (no launch buttons)
        connection_status = QHBoxLayout()
        row3, self.rpi_status, _ = self.make_status_row(
            "Connected to RPI:", "CONNECTED", "green", launch=False
        )
        connection_status.addLayout(row3)

        row4, self.controller_status, _ = self.make_status_row(
            "Controller connected:", "DISCONNECTED", "red", launch=False
        )
        connection_status.addLayout(row4)

        connection_status.addStretch()
        refresh = QPushButton("REFRESH")
        refresh.clicked.connect(self.refresh_connections)
        connection_status.addWidget(refresh)

        main_layout.addLayout(connection_status)

        main_layout.addWidget(self.make_seperator())

        # Section 4: Telemetry Visualization (embed widget directly)
          # Ensure this widget exists
        # telemetry_label = QLabel("Telemetry Visualization:")
        # telemetry_label.setFont(QFont("", weight=QFont.Weight.Bold))
        # telemetry_row = QHBoxLayout()
        # telemetry_row.addWidget(telemetry_label, alignment=Qt.AlignmentFlag.AlignLeft)
        self.telemetry_widget = TelemetryPlotWidget(self)
        # telemetry_row.addWidget(self.telemetry_widget, alignment=Qt.AlignmentFlag.AlignLeft)
        main_layout.addWidget(self.telemetry_widget)

        main_layout.addStretch()

        self.setLayout(main_layout)
        
        # Initial status check
        self.refresh_connections()


def main():
    app = QApplication(sys.argv)
    win = Dashboard()
    win.show()
    sys.exit(app.exec())
