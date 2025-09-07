import sys
from collections import deque
from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtGui import QPixmap, QPainter, QColor, QFont
from PyQt6.QtCore import Qt, QTimer, qInfo
from pathlib import Path
import random

from ament_index_python.packages import get_package_share_directory

pkg_name = "mira2_dashboard"
pkg_share = Path(get_package_share_directory(pkg_name))
image_path = pkg_share / "static" / "thruster-layout.png"

qInfo(f"Loading thruster image from: {image_path}")


class ThrusterWidget(QWidget):
    def __init__(self, parent=None, _s = None):
        super().__init__(parent)

        self.max_points = 100
        # Create 8 deques, one per thruster
        self.thruster_pwms = [deque(maxlen=self.max_points) for _ in range(8)]

        self.image = QPixmap(image_path.as_posix())

        # Fill with initial dummy data - using values from your image
        initial_values = [1368, 1229, 1623, 1430, 1726, 1399, 1847, 1847]
        for i, dq in enumerate(self.thruster_pwms):
            dq.append(initial_values[i])

        # Update periodically
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateData)
        self.timer.start(1000)

    def updateData(self):
        """Simulate PWM updates"""
        initial_values = [1368, 1229, 1623, 1430, 1726, 1399, 1847, 1847]
        for i, dq in enumerate(self.thruster_pwms):
            dq.append(initial_values[i])
        # for dq in self.thruster_pwms:
        #     dq.append(random.randint(1100, 1900))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)

        # Scale the image to fit the widget
        widget_width = self.width()
        widget_height = self.height()
        scaled_image = self.image.scaled(
            widget_width,
            widget_height,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        
        # Center the scaled image
        image_width = scaled_image.width()
        image_height = scaled_image.height()
        x_offset = (widget_width - image_width) // 2
        y_offset = (widget_height - image_height) // 2
        
        painter.drawPixmap(x_offset, y_offset, scaled_image)

        # Font scaling relative to widget height
        font_size = max(10, int(widget_height * 0.025))  # Slightly smaller font
        font = QFont("Arial", font_size, QFont.Weight.Bold)
        painter.setFont(font)
        painter.setPen(QColor("white"))

        # Corrected positions based on the actual thruster locations in your image
        # These coordinates are normalized to the image dimensions (0-1 ratios)
        positions = {
            1: (0.75, 0.08),   # Top right thruster (green)
            2: (0.25, 0.08),   # Top left thruster (green) 
            3: (0.75, 0.92),   # Bottom right thruster (blue)
            4: (0.25, 0.92),   # Bottom left thruster (blue)
            5: (0.92, 0.32),   # Right upper thruster (green fan)
            6: (0.08, 0.32),   # Left upper thruster (blue fan)
            7: (0.92, 0.68),   # Right lower thruster (blue fan)
            8: (0.08, 0.68),   # Left lower thruster (green fan)
        }

        # Draw the latest PWM value for each thruster
        for i, dq in enumerate(self.thruster_pwms, start=1):
            if dq:
                pwm = dq[-1]
                x_ratio, y_ratio = positions[i]
                
                # Calculate position relative to the scaled and centered image
                x = x_offset + int(image_width * x_ratio)
                y = y_offset + int(image_height * y_ratio)
                
                # Center the text on the position
                text = str(pwm)
                text_rect = painter.fontMetrics().boundingRect(text)
                x -= text_rect.width() // 2
                y += text_rect.height() // 4  # Slight vertical adjustment
                
                painter.drawText(x, y, text)