import sys
import subprocess
import threading
import time
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QTextEdit, QMessageBox, QHBoxLayout
)
from PyQt5.QtCore import pyqtSignal, QObject
import RPi.GPIO as GPIO  # Import GPIO library

class AGVControlGUI(QWidget):
    # Signal to update the status in the main thread
    status_signal = pyqtSignal(str, bool)  # Add a bool to indicate error messages

    def __init__(self):
        super().__init__()

        self.setWindowTitle("AGV Control Panel")
        self.setGeometry(100, 100, 800, 400)

        # Status variables
        self.radar_on = False
        self.navigation_on = False

        # Initialize the UI
        self.init_ui()

        # Connect the signal to update the status in the main thread
        self.status_signal.connect(self.add_status_message)

    def init_ui(self):
        # Buttons
        self.radar_button = QPushButton("Radar ON")
        self.radar_button.setCheckable(True)
        self.radar_button.clicked.connect(self.radar_control)

        self.navigation_button = QPushButton("Navigation ON")
        self.navigation_button.setCheckable(True)
        self.navigation_button.clicked.connect(self.navigation_control)
        self.navigation_button.setEnabled(False)  # Initially disabled

        self.clear_button = QPushButton("Clear Logs")
        self.clear_button.clicked.connect(self.clear_status)

        # QTextEdit to display status messages
        self.status_text_edit = QTextEdit()
        self.status_text_edit.setReadOnly(True)

        # Layout setup
        button_layout = QVBoxLayout()
        button_layout.addWidget(self.radar_button)
        button_layout.addWidget(self.navigation_button)
        button_layout.addWidget(self.clear_button)

        main_layout = QHBoxLayout()
        main_layout.addLayout(button_layout)
        main_layout.addWidget(self.status_text_edit)

        self.setLayout(main_layout)

        # Apply basic styling
        self.setStyleSheet(
            """
            QPushButton {
                font-size: 16px;
                padding: 10px;
            }
            QTextEdit {
                font-size: 14px;
                background-color: #f4f4f4;
                border: 1px solid #ccc;
            }
            """
        )

    def radar_control(self):
        if self.radar_button.isChecked():  # Turn Radar ON
            self.radar_on = True
            self.radar_button.setText("Radar OFF")
            self.status_signal.emit("Radar system turning ON...", False)

            # Start radar opening in a separate thread
            threading.Thread(target=self.radar_open, daemon=True).start()

            # Enable the Navigation button
            self.navigation_button.setEnabled(True)
        else:  # Turn Radar OFF
            self.radar_on = False
            self.radar_button.setText("Radar ON")
            self.status_signal.emit("Radar system turning OFF...", False)

            # Start radar closing in a separate thread
            threading.Thread(target=self.radar_close, daemon=True).start()

            # Disable the Navigation button
            self.navigation_button.setEnabled(False)

    def radar_open(self):
        try:
            self.radar_high()
            subprocess.run([
                'gnome-terminal', '--', 'bash', '-c', "roslaunch myagv_odometry myagv_active.launch; exec $SHELL"
            ])
            self.status_signal.emit("Radar system started successfully.", False)
        except Exception as e:
            self.status_signal.emit(f"Error starting radar: {e}", True)

    def radar_close(self):
        try:
            self.radar_low()
            subprocess.run(
                "ps -ef | grep -E 'myagv_active.launch' | grep -v 'grep' | awk '{print $2}' | xargs kill -2",
                shell=True
            )
            self.status_signal.emit("Radar system stopped successfully.", False)
        except Exception as e:
            self.status_signal.emit(f"Error stopping radar: {e}", True)

    def radar_high(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20, GPIO.OUT)
        GPIO.output(20, GPIO.HIGH)

    def radar_low(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20, GPIO.OUT)
        GPIO.output(20, GPIO.LOW)

    def navigation_control(self):
        if self.navigation_button.isChecked():  # Turn Navigation ON
            if not self.radar_on:
                self.status_signal.emit("Error: Radar must be ON before starting Navigation.", True)
                self.navigation_button.setChecked(False)
                return

            self.navigation_on = True
            self.navigation_button.setText("Navigation OFF")
            self.status_signal.emit("Navigation system turning ON...", False)

            # Start navigation opening in a separate thread
            threading.Thread(target=self.navigation_open, daemon=True).start()
        else:  # Turn Navigation OFF
            self.navigation_on = False
            self.navigation_button.setText("Navigation ON")
            self.status_signal.emit("Navigation system turning OFF...", False)

            # Start navigation closing in a separate thread
            threading.Thread(target=self.navigation_close, daemon=True).start()

    def navigation_open(self):
        try:
            subprocess.run([
                'gnome-terminal', '--', 'bash', '-c', "roslaunch myagv_navigation navigation_active.launch; exec $SHELL"
            ])
            self.status_signal.emit("Navigation system started successfully.", False)
        except Exception as e:
            self.status_signal.emit(f"Error starting navigation: {e}", True)

    def navigation_close(self):
        try:
            subprocess.run(
                "ps -ef | grep -E 'navigation_active.launch' | grep -v 'grep' | awk '{print $2}' | xargs kill -2",
                shell=True
            )
            self.status_signal.emit("Navigation system stopped successfully.", False)
        except Exception as e:
            self.status_signal.emit(f"Error stopping navigation: {e}", True)

    def add_status_message(self, message, is_error):
        """Update the status message in the QTextEdit."""
        color = "red" if is_error else "black"
        self.status_text_edit.append(f'<span style="color: {color};">{message}</span>')

    def clear_status(self):
        self.status_text_edit.clear()

# Main function to run the application
def main():
    app = QApplication(sys.argv)
    window = AGVControlGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
