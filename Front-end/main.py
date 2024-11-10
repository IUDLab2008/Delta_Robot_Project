import sys
import serial.tools.list_ports
import struct
import serial
import os  
from PyQt6 import uic, QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QApplication, QWidget, QGraphicsDropShadowEffect, QSizeGrip, QPushButton, QMessageBox
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from qt_material import *
from delta_ui import Ui_DeltaGUI
import icons_rc
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../Back-end')))
import converted as cv


class SerialPortScanner(QThread):
    ports_updated = pyqtSignal(list)
    def __init__(self):
        super().__init__()
        self.running = True

    def run(self):
        previous_ports = set()
        while self.running:
            current_ports = set([port.device for port in serial.tools.list_ports.comports()])
            if current_ports != previous_ports:
                self.ports_updated.emit(list(current_ports))
                previous_ports = current_ports
            self.msleep(1000)  

    def stop(self):
        self.running = False
        
class SerialReader(QThread):
    data_received = pyqtSignal(float, float, float)  

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port 
        self.baudrate = baudrate
        self.running = True

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Successfully connected to {self.port} at baud rate {self.baudrate}")

            while self.running:
                if ser.in_waiting >= 12:  
                    data = ser.read(12)  
                    angles = struct.unpack('fff', data)  
                    self.data_received.emit(*angles)  

        except serial.SerialException as e:
            print(f"Error: {e}")
        finally:
            if ser.is_open:
                ser.close()
                print("Connection closed.")
                
    def stop(self):
        self.running = False
        self.wait()


class Window(QWidget):
    def __init__(self):
        super().__init__()
        self.ui = Ui_DeltaGUI()
        self.ui.setupUi(self)
        
        self.status = "Disconnected"  # Status of the connection
        self.serial_connection = None  # Serial connection object

        # Load the stylesheet
        self.load_stylesheet('styles.css')
        # Set the size
        self.resize(950, 630)  
        # Delete title bar
        self.setWindowFlag(QtCore.Qt.WindowType.FramelessWindowHint)
        # Window Size grip
        QSizeGrip(self.ui.size_grip)
        # Set up blinking for lb_status
        self.blink_status_label()
        
        # SET UP SHADOW
        ###########################################################################################################
        # Set the background to transparent
        self.setAttribute(QtCore.Qt. WidgetAttribute.WA_TranslucentBackground)
        # Shadow effect
        self.shadow = QGraphicsDropShadowEffect()
        self.shadow.setBlurRadius(50)
        self.shadow.setXOffset(0)
        self.shadow.setYOffset(0)
        self.shadow.setColor(QtCore.Qt.GlobalColor.black)
        self.setGraphicsEffect(self.shadow)
        ###########################################################################################################
        
        # SET UP MENU
        ###########################################################################################################
        # Set the button to serial page
        self.ui.pbtn_serial.clicked.connect(lambda: self.ui.stackedWidget.setCurrentWidget(self.ui.serial_page))
        # Set the button to setting page
        self.ui.pbtn_setting.clicked.connect(lambda: self.ui.stackedWidget.setCurrentWidget(self.ui.setting_page))
        ###########################################################################################################

        # ENABLE SENDING DATA
        ###########################################################################################################
        self.ui.le_input_angle1.setEnabled(False)
        self.ui.le_input_angle2.setEnabled(False)
        self.ui.le_input_angle3.setEnabled(False)
        self.ui.le_input_x.setEnabled(False)
        self.ui.le_input_y.setEnabled(False)
        self.ui.le_input_z.setEnabled(False)
        self.ui.cbb_manual_mode.setEnabled(False)
        self.ui.le_auto_mode.setEnabled(False)
        self.ui.pbtn_add.setEnabled(False)
        ###########################################################################################################
        
        # SET UP BUTTONS 
        ###########################################################################################################
        # Set cursor hand for buttons
        self.set_cursor_hand_for_buttons()
        # Minimize window
        self.ui.pbtn_minimize.clicked.connect(lambda: self.showMinimized())
        # Restore window
        self.ui.pbtn_restore.clicked.connect(lambda: self.restore_or_maximize_window())
        # Close window
        self.ui.pbtn_close.clicked.connect(lambda: self.close())
        # Left menu 
        self.ui.pbtn_menu.clicked.connect(lambda: self.slide_left_menu()) 
        # Connect button
        self.ui.pbtn_connect.clicked.connect(lambda: self.connect_serial())
        # Disconnect button
        self.ui.pbtn_disconnect.clicked.connect(lambda: self.disconnect_serial())
        # Pause button
        self.ui.pbtn_pause.clicked.connect(lambda: self.pause_serial())
        # Send button
        self.ui.pbtn_send.clicked.connect(lambda: self.send_data()) 
        # Add button
        self.ui.pbtn_add.clicked.connect(lambda: self.add_file())
        ###########################################################################################################
        
        # SET UP COMBOBOXES
        ###########################################################################################################
        # Port
        self.serial_scanner = SerialPortScanner()
        self.serial_scanner.ports_updated.connect(self.update_com_ports)
        self.serial_scanner.start()
        # Baudrate
        # Control mode
        self.ui.cbb_control_mode.currentIndexChanged.connect(self.control_mode)
        # Manual mode
        self.ui.cbb_manual_mode.currentIndexChanged.connect(self.manual_mode)
        ###########################################################################################################
    
    # Load the stylesheet
    def load_stylesheet(self, path):
        with open(path, 'r') as file:
            stylesheet = file.read()
        self.setStyleSheet(stylesheet)
            
    # Blink status label
    def blink_status_label(self):
        self.blink_state = True
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.toggle_status_label)
        self.timer.start(500)  
        
    # Toggle status label
    def toggle_status_label(self):
        if self.blink_state:
            if self.status == "Connected":
                self.ui.lb_status.setText("CONNECTED")
                self.ui.lb_status.setStyleSheet("color: rgb(0, 255, 0);")
                self.ui.lb_icon_status.setPixmap(QtGui.QPixmap(":/icons/icons8-success-48-green.png"))
            elif self.status == "Pause":
                self.ui.lb_status.setText("PAUSE")
                self.ui.lb_status.setStyleSheet("color: rgb(255, 255, 0);")
                self.ui.lb_icon_status.setPixmap(QtGui.QPixmap(":/icons/icons8-pause-50-yellow.png"))  
            else:
                self.ui.lb_status.setText("DISCONNECTED")
                self.ui.lb_status.setStyleSheet("color: rgb(255, 0, 0);")
                self.ui.lb_icon_status.setPixmap(QtGui.QPixmap(":/icons/icons8-warning-50-red.png"))
        else:
            if self.status == "Connected":
                self.ui.lb_status.setText("CONNECTED")
                self.ui.lb_status.setStyleSheet("color: rgb(255, 255, 255);")
                self.ui.lb_icon_status.setPixmap(QtGui.QPixmap(":/icons/icons8-success-48-white.png"))
            elif self.status == "Pause":
                self.ui.lb_status.setText("PAUSE")
                self.ui.lb_status.setStyleSheet("color: rgb(255, 255, 255);")
                self.ui.lb_icon_status.setPixmap(QtGui.QPixmap(":/icons/icons8-pause-50-white.png"))  
            else:
                self.ui.lb_status.setText("DISCONNECTED")
                self.ui.lb_status.setStyleSheet("color: rgb(255, 255, 255);")
                self.ui.lb_icon_status.setPixmap(QtGui.QPixmap(":/icons/icons8-warning-50-white.png"))
        self.blink_state = not self.blink_state
        
    # Set cursor hand for buttons
    def set_cursor_hand_for_buttons(self):
        for widget in self.findChildren(QPushButton):
            widget.setCursor(Qt.CursorShape.PointingHandCursor)
            
    # Restore or maximize window function
    def restore_or_maximize_window(self):
        if self.isMaximized():
            self.showNormal()
            self.ui.pbtn_restore.setIcon(QIcon(":/icons/icons8-maximize-window-50.png"))
        else:
            self.showMaximized()
            self.ui.pbtn_restore.setIcon(QIcon(":/icons/icons8-restore-down-52.png"))
            
    # Override mouse events to enable window dragging
    def mousePressEvent(self, event):
        if self.isMaximized():
            return 
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.old_pos = event.globalPosition().toPoint()
    def mouseMoveEvent(self, event):
        if self.old_pos is not None:
            delta = event.globalPosition().toPoint() - self.old_pos
            self.move(self.pos() + delta)
            self.old_pos = event.globalPosition().toPoint()
    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.old_pos = None   
               
    # Slide left menu function
    def slide_left_menu(self):
        width = self.ui.left_menu_cont_frame.width()
        new_width = 50
        if width == 50:
            new_width = 150
        self.animation = QtCore.QPropertyAnimation(self.ui.left_menu_cont_frame, b"minimumWidth")
        self.animation.setDuration(250)
        self.animation.setStartValue(width)
        self.animation.setEndValue(new_width)
        self.animation.setEasingCurve(QtCore.QEasingCurve.Type.InOutQuart)
        self.animation.start()        
            
    # Update the com ports
    def update_com_ports(self, ports):
        self.ui.cbb_port.clear()
        self.ui.cbb_port.addItems(ports)
        
    # Control mode function
    def control_mode(self):
        if self.ui.cbb_control_mode.currentText() == "Auto":
            self.ui.le_auto_mode.setEnabled(True)
            self.ui.le_auto_mode.setStyleSheet("background-color: rgb(255, 255, 255);")
            self.ui.pbtn_add.setEnabled(True)
            self.ui.pbtn_add.setStyleSheet("background-color: rgb(0, 0, 0);")
            self.ui.cbb_manual_mode.setEnabled(False)
            self.ui.cbb_manual_mode.setStyleSheet("background-color: rgb(200, 200, 200);")
            self.manual_mode()
        else:
            self.ui.cbb_manual_mode.setEnabled(True)
            self.ui.cbb_manual_mode.setStyleSheet("background-color: rgb(255, 255, 255);")
            self.ui.le_auto_mode.setEnabled(False)
            self.ui.le_auto_mode.setStyleSheet("background-color: rgb(200, 200, 200);")
            self.ui.pbtn_add.setEnabled(False)
            self.ui.pbtn_add.setStyleSheet("background-color: rgb(200, 200, 200);")
            self.manual_mode()
        
    # Manual mode function
    def manual_mode(self):
        if self.ui.cbb_manual_mode.currentText() == "Cartesian" and self.ui.cbb_control_mode.currentText() == "Manual":
            self.ui.le_input_x.setEnabled(True)
            self.ui.le_input_x.setStyleSheet("background-color: rgb(255, 255, 255);")
            self.ui.le_input_y.setEnabled(True)
            self.ui.le_input_y.setStyleSheet("background-color: rgb(255, 255, 255);")
            self.ui.le_input_z.setEnabled(True)
            self.ui.le_input_z.setStyleSheet("background-color: rgb(255, 255, 255);")
            self.ui.le_input_angle1.setEnabled(False)
            self.ui.le_input_angle1.setStyleSheet("background-color: rgb(200, 200, 200);")
            self.ui.le_input_angle2.setEnabled(False)
            self.ui.le_input_angle2.setStyleSheet("background-color: rgb(200, 200, 200);")
            self.ui.le_input_angle3.setEnabled(False)
            self.ui.le_input_angle3.setStyleSheet("background-color: rgb(200, 200, 200);")   
        elif self.ui.cbb_manual_mode.currentText() == "Angle" and self.ui.cbb_control_mode.currentText() == "Manual":
            self.ui.le_input_angle1.setEnabled(True)
            self.ui.le_input_angle1.setStyleSheet("background-color: rgb(255, 255, 255);")
            self.ui.le_input_angle2.setEnabled(True)
            self.ui.le_input_angle2.setStyleSheet("background-color: rgb(255, 255, 255);")
            self.ui.le_input_angle3.setEnabled(True)
            self.ui.le_input_angle3.setStyleSheet("background-color: rgb(255, 255, 255);")
            self.ui.le_input_x.setEnabled(False)
            self.ui.le_input_x.setStyleSheet("background-color: rgb(200, 200, 200);")
            self.ui.le_input_y.setEnabled(False)
            self.ui.le_input_y.setStyleSheet("background-color: rgb(200, 200, 200);")
            self.ui.le_input_z.setEnabled(False)
            self.ui.le_input_z.setStyleSheet("background-color: rgb(200, 200, 200);")
        else:
            self.ui.le_input_x.setEnabled(False)
            self.ui.le_input_x.setStyleSheet("background-color: rgb(200, 200, 200);")
            self.ui.le_input_y.setEnabled(False)
            self.ui.le_input_y.setStyleSheet("background-color: rgb(200, 200, 200);")
            self.ui.le_input_z.setEnabled(False)
            self.ui.le_input_z.setStyleSheet("background-color: rgb(200, 200, 200);")
            self.ui.le_input_angle1.setEnabled(False)
            self.ui.le_input_angle1.setStyleSheet("background-color: rgb(200, 200, 200);")
            self.ui.le_input_angle2.setEnabled(False)
            self.ui.le_input_angle2.setStyleSheet("background-color: rgb(200, 200, 200);")
            self.ui.le_input_angle3.setEnabled(False)
            self.ui.le_input_angle3.setStyleSheet("background-color: rgb(200, 200, 200);")

    # Connect serial function
    def connect_serial(self):
        port = self.ui.cbb_port.currentText()
        baudrate = self.ui.cbb_baud.currentText()
        try:
            self.serial_reader = SerialReader(port, int(baudrate))
            self.serial_reader.data_received.connect(self.display_data)  
            self.serial_reader.start()  

            self.show_message("Success", f"Successfully connected to {port} at {baudrate} baud.")
            self.status = "Connected"

        except serial.SerialException as e:
            self.show_message("Error", f"Error connecting to {port}: {e}")
            self.status = "Disconnected"
            
    # Disconnect serial function
    def disconnect_serial(self):
        if self.serial_connection and self.serial_connection.isOpen():
            self.serial_connection.close()
            if self.serial_reader:
                self.serial_reader.stop()
                self.serial_reader = None
            self.show_message("Success", "Successfully disconnected.")
            self.status = "Disconnected"  
        else:
            self.show_message("Error", "No connection to disconnect.")
            
    # Pause serial function
    def pause_serial(self):
        if self.serial_connection and self.serial_connection.isOpen():
            self.show_message("Pause", "Connection paused.")
            self.status = "Pause"  
        else:
            print("No connection to pause.")
            self.show_message("Error", "No connection to pause.")
            
    # Process serial data and display angles and coordinates
    def process_serial_data(self, data):
        if len(data) == 12:
            angles = struct.unpack('fff', data)
            self.display_data(*angles)
    
    # Display angles and coordinates
    def display_data(self, angle1, angle2, angle3):
        self.ui.lb_display_angle1.setText(f"{angle1:.2f}°")
        self.ui.lb_display_angle2.setText(f"{angle2:.2f}°")
        self.ui.lb_display_angle3.setText(f"{angle3:.2f}°")
        print(f"Angle 1: {angle1:.2f}°, Angle 2: {angle2:.2f}°, Angle 3: {angle3:.2f}°")
        
        # # Convert angles to x, y, z coordinates
        # x, y, z = cv._Forward_Kinematic(angle1, angle2, angle3)

        # # Display x, y, z values on the App
        # self.ui.lb_display_x.setText(f"{x:.2f}")
        # self.ui.lb_display_y.setText(f"{y:.2f}")
        # self.ui.lb_display_z.setText(f"{z:.2f}")
        
    # Sending data to the serial port
    def send_data(self):
        # Get data from the input fields
        try:
            angle1 = float(self.ui.le_input_angle1.text())
            angle2 = float(self.ui.le_input_angle2.text())
            angle3 = float(self.ui.le_input_angle3.text())
            
            x = float(self.ui.le_input_x.text())
            y = float(self.ui.le_input_y.text())
            z = float(self.ui.le_input_z.text())
            
            # Creat object "_element" fron the converted.py
            element_angle = float(cv._element(angle1, angle2, angle3))
            element_xyz = float(cv._element(cv._Inverse_Kinematic(x, y, z)))
            
            # Create object "_element" from the converted.py
            element_angle = cv._element(angle1, angle2, angle3)
            element_xyz = cv._element(cv._Inverse_Kinematic(x, y, z))
            
            # Convert to string and encode before sending
            element_angle_str = f"{element_angle[0]},{element_angle[1]},{element_angle[2]}"
            element_xyz_str = f"{element_xyz[0]},{element_xyz[1]},{element_xyz[2]}"
            
            # Append the object to the _Segment list
            # cv._Segment.append(element_angle)
            # cv._Segment.append(element_xyz)
            port = self.ui.cbb_port.currentText()
            baud_rate = self.ui.cbb_baud.currentText()
            ser = serial.Serial(port, baud_rate)
            
            ser.write(element_xyz.encode())
            ser.write(element_angle.encode())
            
            # print(cv._Segment)
            
        except ValueError:
            self.show_message("Error", "Please enter valid angles.")
            return    
    
    # Add file function
    def add_file(self):
        file_dialog = QtWidgets.QFileDialog(self)
        file_dialog.setFileMode(QtWidgets.QFileDialog.FileMode.ExistingFile)
        file_dialog.setNameFilter("Text Files (*.txt);;All Files (*)")
        if file_dialog.exec():
            file_path = file_dialog.selectedFiles()[0]
            self.ui.le_auto_mode.setText(file_path)
            cv._File_Reading(file_path)

        
    # Show messages tab
    def show_message(self, title, message):
        msg_box = QMessageBox()
        msg_box.setWindowTitle(title)
        msg_box.setText(message)
        msg_box.setIcon(QMessageBox.Icon.Information if title == "Success" else QMessageBox.Icon.Critical)
        msg_box.setStandardButtons(QMessageBox.StandardButton.Ok)
        msg_box.exec()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec())
