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
import delta_serial as ser
        

class Window(QWidget):
    def __init__(self):
        super().__init__()
        self.ui = Ui_DeltaGUI()
        self.ui.setupUi(self)
        self.status = "Disconnected"
        
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

        # ENABLE OBJECT BEFORE CONNECTING
        ###########################################################################################################
        self.ui.cbb_control_mode.setEnabled(False)
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
        self.serial_scanner = ser.SerialPortScanner()
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
            self.serial_handle = ser.SerialHandle(port, int(baudrate))
            self.serial_handle.data_received.connect(self.display_data)  
            self.serial_handle.start()  
            self.show_message("Success", f"Successfully connected to {port} at {baudrate} baud.")
            self.status = "Connected"
            self.ui.cbb_control_mode.setEnabled(True)
            self.ui.cbb_control_mode.setStyleSheet("background-color: rgb(255, 255, 255);")
            
        except serial.SerialException as e:
            self.show_message("Error", f"Error connecting to {port}: {e}")
            self.status = "Disconnected"
    
    # Disconnect serial function
    def disconnect_serial(self):
        try: 
            if hasattr(self, 'serial_handle') and self.serial_handle.is_running:
                self.serial_handle.stop()
                self.serial_handle.wait()
                self.show_message("Success", "Disconnected successfully.")
                self.status = "Disconnected" 
                self.ui.cbb_control_mode.setEnabled(False)
                self.ui.cbb_control_mode.setStyleSheet("background-color: rgb(200, 200, 200);")
            else:
                self.show_message("Warning", "No active serial connection to disconnect.")
        except Exception as e:
            self.show_message("Error", f"Error while disconnecting: {e}")            

    # Pause serial function
    def pause_serial(self):
        try:
            if hasattr(self, 'serial_handle') and self.serial_handle.is_running:
                self.serial_handle.pause()
                self.show_message("Success", "Serial reading paused.")
                self.status = "Paused"
            else:
                self.show_message("Warning", "No active serial connection to pause.")
        except Exception as e:
            self.show_message("Error," f"Error while pause: {e}")
    
            
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
        if hasattr(self, 'serial_connection'):
            self.serial_handle.pause()
        # Get data from the input fields
        try:
            if self.ui.cbb_manual_mode.currentText() == "Angle" and self.ui.cbb_control_mode.currentText() == "Manual" and self.status == "Connected": 
                para1 = float(self.ui.le_input_angle1.text())
                para2 = float(self.ui.le_input_angle2.text())
                para3 = float(self.ui.le_input_angle3.text())
            elif self.ui.cbb_manual_mode.currentText() == "Cartesian" and self.ui.cbb_control_mode.currentText() == "Manual" and self.status == "Connected": 
                para1 = float(self.ui.le_input_x.text())
                para2 = float(self.ui.le_input_y.text())
                para3 = float(self.ui.le_input_z.text())
            
            port = self.ui.cbb_port.currentText()
            baud_rate = int(self.ui.cbb_baud.currentText())
            print(f"Connecting to port: {port} with baud rate: {baud_rate}")
            self.serial_handle.send_data(self.ui.cbb_manual_mode.currentText(), para1, para2, para3)
           
            
        except serial.SerialException as e:
            print(f"Serial error: {e}")
        except Exception as e:
            print(f"Error: {e}")   
        finally:
            # Tiếp tục việc nhận dữ liệu
            if hasattr(self, 'serial_connection'):
                self.serial_connection.resume()
    
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
