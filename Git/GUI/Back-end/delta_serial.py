import sys
import serial.tools.list_ports
import struct
import serial
import os
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
import converted as cv
from serial import PARITY_EVEN, STOPBITS_ONE


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
        self.wait()

class SerialHandle(QThread):
    # Package data received from delta
    data_received = pyqtSignal(float, float, float)  

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port 
        self.baudrate = baudrate
        self.parity = PARITY_EVEN
        self.stopbits = STOPBITS_ONE
        self.is_running = True
        self.is_paused = False
        self.ser = None
    # Receiving data from delta
    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, self.parity, self.stopbits, timeout=1)
            print(f"Successfully connected to {self.port} at baud rate {self.baudrate}")

            while self.is_running:
                if not self.is_paused and self.ser.in_waiting >= 13: 
                    # Read 12 bytes of data (3 floats)
                    data = self.ser.read(12)  
                    # Read '\n' character
                    terminator = self.ser.read(1)  
                    if terminator == b'\n':
                        # Unpack the 3 floats
                        angles = struct.unpack('fff', data) 
                        self.data_received.emit(*angles)  
                    else:
                        print("Invalid terminator received")

        except serial.SerialException as e:
            print(f"Error: {e}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("Connection closed.")
                
    def stop(self):
        self.is_running = False
        self.is_paused = False
        self.wait()
    
    def pause(self):
        self.is_paused = True
    
    def resume(self):
        self.is_paused = False

    # Flip byte order in Float
    def float_to_uint8_array_flipped(self, value):
    # Convert the float to a 4-byte array
        # '<f' = little-endian single-precision float
        packed = struct.pack('<f', value)
        # Convert bytes to a list of uint8 values
        uint8_array = list(packed)  
        # Reverse the order of the array
        flipped_array = uint8_array[::-1]
        return flipped_array
    
    # Sending data to delta
    def send_data(self, mode, para1, para2, para3):
        try:
            if self.ser and self.ser.is_open:
                # If mode is Cartesian, inversing x, y, z
                if mode == "Cartesian":
                    element = cv._element(cv._Inverse_Kinematic(para1, para2, para3))
                else:
                    element = cv._element(para1, para2, para3)
                # Package the data
                bytes_to_send = []
                for i in range(1, 4):
                    value = element._get_Value(i)
                    bytes_to_send.extend(self.float_to_uint8_array_flipped(value))
                    
                # Send all data at once
                self.ser.write(bytearray(bytes_to_send))
                # Sending '\n' character to indicate the end of the data
                self.ser.write(b'\n')
                
                #print(f"Sent: {element_str_1}")
                # Sending the data
                # self.ser.write(bytearray(temporary_storage))
                
            else:
                print("Serial port is not open.")
        except serial.SerialException as e:
            print(f"Serial error: {e}")
        except Exception as e: 
            print(f"Error: {e}")
            
