# -*- coding: utf-8 -*-
import math
import queue
import serial
import struct
import threading

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Button, TextBox
mpl.rcParams['agg.path.chunksize'] = 10000

class PenConGui(object):
    
    def __init__(self):
        self.fig = plt.figure(constrained_layout=True)
        self.fig.suptitle("Pendulum Controller")
        
        gs = GridSpec(5, 5, figure=self.fig, height_ratios=[.225,.225,.225,.225,.1])
        
        self.ax_position = self.fig.add_subplot(gs[:1, :])
        self.ax_position.set_ylim([-0.20, 0.20])
        
        self.ax_velocity = self.fig.add_subplot(gs[1:2, :])
        self.ax_velocity.set_ylim([-0.020, 0.020])
        
        self.ax_command = self.fig.add_subplot(gs[2:3, :])
        self.ax_command.set_ylim([-1, 1])
        
        self.ax_angle = self.fig.add_subplot(gs[3:4, :])
        self.ax_angle.set_ylim([-math.pi, math.pi])
        
        self.ax_button_start = self.fig.add_subplot(gs[4, :1])
        self.ax_button_turbo = self.fig.add_subplot(gs[4, 1:2])
        self. ax_button_calib = self.fig.add_subplot(gs[4, 2:3])
        self.ax_button_meta = self.fig.add_subplot(gs[4, 3:4])
        self.ax_button_exit = self.fig.add_subplot(gs[4, 4:])
        
        self.button_start = Button(self.ax_button_start, 'Start')
    
        self.button_turbo = Button(self.ax_button_turbo, 'Turbo')
    
        self.button_calib = Button(self.ax_button_calib, 'Calibrate')
        
        self.button_exit = Button(self.ax_button_exit, 'Exit')

        

class PendulumController(object):
    RADS_PER_COUNT = math.pi/512.0
    METERS_PER_COUNT = 0.00009347
        
    MODE_BUTTON_CONTROL =           b'\x00'
    MODE_CALIBRATE =                b'\x01' 
    MODE_UART_CONTROL =             b'\x02' 
    MODE_COSINE_CONTROL =           b'\x03'
    MODE_STEP_CONTROL =             b'\x04'
    MODE_PID_ANGLE_SPEED_CONTROL =  b'\x05'
    MODE_PID_ANGLE_POS_CONTROL =    b'\x06'
    
    MESSAGE_HEADER =                b'DEF'
    
    STATUS_HEADER = b'ABC'
    STATUS_FORMAT = "<iiiiic"
    STATUS_LENGTH = struct.calcsize(STATUS_FORMAT)+len(STATUS_HEADER)
    
    MAX_DATAPOINTS = 1000
    PLOT_WINDOW = 100
    
    def __init__(self, port='COM7'):
        self.ser = serial.Serial(port=port,
                            baudrate=115200,
                            bytesize=8,
                            parity='N',
                            stopbits=1,
                            timeout=None)

        self.gui = PenConGui()
        self.gui.button_calib.on_clicked(self.calibrate)
        self.gui.button_exit.on_clicked(self.shutdown)
        
        
        x = np.arange(0, self.PLOT_WINDOW)
        self.angle_plot, = self.gui.ax_angle.plot(x, x)
        self.motor1_counts_plot, = self.gui.ax_position.plot(x, x)
        self.motor1_cps_plot, = self.gui.ax_velocity.plot(x, x)
        self.motor1_command_plot, = self.gui.ax_command.plot(x, x)
        
        self.ani = animation.FuncAnimation(self.gui.fig, 
                                           self.animate_and_store, 
                                           init_func=self.init, 
                                           interval=10,
                                           blit=True,
                                           frames=self.MAX_DATAPOINTS, 
                                           repeat=True)
    
        self.dataqueue = queue.Queue(maxsize=self.PLOT_WINDOW)
        
        self.plotdata = np.empty((self.PLOT_WINDOW, 5))
        
        self.stopping = threading.Event()   
        self.serial_read_thread = threading.Thread(target=self.serial_read_loop)
        self.serial_read_thread.start()
        
        self.data_write_thread = threading.Thread(target=self.data_write_loop)
        self.data_write_thread.start()
                
    def __del__(self):
        self.close_serial()
    
    def close_serial(self):
        self.ser.close()
        print("serial port closed")
        
        
    def set_button_mode(self):
        self.ser.write(self.MESSAGE_HEADER)
        self.ser.write(self.MODE_BUTTON_CONTROL)
        self.ser.write(b'\n')
    
    def calibrate(self, event=None):
        self.ser.write(self.MESSAGE_HEADER)
        self.ser.write(self.MODE_CALIBRATE)
        self.ser.write(b'\n')
        
    def set_motor_speed(self, normalized_speed):
        self.ser.write(self.MESSAGE_HEADER)
        self.ser.write(self.MODE_UART_CONTROL)
        self.ser.write(bytes(struct.pack('f', normalized_speed)))
        self.ser.write(b'\n')
    
    def set_cosine_mode(self, cosinetriplets):
        #cosinetriplets should be an iterable of tuple triplets for magnitude, freq., and phase
        self.ser.write(self.MESSAGE_HEADER)
        self.ser.write(self.MODE_COSINE_CONTROL)
        for triplet in cosinetriplets:
            self.ser.write(bytes(struct.pack('fff', triplet[0], triplet[1], triplet[2])))
        self.ser.write(b'\n')

    def set_pid_speed_mode(self, kP, kI, kD):
        self.ser.write(self.MESSAGE_HEADER)
        self.ser.write(self.MODE_PID_ANGLE_SPEED_CONTROL)
        self.ser.write(bytes(struct.pack('fff', kP, kI, kD)))
        self.ser.write(b'\n')
    
    def set_pid_position_mode(self, kP, kI, kD):
        self.ser.write(self.MESSAGE_HEADER)
        self.ser.write(self.MODE_PID_ANGLE_POS_CONTROL)
        self.ser.write(bytes(struct.pack('fff', kP, kI, kD)))
        self.ser.write(b'\n')
    
    def serial_read_loop(self):
        while(not self.stopping.wait(0.001)):
            if self.ser.in_waiting > self.STATUS_LENGTH:
                if self.ser.read(1) == b'A':
                    if (self.ser.read(2) == b'BC'):
                        line = self.ser.readline()
                        if(len(line)==21):
                            rawvalues = struct.unpack(self.STATUS_FORMAT, line)[:-1]
                            print(self.convert_values(rawvalues))
                            self.dataqueue.put(self.convert_values(rawvalues))
        self.close_serial()
                            
    def data_write_loop(self):
        while(not self.stopping.wait(0.001)):
            try:
                newrow = self.dataqueue.get_nowait()
                self.plotdata = np.vstack((self.plotdata[1:, :], newrow))
            except queue.Empty:
                pass
                            
    def shutdown(self, event=None):
        self.stopping.set()
        plt.close('all')
        
    def convert_values(self, rawvalues):
        timestamp_s = rawvalues[0]/1000.0
        angle_rad = rawvalues[1]*self.RADS_PER_COUNT
        position_m = rawvalues[2]*self.METERS_PER_COUNT
        velocity_mps = rawvalues[3]*self.METERS_PER_COUNT
        motor_command  = rawvalues[4]/10000.0
        
        return timestamp_s, angle_rad, position_m, velocity_mps, motor_command
        
    
    def init(self):
        self.angle_plot.set_ydata([np.nan] * self.PLOT_WINDOW)
        self.motor1_counts_plot.set_ydata([np.nan] * self.PLOT_WINDOW)
        self.motor1_cps_plot.set_ydata([np.nan] * self.PLOT_WINDOW)
        self.motor1_command_plot.set_ydata([np.nan] * self.PLOT_WINDOW)
        return self.angle_plot, self.motor1_counts_plot, self.motor1_cps_plot, self.motor1_command_plot
            
    def animate_and_store(self, i):
        self.angle_plot.set_ydata(self.plotdata[:,1])
        self.motor1_counts_plot.set_ydata(self.plotdata[:,2])
        
        self.motor1_cps_plot.set_data(np.transpose(self.plotdata[:,(0,3)]))
        self.motor1_command_plot.set_data(np.transpose(self.plotdata[:,(0,4)]))
        
        return self.angle_plot, self.motor1_counts_plot, self.motor1_cps_plot, self.motor1_command_plot
    
if __name__ == "__main__":
    pendcon = PendulumController(port="COM8")
    
    plt.show()