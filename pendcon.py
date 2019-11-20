# -*- coding: utf-8 -*-
import datetime
import math
import queue
import serial
import struct
import threading

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Button, TextBox

import matplotlib.style as mplstyle
mplstyle.use('fast')

class PenConGui(object):
    
    def __init__(self):
        self.fig = plt.figure(tight_layout=False)
        
        gs = GridSpec(4, 6, figure=self.fig, height_ratios=[.4,.4,.1,.1])
        
        self.ax_position = self.fig.add_subplot(gs[:1, :])
        self.ax_position.set_ylim([-0.5, 0.50])
        
        self.ax_velocity = self.fig.add_subplot(gs[1:2, :])
        self.ax_velocity.set_ylim([-2, 2])
                
        self.ax_notebox = self.fig.add_subplot(gs[2:3, :])
        self.textbox_note = TextBox(self.ax_notebox, label=None, initial="Notes")
        
        
        self.ax_button_record = self.fig.add_subplot(gs[-1, :1])
        self.ax_button_disable = self.fig.add_subplot(gs[-1, 1:2])
        self.ax_button_cosine = self.fig.add_subplot(gs[-1, 2:3])
        self.ax_button_step = self.fig.add_subplot(gs[-1, 3:4])
        self.ax_button_calib = self.fig.add_subplot(gs[-1, 4:5])
        self.ax_button_exit = self.fig.add_subplot(gs[-1, 5:])
        
        self.button_record = Button(self.ax_button_record, 'Record')
        self.button_record.hovercolor = "red"
        self.button_disable = Button(self.ax_button_disable, 'Disable')
        self.button_cosine = Button(self.ax_button_cosine, 'Cosine')
        self.button_step = Button(self.ax_button_step, 'Step')
        self.button_calib = Button(self.ax_button_calib, 'Calibrate')    
        self.button_exit = Button(self.ax_button_exit, 'Exit')


class PendulumController(object):
    PENDULUM_LENGTH_M = 0.335
    RADS_PER_COUNT = math.pi/512.0
    METERS_PER_COUNT = 0.00009347
        
    MODE_BUTTON_CONTROL =           b'\x00'
    MODE_CALIBRATE =                b'\x01' 
    MODE_UART_CONTROL =             b'\x02' 
    MODE_COSINE_CONTROL =           b'\x03'
    MODE_STEP_CONTROL =             b'\x04'
    MODE_PID_ANGLE_SPEED_CONTROL =  b'\x05'
    MODE_PID_ANGLE_POS_CONTROL =    b'\x06'
    
    ACTION_CODE =                   b'\x0A'
    ACTION_DISABLE =                b'\x00' 
    ACTION_ENABLE =                 b'\x01' 
    ACTION_RESET_CLOCK =            b'\x02' 
    
    MESSAGE_HEADER =                b'DEF'
    
    STATUS_HEADER = b'ABC'
    STATUS_FORMAT = "<fiiiic"
    STATUS_LENGTH = struct.calcsize(STATUS_FORMAT)+len(STATUS_HEADER)
    
    MAX_DATAPOINTS = 1000
    PLOT_WINDOW = 100
    
    def __init__(self, port='COM1', cosines=[(0,0,0)], step_fns=[(0,0)]):
        self.ser = serial.Serial(port=port,
                            baudrate=115200,
                            bytesize=8,
                            parity='N',
                            stopbits=1,
                            timeout=None)
        
        self.cosines = cosines
        self.step_fns = step_fns

        self.gui = PenConGui()
        self.gui.button_record.on_clicked(self.record_button_cb)
        self.gui.button_disable.on_clicked(self.action_disable)
        self.gui.button_step.on_clicked(self.step_button_cb)
        self.gui.button_calib.on_clicked(self.calibrate)
        self.gui.button_exit.on_clicked(self.shutdown)
        self.gui.button_cosine.on_clicked(self.cosine_button_cb)
        
        
        x = np.arange(0, self.PLOT_WINDOW)
        self.motor1_counts_plot, = self.gui.ax_position.plot(x, x)
        self.angle_plot, = self.gui.ax_position.plot(x, x)
        self.gui.ax_position.legend(["Cart", "Endpoint"])
        
        self.motor1_cps_plot, = self.gui.ax_velocity.plot(x, x)
        self.motor1_command_plot, = self.gui.ax_velocity.plot(x, x)
        
        self.ani = animation.FuncAnimation(self.gui.fig, 
                                           self.animate_and_store, 
                                           init_func=self.init, 
                                           interval=100,
                                           blit=False,
                                           frames=self.data_write_loop, 
                                           repeat=True)
    
        self.dataqueue = queue.Queue(maxsize=self.PLOT_WINDOW)
        self.plotdata = np.empty((self.PLOT_WINDOW, 5))
        
        self.recorddata = np.empty((self.MAX_DATAPOINTS, 5))
        self.record_count = 0
        self.recording = threading.Event()
        
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
        self.action_enable()
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
            print(triplet)
            self.ser.write(bytes(struct.pack('fff', triplet[0], triplet[1], triplet[2])))
        self.ser.write(b'\n')
        
    def cosine_button_cb(self, event):
        self.set_cosine_mode(self.cosines)
        self.action_enable()
        self.action_reset_clock()
        
    def set_step_mode(self, steppairs):
        #steppairs should be an iterable of tuple pairs for magnitude, and phase
        self.ser.write(self.MESSAGE_HEADER)
        self.ser.write(self.MODE_STEP_CONTROL)
        for pair in steppairs:
            print(pair)
            self.ser.write(bytes(struct.pack('ff', pair[0], pair[1])))
        self.ser.write(b'\n')
        
    def step_button_cb(self, event):
        self.set_step_mode(self.step_fns)
        self.action_enable()
        self.action_reset_clock()
    
    def record_button_cb(self, event):
        self.action_enable()
        self.action_reset_clock()
        
        self.gui.button_record.set_active(False)
        
        self.record_count = 0;
        self.recording.set()
        
    
    def action_enable(self, event=None):
        self.ser.write(self.MESSAGE_HEADER)
        self.ser.write(self.ACTION_CODE)
        self.ser.write(self.ACTION_ENABLE)
        self.ser.write(b'\n')
    
    def action_disable(self, event=None):
        self.ser.write(self.MESSAGE_HEADER)
        self.ser.write(self.ACTION_CODE)
        self.ser.write(self.ACTION_DISABLE)
        self.ser.write(b'\n')
        
    def action_reset_clock(self):
        self.ser.write(self.MESSAGE_HEADER)
        self.ser.write(self.ACTION_CODE)
        self.ser.write(self.ACTION_RESET_CLOCK)
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
        while(not self.stopping.wait(0.0)):
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
        while(not self.stopping.wait(0.0)):
            while(not self.dataqueue.empty()):
                try:
                    newrow = self.dataqueue.get_nowait()
                    self.plotdata = np.vstack((self.plotdata[1:, :], newrow))
                    
                    if self.recording.is_set():
                        self.recorddata[self.record_count, :] = newrow
                        self.record_count+=1
                        
                        if self.record_count >= self.MAX_DATAPOINTS:
                            self.recording.clear()
                            
                            np.savetxt("pendulum_data_"+datetime.datetime.now().strftime("%B%d%Y_%I%M%p")+".csv",
                                       self.recorddata, delimiter=',',
                                       header=self.gui.textbox_note.text+"\n"+
                                       "timestamp_s, angle_rad, position_m, velocity_mps, motor_command")
                            
                            self.gui.button_record.set_active(True)
                            
                except queue.Empty:
                    pass
            yield self.plotdata
                            
    def shutdown(self, event=None):
        self.stopping.set()
        plt.close('all')

    def convert_values(self, rawvalues):
        timestamp_s = rawvalues[0]
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
    
    def animate_and_store(self, data):
        self.gui.ax_position.set_xlim([data[-1, 0]-(self.PLOT_WINDOW*0.01), data[-1, 0]])
        self.angle_plot.set_data(data[:,0],np.sin(data[:,1])*self.PENDULUM_LENGTH_M)
        self.motor1_counts_plot.set_data(data[:,0], data[:,2])
#        self.angle_plot.set_ydata(np.sin(data[:,1])*self.PENDULUM_LENGTH_M)
#        self.motor1_counts_plot.set_ydata(data[:,2])
        self.motor1_cps_plot.set_ydata(data[:,3])
        self.motor1_command_plot.set_ydata(data[:,4])
        
        return self.angle_plot, self.motor1_counts_plot, self.motor1_cps_plot, self.motor1_command_plot, self.gui.ax_position
    
if __name__ == "__main__":
    pendcon = PendulumController(port="COM10",
                                 cosines=[(0.4, 10, 0)], step_fns=[(0.5, 1), (-0.5, 1.5), (0.5, 10), (-0.5, 10.5)])
    plt.show()