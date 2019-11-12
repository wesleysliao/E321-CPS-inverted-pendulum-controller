# -*- coding: utf-8 -*-

import struct
import serial
import threading
import queue

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation



class PendulumController(object):
    MESSAGE_HEADER =                b'DEF'
    MODE_BUTTON_CONTROL =           b'\x00'
    MODE_CALIBRATE =                b'\x01' 
    MODE_UART_CONTROL =             b'\x02' 
    MODE_COSINE_CONTROL =           b'\x03'
    MODE_STEP_CONTROL =             b'\x04'
    MODE_PID_ANGLE_SPEED_CONTROL =  b'\x05'
    MODE_PID_ANGLE_POS_CONTROL =    b'\x06'
    
    STATUS_FORMAT = "<iiiiic"
    
    MAX_DATAPOINTS = 1000
    PLOT_WINDOW = 100
    
    def __init__(self, port='COM7', ax=None):
        self.ser = serial.Serial(port=port,
                            baudrate=115200,
                            bytesize=8,
                            parity='N',
                            stopbits=1,
                            timeout=None)
        
        self.data = np.empty((self.MAX_DATAPOINTS, 5), dtype=int)
        self.datapoints = 0
        
        self.dataqueue = queue.Queue(maxsize=self.MAX_DATAPOINTS)
        
        x = np.arange(0, self.PLOT_WINDOW)
        self.angle_plot, = ax.plot(x, np.sin(x))
        self.motor1_counts_plot, = ax.plot(x, np.sin(x+1))
        self.motor1_cps_plot, = ax.plot(x, np.sin(x+2))
        self.motor1_command_plot, = ax.plot(x, np.sin(x+3))
                
    def __del__(self):
        self.close_serial()
    
    def close_serial(self):
        self.ser.close()
        print("serial port closed")
        
        
    def set_button_mode(self):
        self.ser.write(self.MESSAGE_HEADER)
        self.ser.write(self.MODE_BUTTON_CONTROL)
        self.ser.write(b'\n')
    
    def calibrate(self):
        self.ser.write(self.MESSAGE_HEADER)
        self.ser.write(self.MODE_CALIBRATE)
        self.ser.write(b'\n')
        
        print("SEND CALIBRATE")
       
        while True:
            try:
                line = self.ser.readline()
                if line == b'READY\r\n':
                    print("CALIBRATION COMPLETED")
                    break
            except KeyboardInterrupt:
                break
            except:
                pass
        
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
    
    def print_console(self):
        while True:
            try:
                self.read_status()
                #print(self.ser.read(1))
            except KeyboardInterrupt:
                break
            except:
                pass
    
    def read_status(self):
        while self.ser.read(1) != b'A':
            pass
        if (self.ser.read(2) == b'BC'):
            line = self.ser.readline()
            data = struct.unpack(self.STATUS_FORMAT, line)
            for value in data[:4]:
                print(value, end=',')
            print(data[4])
            return data
    
    def captureloop(self):
        self.ser.reset_input_buffer()
        while self.datapoints < self.MAX_DATAPOINTS:
            try:
                self.dataqueue.put(self.read_status())
            except:
                pass
        
        self.close_serial()
    
    def start_capture(self):
        cap = threading.Thread(target=self.captureloop)
        cap.start()
        return cap
    
    def init(self):
        self.angle_plot.set_ydata([np.nan] * self.PLOT_WINDOW)
        self.motor1_counts_plot.set_ydata([np.nan] * self.PLOT_WINDOW)
        self.motor1_cps_plot.set_ydata([np.nan] * self.PLOT_WINDOW)
        self.motor1_command_plot.set_ydata([np.nan] * self.PLOT_WINDOW)
        return self.angle_plot, self.motor1_counts_plot, self.motor1_cps_plot, self.motor1_command_plot
            
    def animate_and_store(self, i):
        self.data[self.datapoints, :] =  self.dataqueue.get()[:5]
        self.datapoints += 1
        if i > self.PLOT_WINDOW:
            self.angle_plot.set_ydata(self.data[i-self.PLOT_WINDOW:i, 1]/512.0)
            self.motor1_counts_plot.set_ydata(self.data[i-self.PLOT_WINDOW:i, 2]/12000.0)
            self.motor1_cps_plot.set_ydata(self.data[i-self.PLOT_WINDOW:i, 3]/20000.0)
            self.motor1_command_plot.set_ydata(self.data[i-self.PLOT_WINDOW:i, 4]/10000.0)
        else:
            self.angle_plot.set_ydata(self.data[:self.PLOT_WINDOW, 1]/512.0)
            self.motor1_counts_plot.set_ydata(self.data[:self.PLOT_WINDOW, 2]/12000.0)
            self.motor1_cps_plot.set_ydata(self.data[:self.PLOT_WINDOW, 3]/20000.0)
            self.motor1_command_plot.set_ydata(self.data[:self.PLOT_WINDOW, 4]/10000.0)
        return self.angle_plot, self.motor1_counts_plot, self.motor1_cps_plot, self.motor1_command_plot

    
    
if __name__ == "__main__":
    print("timestamp_us, angle_pot, motor1_counts, motor1_cps, motor1_command")
    fig, ax = plt.subplots()

    pendcon = PendulumController(ax=ax)
    
    capturethread = pendcon.start_capture()
    
    ani = animation.FuncAnimation(
        fig, pendcon.animate_and_store, init_func=pendcon.init, interval=10, blit=True, frames=pendcon.MAX_DATAPOINTS, repeat=False)
    
    plt.show()
    
#    while(capturethread.is_alive):
#        time.sleep(0.1)
#    np.savetxt("out.csv", pendcon.data, delimiter=',', header="timestamp_us, angle_pot, motor1_counts, motor1_cps, motor1_command")
    
    
    
    