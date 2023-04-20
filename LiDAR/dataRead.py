import serial
import matplotlib.pyplot as plt
import statistics
import math

class LidarData():
    
    def __init__(self):        
        self.DATA_LENGTH = 7        # data length : angle, speed, distance 1 - 4, checksum
        self.MAX_DISTANCE = 3000    # in mm
        self.MIN_DISTANCE = 100     # in mm
        self.port = 'COM7'          # SPECIFY serial port 
        self.MAX_DATA_SIZE = 360    # resolution : 1 degree
        self.ser = None
        self.BAUDRATE = 115200

        self.data = {   # sensor data 
            'angles'    : [],
            'distances' : [],
            'speed'     : [],
            'signal_strength' : [], # TODO:
            'checksum'  : [] 
        }
        
        # setup plot
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_rmax(300)
        
        self.connectSerial(self.port, self.BAUDRATE)   # setup serial communication
        
    def connectSerial(self, port: str, baudrate: int) -> bool:
        try: 
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.ser.reset_input_buffer()
            print(f'Serial connection established @ {port}')
            return True # return True to confirm connection
        except Exception as e: 
            print(e)
            return False
        
    def plotData(self) -> None: # plot data on a polar plot
        angles, distances = [], []
        
        for p in range(3, len(self.data['angles'])-3):
            
            # ------ not sure what I'm doing here... ----------------
            # TODO: implement some sort of filter to remove outliers
            if (p > len(self.data['angles']) - 3) : break
            sample = self.data['distances'][p-3:p+3]
            std = statistics.stdev(sample)
            if abs(self.data['distances'][p]-statistics.mean(sample)) < std:
                angles.append(self.data['angles'][p])
                distances.append(self.data['distances'][p])
            # ------------ filter END -------------------------------
                
        self.ax.clear() # clear current plot
        plt.plot(angles, distances, ".")    # plot the points
        self.ax.set_rmax(self.MAX_DISTANCE)
        self.data['angles'].clear()
        self.data['distances'].clear()
        plt.draw()
        plt.pause(0.001)
                                
                
    def updateData(self) -> None: 
        while True:
            try: 
                if self.ser.in_waiting > 0:
                    
                    try: # try read serial inputs; if unsuccessful, ignore the current input data
                        line = self.ser.readline().decode().rstrip()
                        sensorData = line.split('\t')  
                        
                    except: # ignore if the data is invalid 
                        continue
                    
                    # convert string to float, then publish topic
                    if len(sensorData) == self.DATA_LENGTH:
                        
                        for i in range(2,6):    # split into four data points
                            try:           
                                # note: angles comes in increment of 4 degrees as each packet contains 4 readings                    
                                angle = (int(sensorData[0]) + i - 1) * math.pi / 180  # angle in radians
                                dist = float(sensorData[i])   # distance in mm
                                print(f'speed : {int(sensorData[1])} RPM, angle : {round(angle * 180 / math.pi)}, dist : {round(dist)}')
                            except: continue
                            
                            # if the data is valid, update sensor data
                            if dist >= self.MIN_DISTANCE and dist <= self.MAX_DISTANCE:
                                
                                # store angular data in radians
                                self.data['angles'].append(angle)
                                
                                # store radial data in mm
                                self.data['distances'].append(dist)
                                
                                # store checksum data for each measurement
                                self.data['checksum'].append(sensorData[-1])

                                # store speed data in RPM
                                self.data['speed'].append(sensorData[1])
                                
                            if len(self.data['angles']) == self.MAX_DATA_SIZE:  # if enough data is available, plot data
                                self.plotData()
                                
            except KeyboardInterrupt:
                exit()

    
    def getDistances(self) -> list: return self.data['distances']
    
    def getAngles(self) -> list: return self.data['angles']
    
if __name__ == '__main__':
    sensor = LidarData()
    sensor.updateData()