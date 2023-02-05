import serial
import matplotlib.pyplot as plt
import statistics

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_rmax(300)
angles = []
distances = []

try:
    ser = serial.Serial('COM7', 115200, timeout=1)
    ser.reset_input_buffer()
    print('ok')
except: 
    raise
    print("error")
    
DATA_LENGTH = 7
count = 0

f = open("data_1.m","w")
f.write("data = [")

cnt = 0
cnt_ok = 0
n = 0
while True:
    try: 
        if ser.in_waiting > 0:
            # try read serial inputs
            try: 
                line = ser.readline().decode().rstrip()
                data = line.split('\t')  
            except: continue
            
            # convert string to float, then publish topic
            if len(data) == DATA_LENGTH:
                for i in range(2,6):
                    try: 
                        txt = f"{int(data[0])+i-1},{data[i]};\n"
                    except Exception as e: 
                        print(e)
                        continue
                    if float(data[i]) > 13 and float(data[i]) < 400:
                        angles.append((int(data[0])+i-1)*3.14159265/180)
                        distances.append(float(data[i]))
                    
                    print(txt)
                    f.write(txt)
                    # print(data[5])
                    # cnt_ok += int(data[5])
                    if len(angles) == 360: 
                        a, d = [], []
                        
                        for p in range(3, len(angles)-3):
                            if (p > len(angles) - 3) : break
                            sample = distances[p-3:p+3]
                            std = statistics.stdev(sample)
                            if abs(distances[p]-statistics.mean(sample)) < 1*std:
                                a.append(angles[p])
                                d.append(distances[p])
                        ax.clear()
                        ax.plot(a,d, ".")
                        ax.set_rmax(300)
                        angles.clear()
                        distances.clear()
                        plt.draw()
                        plt.pause(0.001)
            # print(cnt_ok/cnt * 100)
            continue
        
    except KeyboardInterrupt:
        f.write("];")
        f.close()
        exit()
    