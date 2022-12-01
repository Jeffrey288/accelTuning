import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import time
from multiprocessing import Process, Queue
from random import *
import serial
import math
import datetime
from matplotlib.animation import FFMpegWriter

# kp 600, kd 40, encoder

buff = ""
triggerTime = 0

sg_window = 7
sg_order = 2

def mainProcess(q):
    ENABLE_BT = False
 
    if ENABLE_BT:
        ser = serial.Serial(port = "COM7", baudrate = 115200, timeout = 0.3)

    start_s = time.time()
    veltimes = []
    vels = []
    raw_vels_x = []
    raw_vels_y = []
    acceltimes = []
    accels = []

    # from https://dsp.stackexchange.com/questions/26248/derive-velocity-and-acceleration-from-position
    def sg_filter(x, m, k=0):
        """
        x = Vector of sample times
        m = Order of the smoothing polynomial
        k = Which derivative
        """
        mid = len(x) // 2        
        a = x - x[mid]
        expa = lambda x: list(map(lambda i: i**x, a))
        A = np.r_[list(map(expa, range(0,m+1)))].transpose()
        Ai = np.linalg.pinv(A)

        return Ai[k]

    def smooth(x, y, size=sg_window, order=sg_order, deriv=1):

        n = len(x)
        m = size

        i = n-m-1
        start, end = i - m, i + m + 1
        f = sg_filter(x[start:end], order, deriv)
        return np.dot(f, y[start:end])

    def appendAccel():
        if (len(veltimes) <= sg_window*2):
            acceltimes.append(veltimes[-1])
            accels.append(0)
        else: # assuming a constant / average acceleration
            if ENABLE_BT:
                accels.append(np.dot([smooth(np.array(veltimes), np.array(raw_vels_x)),
                                        smooth(np.array(veltimes), np.array(raw_vels_y))], 
                                            [0, 1]))
            else:
                accels.append(smooth(np.array(veltimes), np.array(vels)))
            acceltimes.append(veltimes[-sg_window])

            # if veltimes[-1] != veltimes[-4]:
            #     accels.append((vels[-1] - vels[-4])/(veltimes[-1] - veltimes[-4])) 
            #     acceltimes.append((veltimes[-1] + veltimes[-4])/2)
            # else:
            #     acceltimes.append(veltimes[-1])
            #     accels.append(accels[-1]) 

    def decodeBuff():   
        global buff
        buff = ''.join(buff.split())
        while "]" in buff:
            term = buff.find("]")
            if term == -1:
                break # wtf this shouldn't happen
            temp = buff[:term+1]
            buff = buff[term+1:]

            try:
                if temp[:len("start:[")] != "start:[":
                    print("continued at start")
                    continue
                posInd = temp.find("pos:")
                if posInd == -1:
                    print("continued at pos")
                    continue
                posTemp = temp[posInd+len("pos:"):temp.find(";",posInd)]
                posTemp = posTemp.split(",")
                posTemp = list(map(float, posTemp))
                
                velInd = temp.find("vel:")
                if velInd == -1:
                    print("continued at vel")
                    continue
                velTemp = temp[velInd+len("vel:"):temp.find(";",velInd)]
                velTemp = velTemp.split(",")
                velTemp = list(map(float, velTemp))

                timeInd = temp.find("time:")
                if timeInd == -1:
                    print("continued at time")
                    continue
                timeTemp = temp[timeInd+len("time:"):temp.find(";",timeInd)]
                timeTemp = float(timeTemp)

                print(posTemp, velTemp, timeTemp)
                veltimes.append(timeTemp)
                vels.append(math.hypot(velTemp[0] + velTemp[1]))
                raw_vels_x.append(velTemp[0])
                raw_vels_y.append(velTemp[1])
                appendAccel()
                q.put([vels[-1], veltimes[-1], accels[-1], acceltimes[-1]])
                
            except Exception as e:
                print("continued at exception")
                print(e)
                continue

    def recvThread():
        global triggerTime, buff
        last_s = time.time()
        tempVel = 0
        while True:
            if ENABLE_BT:
                temp = ser.read()
                buff += temp.decode("utf-8")
            if (time.time() - last_s) > 0.01:
                last_s = time.time()

                if ENABLE_BT:
                    print(buff) 
                    decodeBuff()
                else:
                    nowTime = float(time.time() - start_s)
                    tempVel += (random()*0.1+1.2) * 0.01
                    if tempVel > 2.9 and triggerTime == 0:
                        triggerTime = nowTime
                    if triggerTime != 0:
                        tempVel = 4 + (random()*0.05-0.025) + math.cos((nowTime - triggerTime) / 2 * 2*math.pi + math.pi)
                    veltimes.append(nowTime)
                    vels.append(tempVel)
                    appendAccel()
                    q.put([vels[-1], veltimes[-1], accels[-1], acceltimes[-1]])
                    
    recvThread()


if __name__ == "__main__":
    
    q = Queue()
    p = Process(target=mainProcess, args=(q,))
    p.start()   

    now = datetime.datetime.now()
    nowString = now.strftime("%d-%m-%Y-%H-%M-%S")
    f = open(f"accelData-{nowString}.txt", "w")

    fig, (ax1, ax2) = plt.subplots(2, 1)
    # matplotlib plots on figures
    # contains ax*e*s

    veltimes = []
    vels = []
    acceltimes = []
    accels = []

    ax1.plot()
    ax1.set_xlabel('time')
    ax1.set_ylabel('vel')
    ax2.plot()
    ax1.set_xlabel('time')
    ax1.set_ylabel('accel')

    dataShown = 1700
    timeRange = 15 + 0.5
    def animate(i):
        global dataShown, timeRange
        while not q.empty():
            (tempVel, tempVeltime, tempAccel, tempAcceltime) = q.get()
            
            veltimes.append(tempVeltime)
            vels.append(tempVel)
            acceltimes.append(tempAcceltime)
            accels.append(tempAccel)
            if (len(veltimes) > sg_window):
                f.write(f"Time: {veltimes[-sg_window]:6.2f}   Vel: {vels[-sg_window]:6.2f} | ")
                f.write(f"Time: {acceltimes[-1]:6.2f}   Accel: {accels[-1]:4.2f} \n")
        ax1.clear()
        ax2.clear()
        if len(veltimes) > 0: 
            ax1.set_xlim(left=veltimes[-1] - timeRange, right=veltimes[-1] + 0.5)
            ax2.set_xlim(left=acceltimes[-1] - timeRange, right=acceltimes[-1] + 0.5)
            ax1.set_ylim(ymin=0, ymax=5)
            ax2.set_ylim(ymin=-8, ymax=8)
        # if 100 < len(veltimes) < 1000:
        #     avgTime = (veltimes[-1] - veltimes[-100]) / 100
        #     dataShown = int(timeRange / avgTime * 1.1)
        ax1.plot(veltimes[-dataShown:], vels[-dataShown:])
        ax2.plot(acceltimes[-dataShown:], accels[-dataShown:])
        # print("fuick")

    # ani = mpl.animation.FuncAnimation(fig, animate, interval=1)
    # writervideo = mpl.animation.FFMpegWriter(fps=60)
    # plt.show()
    # ani.save('increasingStraightLine.mp4')


    # Create the ffmpeg writer
    mpl.rcParams['animation.ffmpeg_path'] = r'C:\\ffmpeg\\bin\\ffmpeg.exe'
    writer = FFMpegWriter(fps=10)
    file_path = f"accelGraph-{nowString}.mp4"

    # Open the mp4 file or create a new one if it does not exist
    with writer.saving(fig, file_path, 100):
        # Show the plot and set blocking to False
        # Display and save each frame
        plt.show(block=False)
        try:
            while True:
                animate(1)
                fig.canvas.flush_events()
                writer.grab_frame()
                plt.pause(1/60)
        except:
            pass
        p.join()
            

            