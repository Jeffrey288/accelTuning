

if __name__ == "__main__":
        
    import matplotlib as mpl
    import matplotlib.pyplot as plt
    import numpy as np
    import time
    from threading import Thread
    from random import *
    import serial
    import math


    """
    start:[
    pos:6.000,0.492,-0.000;
    vel:-0.000,-0.000,-0.000;
    time:455.496;
    ]
    start:[
    pos:6.000,0.492,-0.000;
    vel:-0.000,-0.000,-0.000;
    time:455.548;
    ]
    """

    ENABLE_BT = False
    USE_THREAD = True
    if ENABLE_BT:
        ser = serial.Serial(port = "COM7", baudrate = 115200, timeout = 0.3)

    fig, (ax1, ax2) = plt.subplots(2, 1)
    # matplotlib plots on figures
    # contains ax*e*s

    start_ms = time.time()
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

    buff = ""

    def appendAccel():
        global veltimes, acceltimes, vels, accels
        if (len(veltimes) <= 1):
            acceltimes.append(veltimes[-1])
            accels.append(0)
        else: # assuming a constant / average acceleration
            if veltimes[-1] != veltimes[-2]:
                acceltimes.append((veltimes[-1] + veltimes[-2])/2)
                accels.append((vels[-1] - vels[-2])/(veltimes[-1] - veltimes[-2])) 

    def decodeBuff():   
        global buff, veltimes, acceltimes, vels, accels
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
                appendAccel()
                
            except:
                print("continued at exception")
                continue

    def recvThread():
        global buff
        last_s = time.time()
        tempVel = 0
        while True:
            if (time.time() - last_s) > 0.01:
                last_s = time.time()

                if ENABLE_BT:
                    temp = ser.readlines()
                    buff += temp.decode("utf-8") 
                    print(buff) 
                    if USE_THREAD:
                        decodeBuff()
                else:
                    if USE_THREAD:
                        nowTime = float(time.time() - start_ms)
                        tempVel += (random()*0.1+1.2) * 0.01
                        if tempVel > 4:
                            tempVel = 4 + (random()*0.05-0.025)
                        veltimes.append(nowTime)
                        vels.append(tempVel)
                        appendAccel()
            
            # print("hi")

    last_s2 = time.time()       
    tempVel2 = 0
    def animate(i):
        global last_s2, tempVel2
        if not USE_THREAD:
            if (time.time() - last_s2) > 0.01:
                last_s2 = time.time()
                if ENABLE_BT:
                    decodeBuff()
                else:
                    nowTime = float(time.time() - start_ms)
                    tempVel2 += (random()*0.1+1.2) * 0.01
                    if tempVel2 > 4:
                        tempVel2 = 4 + (random()*0.05-0.025)
                    veltimes.append(nowTime)
                    vels.append(tempVel2)
                    appendAccel()


        ax1.clear()
        ax1.plot(veltimes, vels)
        ax2.clear()
        ax2.plot(acceltimes, accels)
        # print("fuick")

    thread = Thread(target=recvThread)
    thread.daemon = True
    if USE_THREAD:
        thread.start()

    ani = mpl.animation.FuncAnimation(fig, animate, interval=1)
    plt.show()