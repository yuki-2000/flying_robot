import serial
from serial.tools import list_ports
import time
import numpy as np
import matplotlib.pyplot as plt

def select_port():
    ser = serial.Serial()
    ser.baudrate = 115200 # ArduinoのSerial.beginで指定した値
    ser.timeout = 0.1       # タイムアウトの時間

    ports = list_ports.comports()    # ポートデータを取得
    
    devices = [info.device for info in ports]

    if len(devices) == 0:
        # シリアル通信できるデバイスが見つからなかった場合
        print("error: device not found")
        return None
    elif len(devices) == 1:
        print("only found %s" % devices[0])
        ser.port = devices[0]
    else:
        # ポートが複数見つかった場合それらを表示し選択させる
        for i in range(len(devices)):
            print("input %3d: open %s" % (i,devices[i]))
        print("input number of target port >> ",end="")
        num = int(input())
        ser.port = devices[num]
    
    # 開いてみる
    try:
        ser.open()
        return ser
    except:
        print("error when opening serial")
        return None

def to_value(s):
    try:
        return int(s)
    except:
        try:
            return int(s,16)
        except:
            return float(s)

def parse_core(buffer,record):
    for line in buffer.split("\n"):
        for col in line.split(","):
            try:
                name,val = col.split(":")
                name = name.strip()
                val = to_value(val.strip())
                record[name] = val
            except:
                pass

def parse(buffer,record):
    last_newline = buffer.rfind("\n")
    if last_newline == -1:
        return buffer
    else:
        for_parse = buffer[:last_newline+1]
        rest = buffer[last_newline+1:]

        parse_core(for_parse,record)

        return rest

class ReatimePlotter:
    def __init__(self,names):
        plt.ion()
        plt.figure()
        self.names = names
        self.time0 = time.time()
        self.xs_list = [[0] * 100 for _ in names]
        self.ys_list = [[0] * 100 for _ in names]
        self.lines = [
            plt.plot(xs, ys, label=name)[0] 
            for xs,ys,name 
            in zip(self.xs_list,self.ys_list,self.names)
        ]
        plt.legend(loc=1)

    def update(self,record):
        t = time.time()-self.time0
        xmin = +1e10 
        xmax = -1e10
        ymin = +1e10
        ymax = -1e10

        for i,name in enumerate(self.names):
            if name in record.keys():
                self.xs_list[i].append(t)
                self.xs_list[i].pop(0)
                self.ys_list[i].append(record[name])
                self.ys_list[i].pop(0)
                self.lines[i].set_xdata(self.xs_list[i])
                self.lines[i].set_ydata(self.ys_list[i])
                xmin = min(xmin,min(self.xs_list[i]))
                xmax = max(xmax,max(self.xs_list[i]))
                ymin = min(ymin,min(self.ys_list[i]))
                ymax = max(ymax,max(self.ys_list[i]))
        plt.xlim(xmin,xmax)
        plt.ylim(ymin,ymax) 
        plt.draw()
        plt.pause(0.01)

def main():
    ser = select_port()
    if ser is None:
        return
    
    buffer = ""
    record = {}
    old_time = time.time()

    print("input variable names to display graph")
    names = [x.strip() for x in input().split(",")]
    plotter = ReatimePlotter(names)

    try:
        while ser.is_open:
            data = ser.read(256)
            if data != b'':
                buffer += data.decode()
                buffer = parse(buffer,record)

            if time.time() - old_time > 0.1:
                old_time = time.time()
                print(record)
                plotter.update(record)
    except:
        pass  
    ser.close()
    plt.close()

if __name__=="__main__":
    main()