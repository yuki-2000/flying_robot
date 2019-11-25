import serial
from serial.tools import list_ports
import time


def select_port():
    ser = serial.Serial()
    ser.baudrate = 115200  # ArduinoのSerial.beginで指定した値
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
            print("input %3d: open %s" % (i, devices[i]))
        print("input number of target port >> ", end="")
        num = int(input())
        ser.port = devices[num]

    # 開いてみる
    try:
        ser.open()
        return ser
    except:
        print("error when opening serial")
        return None


def make_send_str(vars):
    buff = ""
    for key in vars.keys():
        buff += "%s\r\n%lf\r\n" % (key, vars[key])
    return buff


def main():
    ser = select_port()
    if ser is None:
        return

    # orange
    # data = {
    #     "TRIM_RUDDER": 60.0,
    #     "TRIM_ELEVATOR": 60.0 ,
    #     "SIGN_RUDDER": 1,
    #     "SIGN_ELEVATOR": 1,
    #     "RUDDER_KP": 1.5,
    #     "RUDDER_KI": 0.0,
    #     "RUDDER_KD": 0.05,
    #     "ELEVATOR_KP": 1.0,
    #     "ELEVATOR_KI": 0.0,
    #     "ELEVATOR_KD": 0.05,
    # }

    # Blackjack
    # data = {
    #     "TRIM_RUDDER": 30.0,
    #     "TRIM_ELEVATOR": 100.0 ,
    #     "SIGN_RUDDER": 1,
    #     "SIGN_ELEVATOR": -1,
    #     "RUDDER_KP": 0.8,
    #     "RUDDER_KI": 0.0,
    #     "RUDDER_KD": 0.05,
    #     "ELEVATOR_KP": 1.0,
    #     "ELEVATOR_KI": 0.0,
    #     "ELEVATOR_KD": 0.05,
    # }
    # teal
    data = {
        "TRIM_RUDDER": 90.0-10,
        "TRIM_ELEVATOR": 90.0 - 10,
        "SIGN_RUDDER": 1,
        "SIGN_ELEVATOR": -1,
        "RUDDER_KP": 0.8,
        "RUDDER_KI": 0.0,
        "RUDDER_KD": 0.00,
        "ELEVATOR_KP": 1.0,
        "ELEVATOR_KI": 0.0,
        "ELEVATOR_KD": 0.00,
    }
    temp = make_send_str(data)
    print(temp)

    old_time = time.time()

    while ser.is_open:
        print(ser.read(256).decode(), end="")

        if time.time() - old_time > 1:
            ser.write(temp.encode())
            old_time = time.time()

        time.sleep(0.01)
    ser.close()


if __name__ == "__main__":
    main()
