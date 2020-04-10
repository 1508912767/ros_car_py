from socket import *
from threading import Thread
import time

SLOTS = "/sys/devices/bone_capemgr.9/slots"
p1_duty = "/sys/devices/ocp.3/pwm_test_P9_16.13/duty"
p2_duty = "/sys/devices/ocp.3/pwm_test_P8_13.12/duty"
p1_period = "/sys/devices/ocp.3/pwm_test_P9_16.13/period"
p2_period = "/sys/devices/ocp.3/pwm_test_P8_13.12/period"
p1_run = "/sys/devices/ocp.3/pwm_test_P9_16.13/run"
p2_run = "/sys/devices/ocp.3/pwm_test_P8_13.12/run"
p1_export = "/sys/class/gpio/export"
p2_export = "/sys/class/gpio/export"
p1_direction = "/sys/class/gpio/gpio44/direction"
p2_direction = "/sys/class/gpio/gpio45/direction"
p1_polarity = "/sys/class/gpio/gpio44/value"
p2_polarity = "/sys/class/gpio/gpio45/value"
p1_encode = "/sys/devices/ocp.3/48302000.epwmss/48302180.eqep/position"
p2_encode = "/sys/devices/ocp.3/48304000.epwmss/48304180.eqep/position"


def main():
    try:
        with open(SLOTS, "a") as f:
            f.write("am33xx_pwm")

        with open(SLOTS, "a") as f:
            f.write("bone_pwm_P8_13")

        with open(SLOTS, "a") as f:
            f.write("bone_pwm_P9_16")

        with open(SLOTS, "a") as f:
            f.write("bone_eqep1")

        with open(SLOTS, "a") as f:
            f.write("bone_eqep2")

        with open(p1_export, "a") as f:
            f.write("44")

        with open(p2_export, "a") as f:
            f.write("45")

        with open(p1_direction, "a") as f:
            f.write("out")

        with open(p2_direction, "a") as f:
            f.write("out")

        with open(p1_period, "w") as f:
            f.write("500000")

        with open(p2_period, "w") as f:
            f.write("500000")
    except:
        print("write wrong")

    try:
        with open(p1_encode, "w") as f:
            f.write("0")
        with open(p2_encode, "w") as f:
            f.write("0")
    except:
        print("encode initialize wrong")


def Receive(clientSocket):
    msg_lists = []

    while True:
        msg_list1 = clientSocket.recv(1024)
        msg_list2 = clientSocket.recv(1024)
        if len(msg_list1) > 0 or len(msg_list2) > 0:
            msg_list1 = int(float(msg_list1)) * 500
            msg_list2 = int(float(msg_list2)) * 500
            msg_lists.append(msg_list1)
            msg_lists.append(msg_list2)
            msg_lists.append(abs(msg_list1))
            msg_lists.append(abs(msg_list2))
            for msg in msg_lists:
                print("recvData:%d" % msg)

            if msg_lists[0] > 0 and msg_lists[1] > 0:
                print("go front")
                try:
                    with open(p1_duty, "w") as f:
                        # f.write(str(msg_lists[2]))
                        f.write("400000")
                    with open(p2_duty, "w") as f:
                        # f.write(str(msg_lists[3]))
                        f.write("400000")
                    with open(p1_polarity, "w") as f:
                        f.write("1")
                    with open(p2_polarity, "w") as f:
                        f.write("0")
                    with open(p1_run, "w") as f:
                        f.write("1")
                    with open(p2_run, "w") as f:
                        f.write("1")
                    msg_lists = []
                except:
                    print("go front wrong")

            elif msg_lists[0] < 0 and msg_lists[1] < 0:
                print("go back")
                try:
                    with open(p1_duty, "w") as f:
                        # f.write(str(msg_lists[2]))
                        f.write("400000")
                    with open(p2_duty, "w") as f:
                        # f.write(str(msg_lists[3]))
                        f.write("400000")
                    with open(p1_polarity, "w") as f:
                        f.write("0")
                    with open(p2_polarity, "w") as f:
                        f.write("1")
                    with open(p1_run, "w") as f:
                        f.write("1")
                    with open(p2_run, "w") as f:
                        f.write("1")
                    msg_lists = []
                except:
                    print("go back wrong")

            elif msg_lists[0] > 0 > msg_lists[1]:
                print("go left")
                try:
                    with open(p1_duty, "w") as f:
                        # f.write(str(msg_lists[2]))
                        f.write("400000")
                    with open(p2_duty, "w") as f:
                        # f.write(str(msg_lists[3]))
                        f.write("400000")
                    with open(p1_polarity, "w") as f:
                        f.write("1")
                    with open(p2_polarity, "w") as f:
                        f.write("1")
                    with open(p1_run, "w") as f:
                        f.write("1")
                    with open(p2_run, "w") as f:
                        f.write("1")
                    msg_lists = []
                except:
                    print("go left wrong")

            elif msg_lists[0] < 0 < msg_lists[1]:
                print("go right")
                try:
                    with open(p1_duty, "w") as f:
                        # f.write(str(msg_lists[2]))
                        f.write("400000")
                    with open(p2_duty, "w") as f:
                        # f.write(str(msg_lists[3]))
                        f.write("400000")
                    with open(p1_polarity, "w") as f:
                        f.write("0")
                    with open(p2_polarity, "w") as f:
                        f.write("0")
                    with open(p1_run, "w") as f:
                        f.write("1")
                    with open(p2_run, "w") as f:
                        f.write("1")
                    msg_lists = []
                except:
                    print("go right wrong")

            elif msg_lists[0] == 0 and msg_lists[1] == 0:
                print("stop")
                try:
                    with open(p1_run, "w") as f:
                        f.write("0")
                    with open(p2_run, "w") as f:
                        f.write("0")
                    msg_lists = []
                except:
                    print("stop wrong")

            else:
                print("put wrong")

            time.sleep(0.1)

        else:
            time.sleep(0.1)
            clientSocket = socket(AF_INET, SOCK_STREAM)
            # clientSocket.connect(("192.168.1.138", 8899))
            clientSocket.connect(("192.168.7.1", 8899))


def Send(clientSocket):
    while clientSocket:
        try:
            with open(p1_encode, "r") as f:
                encode_data_left = str(f.read())
                print("encode1:%s" % encode_data_left)
                clientSocket.send(encode_data_left.encode("utf-8"))
            with open(p2_encode, "r") as f:
                encode_data_right = str(f.read())
                print("encode2:%s" % encode_data_right)
                clientSocket.send(encode_data_right.encode("utf-8"))
        except:
            print("read wrong")
        time.sleep(0.1)


if __name__ == "__main__":
    main()

    client_Socket = socket(AF_INET, SOCK_STREAM)
    # clientSocket.connect(("192.168.1.151", 8899))
    client_Socket.connect(("192.168.7.1", 8899))

    # t1 = Thread(target=Receive, args=(client_Socket,))
    t2 = Thread(target=Send, args=(client_Socket,))
    # t1.start()
    t2.start()
    # t1.join()
    # t2.join()
    Receive(client_Socket)
