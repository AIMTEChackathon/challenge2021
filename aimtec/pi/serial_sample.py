import serial
import threading
import time
import traceback

MAX_SIZE=1024

ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)

state = None

def get_parenthesis(string):
    # Tady je to velky spatny
    return string.split("(")[1].split(")")[0]

def parse_state(line):
    lift, odo, ladis = line.split()
    lift = get_parenthesis(lift)
    lift0, lift1 = lift.split(",")
    lift0 = int(lift0)
    lift1 = int(lift1)
    odo = get_parenthesis(odo)
    odo0, odo1 = odo.split(",")
    odo0 = int(odo0)
    odo1 = int(odo1)
    ladis = get_parenthesis(ladis)
    ladis = int(ladis)
    return {"lift": (lift0, lift1),
            "odo": (odo0, odo1),
            "ladis": ladis}

def read_thread(port):
    global state
    while True:
        line = port.readline().strip().decode("ascii", errors="ignore")
        if not line:
            # nothing read
            continue
        if line.startswith("/*"):
            print("Received comment", line)
        else:
            try:
                state = parse_state(line)
            except:
                print("Unable to parse", line)
                #traceback.print_exc()
            print("Received state", state)

read_thread = threading.Thread(target=read_thread, args=(ser,))
read_thread.setDaemon(True)
read_thread.start()

print("Sleeping 60")
time.sleep(60)

