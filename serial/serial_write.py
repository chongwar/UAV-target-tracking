import serial
import serial.tools.list_ports

for port in list(serial.tools.list_ports.comports()):
    print(port)

port = '/dev/ttyUSB0'
bps = 57600
time_wait = 5
msg = 'hello...'

def write(port=port, bps=bps, time_wait=time_wait):
    try:
        ser = serial.Serial(port, bps, timeout=time_wait)
        write_len = ser.write("hi this is wang".encode("utf-8"))
        print("success write: ", write_len)
        ser.close()
    except Exception as e:
        print(e)


def read(port=port, bps=bps, time_wait=0):
    while True:
        ser = serial.Serial(port, bps, timeout=time_wait)
        read_msg = ser.read(ser.in_waiting)
        if len(read_msg) > 0:
            print(read_msg, type(read_msg))
            print('read {} {} from port.'.format(len(read_msg), type(read_msg)))
            break
    

if __name__ == '__main__':
    # write()
    read()
