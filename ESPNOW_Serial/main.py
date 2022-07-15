import sys
import time
from ctypes import *
from enum import Enum
import serial
import serial.tools.list_ports


class Packet(Structure):

    _pack = 1  # 1byte 정렬

    _fields_ = [("device_led", c_uint8),
                ("RED", c_uint8),
                ("GREEN", c_uint8),
                ("BLUE", c_uint8),
                ("brightness", c_uint8),
                ("style", c_uint8),
                ("wait_time", c_uint8),
                ("checksum", c_uint8)]


def read_packet_data(fields):
    print("------------------packet------------------------")
    print(f"device_led : {fields.device_led}\n" +
          f"RED   : {fields.RED}\n" +
          f"GREEN : {fields.GREEN}\n" +
          f"BLUE  : {fields.BLUE}\n" +
          f"brightness : {fields.brightness}\n" +
          f"style : {fields.style}\n" +
          f"wait_time : {fields.wait_time}\n" +
          f"checksum : {fields.checksum}")
    print("------------------------------------------------")
    print(f"bytes : {bytes(fields)}")


class STYLE(Enum):
    NONE = 0
    oneColor = 1
    chase = 2
    rainbow = 3
    colorWipe = 4
    chase_rainbow = 5


def set_packet(device_led, rgb_list, brightness, style, wait_time):
    data = Packet(device_led, rgb_list[0], rgb_list[1], rgb_list[2], brightness, style, wait_time, 0)
    read_packet_data(data)
    return data


# 세팅 끝 알려주는 메서드
def readUntilString(ser, exitcode=b'Setup_Done'):
    count = 0
    while True:
        data = ser.read_until()
        print(data)
        # print(count)
        if data == b'':
            count = count + 1
        else:
            count = 0

        if exitcode in data or count > 50:
            return print("====Serial Now available====")

def clear_serial_buffer(ser, delay):
    close_time = time.time() + delay
    while True:
        # if py_serial.readable():
        res = ser.readline()
        # print(res[:len(res)-1].decode('utf-8').rstrip()
        print(res[:len(res) - 1].decode().rstrip())

        if time.time() > close_time:
            break


def serial_ports(com_port=None):
    ports = serial.tools.list_ports.comports()
    uart_port = ['CP210x', 'CH340', 'CH340K', 'CH9102']
    dic = {}

    if com_port is not None:
        dic[com_port] = "manually"
        return dic

    for port, desc, hwid in sorted(ports):
        # print("{}: {} [{}]".format(port, desc, hwid))
        for uart in uart_port:
            if uart in desc:
                # print(uart)
                dic[port] = uart
                # print(dic)

    if len(dic.items()) > 0:
        return dic


def connect_port(com_port=None):
    connected_ports = serial_ports(com_port)
    board_port = list(connected_ports.keys())
    # print(board_port[0])
    return board_port[0]


def serial_receive_callback(ser, data):
    recv_data = ser.read(data)
    recv_data = Packet.from_buffer_copy(recv_data)
    read_packet_data(recv_data)
   # return recv_data


def set_rainbow_color(ser, address):
    rainbow_list = [[255, 0, 0],  # Red
                    [255, 140, 0],  # Orange
                    [255, 255, 0],  # Yellow
                    [0, 255, 0],  # Green
                    [0, 0, 255],  # Blue
                    [18, 0, 255],  # Indigo
                    [255, 0, 255],  # Purple
                    [255, 255, 255]]

    for led_number in range(7):
        trans = set_packet(address + led_number, rainbow_list[led_number], 50, STYLE.oneColor.value, 10)
        send_data = ser.write(bytes(trans))
        time.sleep(0.02)


def on_off_led(ser):
    for led_num in range(0x10, 0x16):
        trans = set_packet(led_num, [255, 43, 123], 50, STYLE.oneColor.value, 20)
        send_data = ser.write(bytes(trans))

        time.sleep(0.1)  # 약간의 딜레이가 없으면 전송이 힘들 수 있음 Serial 인식 시간

        trans = set_packet(led_num, [255, 43, 123], 0, STYLE.oneColor.value, 20)
        send_data = ser.write(bytes(trans))

        time.sleep(0.1)


def Wheel(WheelPos):
    WheelPos = 255 - WheelPos
    if WheelPos < 85:
        return [255 - WheelPos * 3, 0, WheelPos * 3]    # R, 0, B

    if WheelPos < 170:
        WheelPos = WheelPos - 85
        return [0, WheelPos * 3, 255 - WheelPos * 3]    # 0, G, B

    WheelPos = WheelPos - 170
    return [WheelPos * 3, 255 - WheelPos * 3, 0]        # R, G, 0


def rainbow_python(ser, address, size, delay, on_off_brightness):
    global pixel_cycle
    global pixel_queue

    for i in range(0, size, 1):
        trans = set_packet(address + i, Wheel((i + pixel_cycle) % 255), on_off_brightness, STYLE.oneColor.value, 1)
        ser.write(bytes(trans))
        time.sleep(delay)
        pixel_cycle = pixel_cycle + 1
        pixel_queue = pixel_queue + 1
        if pixel_cycle >= 256:
            pixel_cycle = 0
        if pixel_queue >= size:
            pixel_queue = 0






if __name__ == '__main__':
    print(serial_ports())
    # py_serial = serial.Serial(port="COM8", baudrate=115200, timeout=0.1)
    # py_serial = serial.Serial(port=connect_port('COM8'), baudrate=115200, timeout=0.1)
    py_serial = serial.Serial(port=connect_port(), baudrate=115200, timeout=0.1)    # 포트 연결

    readUntilString(py_serial)

    clear_serial_buffer(py_serial, 1)

    # Global value
    pixel_cycle = 0
    pixel_queue = 0

    # led_num, rgb_list, brightness, style, wait
    while True:
        for i in range(10):
            rainbow_python(py_serial, 0x10, 8, 0.01, 10)
            rainbow_python(py_serial, 0x10, 8, 0.01, 10)
            if i == 4:
                trans = set_packet(0x10, [0, 0, 0], 0, STYLE.oneColor.value, 10)
                py_serial.write(bytes(trans))
                time.sleep(0.5)





        # for i in range(8):
        #     trans = set_packet(0x50 + i, [255, 43, 123], 10, STYLE.oneColor.value, 10)
        #     py_serial.write(bytes(trans))
        #     time.sleep(0.1)
        #
        #     if i == 3:
        #         trans = set_packet(0x50, [255, 43, 123], 0, STYLE.oneColor.value, 10)
        #         py_serial.write(bytes(trans))
        #
        #
        # for j in range(8):
        #     trans = set_packet(0x50 + j, [0, 0, 0], 10, STYLE.oneColor.value, 10)
        #     py_serial.write(bytes(trans))
        #     time.sleep(0.1)




        # for i in range(6):
        #     set_rainbow_color(py_serial, 0x10 + i)
        #     if i == 2:
        #         trans = set_packet(0x10, [0, 0, 0], 0, STYLE.oneColor.value, 10)
        #         py_serial.write(bytes(trans))
        #         time.sleep(0.3)



    # while True:
    #     pass
        # for led_num in range(0x40, 0x48):

        # on_off_led(py_serial)
        # trans = set_packet(0x10, [255, 43, 123], 10, STYLE.rainbow.value, 2)
        # send_data = py_serial.write(bytes(trans))




        # for led_num in range(0x10, 0x16):
        #     trans = set_packet(led_num, [255, 43, 123], 50, STYLE.oneColor.value, 20)
        #     send_data = py_serial.write(bytes(trans))
        #
        #     time.sleep(0.1)  # 약간의 딜레이가 없으면 전송이 힘들 수 있음 Serial 인식 시간
        #
        #     trans = set_packet(led_num, [255, 43, 123], 0, STYLE.oneColor.value, 20)
        #     send_data = py_serial.write(bytes(trans))
        #
        #     time.sleep(0.05)

        # set_rainbow_color(py_serial)







    # change WiFi
    # trans = set_packet(255, 255, [0, 0, 0], 0, 1, 20)
    # send_data = py_serial.write(bytes(trans))

    # serial_receive_callback(py_serial, send_data)

    py_serial.close()
