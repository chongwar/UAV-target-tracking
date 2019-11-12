from ctypes import *
from collections import namedtuple
import os

control = CDLL('./control.so')


def function_33(*args):
    tmp = []
    for index, i in enumerate(*args):
        tmp.append(i)
    _function_33 = control.function_33
    _function_33(c_ushort(tmp[0]), c_char(tmp[1]), c_float(tmp[2]),
                 c_float(tmp[3]), c_float(tmp[4]),c_float(tmp[5]))


def function_32(*args):
    tmp = []
    for index, i in enumerate(*args):
        tmp.append(i)
    _function_32 = control.function_32
    _function_32.restype = c_char_p
    plane_info = _function_32(c_ushort(tmp[0]), c_char(tmp[1]))
    return plane_info


def function_31(*args):
    tmp = []
    for index, i in enumerate(*args):
        tmp.append(i)
    _function_31 = control.function_31
    _function_31(c_ushort(tmp[0]), c_short(tmp[1]), c_short(tmp[2]),
                 c_short(tmp[3]), c_short(tmp[4]), c_short(tmp[5]), c_short(tmp[6]))


if __name__ == '__main__':
    os.system('picocom -b 57600 /dev/ttyUSB0')    

    func_31 = namedtuple('func_31', ['pkg_num', 'pitch', 'roll', 'yaw', 'thrust', 'x_1', 'x_2'])
    func_32 = namedtuple('func_32', ['pkg_num', 'load_msg'])
    func_33 = namedtuple('func_33', ['pkg_num', 'ctrl_mode', 'offset_x', 'offset_y', 'height', 'yaw'])
    pkg_num_1 = 1
    pkg_num_2 = 2
    pkg_num_3 = 3
    pkg_num_4 = 4
    pkg_num_5 = 5
    f_33_1 = func_33(pkg_num_1, 2, 0.2, 0.2, 0.2, 0)
    f_33_2 = func_33(pkg_num_2, 2, 0.2, -0.2, 0.5, 90)
    f_33_3 = func_33(pkg_num_3, 2, 0.2, 0.2, 0.5, 180)
    f_32 = func_32(pkg_num_4, 1)
    f_31 = func_31(pkg_num_5, 0, 0, 0, 550, 0, 0)

    function_33(f_33_1)
    function_33(f_33_2)
    function_33(f_33_3)
    plane_info = function_32(f_32)
    function_31(f_31)
