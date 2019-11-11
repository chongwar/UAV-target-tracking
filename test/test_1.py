from ctypes import *
from collections import namedtuple

# a = c_byte(2)
# print(a)

def abc(*args):
    print('*****')
    print(type(*args))
    for index, i in enumerate(*args):
        print(index, i)


if __name__ == '__main__':
    a = namedtuple('a', ['x', 'y', 'z'])
    t = a(1, 2, 3)
    # print(type(t))
    # print(t)
    t_1 = [4, 'ff', c_int(8)]
    abc(t)
    abc(t_1)
    # a = 0x226
    # print(a)

