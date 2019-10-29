from collections import deque, namedtuple
import time

kp = 1
ki = 1
kd = 1
# load_len_31 = 12
# load_len_32 = 1
# load_len_33 = 44
pkg_num = 0
func_id = [0, 31, 32, 33]
func_31 = namedtuple('func_31', ['pitch', 'roll', 'yaw', 'thrust', 'x_1', 'x_2'])
func_32 = namedtuple('func_32', ['load_msg'])
func_33 = namedtuple('func_33', ['ctrl_mode', 'offset_x', 'offset_y', 'height'])
# initial func31, func32, func33
f_31 = func_31(0, 0, 0, 0, 0, 0)
f_32 = func_32(2)
f_33 = func_33(4, 0, 0, 0)


def get_plane_param(pkg_num, load_msg):
    f_32 = func_32(load_msg)
    info = {}
    return info


def get_bbox():
    bbox = namedtuple('bbox', ['x', 'y'])
    posi_bbox = bbox(400, 400)
    return posi_bbox


def get_dis():
    dis = 0
    return dis


def adjust(posi_std, posi_now, error, dis=False):
    error_now = posi_now - posi_std
    delta = -1
    if abs(error_now) <= 10 and not dis:
        return delta
    if abs(error_now) <= 5 and dis:
        return delta
    error.appendleft(error_now)
    error.pop()
    kp_multi = error[0] - error[1]
    ki_multi = error[0]
    kd_multi = error[0] - 2 * error[1] + error[2]
    delta = kp * kp_multi + ki * ki_multi + kd * kd_multi
    return delta


def send(pkg_num, func_id, func_param):
    if func_id == 31:
        return 31
    # if func_id == 32:
    #     return []
    if func_id == 33:
        return 33


error_x = deque([200, 300, 400])
# error_y = deque([200, 300, 400])
error_dis = deque([10, 15, 20])

img_center = namedtuple('img_center', ['x', 'y'])
posi_std = img_center(1920 / 2, 1080 / 2)
dis_std = 4
init_x = 0.5,
init_dis = 5
prev_x, prev_dis = init_x, init_dis

# start the plane
f_33.ctrl_mode = 4
send(pkg_num, 33, f_33)
pkg_num += 1

time_start = time.perf_counter()
while True:
    if time.perf_counter() - time_start >= 0.1:
        plane_info = get_plane_param(pkg_num, 1)
        pkg_num += 1
        posi_bbox = get_bbox()
        dis_now = get_dis()
        delta_x = adjust(posi_std.x, posi_bbox.x, error_x)
        delta_dis = adjust(dis_std, dis_now, error_dis, dis=True)

        if delta_x == -1 and delta_dis == -1:
            f_33 = func_33(2, 0, 0, 2)

        elif delta_x == -1:
            dis = prev_dis + delta_dis
            f_33 = func_33(2, 0, dis, 2)
            prev_dis = dis

        elif delta_dis == -1:
            x = prev_x + delta_x
            f_33 = func_33(2, x, 0, 2)
            prev_x = x

        else:
            dis = prev_dis + delta_dis
            x = prev_x + delta_x
            f_33 = func_33(2, x, dis, 2)
            prev_dis = dis
            prev_x = x

        send(pkg_num, func_id[3], f_33)
        pkg_num += 1

        time_start = time.perf_counter()
