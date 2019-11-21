from _control import function_33, function_32, function_31
from collections import deque, namedtuple
import time

kp = 1
ki = 0.7
kd = 0.3
kp_leftright = 0.005
kp_frontback = 1.5
kp_updown = 0.5

func_id = [0, 31, 32, 33]
_func_31 = namedtuple('_func_31', ['pkg_num', 'pitch', 'roll', 'yaw', 'thrust', 'x_1', 'x_2'])
_func_32 = namedtuple('_func_32', ['pkg_num', 'load_msg'])
_func_33 = namedtuple('_func_33', ['pkg_num', 'ctrl_mode', 'offset_x', 'offset_y', 'height', 'yaw'])


def get_bbox():
    bbox = namedtuple('bbox', ['x', 'y'])
    posi_bbox = bbox(400, 400)
    return posi_bbox


def get_dis():
    dis = 0
    return dis


# def adjust(posi_std, posi_now, error, dis=False):
#     error_now = posi_now - posi_std
#     delta = -1
#     if abs(error_now) <= 10 and not dis:
#         return delta
#     if abs(error_now) <= 5 and dis:
#         return delta
#     error.appendleft(error_now)
#     error.pop()
#     kp_multi = error[0] - error[1]
#     ki_multi = error[0]
#     kd_multi = error[0] - 2 * error[1] + error[2]
#     delta = kp * kp_multi + ki * ki_multi + kd * kd_multi
#     return delta


def adjust_lr(posi_std, posi_now):
    error_now = posi_now - posi_std
    delta = -1
    if abs(error_now) <= 10:
        return  delta
    delta = kp_leftright * error_now
    return delta


def adjust_fb(posi_std, posi_now):
    error_now = posi_now - posi_std
    delta = -1
    if abs(error_now) <= 10:
        return  delta
    delta = kp_frontback * error_now
    return delta


def adjust_ud(posi_std, posi_now):
    error_now = posi_now - posi_std
    delta = -1
    if abs(error_now) <= 10:
        return  delta
    delta = kp_updown * error_now
    return delta


def send(func_id, func_param):
    if func_id == 31:
        function_31(func_param)
        return 31
    if func_id == 32:
        plane_info = function_32(func_param)
        return plane_info
    if func_id == 33:
        function_33(func_param)
        return 33


def get_plane_param(pkg_num, load_msg):
    """
    get parameters of the plane
    :param pkg_num: package number
    :param load_msg: load mode [0: stop output;
                                1: output status of plane (50Hz)
                                2: output time data (one frame)
                                3: output position data (one frame)]
    :return: parameters of the plane (dict)
    """
    f_32_param = _func_32(pkg_num, load_msg)
    info = send(func_id[2], f_32_param)
    return info


def auto_track():
    """
    function id: 31
    :return:
    """
    pass


def pid():
    pkg_num = 0
    # error_x = deque([0, 0, 0])
    # # error_y = deque([200, 300, 400])
    # error_dis = deque([0, 0, 0])
    error_lr = 0

    img_center = namedtuple('img_center', ['x', 'y'])
    posi_std = img_center(1920 / 2, 1080 / 2)
    dis_std = 4
    # init_x = 0.5
    # init_dis = 5
    # prev_x = init_x
    # prev_dis = init_dis

    # start the plane
    f_33_param = _func_33(pkg_num, 4, 0, 0, 0, 0)
    tmp = send(func_id[3], f_33_param)
    pkg_num += 1

    time_start = time.perf_counter()
    while True:
        if pkg_num == 255:
            pkg_num = 0
        # if pkg_num == 5:
            # break
        if time.perf_counter() - time_start >= 0.5:
            plane_info = get_plane_param(pkg_num, 1)
            pkg_num += 1
            posi_bbox = get_bbox()
            dis_now = get_dis()
            delta_x = adjust_lr(posi_std.x, posi_bbox.x)
            delta_dis = adjust_fb(dis_std, dis_now)

            if delta_x == -1 and delta_dis == -1:
                f_33_param = _func_33(pkg_num, 2, 0, 0, 2, 0)
                auto_track()

            elif delta_x == -1:
                # dis = prev_dis + delta_dis
                # f_33_param = _func_33(pkg_num, 2, 0, dis, 2, 0)
                # prev_dis = dis
                f_33_param = _func_33(pkg_num, 2, 0, delta_dis, 2, 0)

            elif delta_dis == -1:
                # x = prev_x + delta_x
                # f_33_param = _func_33(pkg_num, 2, x, 0, 2, 0)
                # prev_x = x
                f_33_param = _func_33(pkg_num, 2, delta_x, 0, 2, 0)

            else:
                # dis = prev_dis + delta_dis
                # x = prev_x + delta_x
                # f_33_param = _func_33(pkg_num, 2, x, dis, 2, 0)
                # prev_dis = dis
                # prev_x = x
                f_33_param = _func_33(pkg_num, 2, delta_x, delta_dis, 2, 0)

            tmp = send(func_id[3], f_33_param)
            pkg_num += 1

            time_start = time.perf_counter()


if __name__ == '__main__':
    pid()
