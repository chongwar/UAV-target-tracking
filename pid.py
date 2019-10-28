from collections import deque, namedtuple
import time

kp = 1
ki = 1
kd = 1
def get_plane_param():
    info = {}
    return info


def get_bbox():
    bbox = namedtuple('bbox', ['x', 'y'])
    posi_bbox = bbox(400, 400)
    return posi_bbox


def adjust(posi_std, posi_bbox, error, dis=False):
    error_now = posi_bbox - posi_std
    delta = -1
    if abs(error_now) <= 10 and not dis:
        return delta
    if abs(error_now) <= 5 and dis:
        return delta
    error.appendleft(error_now)
    kp_multi = error[0] - error[1]
    ki_multi = error[0]
    kd_multi = error[0] - 2 * error[1] + error[2]
    delta = kp * kp_multi + ki * ki_multi + kd * kd_multi
    return delta


def send(msg):
    pass


error_x = deque([200, 300, 400])
# error_y = deque([400, 400, 400])
error_f = deque([10, 15, 20])
img_center = namedtuple('img_center', ['x', 'y'])
posi_std = img_center(1920 / 2, 1080 / 2)
dis_std = 5
dis_now = 25
init_x, init_f = 0.5, 5
count = 0
time_start = time.perf_counter()
prev_x, prev_f = init_x, init_f

while True:
    if time.perf_counter() - time_start >= 0.05:
        count += 1
        # if count == 1:
        #     prev_x, prev_f = init_x, init_f
        plane_info = get_plane_param()
        posi_bbox = get_bbox()
        delta_x = adjust(posi_std.x, posi_bbox.x, error_x)
        delta_f = adjust(dis_std, dis_now, error_f, dis=False)
        if delta_x == -1 and delta_f == -1:
            send((0, 10))
            break
        if delta_x == -1:
            f = prev_f + delta_f
            send((0, f))
            prev_f = f
            continue
        if delta_f == -1:
            x = prev_x + delta_x
            send((x, 0.5))
            prev_x = x
            continue
        x, f = prev_x + delta_x, prev_f + delta_f
        send((x, f))
        prev_x, prev_f = x, f

