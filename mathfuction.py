from numpy import *
from math import *



def inverseKinematics(x0, y0, z0, angle=180):
    l1 = 10.35     #三段机械臂长度 单位：cm
    l2 = 8.8
    l3 = 17
    larm = l1 + l2 + l3  # 机械臂总长
    eh = 0  # 垂直误差
    ANGLE_ERR = 180
    j1 = j2 = j3 = j4 = 0
    x0 = x0 * larm
    y0 = y0 * larm
    z0 = z0 * larm
    distance = sqrt(x0 ** 2 + y0 ** 2 + z0 ** 2)
    print(x0,y0,z0)
    if distance > (l1 + l2 + l3):
        print("超出机械臂范围，无解")
        return False, None, None, None, None
    if x0 == 0 and y0 == 0:
        j1 = 0
    else:
        j1 = arctan2(y0, x0)

    if j1 > radians(135):
        j1 = j1 - radians(180)
        angle_end = -1 * angle
    if j1 < radians(-135):
        j1 = j1 + radians(180)
        angle_end = -1 * angle

    if x0 != 0:
        len = x0 / cos(j1)
    else:
        len = abs(y0)

    count = 0
    angle_up = angle_low = angle
    up_true_low_false = True
    is_out_range = False

    while count <= ANGLE_ERR:
        rad = radians(angle)
        a = len - l3 * sin(rad)
        b = z0 - eh - l3 * cos(rad)

        cos_j3 = ((pow(a, 2) + pow(b, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l2 * l1))
        if abs(cos_j3) <= 1:
            sin_j3 = sqrt(1 - pow(cos_j3, 2))
            j3 = atan2(sin_j3, cos_j3)
            k1 = l1 + l2 * cos_j3
            k2 = l2 * sin_j3
            w = atan2(k2, k1)
            j2 = atan2(a, b) - w

            j4 = rad - j2 - j3

            x1 = (l1 * sin(j2) + l2 * sin(j2 + j3) + l3 * sin(j2 + j3 + j4)) * cos(j1)
            y1 = (l1 * sin(j2) + l2 * sin(j2 + j3) + l3 * sin(j2 + j3 + j4)) * sin(j1)
            z1 = l1 * cos(j2) + l2 * cos(j2 + j3) + l3 * cos(j2 + j3 + j4) + eh
            deg_j1 = degrees(j1)
            deg_j2 = degrees(j2)
            deg_j3 = degrees(j3)
            deg_j4 = degrees(j4) - 15

            print("结果：j1: {} ,j2: {} ,j3: {} ,j4: {} ,angle: {} ".format(deg_j1, deg_j2, deg_j3, deg_j4, angle))
            print("运动学正解结果：x:%f,y:%f,z:%f\r\n" % (x1, y1, z1))

            if deg_j1 <= 135 and deg_j1 >= -135 and deg_j2 <= 90 and deg_j2 >= -90 and deg_j3 <= 135 and deg_j3 >= -135 and deg_j4 <= 90 and deg_j4 >= -135:
                return True,deg_j1,deg_j2, deg_j3, deg_j4,
            else:
                print("超出约束")
                is_out_range = True
        # 试试两边的角度有没有解
        if abs(cos_j3) > 1 or is_out_range == True:
            if up_true_low_false == True:
                angle_up = angle_up + 1
                angle = angle_up

            if up_true_low_false == False:
                angle_low = angle_low - 1
                angle = angle_low
                count += 1
            up_true_low_false = bool(1 - up_true_low_false)
    if count > ANGLE_ERR:
        print("无解")
        return False, None, None, None, None


def distanceCalculat(x1, y1, z1, x2, y2, z2):
    x = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2))
    return x
def distance_two_points(x1,y1,x2,y2):
    x = sqrt((pow(x1-x2,2)+pow(y1-y2,2)))
    return x
def trangleIncenterCalculate(handlist):
    wrist = 0
    index_finger_mcp = 1  # index_finger_mcp=5;
    pinky_mcp = 2  # pinky_mcp=17
    x1 = handlist[wrist].x
    y1 = handlist[wrist].y
    z1 = handlist[wrist].z
    x2 = handlist[index_finger_mcp].x
    y2 = handlist[index_finger_mcp].y
    z2 = handlist[index_finger_mcp].z
    x3 = handlist[pinky_mcp].x
    y3 = handlist[pinky_mcp].y
    z3 = handlist[pinky_mcp].z
    a = distanceCalculat(x2, y2, z2, x3, y3, z3)
    c = distanceCalculat(x1, y1, z1, x2, y2, z2)
    b = distanceCalculat(x1, y1, z1, x3, y3, z3)
    incenter_x = (a * x1 + b * x2 + c * x3) / (a + b + c)
    incenter_y = (a * y1 + b * y2 + c * y3) / (a + b + c)
    incenter_z = (a * z1 + b * z2 + c * z3) / (a + b + c)
    coordinate = [incenter_x, incenter_y, incenter_z]
    return coordinate


class HandList:
    x = 0
    y = 0
    z = 0

    def __init__(self, xin, yin, zin):
        self.x = xin
        self.y = yin
        self.z = zin

def coordinateTrans(point):
    point.x=(point.x-0.5)*-1
    point.y=(point.y-1)*-1
    point.z=point.z
    return  point

inverseKinematics(sqrt(3), 1.5, 2 * sqrt(3), 180)
inverseKinematics(-sqrt(3), 1.5, 2 * sqrt(3), 180)
