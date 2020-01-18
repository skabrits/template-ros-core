from copy import copy

utk_bot = 55 / 3
import math

R1 = None
r1 = None
R2 = None
r2 = None
R3 = None
r3 = None
R4 = None
r4 = None
R5 = None
r5 = None
R6 = None
r6 = None
Rmap = None


def calc_belong(circle_x, circle_y, x, y, is_smez=1):
    ro = ((circle_x - x) ** 2 + (circle_y - y) ** 2)
    ro = math.sqrt(ro)
    if ro == 0:
        return 0, 0
    sin = (x - circle_x) / ro
    sina = sin
    if sin == 0:
        sina = 1
    angle = (1 - is_smez) / 2 * math.pi + is_smez * sina / abs(sina) * math.asin(sin) + 0.5 * (
            1 - sina / abs(sina)) * math.pi
    return angle, ro


class Road:
    def __init__(self, x1, y1, seg, sign, roadId, direct=1):
        self.direct = direct
        self.id = roadId
        self.b_cord = x1, y1
        self.R1 = None
        self.R2 = None
        coordinates = self.b_cord
        x, y = coordinates
        x_s, y_s = (x, y)
        sign = sign
        sequence = seg
        self.segm_types = sequence
        self.segments = list()
        # segmentum
        for segmentum in sequence:
            if segmentum == 'v':
                curr_segm = Vert_road((x_s, y_s))
                self.segments.append(curr_segm)
                x_s, y_s, sign = curr_segm.cord_next(sign)
            if segmentum == 'h':
                curr_segm = Horizontal_road((x_s, y_s))
                self.segments.append(curr_segm)
                x_s, y_s, sign = curr_segm.cord_next(sign)
            if segmentum[:-1] == 'turnrd':
                if segmentum[-1] == 'm':
                    r1 = 0 * utk_bot
                    r2 = 1.5 * utk_bot
                elif segmentum[-1] == 'b':
                    r1 = 1.5 * utk_bot
                    r2 = 3 * utk_bot
                curr_segm = circle_road_right_down((x_s, y_s, r1, r2))
                self.segments.append(curr_segm)
                x_s, y_s, sign = curr_segm.cord_next(sign)
            if segmentum[:-1] == 'turnru':
                if segmentum[-1] == 'm':
                    r1 = 0 * utk_bot
                    r2 = 1.5 * utk_bot
                else:
                    r1 = 1.5 * utk_bot
                    r2 = 3 * utk_bot
                curr_segm = circle_road_right_upper((x_s, y_s, r1, r2))
                self.segments.append(curr_segm)
                x_s, y_s, sign = curr_segm.cord_next(sign)
            if segmentum[:-1] == 'turnld':
                if segmentum[-1] == 'm':
                    r1 = 0 * utk_bot
                    r2 = 1.5 * utk_bot
                else:
                    r1 = 1.5 * utk_bot
                    r2 = 3 * utk_bot
                curr_segm = circle_road_left_down((x_s, y_s, r1, r2))
                self.segments.append(curr_segm)
                x_s, y_s, sign = curr_segm.cord_next(sign)
            if segmentum[:-1] == 'turnlu':
                if segmentum[-1] == 'm':
                    r1 = 0 * utk_bot
                    r2 = 1.5 * utk_bot
                else:
                    r1 = 1.5 * utk_bot
                    r2 = 3 * utk_bot
                curr_segm = circle_road_left_upper((x_s, y_s, r1, r2))
                self.segments.append(curr_segm)
                x_s, y_s, sign = curr_segm.cord_next(sign)

    def is_on_road(self, x, y):
        is_on_road_v = False
        for segmentum in self.segments:
            if segmentum.is_in_phragment(x, y):
                is_on_road_v = True
        return is_on_road_v

    def get_road_len(self):
        length = 0
        for segment in self.segments:
            length += segment.dlina()
        return length

    def get_route_len(self, x, y):
        length = 0
        for i, segment in enumerate(self.segments):
            result = segment.is_in_phragment(x, y)
            if not result:
                length += segment.dlina()  # указание принадлежности к классу
            if result:
                if self.segm_types[i] == 'v':
                    x_e = segment.base_x + segment.len / 2
                    y_e = segment.base_y

                    x_w = x_e
                    y_w = y
                    length += abs(y - y_e)
                if self.segm_types[i] == 'h':
                    x_e = segment.base_x
                    y_e = segment.base_y + segment.wid / 2

                    x_w = x
                    y_w = y_e
                    length += abs(x - x_e)
                if self.segm_types[i][:-1] == 'turnrd':
                    ang, ro = calc_belong(segment.circle_x, segment.circle_y, x, y)
                    length += segment.dlina() * ang / (math.pi / 2)
                if self.segm_types[i][:-1] == 'turnru':
                    ang, ro = calc_belong(segment.circle_x, segment.circle_y, x, y, is_smez=-1)
                    length += segment.dlina() * (ang - math.pi / 2) / (math.pi / 2)
                if self.segm_types[i][:-1] == 'turnlu':
                    ang, ro = calc_belong(segment.circle_x, segment.circle_y, x, y)
                    length += segment.dlina() * (ang - math.pi) / (math.pi / 2)
                if self.segm_types[i][:-1] == 'turnld':
                    ang, ro = calc_belong(segment.circle_x, segment.circle_y, x, y, is_smez=-1)
                    length += segment.dlina() * (ang - 3 * math.pi / 2) / (math.pi / 2)
                return (1 - self.direct) / 2 * self.get_road_len() + self.direct * length
        return length


class Vert_road:
    def __init__(self, cord_b):
        self.base_x, self.base_y = cord_b
        self.len = 1.5 * utk_bot
        self.wid = 3 * utk_bot

    def is_in_phragment(self, x, y):
        s_v = 1
        if self.sign == '-':
            s_v = -1
            if self.base_x <= x <= self.base_x + self.len and self.base_y >= y >= self.base_y + s_v * self.wid:
                return True
            else:
                return False
        else:
            if self.base_x <= x <= self.base_x + self.len and self.base_y <= y <= self.base_y + s_v * self.wid:
                return True
            else:
                return False

    def dlina(self):
        return self.wid

    def cord_next(self, sign):
        self.sign = sign
        if sign == '+':
            self.next_x = self.base_x
            self.next_y = self.base_y + self.wid
        elif sign == '-':
            self.next_x = self.base_x
            self.next_y = self.base_y - self.wid
        return (self.next_x, self.next_y, self.sign)


class Horizontal_road:
    def __init__(self, cord_b):
        self.base_x, self.base_y = cord_b
        self.len = 3 * utk_bot
        self.wid = 1.5 * utk_bot

    def is_in_phragment(self, x, y):
        s_v = 1
        if self.sign == '-':
            s_v = -1
            if (self.base_x >= x >= self.base_x + s_v * self.len and self.base_y >= y >= self.base_y - self.wid):
                return True
            else:
                return False
        else:
            if (self.base_x <= x <= self.base_x + s_v * self.len and self.base_y >= y >= self.base_y - self.wid):
                return True
            else:
                return False

    def dlina(self):
        return self.len

    def cord_next(self, sign):
        self.sign = sign
        if sign == '+':
            self.next_x = self.base_x + self.len
            self.next_y = self.base_y
        elif sign == '-':
            self.next_x = self.base_x - self.len
            self.next_y = self.base_y
        return (self.next_x, self.next_y, self.sign)


class circle_road_right_down:
    def __init__(self, cord_b):
        self.base_x, self.base_y, self.r1, self.r2 = cord_b
        self.len = math.pi / 2 * (self.r1 + self.r2) / 2
        self.circle_x = self.base_x
        self.circle_y = self.base_y + self.r1

    def is_in_phragment(self, x, y):
        angle, ro = calc_belong(self.circle_x, self.circle_y, x, y)
        if ro == 0:
            return True
        if 0 <= angle <= math.pi / 2 and self.r1 <= ro <= self.r2:
            return True
        else:
            return False

    def dlina(self):
        return self.len

    def cord_next(self, sign):
        self.sign = '+'
        self.next_x = self.base_x + self.r1
        self.next_y = self.base_y + self.r1
        return (self.next_x, self.next_y, self.sign)


class circle_road_right_upper:
    def __init__(self, cord_b):
        self.base_x, self.base_y, self.r1, self.r2 = cord_b
        self.len = math.pi / 2 * (self.r1 + self.r2) / 2
        self.circle_x = self.base_x
        self.circle_y = self.base_y - self.r2

    def is_in_phragment(self, x, y):
        angle, ro = calc_belong(self.circle_x, self.circle_y, x, y, is_smez=-1)
        if ro == 0:
            return True
        if math.pi / 2 <= angle <= math.pi and self.r1 <= ro <= self.r2:
            return True
        else:
            return False

    def dlina(self):
        return self.len

    def cord_next(self, sign):
        self.sign = '-'
        self.next_x = self.base_x + self.r1
        self.next_y = self.base_y - self.r2
        return (self.next_x, self.next_y, self.sign)


class circle_road_left_upper:
    def __init__(self, cord_b):
        self.base_x, self.base_y, self.r1, self.r2 = cord_b
        self.len = math.pi / 2 * (self.r1 + self.r2) / 2
        self.circle_x = self.base_x + self.r2
        self.circle_y = self.base_y

    def is_in_phragment(self, x, y):
        angle, ro = calc_belong(self.circle_x, self.circle_y, x, y)
        if ro == 0:
            return True
        if math.pi <= angle <= math.pi * (3 / 2) and self.r1 <= ro <= self.r2:
            return True
        else:
            return False

    def dlina(self):
        return self.len

    def cord_next(self, sign):
        self.sign = '+'
        self.next_x = self.base_x + self.r2
        self.next_y = self.base_y + self.r2
        return (self.next_x, self.next_y, self.sign)


class circle_road_left_down:
    def __init__(self, cord_b):
        self.base_x, self.base_y, self.r1, self.r2 = cord_b
        self.len = math.pi / 2 * (self.r1 + self.r2) / 2
        self.circle_x = self.base_x + self.r2
        self.circle_y = self.base_y

    def is_in_phragment(self, x, y):
        angle, ro = calc_belong(self.circle_x, self.circle_y, x, y, is_smez=-1)
        if ro == 0:
            return True
        if math.pi * 3 / 2 <= angle <= math.pi * 2 and self.r1 <= ro <= self.r2:
            return True
        else:
            return False

    def dlina(self):
        return self.len

    def cord_next(self, sign):
        self.sign = '+'
        self.next_x = self.base_x + self.r2
        self.next_y = self.base_y - self.r1
        return (self.next_x, self.next_y, self.sign)


def init_lib():
    roadId = 0
    segm = ['v', 'turnldb', 'h', 'h', 'h', 'turnrdb', 'v']
    R1 = Road(0 * utk_bot, 6 * utk_bot, segm, '-', roadId)
    roadId += 1
    segm = ['v', 'turnldm', 'h', 'h', 'h', 'turnrdm', 'v']
    r1 = Road(1.5 * utk_bot, 6 * utk_bot, segm, '-', roadId, direct=-1)
    roadId += 1
    segm = ['h', 'turnrub', 'v']
    R2 = Road(9 * utk_bot, 15 * utk_bot, segm, '+', roadId, direct=-1)
    roadId += 1
    segm = ['h', 'turnrum', 'v']
    r2 = Road(9 * utk_bot, 13.5 * utk_bot, segm, '+', roadId)
    roadId += 1
    segm = ['h']
    R3 = Road(9 * utk_bot, 9 * utk_bot, segm, '+', roadId, direct=-1)
    roadId += 1
    r3 = Road(9 * utk_bot, 7.5 * utk_bot, segm, '+', roadId)
    roadId += 1
    segm = ['v', 'turnlub', 'h']
    R4 = Road(0 * utk_bot, 9 * utk_bot, segm, '+', roadId, direct=-1)
    roadId += 1
    segm = ['v', 'turnlum', 'h']
    r4 = Road(1.5 * utk_bot, 9 * utk_bot, segm, '+', roadId)
    roadId += 1
    segm = ['v']
    R5 = Road(6 * utk_bot, 12 * utk_bot, segm, '-', roadId)
    roadId += 1
    r5 = Road(7.5 * utk_bot, 12 * utk_bot, segm, '-', roadId, direct=-1)
    roadId += 1
    segm = ['h']
    R6 = Road(3 * utk_bot, 9 * utk_bot, segm, '+', roadId, direct=-1)
    roadId += 1
    r6 = Road(3 * utk_bot, 7.5 * utk_bot, segm, '+', roadId)
    R1.Road1 = R2
    R1.Road2 = R3
    R2.Road1 = R4
    R2.Road2 = R5
    R3.Road1 = r5
    R3.Road2 = R6
    R4.Road1 = R1
    R4.Road2 = r6
    R5.Road1 = R6
    R5.Road2 = r3
    R6.Road1 = r4
    R6.Road2 = R1
    r1.Road1 = r6
    r1.Road2 = r4
    r2.Road1 = R3
    r2.Road2 = r1
    r3.Road1 = r1
    r3.Road2 = R2
    r4.Road1 = R5
    r4.Road2 = r2
    r5.Road1 = r2
    r5.Road2 = R4
    r6.Road1 = r3
    r6.Road2 = r5

    self.Rmap = [R1, r1, R2, r2, R3, r3, R4, r4, R5, r5, R6, r6]
    return self.Rmap, R1, r1, R2, r2, R3, r3, R4, r4, R5, r5, R6, r6


def find_route(roadFrom, id_to):
    mini = -1
    min_route = []

    def findRoute(roadFrom, id_to, route, mini, min_route):
        l = route.__len__()
        if roadFrom.id == id_to:
            if len(route) < mini or mini == -1:
                mini = len(route)
                min_route = route[:]
            return mini, min_route
        if route.__len__() >= 10:
            return mini, min_route
        route.append(1)
        tmp = route[:]
        mini, min_route = findRoute(roadFrom.Road1, id_to, route, mini, min_route)
        # print('tmp', l, tmp)
        tmp[-1] = 2
        route = tmp
        mini, min_route = findRoute(roadFrom.Road2, id_to, route, mini, min_route)
        return mini, min_route

    mini, min_route = findRoute(roadFrom, id_to, [], mini, min_route)
    return min_route


def det_points_road(x, y, Rmap):
    c_road = None

    for road in Rmap:
        if road.is_on_road(x, y):
            c_road = road

    if c_road is None:
        return "Not on the road"
    else:
        return c_road
        # c_road.id, c_road.segments

# cord = [(x/10*utk_bot,y/10*utk_bot) for x in range(150) for y in range(150)]
# j = 0
# print(cord)
# while j < len(cord):
#     should_d = True
#
#     for road in Rmap:
#         if j >= len(cord):
#             break
#         if road.is_on_road(cord[j][0],cord[j][1]):
#             should_d = False
#     if should_d:
#         cord.pop(j)
#     else:
#         j += 1
# print (cord)
# print((13*utk_bot,12*utk_bot) in set(cord))
# print((13*utk_bot,12*utk_bot))
# import  matplotlib.pyplot as plt
# xes = [x for x,y in cord]
# yes = [y for x,y in cord]
# plt.plot(xes,yes,'bs')
# plt.show()
# print(r1.get_route_len(220,50))
