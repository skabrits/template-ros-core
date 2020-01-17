from copy import copy

utk_bot = 55/3
import math


roadId = 0


def calc_belong(circle_x, circle_y, x, y, is_smez=1):
    ro = ((circle_x - x) ** 2 + (circle_y - y) ** 2)
    ro = math.sqrt(ro)
    if ro == 0:
        return 0,0
    sin = (x - circle_x) / ro
    sina = sin
    if sin == 0:
        sina = 1
    angle = (1 - is_smez)/2*math.pi + is_smez * sina / abs(sina) * math.asin(sin) + 0.5 * (1 - sina / abs(sina)) * math.pi
    return angle, ro

class Road:
    def __init__(self,x1,y1,seg,sign,direct=1):
        self.direct = direct
        global roadId
        self.id = roadId
        roadId += 1
        self.b_cord = x1,y1
        self.R1=None
        self.R2=None
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

    def is_on_road(self,x,y):
        is_on_road_v = False
        for segmentum in self.segments:
            if segmentum.is_in_phragment(x,y):
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
                    x_e = segment.base_x + segment.len/2
                    y_e = segment.base_y

                    x_w = x_e
                    y_w = y
                    length += abs(y - y_e)
                if self.segm_types[i] == 'h':
                    x_e = segment.base_x
                    y_e = segment.base_y + segment.wid/2

                    x_w = x
                    y_w = y_e
                    length += abs(x - x_e)
                if self.segm_types[i][:-1] == 'turnrd':
                    ang, ro = calc_belong(segment.circle_x, segment.circle_y, x, y)
                    length += segment.dlina() * ang/(math.pi/2)
                if self.segm_types[i][:-1] == 'turnru':
                    ang, ro = calc_belong(segment.circle_x, segment.circle_y, x, y, is_smez=-1)
                    length += segment.dlina() * (ang-math.pi/2) / (math.pi / 2)
                if self.segm_types[i][:-1] == 'turnlu':
                    ang, ro = calc_belong(segment.circle_x, segment.circle_y, x, y)
                    length += segment.dlina() * (ang - math.pi) / (math.pi / 2)
                if self.segm_types[i][:-1] == 'turnld':
                    ang, ro = calc_belong(segment.circle_x, segment.circle_y, x, y, is_smez=-1)
                    length += segment.dlina() * (ang - 3 * math.pi / 2) / (math.pi / 2)
                return (1-self.direct)/2*self.get_road_len()+self.direct*length
        return length


class Vert_road:
    def __init__(self,cord_b):
        self.base_x, self.base_y = cord_b
        self.len = 1.5 * utk_bot
        self.wid = 3 * utk_bot
    def is_in_phragment(self,x,y):
        s_v = 1
        if self.sign == '-':
            s_v = -1
            if self.base_x <= x <= self.base_x + self.len and self.base_y >= y >= self.base_y + s_v * self.wid:
                return True
            else:
                return False
        else:
            if self.base_x<=x<=self.base_x+self.len and self.base_y<=y<=self.base_y+s_v*self.wid:
                return True
            else:
                return False
    def dlina(self):
        return self.wid
    def cord_next(self,sign):
        self.sign = sign
        if sign == '+':
            self.next_x = self.base_x
            self.next_y = self.base_y + self.wid
        elif sign == '-':
            self.next_x = self.base_x
            self.next_y = self.base_y - self.wid
        return (self.next_x, self.next_y,self.sign)

class Horizontal_road:
    def __init__(self,cord_b):
        self.base_x, self.base_y = cord_b
        self.len = 3 * utk_bot
        self.wid = 1.5 * utk_bot
    def is_in_phragment(self,x,y):
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
    def cord_next(self,sign):
        self.sign = sign
        if sign == '+':
            self.next_x = self.base_x+self.len
            self.next_y = self.base_y
        elif sign == '-':
            self.next_x = self.base_x-self.len
            self.next_y = self.base_y
        return (self.next_x, self.next_y,self.sign)

class circle_road_right_down:
    def __init__(self,cord_b):
        self.base_x, self.base_y, self.r1, self.r2 = cord_b
        self.len = math.pi / 2 * (self.r1 + self.r2) / 2
        self.circle_x = self.base_x
        self.circle_y = self.base_y + self.r1
    def is_in_phragment(self,x,y):
        angle, ro = calc_belong(self.circle_x, self.circle_y, x, y)
        if ro == 0:
            return True
        if 0<=angle<=math.pi/2 and self.r1<=ro<=self.r2:
            return True
        else:
            return False
    def dlina(self):
        return self.len
    def cord_next(self,sign):
        self.sign='+'
        self.next_x=self.base_x+self.r1
        self.next_y=self.base_y+self.r1
        return(self.next_x,self.next_y,self.sign)

class circle_road_right_upper:
    def __init__(self,cord_b):
        self.base_x, self.base_y, self.r1, self.r2 = cord_b
        self.len = math.pi / 2 * (self.r1 + self.r2) / 2
        self.circle_x = self.base_x
        self.circle_y = self.base_y - self.r2
    def is_in_phragment(self,x,y):
        angle, ro = calc_belong(self.circle_x, self.circle_y, x, y, is_smez=-1)
        if ro == 0:
            return True
        if math.pi/2<=angle<=math.pi and self.r1<=ro<=self.r2:
            return True
        else:
            return False
    def dlina(self):
        return self.len
    def cord_next(self,sign):
        self.sign = '-'
        self.next_x = self.base_x + self.r1
        self.next_y = self.base_y - self.r2
        return (self.next_x, self.next_y, self.sign)



class circle_road_left_upper:
    def __init__(self,cord_b):
        self.base_x, self.base_y, self.r1, self.r2 = cord_b
        self.len = math.pi / 2 * (self.r1 + self.r2) / 2
        self.circle_x = self.base_x + self.r2
        self.circle_y = self.base_y
    def is_in_phragment(self,x,y):
        angle, ro = calc_belong(self.circle_x, self.circle_y, x, y)
        if ro == 0:
            return True
        if math.pi<=angle<=math.pi*(3/2) and self.r1<=ro<=self.r2:
            return True
        else:
            return False
    def dlina(self):
        return self.len
    def cord_next(self,sign):
        self.sign = '+'
        self.next_x = self.base_x + self.r2
        self.next_y = self.base_y + self.r2
        return (self.next_x, self.next_y,self.sign)

class circle_road_left_down:
    def __init__(self,cord_b):
        self.base_x, self.base_y, self.r1, self.r2 = cord_b
        self.len = math.pi / 2 * (self.r1 + self.r2) / 2
        self.circle_x = self.base_x + self.r2
        self.circle_y = self.base_y
    def is_in_phragment(self,x,y):
        angle, ro = calc_belong(self.circle_x, self.circle_y, x, y, is_smez=-1)
        if ro == 0:
            return True
        if math.pi*3/2<=angle<=math.pi*2 and self.r1<=ro<=self.r2:
            return True
        else:
            return False

    def dlina(self):
        return self.len
    def cord_next(self,sign):
        self.sign = '+'
        self.next_x = self.base_x + self.r2
        self.next_y = self.base_y - self.r1
        return (self.next_x, self.next_y,self.sign)



segm = ['v','turnldb','h','h','h','turnrdb','v']
R1 = Road(0*utk_bot,6*utk_bot,segm,'-')
segm = ['v','turnldm','h','h','h','turnrdm','v']
r1 = Road(1.5*utk_bot,6*utk_bot,segm,'-',direct = -1)
segm = ['h','turnrub','v']
R2 = Road(9*utk_bot,15*utk_bot,segm,'+',direct = -1)
segm = ['h','turnrum','v']
r2 = Road(9*utk_bot,13.5*utk_bot,segm,'+')
segm = ['h']
R3 = Road(9*utk_bot,9*utk_bot,segm,'+', direct = -1)
r3 = Road(9*utk_bot,7.5*utk_bot,segm,'+')
segm = ['v','turnlub','h']
R4 = Road(0*utk_bot,9*utk_bot,segm,'+',direct = -1)
segm = ['v','turnlum','h']
r4 = Road(1.5*utk_bot,9*utk_bot,segm,'+')
segm = ['v']
R5 = Road(6*utk_bot,12*utk_bot,segm,'-')
r5 = Road(7.5*utk_bot,12*utk_bot,segm,'-',direct = -1)
segm = ['h']
R6 = Road(3*utk_bot,9*utk_bot,segm,'+',direct = -1)
r6 = Road(3*utk_bot,7.5*utk_bot,segm,'+')
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



min = -1
min_route = []


def findRoute(roadFrom, id_to, route):
    l = route.__len__()
    if roadFrom.id == id_to:
        global min, min_route
        if len(route) < min or min == -1:
            min = len(route)
            min_route = route[:]
        return
    if route.__len__() >= 10:
        return
    route.append(1)
    tmp = route[:]
    findRoute(roadFrom.Road1, id_to, route)
    # print('tmp', l, tmp)
    tmp[-1] = 2
    route = tmp
    findRoute(roadFrom.Road2, id_to, route)


findRoute(R3, r3.id, [])

print(min_route)


c_road = None
Rmap = [R1,r1,R2,r2,R3,r3,R4,r4,R5,r5,R6,r6]
for road in Rmap:
    if road.is_on_road(7.5*utk_bot,1.5*utk_bot):
        c_road = road

if c_road==None:
    print("Not on the road")
else:
    print(c_road.id,"\n\n", c_road.segments)

cord = [(x/10*utk_bot,y/10*utk_bot) for x in range(150) for y in range(150)]
j = 0
# print(cord)
while j < len(cord):
    should_d = True

    for road in Rmap:
        if j >= len(cord):
            break
        if road.is_on_road(cord[j][0],cord[j][1]):
            should_d = False
    if should_d:
        cord.pop(j)
    else:
        j += 1
print (cord)
print((13*utk_bot,12*utk_bot) in set(cord))
print((13*utk_bot,12*utk_bot))
import  matplotlib.pyplot as plt
xes = [x for x,y in cord]
yes = [y for x,y in cord]
plt.plot(xes,yes,'bs')
plt.show()
print(r1.get_route_len(220,50))