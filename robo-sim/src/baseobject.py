from __future__ import division 
from math import *

class BaseObj():
    ROBOT = 0
    WALL = 1
    LINE = 2
    CONE = 3
    TARGET = 4
    CARPET = 5

    def __init__(self, type):
        self.type = type

    def rotate(self, x, y, ox, oy, angle):
        nx = ox - ((x-ox) * cos(angle) + (y-oy) * sin(angle))
        ny = oy + (y-oy) * cos(angle) - (x-ox) * sin(angle)
        return tuple([nx, ny])

    def rotate_about_origin(self, x, y, angle):
        nx = x * cos(angle) - y * sin(angle)
        ny = y * cos(angle) + x * sin(angle)
        return tuple([nx, ny])

    def translate(self, origin_x, origin_y, x, y):
        return tuple([x - origin_x, y - origin_y])

    def dist(self, x1, y1, x2, y2):
        xd = x1-x2
        yd = y1-y2
        return sqrt(xd*xd+yd*yd)

    def closest_to_line(self, x, y, lx1, ly1, lx2, ly2):
        x1r,y1r = self.translate(lx1, ly1, x,y)
        x2r,y2r = self.translate(lx1, ly1, lx2, ly2)
        if (x2r == 0):
            if y2r > 0:
                angle = pi / 2
            else:
                angle = 3 * pi / 2
        else:
            angle = atan(y2r/x2r)
        #print(lx1,ly1," ",lx2,ly2,"angle=", 180 *angle / pi)

        x1r,y1r = self.rotate_about_origin(x1r, y1r, -angle)
        x2r,y2r = self.rotate_about_origin(x2r, y2r, -angle)        
        if (y2r > 1e-6 or y2r < -1e-6):
            print("**** Error in maths ******")

        x_c = x1r
        if x2r > 0:
            if (x_c < 0):
                #print("LLimit")
                x_c = 0
            if (x_c > x2r):
                #print("ULimit")
                x_c = x2r
        else:
            if (x_c > 0):
                #print("LLimit")
                x_c = 0
            if (x_c < x2r):
                #print("ULimit")
                x_c = x2r
        #print("x_c: ", x_c)
        y_c = 0
        
        x_c,y_c = self.rotate_about_origin(x_c, y_c, angle)
        x_c,y_c = self.translate(-lx1, -ly1, x_c, y_c)
        dist = self.dist(x,y,x_c,y_c)
        return dist,x_c,y_c
        
