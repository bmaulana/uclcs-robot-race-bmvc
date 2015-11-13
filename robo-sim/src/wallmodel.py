from __future__ import division 
from src.baseobject import BaseObj
from src.visualize import *

class WallModel(BaseObj):
    def __init__(self, display, x1, y1, x2, y2):
        BaseObj.__init__(self, BaseObj.WALL)
        zoom = display.zoom

        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        #fudge factor - to avoid rounding errors resulting in values
        #where a valid intersection is ignored because it's not "in"
        #the object.
        self.fudge = 0.000001 
        self.vis = WallVis(display, x1, y1, x2, y2)
        if x1 < x2:
            self.minx = x1
            self.maxx = x2
        else:
            self.minx = x2
            self.maxx = x1

        if y1 < y2:
            self.miny = y1
            self.maxy = y2
        else:
            self.miny = y2
            self.maxy = y1
        self.draw()

    def draw(self):
        self.vis.draw()

    def redraw(self):
        self.vis.redraw()

    def erase(self):
        self.vis.erase()

    def closest(self, x1, y1):
        return self.closest_to_line(x1, y1, self.x1, self.y1, self.x2, self.y2)
        

    def intersects(self, x1,y1,x2,y2):
        #print("checking bb");
        # first check bounding box
        slop = 5
        if x1 < self.minx-slop and x2 < self.minx-slop:
            return -1,x2,y2,(0,0,0)
        if x1 > self.maxx+slop and x2 > self.maxx+slop:
            return -1,x2,y2,(0,0,0)
        if y1 < self.miny-slop and y2 < self.miny-slop:
            return -1,x2,y2,(0,0,0)
        if y1 > self.maxy+slop and y2 > self.maxy+slop:
            return -1,x2,y2,(0,0,0)
        #print("bb intersects")

        wall_x1,wall_y1 = self.translate(x1,y1, self.x1, self.y1)
        wall_x2,wall_y2 = self.translate(x1,y1, self.x2, self.y2)
        his_x2,his_y2 = self.translate(x1,y1, x2, y2)

        if (his_x2 == 0):
            #his line is vertical
            #print("his is vertical")
            if(wall_x2 == wall_x1):
                #parallel vertical lines
                return -2,x2,y2, (0, 0, 0)
            else:
                k2 = (wall_y2 - wall_y1)/(wall_x2 - wall_x1)
                k1 = 0
                c = 0
                x = 0
                y = wall_y1 - k2*wall_x1
        else:
            #his line is not vertical
            #print("his is not vertical")
            if (wall_x2 == wall_x1):
                #print("mine is vertical")
                #my line is vertical
                k1 = his_y2/his_x2
                k2 = 0
                c = 0
                x = wall_x1
                y = k1 * x
            else:
                #print("neither is vertical")
                #my line is not vertical
                k1 = his_y2/his_x2
                k2 = (wall_y2 - wall_y1)/(wall_x2 - wall_x1)
                c = wall_y1 - (k2*wall_x1)
                if (k1 == k2):
                    #lines are parallel
                    return -1,x2,y2, (0, 0, 0)
                else:
                    x = c/(k1-k2)
                    y = k1 * x

        #is the intersection point in range?
        i_sqr = x*x + y*y
        #1.02 adds some slop
        range_sqr = 1.02*(his_x2*his_x2 + his_y2*his_y2)
        if i_sqr > range_sqr:
            #print("not in range1")
            return -2,x2,y2,(0,0,0)

        #also needs to be in range of x2 for collision detection
        xdiff = x - his_x2
        ydiff = y - his_y2
        i2_sqr = xdiff*xdiff + ydiff*ydiff
        if i2_sqr > range_sqr:
            #print("not in range2: i2_sqr = ", i2_sqr, " range_sqr=", range_sqr)
            return -2,x2,y2,(0,0,0)
        

        #is the intersection point in the object limits?
        i_x,i_y = self.translate(-x1,-y1, x, y)
        if i_x < self.minx - self.fudge or i_x > self.maxx + self.fudge:
            #print("not in limits1")
            return -2,x2,y2, (0, 0, 0)
        if i_y < self.miny - self.fudge or i_y > self.maxy + self.fudge:
            #print("not in limits2")
            return -2,x2,y2, (0, 0, 0)

        i_dist = sqrt(i_sqr)

        #print("i_sqr=", i_sqr, "i2_sqr=", i2_sqr, "range_sqr=", range_sqr)

        #returns a tuple:
        # i_dist: distance to object
        # i_x, i_y: position of intersection point
        # (k1,k2,c) line constants we'll use later to calculate noise
        return i_dist, i_x, i_y, (k1, k2, c)

    def add_noise(self, x1, y1, x2, y2, noisy_dist, consts):
        k1,k2,c = consts
        if k1 == 0 and c == 0:
            if k2 == 0:
                #nothing to calculate - beam must be on a boundary
                #wall, so just scale range appropriately
                dx = x2-x1
                dy = y2-y1
                dist = sqrt(dx*dx + dy*dy)
                scale = noisy_dist/dist
                x = x1+dx*scale
                y = y1+dy*scale
                return x,y
            x = 0
            wall_x1,wall_y1 = self.translate(x1,y1, self.x1, self.y1)
            y = wall_y1 - k2*wall_x1
        elif k2 == 0 and c == 0:
            wall_x1,wall_y1 = self.translate(x1,y1, self.x1, self.y1)
            x = wall_x1
            y = k1 * x
        else:
            x = c/(k1-k2)
            y = k1 * x
        dist = sqrt(x*x + y*y)
        x = x * noisy_dist/dist
        y = y * noisy_dist/dist
        i_x,i_y = self.translate(-x1,-y1, x, y)
        return i_x, i_y
    
