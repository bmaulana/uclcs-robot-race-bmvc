from __future__ import division 
from src.baseobject import BaseObj
from src.visualize import *

class ConeModel(BaseObj):
    def __init__(self, display, x, y, r):
        BaseObj.__init__(self, BaseObj.CONE)
        zoom = display.zoom

        self.x = x
        self.y = y
        self.r = r
        self.vis = ConeVis(display, x, y, r)
        self.minx = x - r
        self.maxx = x + r
        self.miny = y - r
        self.maxy = y + r
        self.draw()

    def draw(self):
        self.vis.draw()

    def redraw(self):
        self.vis.redraw()

    def erase(self):
        self.vis.erase()

    def closest(self, ax, ay):
        #compute the euclidean distance between ax,ay and x,y
        len_ab = sqrt( (self.x-ax)*(self.x-ax) + (self.y-ay)*(self.y-ay) )

        #compute the direction vector d from a to cone
        dx = (ax-self.x)
        dy = (ay-self.y)

        # Now the line equation is 
        #    x = dx*t + self.x, y = dy*t + self.y with 0 <= t <= 1.

        # compute the value t of the closest point to the circle center
        t = self.r/len_ab

        # This is the projection of cone center on the line from a to b.

        # compute the coordinates of the point e on line and closest
        # to cone center
        ex = t*dx+self.x
        ey = t*dy+self.y

        dist = sqrt((ex-ax)*(ex-ax) + (ey-ay)*(ey-ay))
        return dist,ex,ey
        

    def intersects(self, ax,ay,bx,by):
        # first check bounding box
        slop = 5
        consts = 0,0,0
        if ax < self.minx-slop and bx < self.minx-slop:
            return -1,bx,by, consts
        if ax > self.maxx+slop and bx > self.maxx+slop:
            return -1,bx,by, consts
        if ay < self.miny-slop and by < self.miny-slop:
            return -1,bx,by, consts
        if ay > self.maxy+slop and by > self.maxy+slop:
            return -1,bx,by, consts

        #compute the euclidean distance between ax,ay and bx,by
        len_ab = sqrt( (bx-ax)*(bx-ax) + (by-ay)*(by-ay) )

        #compute the direction vector d from a to b
        dx = (bx-ax)/len_ab
        dy = (by-ay)/len_ab

        # Now the line equation is 
        #    x = dx*t + ax, y = dy*t + ay with 0 <= t <= 1.

        # compute the value t of the closest point to the circle center
        t = dx*(self.x-ax) + dy*(self.y-ay)

        # This is the projection of cone center on the line from a to b.

        # compute the coordinates of the point e on line and closest
        # to cone center
        ex = t*dx+ax
        ey = t*dy+ay

        # compute the euclidean distance from e to cone centre
        len_ec = sqrt( (ex-self.x)*(ex-self.x) + (ey-self.y)*(ey-self.y) )

        #test if the line intersects the circle
        if( len_ec < self.r ):
            # compute distance from t to circle intersection point
            dt = sqrt( self.r*self.r  - len_ec*len_ec)
        
            # compute first intersection point
            fx = (t-dt)*dx + ax
            fy = (t-dt)*dy + ay
            dist = sqrt( (fx-ax)*(fx-ax) + (fy-ay)*(fy-ay))

            consts = dx,dy,t,dt,ax,ay
            return dist,fx,fy,consts

        # else test if the line is tangent to circle
        else:
            if( len_ec == self.r ):
                # tangent point to circle is E
                dist = sqrt( (ex-ax)*(ex-ax) + (ey-ay)*(ey-ay))
                return dist,ex,ey, consts
            else:
                # line doesn't touch circle
                return -1,bx,by, consts


    def add_noise(self, x1, y1, x2, y2, noisy_dist, consts):
        dx,dy,t,dt,ax,ay = consts
        fx = (t-dt)*dx + ax
        fy = (t-dt)*dy + ay
        orig_dist = sqrt( (fx-ax)*(fx-ax) + (fy-ay)*(fy-ay))
        dist_scale = noisy_dist/orig_dist
        
        x = dist_scale*(t-dt)*dx + ax
        y = dist_scale*(t-dt)*dy + ay
        return x,y
