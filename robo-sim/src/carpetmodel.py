from __future__ import division 
from src.baseobject import BaseObj
from src.visualize import *

class CarpetModel(BaseObj):
    def __init__(self, display, x1, y1, x2, y2, x3, y3, x4, y4, drag1, drag2):
        BaseObj.__init__(self, BaseObj.CARPET)
        zoom = display.zoom

        self.coords = [x1,y1, x2, y2, x3, y3, x4, y4]
        self.drag1 = drag1
        self.drag2 = drag2
        self.vis = CarpetVis(display, self.coords)

        #compute bounding box
        self.minx = min(x1,x2,x3,x4)
        self.maxx = max(x1,x2,x3,x4)
        self.miny = min(y1,y2,y3,y4)
        self.maxy = max(y1,y2,y3,y4)
        self.draw()

    def draw(self):
        self.vis.draw()

    def redraw(self):
        self.vis.redraw()

    def erase(self):
        self.vis.erase()

    def get_drag(self, (wx,wy), prev_drag):
        #only return edge drag if we're not going from centre to edge
        if prev_drag != self.drag2 and self.is_on_edge(wx,wy):
            return self.drag1
        elif self.is_within(wx,wy):
            return self.drag2
        else:
            return 0

    def is_within(self, x, y):
        if x < self.minx:
            return False
        if x > self.maxx:
            return False
        if y < self.miny:
            return False
        if y > self.maxy:
            return False

        #fix this!
        return True

    def is_on_edge(self, x, y):
        if x < self.minx:
            return False
        if x > self.maxx:
            return False
        if y < self.miny:
            return False
        if y > self.maxy:
            return False

        closest_dist = 1000
        closest_coords = (0,0)
        for i in range(0,4):
            dist,xl,yl = self.closest_to_line(x, y, 
                                              self.coords[(i*2)%8], 
                                              self.coords[(i*2+1)%8], 
                                              self.coords[(i*2+2)%8], 
                                              self.coords[(i*2+3)%8])
            if dist < closest_dist:
                closest_dist = dist
                closest_coords = (xl,yl)
        if closest_dist < 3:
            return True
        return False
        

