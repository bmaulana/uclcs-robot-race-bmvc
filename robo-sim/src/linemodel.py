from __future__ import division 
from src.baseobject import BaseObj
from src.visualize import *

class LineModel(BaseObj):
    def __init__(self, display, x1, y1, x2, y2):
        BaseObj.__init__(self, BaseObj.LINE)
        zoom = display.zoom

        # we extend the line a few pixels beyond the end position so
        # we don't end up with gaps between consecutive lines
        extension = 3

        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

        if x1 == x2:
            if y2 > y1:
                self.y2 = y2 + extension
                self.y1 = y1 - extension
            else:
                self.y2 = y2 - extension
                self.y1 = y1 + extension
        else:
            angle = atan2((y2 - y2), (x2 - x1))
            self.y2 = y2 + (extension * sin(angle))
            self.x2 = x2 + (extension * cos(angle))
            self.y1 = y1 - (extension * sin(angle))
            self.x1 = x1 - (extension * cos(angle))
            

        self.vis = LineVis(display, x1, y1, x2, y2)
        if x1 < x2:
            self.minx = self.x1
            self.maxx = self.x2
        else:
            self.minx = self.x2
            self.maxx = self.x1

        if y1 < y2:
            self.miny = self.y1
            self.maxy = self.y2
        else:
            self.miny = self.y2
            self.maxy = self.y1
        self.draw()

    def draw(self):
        self.vis.draw()

    def redraw(self):
        self.vis.redraw()

    def erase(self):
        self.vis.erase()

    def closest(self, x1, y1):
        return self.closest_to_line(x1, y1, self.x1, self.y1, self.x2, self.y2)

        
