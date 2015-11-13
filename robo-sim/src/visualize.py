#!/opt/local/bin/python3.2
from __future__ import division 
import sys
from . import sgc
from sgc.locals import *
import pygame
from pygame import SRCALPHA
from src.constants import L,M,R,FRONT,SIDE
PYTHON_MAJOR = sys.version_info[0]

# if PYTHON_MAJOR == 2:
#     from Tkinter import *
# else:
#     from tkinter import *
from math import *

#initialize colors
red = pygame.Color("red")
palered = pygame.Color(255,150,150)
blue = pygame.Color("blue")
paleblue = pygame.Color(150,150,255, 100)
grey = pygame.Color("grey")
darkgrey = pygame.Color("darkgrey")
black = pygame.Color("black")
white = pygame.Color("white")
green = pygame.Color("green")
pink = pygame.Color("pink")
darkgreen = pygame.Color("darkgreen")
navyblue = pygame.Color("navyblue")

class BaseObjVis():
    def __init__(self, display):
        #self.canvas = display.canvas
        self.display = display
        self.g_screen = display.g_screen()



    def draw_self(self):
        print("base draw self");

class WallVis(BaseObjVis):
    def __init__(self, display, x1, y1, x2, y2):
        BaseObjVis.__init__(self, display)
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def draw(self):
        zoom = self.display.zoom
        x_offset = self.display.x_offset
        y_offset = self.display.y_offset
        pygame.draw.line(self.g_screen, black, 
                         ((self.x1 - x_offset)*zoom,
                          (self.y1 - y_offset)*zoom),
                         ((self.x2 - x_offset)*zoom,
                          (self.y2 - y_offset)*zoom), 3)


    def redraw(self):
        self.draw()


class ConeVis(BaseObjVis):
    def __init__(self, display, x, y, r):
        BaseObjVis.__init__(self, display)
        self.x1 = x-r
        self.y1 = y-r
        self.x2 = x+r
        self.y2 = y+r
        self.x = x
        self.y = y
        self.r = r

    def draw(self):
        zoom = self.display.zoom
        x_offset = self.display.x_offset
        y_offset = self.display.y_offset
        x = int((self.x - x_offset)*zoom)
        y = int((self.y - y_offset)*zoom)
        r = int(self.r*zoom)
        pygame.draw.circle(self.g_screen, black, (x, y), r, 3)
        

    def redraw(self):
        self.draw()


class TargetVis(BaseObjVis):
    def __init__(self, display, x, y, r, n):
        BaseObjVis.__init__(self, display)
        self.x1 = x-r
        self.y1 = y-r
        self.x2 = x+r
        self.y2 = y+r
        self.x = x
        self.y = y
        self.r = r
        self.num = n
        self.font = Font["mono"]
        self.basefontsize = 48
        self.fontsize = self.basefontsize
        self.font = pygame.font.SysFont("Arial", self.fontsize)
        size = 2*r,2*r
        self.surface = pygame.Surface(size,SRCALPHA).convert_alpha()
        self.surface.fill((255,255,255,0))
        self.text = self.font.render(self.num, True, palered)
        w,h = self.text.get_size()
        #self.surface.blit(self.text, (0,0))
        self.surface.blit(self.text, (r-w/2, r-h/2))
        pygame.draw.circle(self.surface, palered, (r, r), r, 3)

    def draw(self):
        zoom = self.display.zoom
        r = int(self.r*zoom)
        fontsize = int(self.basefontsize * zoom)
        if fontsize != self.fontsize:
            self.fontsize = fontsize
            self.font = pygame.font.SysFont("Arial", self.fontsize)
            size = int(zoom*2*self.r)
            self.surface = pygame.Surface((size,size),SRCALPHA).convert_alpha()
            self.surface.fill((255,255,255,0))
            self.text = self.font.render(self.num, True, palered)
            w,h = self.text.get_size()
            self.surface.blit(self.text, (r-w/2, r-h/2))
            pygame.draw.circle(self.surface, palered, (r, r), r, int(zoom*3))

        x_offset = self.display.x_offset
        y_offset = self.display.y_offset
        x = int((self.x - x_offset)*zoom)
        y = int((self.y - y_offset)*zoom)
        #pygame.draw.circle(self.g_screen, palered, (x, y), r, 3)
        w,h = self.surface.get_size()
        self.g_screen.blit(self.surface,(x - w/2, y - h/2))
        #self.g_screen.blit(self.surface,(x, y))
        

    def redraw(self):
        self.draw()


class CarpetVis(BaseObjVis):
    def __init__(self, display, coords):
        BaseObjVis.__init__(self, display)
        self.coords = coords
        self.create_surface()
        self.prevzoom = self.display.zoom

    def create_surface(self):
        zoom = self.display.zoom
        self.minx = min(self.coords[0], self.coords[2], self.coords[4], self.coords[6])
        self.maxx = max(self.coords[0], self.coords[2], self.coords[4], self.coords[6])
        self.miny = min(self.coords[1], self.coords[3], self.coords[5], self.coords[7])
        self.maxy = max(self.coords[1], self.coords[3], self.coords[5], self.coords[7])
        size = (self.maxx-self.minx)*zoom+2, (self.maxy-self.miny)*zoom+2
        self.surface = pygame.Surface(size,SRCALPHA).convert_alpha()
        self.surface.fill((255,255,255,0))
        polycoords = [(zoom*(self.coords[0] - self.minx), 
                       zoom*(self.coords[1] - self.miny)),\
                      (zoom*(self.coords[2] - self.minx), 
                       zoom*(self.coords[3] - self.miny)),\
                      (zoom*(self.coords[4] - self.minx), 
                       zoom*(self.coords[5] - self.miny)),\
                      (zoom*(self.coords[6] - self.minx),
                       zoom*(self.coords[7] - self.miny))]
        pygame.draw.polygon(self.surface, paleblue, polycoords, 0)
        pygame.draw.polygon(self.surface, navyblue, polycoords, 1)

    def draw(self):
        zoom = self.display.zoom
        if zoom != self.prevzoom:
            self.prevzoom = zoom
            self.create_surface()

        x_offset = self.display.x_offset
        y_offset = self.display.y_offset
        x = int((self.minx - x_offset)*zoom)
        y = int((self.miny - y_offset)*zoom)
        self.g_screen.blit(self.surface,(x, y))
        

    def redraw(self):
        self.draw()


class LineVis(BaseObjVis):
    def __init__(self, display, x1, y1, x2, y2):
        BaseObjVis.__init__(self, display)
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def draw(self):
        zoom = self.display.zoom
        x_offset = self.display.x_offset
        y_offset = self.display.y_offset
        pygame.draw.line(self.g_screen, darkgrey, 
                         ((self.x1 - x_offset)*zoom,
                          (self.y1 - y_offset)*zoom),
                         ((self.x2 - x_offset)*zoom,
                          (self.y2 - y_offset)*zoom), 9)
        pygame.draw.line(self.g_screen, white, 
                         ((self.x1 - x_offset)*zoom,
                          (self.y1 - y_offset)*zoom),
                         ((self.x2 - x_offset)*zoom,
                          (self.y2 - y_offset)*zoom), 3)

    def redraw(self):
        self.draw()

class Blob():
    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.blob = 0
        self.color = color

    def redraw(self, display):
        zoom = display.zoom
        x_offset = display.x_offset
        y_offset = display.y_offset
        x1 = (self.x - 2 - x_offset)*zoom
        y1 = (self.y - 2 - y_offset)*zoom
        x2 = (self.x + 2 - x_offset)*zoom
        y2 = (self.y + 2 - y_offset)*zoom

        rect = pygame.Rect(x1-2,y1-2,4,4)
        color = pygame.Color(self.color)
        pygame.draw.rect(display.g_screen(), color, rect, 0)


class RobotVis(BaseObjVis):
    def __init__(self, display, x, y, angle, size, \
                     max_front_ir_range, max_side_ir_range):
        BaseObjVis.__init__(self, display)
        self.display = display
        self.count = 0
        self.x = x
        self.y = y
        self.size = size
        self.max_front_ir_range = max_front_ir_range
        self.max_side_ir_range = max_side_ir_range
        self.angle = angle * (pi / 180)
        self.ir_angle_left = 0
        self.ir_angle_right = 0
        self.ir_beam_front_on = True
        self.ir_beam_front_left = 0
        self.ir_beam_front_right = 0
        self.ir_beam_side_on = True
        self.ir_beam_side_left = 0
        self.ir_beam_side_right = 0
        self.us_beam_on = True
        self.us_beam = []
        self.blob = []
        self.trail = []
        self.points = []
        x_offset = self.display.x_offset
        y_offset = self.display.y_offset
        zoom = self.display.zoom
        self.prevzoom = zoom
        self.basefontsize = 16
        self.precalculate_text(zoom)
        self.sensor_range = [0,0,0,0,0]
            
        self.draw()
#        self.draw(0,0)
        x1 = (x-5)*zoom
        y1 = (y-5)*zoom
        x2 = (x+5)*zoom
        y2 = (y+5)*zoom
#         self.blob.append(self.canvas.create_polygon(x1,y1,x2,y1,x2,y2,x1,y2, fill="red"))
#         self.blob.append(self.canvas.create_polygon(x1,y1,x2,y1,x2,y2,x1,y2, fill="red"))
#         self.blob.append(self.canvas.create_polygon(x1,y1,x2,y1,x2,y2,x1,y2, fill="red"))
#         self.blob.append(self.canvas.create_polygon(x1,y1,x2,y1,x2,y2,x1,y2, fill="red"))
#         self.blob.append(self.canvas.create_polygon(x1,y1,x2,y1,x2,y2,x1,y2, fill="red"))
#         self.blob.append(self.canvas.create_polygon(x1,y1,x2,y1,x2,y2,x1,y2, fill="red"))
#         self.blob.append(self.canvas.create_polygon(x1,y1,x2,y1,x2,y2,x1,y2, fill="pink"))
#         self.blob.append(self.canvas.create_polygon(x1,y1,x2,y1,x2,y2,x1,y2, fill="pink"))
#         self.blob.append(self.canvas.create_polygon(x1,y1,x2,y1,x2,y2,x1,y2, fill="pink"))

    def precalculate_text(self, zoom):
        self.fontsize = int(self.basefontsize*zoom)
        self.font = pygame.font.SysFont("Arial", self.fontsize)
        self.nums=[]
        for i in range(0,400):
            self.nums.append(self.font.render(str(i), True, red))

    def set_display_text(self, line, text):
        self.display.set_display_text(line, text)

    def set_sensor_range(self, sensornum, value):
        self.sensor_range[sensornum] = value

    def set_sensor_visibility(self, ir_front, ir_side, us, line):
        self.ir_beam_front_on = ir_front
        self.ir_beam_side_on = ir_side
        self.us_beam_on = us
        self.display.set_sensor_visibility(ir_front, ir_side, us, line)
        
    def rotate(self, x, y, ox, oy, angle):
        nx = ox - ((x-ox) * cos(angle) + (y-oy) * sin(angle))
        ny = oy + (y-oy) * cos(angle) - (x-ox) * sin(angle) 
        return tuple([nx, ny])

    def draw(self):
        x_offset = self.display.x_offset
        y_offset = self.display.y_offset
        zoom = self.display.zoom
        if (zoom != self.prevzoom):
            self.precalculate_text(zoom)
            self.prevzoom = zoom
        self.redraw_trail()
        self.redraw_points()
        
        x = (self.x - x_offset) * zoom
        y = (self.y - y_offset)* zoom
        size = self.size * zoom
        
        #df is half the width of the front straight side
        df = 0.3*size
        #dr is half the width of the robot
        dr = 0.5*size

        x1 = x - dr
        y1 = y - df
        x2 = x - df
        y2 = y - dr
        
        x3 = x + df
        y3 = y - dr
        x4 = x + dr
        y4 = y - df

        x5 = x + dr
        y5 = y + df
        x6 = x + df
        y6 = y + dr

        x7 = x - df
        y7 = y + dr
        x8 = x - dr
        y8 = y + df
        x1,y1 = self.rotate(x1,y1,x, y, self.angle)
        x2,y2 = self.rotate(x2,y2,x, y, self.angle)
        x3,y3 = self.rotate(x3,y3,x, y, self.angle)
        x4,y4 = self.rotate(x4,y4,x, y, self.angle)
        x5,y5 = self.rotate(x5,y5,x, y, self.angle)
        x6,y6 = self.rotate(x6,y6,x, y, self.angle)
        x7,y7 = self.rotate(x7,y7,x, y, self.angle)
        x8,y8 = self.rotate(x8,y8,x, y, self.angle)
#        self.base = self.canvas.create_polygon(x1,y1,x2,y2,x3,y3,x4,y4,
#                                               x5,y5,x6,y6,x7,y7,x8,y8, fill="darkgrey")

        poly = [(x1,y1),(x2,y2),(x3,y3),(x4,y4),(x5,y5),(x6,y6),(x7,y7),(x8,y8)]
        pygame.draw.polygon(self.g_screen, darkgrey, poly, 0)

        x1 = x - size/2
        y1 = y - size/5
        x2 = x1 + size/10
        y2 = y1
        x3 = x2
        y3 = y + size/5
        x4 = x1
        y4 = y3
        x1,y1 = self.rotate(x1,y1,x, y, self.angle)
        x2,y2 = self.rotate(x2,y2,x, y, self.angle)
        x3,y3 = self.rotate(x3,y3,x, y, self.angle)
        x4,y4 = self.rotate(x4,y4,x, y, self.angle)
#        self.left_wheel = self.canvas.create_polygon(x1,y1,x2,y2,x3,y3,x4,y4, fill="red")
        poly = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
        pygame.draw.polygon(self.g_screen, red, poly, 0)

        x1 = x + size/2
        y1 = y - size/5
        x2 = x1 - size/10
        y2 = y1
        x3 = x2
        y3 = y + size/5
        x4 = x1
        y4 = y3
        x1,y1 = self.rotate(x1,y1,x, y, self.angle)
        x2,y2 = self.rotate(x2,y2,x, y, self.angle)
        x3,y3 = self.rotate(x3,y3,x, y, self.angle)
        x4,y4 = self.rotate(x4,y4,x, y, self.angle)
#        self.right_wheel = self.canvas.create_polygon(x1,y1,x2,y2,x3,y3,x4,y4, fill="red")
        poly = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
        pygame.draw.polygon(self.g_screen, red, poly, 0)

        x1 = x - (2.5*size)/40
        y1 = y - (12*size)/40
        x2 = x1 + size/20
        y2 = y1
        x3 = x2
        y3 = y - (10*size)/40
        x4 = x1
        y4 = y3
        x1,y1 = self.rotate(x1,y1,x, y, self.angle)
        x2,y2 = self.rotate(x2,y2,x, y, self.angle)
        x3,y3 = self.rotate(x3,y3,x, y, self.angle)
        x4,y4 = self.rotate(x4,y4,x, y, self.angle)
#        self.us_sensor1 = self.canvas.create_polygon(x1,y1,x2,y2,x3,y3,x4,y4, fill="blue")
        poly = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
        pygame.draw.polygon(self.g_screen, blue, poly, 0)

        x1 = x + (0.5*size)/40
        y1 = y - (12*size)/40
        x2 = x1 + size/20
        y2 = y1
        x3 = x2
        y3 = y - (10*size)/40
        x4 = x1
        y4 = y3
        x1,y1 = self.rotate(x1,y1,x, y, self.angle)
        x2,y2 = self.rotate(x2,y2,x, y, self.angle)
        x3,y3 = self.rotate(x3,y3,x, y, self.angle)
        x4,y4 = self.rotate(x4,y4,x, y, self.angle)
#        self.us_sensor2 = self.canvas.create_polygon(x1,y1,x2,y2,x3,y3,x4,y4, fill="blue")
        poly = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
        pygame.draw.polygon(self.g_screen, blue, poly, 0)
        self.do_front_ir_display(x_offset, y_offset)
        self.do_side_ir_display(x_offset, y_offset)
        self.do_us_display(x_offset, y_offset)


    def do_front_ir_display(self, x_offset, y_offset):
        x = (self.x - x_offset) * self.display.zoom
        y = (self.y - y_offset) * self.display.zoom
        size = self.size * self.display.zoom
        max_ir_range = self.max_front_ir_range * self.display.zoom
        x1 = x - (15*size)/40
        y1 = y - (16*size)/40
        x2 = x1 + (5*size)/40
        y2 = y1
        x3 = x2
        y3 = y - (14*size)/40
        x4 = x1
        y4 = y3
        cx = x - (12.5*size)/40
        cy = y - (15*size)/40
        x1,y1 = self.rotate(x1,y1,cx, cy, self.ir_angle_right)
        x1,y1 = self.rotate(x1,y1,x, y, self.angle)
        x2,y2 = self.rotate(x2,y2,cx, cy, self.ir_angle_right)
        x2,y2 = self.rotate(x2,y2,x, y, self.angle)
        x3,y3 = self.rotate(x3,y3,cx, cy, self.ir_angle_right)
        x3,y3 = self.rotate(x3,y3,x, y, self.angle)
        x4,y4 = self.rotate(x4,y4,cx, cy, self.ir_angle_right)
        x4,y4 = self.rotate(x4,y4,x, y, self.angle)
        #self.ir_front_right = self.canvas.create_polygon(x1,y1,x2,y2,x3,y3,x4,y4, fill="blue")
        poly = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
        pygame.draw.polygon(self.g_screen, blue, poly, 0)

        x1 = x + (15*size)/40
        y1 = y - (16*size)/40
        x2 = x1 - (5*size)/40
        y2 = y1
        x3 = x2
        y3 = y - (14*size)/40
        x4 = x1
        y4 = y3
        cx = x + (12.5*size)/40
        cy = y - (15*size)/40
        x1,y1 = self.rotate(x1,y1,cx, cy, self.ir_angle_left)
        x1,y1 = self.rotate(x1,y1,x, y, self.angle)
        x2,y2 = self.rotate(x2,y2,cx, cy, self.ir_angle_left)
        x2,y2 = self.rotate(x2,y2,x, y, self.angle)
        x3,y3 = self.rotate(x3,y3,cx, cy, self.ir_angle_left)
        x3,y3 = self.rotate(x3,y3,x, y, self.angle)
        x4,y4 = self.rotate(x4,y4,cx, cy, self.ir_angle_left)
        x4,y4 = self.rotate(x4,y4,x, y, self.angle)
        #self.ir_front_left = self.canvas.create_polygon(x1,y1,x2,y2,x3,y3,x4,y4, fill="green")
        poly = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
        pygame.draw.polygon(self.g_screen, green, poly, 0)

        if self.ir_beam_front_on:
            x1 = x - (12.5*size)/40
            y1 = y - (16*size)/40
            x2 = x1
            y2 = y1 - max_ir_range
            cx = x - (12.5*size)/40
            cy = y - (15*size)/40
            x1,y1 = self.rotate(x1,y1,cx, cy, self.ir_angle_right)
            x1,y1 = self.rotate(x1,y1,x, y, self.angle)
            x2,y2 = self.rotate(x2,y2,cx, cy, self.ir_angle_right)
            x2,y2 = self.rotate(x2,y2,x, y, self.angle)
            pygame.draw.line(self.g_screen, palered, (x1,y1), (x2,y2), 1)

            s_range = self.sensor_range[1]
            x3,y3 = self.calc_number_posn(s_range, self.max_front_ir_range,x1,y1,x2,y2)
            if s_range >= 0 and s_range < 400:
                surface = self.nums[s_range]
                w,h = surface.get_size()
                pos = (int(x3-w/2), int(y3-h/2))
                self.g_screen.blit(surface, pos)

            x1 = x + (12.5*size)/40
            y1 = y - (16*size)/40
            x2 = x1
            y2 = y1 - max_ir_range
            cx = x + (12.5*size)/40
            cy = y - (15*size)/40
            x1,y1 = self.rotate(x1,y1,cx, cy, self.ir_angle_left)
            x1,y1 = self.rotate(x1,y1,x, y, self.angle)
            x2,y2 = self.rotate(x2,y2,cx, cy, self.ir_angle_left)
            x2,y2 = self.rotate(x2,y2,x, y, self.angle)
            pygame.draw.line(self.g_screen, palered, (x1,y1), (x2,y2), 1)
            s_range = self.sensor_range[0]
            x3,y3 = self.calc_number_posn(s_range, self.max_front_ir_range,x1,y1,x2,y2)
            if s_range >= 0 and s_range < 400:
                surface = self.nums[s_range]
                w,h = surface.get_size()
                pos = (int(x3-w/2), int(y3-h/2))
                self.g_screen.blit(surface, pos)

    def do_side_ir_display(self, x_offset, y_offset):
        x = (self.x - x_offset) * self.display.zoom
        y = (self.y - y_offset) * self.display.zoom
        size = self.size * self.display.zoom
        side_angle = 3.14*90.0/180.0
        max_ir_range = self.max_side_ir_range * self.display.zoom
        x1 = x - (16*size)/40
        y1 = y + (14*size)/40
        x2 = x1 + (2*size)/40
        y2 = y1
        x3 = x2
        y3 = y + (12*size)/40
        x4 = x1
        y4 = y3
        cx = x - (12.5*size)/40
        cy = y + (15*size)/40
        #x1,y1 = self.rotate(x1,y1,cx, cy, -side_angle)
        x1,y1 = self.rotate(x1,y1,x, y, self.angle)
        #x2,y2 = self.rotate(x2,y2,cx, cy, -side_angle)
        x2,y2 = self.rotate(x2,y2,x, y, self.angle)
        #x3,y3 = self.rotate(x3,y3,cx, cy, -side_angle)
        x3,y3 = self.rotate(x3,y3,x, y, self.angle)
        #x4,y4 = self.rotate(x4,y4,cx, cy, -side_angle)
        x4,y4 = self.rotate(x4,y4,x, y, self.angle)
        #self.ir_side_right = self.canvas.create_polygon(x1,y1,x2,y2,x3,y3,x4,y4, fill="blue")
        poly = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
        pygame.draw.polygon(self.g_screen, blue, poly, 0)

        x1 = x + (16*size)/40
        y1 = y + (14*size)/40
        x2 = x1 - (2*size)/40
        y2 = y1
        x3 = x2
        y3 = y + (12*size)/40
        x4 = x1
        y4 = y3
        cx = x + (12.5*size)/40
        cy = y + (15*size)/40
        #x1,y1 = self.rotate(x1,y1,cx, cy, side_angle)
        x1,y1 = self.rotate(x1,y1,x, y, self.angle)
        #x2,y2 = self.rotate(x2,y2,cx, cy, side_angle)
        x2,y2 = self.rotate(x2,y2,x, y, self.angle)
        #x3,y3 = self.rotate(x3,y3,cx, cy, side_angle)
        x3,y3 = self.rotate(x3,y3,x, y, self.angle)
        #x4,y4 = self.rotate(x4,y4,cx, cy, side_angle)
        x4,y4 = self.rotate(x4,y4,x, y, self.angle)
        #self.ir_side_left = self.canvas.create_polygon(x1,y1,x2,y2,x3,y3,x4,y4, fill="green")
        poly = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
        pygame.draw.polygon(self.g_screen, green, poly, 0)

        if self.ir_beam_side_on:
            x1 = x - (16*size)/40
            y1 = y + (13*size)/40
            x2 = x1 - max_ir_range
            y2 = y1 
            #cx = x - (16*size)/40
            #cy = y + (13*size)/40
            #x1,y1 = self.rotate(x1,y1,cx, cy, -side_angle)
            x1,y1 = self.rotate(x1,y1,x, y, self.angle)
            #x2,y2 = self.rotate(x2,y2,cx, cy, -side_angle)
            x2,y2 = self.rotate(x2,y2,x, y, self.angle)
            #self.ir_beam_side_right = self.canvas.create_line(x1,y1,x2,y2, fill="navyblue")
            pygame.draw.line(self.g_screen, palered, (x1,y1), (x2,y2), 1)
            s_range = self.sensor_range[3]
            x3,y3 = self.calc_number_posn(s_range, self.max_side_ir_range,x1,y1,x2,y2)
            if s_range >= 0 and s_range < 400:
                surface = self.nums[s_range]
                w,h = surface.get_size()
                pos = (int(x3-w/2), int(y3-h/2))
                self.g_screen.blit(surface, pos)

            x1 = x + (16*size)/40
            y1 = y + (13*size)/40
            x2 = x1 + max_ir_range
            y2 = y1
            #cx = x + (16*size)/40
            #cy = y + (13*size)/40
            #x1,y1 = self.rotate(x1,y1,cx, cy, side_angle)
            x1,y1 = self.rotate(x1,y1,x, y, self.angle)
            #x2,y2 = self.rotate(x2,y2,cx, cy, side_angle)
            x2,y2 = self.rotate(x2,y2,x, y, self.angle)
            #self.ir_beam_side_left = self.canvas.create_line(x1,y1,x2,y2, fill="darkgreen")
            pygame.draw.line(self.g_screen, palered, (x1,y1), (x2,y2), 1)
            s_range = self.sensor_range[2]
            x3,y3 = self.calc_number_posn(s_range, self.max_side_ir_range,x1,y1,x2,y2)
            if s_range >= 0 and s_range < 400:
                surface = self.nums[s_range]
                w,h = surface.get_size()
                pos = (int(x3-w/2), int(y3-h/2))
                self.g_screen.blit(surface, pos)

    def calc_number_posn(self, s_range, max_range, x1, y1, x2, y2):
        frac = s_range / (0.26*max_range)
        if frac > 0.2:
            x = int(x1*0.8 + x2*0.2)
            y = int(y1*0.8 + y2*0.2)
        else:
            frac = (s_range + 5) / (0.26 * max_range)
            x = int(x1*(1-frac) + x2*frac)
            y = int(y1*(1-frac) + y2*frac)
        return x,y

    def do_us_display(self, x_offset, y_offset):
        x = (self.x - x_offset)* self.display.zoom
        y = (self.y - y_offset)* self.display.zoom
        size = self.size * self.display.zoom
        for beam in self.us_beam:
            self.canvas.delete(beam)
        while self.us_beam != []:
            self.us_beam.pop()
        if self.us_beam_on == False:
            return
        o_x1 = x 
        o_y1 = y - (12*size)/40 
        o_x1,o_y1 = self.rotate(o_x1,o_y1,x, y, self.angle)
        for i in [0.5,1,1.5,2,2.5,3,3.5,4]:
            x2 = o_x1 + size*i
            y2 = o_y1 + size*i
            x1 = o_x1 - size*i
            y1 = o_y1 - size*i
            #arc = self.canvas.create_arc(x1,y1,x2,y2, extent=46, start= 67 - (180 * self.angle / pi), style="arc", outline="grey", width = 3)

            rect = pygame.Rect(x1,y1,2*size*i, 2*size*i)
            #1.17 radians = 67 degrees
            start = 1.17 -  self.angle
            #1.97 radians = 113 degrees
            end = 1.97 - self.angle
            pygame.draw.arc(self.g_screen, grey, rect, start, end, 3)
            #self.us_beam.append(arc)

#     def front_ir_erase(self):
#         self.canvas.delete(self.ir_front_left)
#         self.canvas.delete(self.ir_front_right)
#         if self.ir_beam_front_left != 0:
#             self.canvas.delete(self.ir_beam_front_left)
#             self.canvas.delete(self.ir_beam_front_right)

#     def side_ir_erase(self):
#         self.canvas.delete(self.ir_side_left)
#         self.canvas.delete(self.ir_side_right)
#         if self.ir_beam_side_left != 0:
#             self.canvas.delete(self.ir_beam_side_left)
#             self.canvas.delete(self.ir_beam_side_right)

#     def erase(self):
#         self.canvas.delete(self.base)
#         self.canvas.delete(self.left_wheel)
#         self.canvas.delete(self.right_wheel)
#         self.canvas.delete(self.us_sensor1)
#         self.canvas.delete(self.us_sensor2)
#         self.front_ir_erase()
#         self.side_ir_erase()

    def set_posn(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

    def set_ir_angle(self, ir_angle_left, ir_angle_right):
        #zero angles are at 45 degrees.
        #invert sign so positive angles are clockwize
        self.ir_angle_left = ((pi/4) - ir_angle_left)
        self.ir_angle_right = (-(pi/4) - ir_angle_right)
        #self.front_ir_erase()
        #self.do_front_ir_display()
        #self.do_us_display()

#    def set_ir_dists(self, ir_dists):
#        self.display.set_ir_front_dists(ir_dists[L]*26.0/100, ir_dists[R]*26/100)
#        self.display.set_ir_side_dists(ir_dists[L+SIDE*2]*26/100, ir_dists[R+SIDE*2]*26/100)
        
#    def set_us_dist(self, dist):
#        self.display.set_us_dist(dist)
        
    def set_blob(self, num, x,y):
        x_offset = self.display.x_offset
        y_offset = self.display.y_offset
        x = x - x_offset
        y = y - y_offset
        zoom = self.display.zoom
        if (num <= 5):
            x1 = (x-5)*zoom
            y1 = (y-5)*zoom
            x2 = (x+5)*zoom
            y2 = (y+5)*zoom
#             if self.blob[num] != 0:
#                 self.canvas.delete(self.blob[num])
            if ((num == 0 or num == 1) and self.ir_beam_front_on == False):
#                self.blob[num] == 0 
                return
            if ((num == 4) and self.us_beam_on == False):
#                self.blob[num] == 0 
                return
            if ((num == 2 or num == 3) and self.ir_beam_side_on == False):
#                self.blob[num] == 0 
                return

#            self.blob[num] = self.canvas.create_polygon(x1,y1,x2,y1,x2,y2,x1,y2, fill="red")
            poly = [(x1,y1),(x2,y1),(x2,y2),(x1,y2)]
            pygame.draw.polygon(self.g_screen, red, poly, 0)
        else:
            x1 = (x-2)*zoom
            y1 = (y-2)*zoom
            x2 = (x+2)*zoom
            y2 = (y+2)*zoom
#            self.canvas.delete(self.blob[num])
#            self.blob[num] = self.canvas.create_polygon(x1,y1,x2,y1,x2,y2,x1,y2, fill="pink")
            poly = [(x1,y1),(x2,y1),(x2,y2),(x1,y2)]
            pygame.draw.polygon(self.g_screen, pink, poly, 0)

    def clear_trail(self):
        while len(self.trail) > 0:
            self.trail.pop(0)
            
    def redraw_trail(self):
        for blob in self.trail:
            blob.redraw(self.display)

    def set_trail(self, x, y):
        blob = Blob(x, y, "brown")
        self.trail.append(blob);
        blob.redraw(self.display)

    def clear_points(self):
        while len(self.points) > 0:
            self.points.pop(0)

    def redraw_points(self):
        for blob in self.points:
            blob.redraw(self.display)

    def set_point(self, x, y):
        blob = Blob(x, y, "red")
        self.points.append(blob);
        blob.redraw(self.display)

