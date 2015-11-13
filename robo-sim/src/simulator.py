from __future__ import division
import sys
PYTHON_MAJOR = sys.version_info[0]
# if PYTHON_MAJOR == 2:
#     from Tkinter import *
# else:
#     from tkinter import *

#from tkinter import *
import time
import os
from src.rsim_display import Display
#from visualize import *

from src.wallmodel import WallModel
from src.robotmodel import RobotModel, L, R
from src.worldmodel import WorldModel
from src.control import Controller




class Simulator():
    def __init__(self, port):
        self.zoom = 1
        self.disp = Display(self.zoom)

        #robot displays by default at 100 pixels square.
        #read robot is approx 26cm square
        #multiply default scaled distances by scale to get cm

        #internally the model works in default scaled pixels.  But it returns
        #cm on the UI and TCP interfaces.
        self.scale = 26.0/100
        timenow = time.time()
        self.robot = RobotModel(self.disp, 100, self.scale, timenow)
        self.world = WorldModel(self.disp, self.robot, timenow)
        self.world.add_default_walls()
        if (port != 0):
            self.controller = Controller(port,self.robot, 1)
        else:
            self.controller = Controller(55443,self.robot, 0)
        self.robot.set_controller(self.controller)
        self.robot.init_robot_posn(500,350,0)
        self.robot.set_ir_angles(0, 0)
        self.robot.stop()
        self.active = True

    def poll_controller(self):
        cont_active = self.controller.poll()
        if cont_active:
            self.disp.set_active();
            
    def process_gui(self):
        self.active = self.disp.process_gui_events()


    def run(self):
        #local variables are fastest and this is the inner loop
        world = self.world
        robot = self.robot
        realtime = time.time()
        starttime = realtime
        cur_time = 0
        iter_time = 0
        sleep_time = 0.005
        try:
            while True:
                #five phases in this loop to allow more accurate position
                #updates than display updates

                #phase 1
                #first update after pygame's sleep
                realtime = time.time()
                if self.active:
                    self.robot.update_position()
                    self.world.check_wheel_drag()
                self.poll_controller()

                #phase 2
                #process the gui
                self.process_gui()
                cur_time = cur_time + 1
                world.set_time(realtime)
                x_offset, y_offset = robot.self_update(cur_time, realtime)
                if self.active:
                    self.world.check_wheel_drag()
                zoom = self.disp.zoom
                x_offset = x_offset - (500/zoom)
                y_offset = y_offset - (350/zoom)
                self.poll_controller()

                #phase 3
                #update sensors, update display
                if self.active:
                    world.sample_all_sensors()
                self.disp.update(x_offset, y_offset)
                if self.active:
                    self.robot.update_position()
                    self.world.check_wheel_drag()
                self.poll_controller()

                #phase 4
                #check collisions
                world.check_collision()
                if self.active:
                    self.robot.update_position()
                    self.world.check_wheel_drag()
                self.poll_controller()

                #phase 5
                #draw the screen
                world.redraw()
                if self.active:
                    self.robot.update_position()
                    self.world.check_wheel_drag()
                self.poll_controller()
                #may sleep in flip
                self.disp.flip()
        except KeyboardInterrupt:
            sys.exit()
                                


