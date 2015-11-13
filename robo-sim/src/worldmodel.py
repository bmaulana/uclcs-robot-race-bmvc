from math import *
from src.baseobject import BaseObj
from src.constants import L,M,R,FRONT,SIDE
from src.robotmodel import RobotModel
from src.wallmodel import WallModel
from src.conemodel import ConeModel
from src.carpetmodel import CarpetModel

class WorldModel():
    def __init__(self, display, robot, time):
        self.display = display
        self.robot = robot
        self.objects = []
        self.time = time
        display.register_world(self)

    def set_time(self, time):
        self.time = time

    def erase_scenery(self):
        #for object in self.objects:
        #    object.erase()
        del self.objects
        self.objects = []

    def add_default_walls(self):
        wall = WallModel(self.display, 0, 0, 2000,0)
        self.add_object(wall)
        wall = WallModel(self.display, 2000, 0, 2000,2000)
        self.add_object(wall)
        wall = WallModel(self.display, 2000, 2000, 0, 2000)
        self.add_object(wall)
        wall = WallModel(self.display, 0, 2000, 0,0)
        self.add_object(wall)

    #use only when loading new configs, not during driving
    def set_robot_position(self, x, y, angle):
        self.robot.init_robot_posn(x, y, angle)

    def add_object(self, object):
        self.objects.append(object)

    def typed_objects(self, type):
        objects = []
        for object in self.objects:
            if object.type == type:
                objects.append(object)
        return objects

    def sample_all_sensors(self):
        self.sample_ir()
        self.sample_us()
        self.sample_linesensors()


    def sample_ir(self):
        sampled_time = self.robot.get_ir_sample_time()
        if (self.time - sampled_time) < 0.040:
            #the sensor updated less than 40ms ago, so don't update it
            #again - the real sensors only sample every 40ms
            return

        for sensor in FRONT,SIDE:
            for side in L,R:
                (x1,y1,x2,y2,maxrange) = self.robot.get_ir_position(sensor,side)
                #x1,y1 is the beam position at the robot end.
                #x2,y2 is the beam position at maxrange

                mindist = 1000000
                objects = self.typed_objects(BaseObj.WALL) \
                    + self.typed_objects(BaseObj.CONE)
                min_object = objects[0]
                for object in objects:

                    #  does the object intersect the beam?
                    dist,x,y,consts = object.intersects(x1,y1,x2,y2)
                    #  x,y are the intersect position.
                    #  consts are the line constants of the beam if it intersects

                    if (dist >= 0):
                        #it intersects
                        if dist < mindist:
                            #it's the closest object seen so far
                            mindist = dist
                            minx = x
                            miny = y
                            minconsts = consts
                            min_object = object
                    else:
                        #it does not intersect
                        if mindist == 1000000:
                            minx = x2
                            miny = y2
                            mindist = maxrange
                            minconsts = (0,0,0)
                    
                mindist = self.robot.set_ir_dist(sensor, side, mindist, self.time)
                minx,miny = min_object.add_noise(x1,y1,x2,y2,mindist,minconsts)
                self.robot.set_blob(self.robot.sensor_num(sensor, side), minx,miny)
                
    def sample_linesensors(self):
        mindist = [1000000,1000000,1000000]
        x,y = self.robot.get_linesensor_positions()
        values = []
        for sensor in L,R,M:
            self.robot.set_blob(6+sensor, x[sensor],y[sensor])
            for object in self.typed_objects(BaseObj.LINE):
                dist,dummy1,dummy2 = object.closest(x[sensor],y[sensor])
                if dist < mindist[sensor]:
                    mindist[sensor] = dist
            if mindist[sensor] < 3:
                values.append(10)
            elif mindist[sensor] < 8:
                values.append(250)
            else:
                values.append(127)
        self.robot.set_linesensor_readings(values)
                
    def sample_us(self):
        (x1,y1,x2,y2,x3,y3,maxrange) = self.robot.get_us_triangle()
        mindist = 1000000
        
        objects = self.typed_objects(BaseObj.WALL) + self.typed_objects(BaseObj.CONE)
        for object in objects:
            dist,x,y = object.closest(x1,y1)
            if (dist >= 0):
                if dist < mindist:
                    angle = self.robot.get_angle_from_us(x,y)
                    if (angle < 0.45 and angle > -0.45):
                        mindist = dist
                        minx = x
                        miny = y
                        minobj = object
        if mindist > maxrange:
            mindist = 1000000
        
        for object in objects:
            dist,x,y,consts = object.intersects(x1,y1,x2,y2)
            if (dist >= 0):
                if dist < mindist:
                    mindist = dist
                    minx = x
                    miny = y
        if mindist > maxrange:
            mindist = 1000000

        for object in objects:
            dist,x,y,consts = object.intersects(x1,y1,x3,y3)
            if (dist >= 0):
                if dist < mindist:
                    mindist = dist
                    minx = x
                    miny = y
        if mindist > maxrange:
            mindist = 1000000
                    
        if mindist < 1000000:
            self.robot.set_blob(4, minx, miny)
        else:
            self.robot.set_blob(4, x1, y1)
        if mindist == 1000000:
            mindist = maxrange
        self.robot.set_us_dist(mindist)

    def report_crash(self, bumper, x, y):
        if bumper == 0 or  bumper == 1:
            self.robot.crash(0)
            self.robot.set_blob(5, x,y)
        elif bumper == 2 or bumper == 3:
            self.robot.crash(1)
            self.robot.set_blob(5, x,y)
        elif bumper >= 4:
            self.robot.crash(2)
            self.robot.set_blob(5, x,y)


    def check_collision(self):
        x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7,x8,y8 = \
            self.robot.get_perimeter()

        xc = (x1+x5)/2;
        yc = (y1+y5)/2;
        # update the display of the robot position, so it can be
        # centered if desired.
        self.display.robot_position(xc,yc);

        self.robot.set_blob(5, x6,y6)
        objects = self.typed_objects(BaseObj.WALL) + self.typed_objects(BaseObj.CONE)
        for object in objects:
            for bumper in [0,1,2,3]:
                if bumper == 0:
                    xt1 = x1; yt1 = y1; xt2 = x2; yt2 = y2
                elif bumper == 1:
                    xt1 = x2; yt1 = y2; xt2 = (x2+x3)/2; yt2 = (y2+y3)/2
                elif bumper == 2:
                    xt1 = (x2+x3)/2; yt1 = (y2+y3)/2; xt2 = x3; yt2 = y3
                elif bumper == 3:
                    xt1 = x3; yt1 = y3; xt2 = x4; yt2 = y4
                dist,x,y,consts = object.intersects(xt1,yt1,xt2,yt2)
                if (dist == -2): 
                    #lines don't intersect but BB does - need to also
                    #check closeness so we don't jump a parallel line.
                    dist,x,y = object.closest(xt1,yt1)
                    if (dist <= 2):
                        self.report_crash(bumper, x, y)
                        return
                    dist,x,y = object.closest(x2,y2)
                    if (dist <= 2):
                        self.report_crash(bumper, x, y)
                        return
                elif (dist >= 0):
                    #we've got a collision
                    self.report_crash(bumper, x, y)
                    return

            for edge in [4,5,6,7,8]:
                if edge == 4:
                    xt1,yt1,xt2,yt2 = x4,y4,x5,y5
                elif edge == 5:
                    xt1,yt1,xt2,yt2 = x5,y5,x6,y6
                elif edge == 6:
                    xt1,yt1,xt2,yt2 = x6,y6,x7,y7
                elif edge == 7:
                    xt1,yt1,xt2,yt2 = x7,y7,x8,y8
                elif edge == 8:
                    xt1,yt1,xt2,yt2 = x8,y8,x1,y1
                dist,x,y,consts = object.intersects(xt1,yt1,xt2,yt2)
#                 if (dist == -2): 
#                     #lines don't intersect but BB does - need to also
#                     #check closeness so we don't jump a parallel line.
#                     dist,x,y = object.closest((xt1+xt2)/2,(yt1+yt2)/2)
#                     if (dist <= 2):
#                         print("mid rear closest", dist, self.robot.size)
#                         self.robot.crash(5)
#                         self.robot.set_blob(5, (xt1+xt2)/2,(yt1+yt2)/2)
#                         return
#                     dist,x,y = object.closest(xt1,yt1)
#                     if (dist <= 2):
#                         print("right rear closest", dist, self.robot.size)
#                         self.robot.crash(4)
#                         self.robot.set_blob(5, xt1,yt1)
#                         return
#                     dist,x,y = object.closest(xt2,yt2)
#                     if (dist <= 2):
#                         print("left rear closest", dist, self.robot.size)
#                         self.robot.crash(3)
#                         self.robot.set_blob(5, xt2,yt2)
#                         return
                if (dist >= 0):
                    #we've got a collision
                    self.report_crash(edge, x, y)
                    return

        self.robot.crash(-1)
        #self.robot.set_blob(5, x,y)

    def check_wheel_drag(self):
        left_wheel, right_wheel = self.robot.get_wheel_pos()
        left_drag, right_drag = self.robot.get_drag()
        objects = self.typed_objects(BaseObj.CARPET)
        max_left_drag = 0
        max_right_drag = 0
        count = 0
        for object in objects:
            count+=1
            drag = object.get_drag(left_wheel, left_drag)
            if drag > max_left_drag:
                max_left_drag = drag
            drag = object.get_drag(right_wheel, right_drag)
            if drag > max_right_drag:
                max_right_drag = drag
        self.robot.set_drag(max_left_drag, max_right_drag)

    def redraw(self):
        for object in self.objects:
            object.redraw()
        self.robot.redraw()
