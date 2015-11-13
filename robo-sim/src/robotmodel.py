from __future__ import division 
from src.baseobject import BaseObj
from src.visualize import *
from src.noisemodel import FrontNoiseModel, SideNoiseModel
from src.constants import L, R, FRONT, SIDE, NO_NOISE, SAMPLED_NOISE,\
    GAUSSIAN_NOISE, ANGULAR_NOISE
from math import *
from random import *
import time

    
class RobotModel(BaseObj):

    def __init__(self, display, size, scale, time):
        BaseObj.__init__(self, BaseObj.ROBOT)
        self.rand = Random()
        self.tick = 0
        self.ir_angle = [0,0] #angle is in radians
        self.target_ir_angle = [0,0] #angle is in radians
        self.target_speed = [0,0]
        self.wheel_speed = [0,0]
        self.wheel_accel = [2,2]
        self.wheel_drag = [0,0]
        self.asr = True #automatic speed regulation
        self.wheel_pos = [0,0]
        self.encoder = [0.0,0.0]
        self.prev_encoder = [0.0,0.0]
        self.real_time = 0
        self.prev_time = 0
        self.last_speed_change = 0
        self.x = 0
        self.y = 0
        self.size = size
        #dist between wheel centres is 23.5cm, robot is 26cm wide
        self.wheelbase = 22.5*self.size/26.0
        self.scale = scale
        self.noise_model = NO_NOISE
        self.noise_value = 0
        self.rand_noise = False
        self.max_ir_range = [0 , 0]
        self.max_ir_range[FRONT] = 100/26 * size
        self.max_ir_range[SIDE] = 40/26 * size
        self.max_us_range = 3 * size
        self.ir_dist = [self.max_ir_range[FRONT], self.max_ir_range[FRONT], 
                        self.max_ir_range[SIDE], self.max_ir_range[SIDE]]
        self.ir_sample_time = time
        self.ir_scan_speed = 20 * 3.14/180  #20 degrees per update
        self.us_dist = self.max_us_range
        self.last_ir_front_reading = 0
        self.last_ir_side_reading = 0
        self.last_us_reading = 0
        self.always_show_front_ir = False
        self.always_show_side_ir = False
        self.always_show_us = False
        self.async_enabled = False
        self.async_update_interval = 100
        self.async_last_update = 0
        self.async_us = False
        self.async_iflr = False
        self.async_islr = False
        self.async_melr = False
        self.angle = 0  #angle is in radians
        self.bump = [False,False]
        self.bump_got = [False,False]
        self.latch_bump = False
        self.line_y = -0.4 * size
        self.line_spacing = 0.05 * size
        self.line_readings = [127,127,127]
        self.last_line_reading = 0
        self.vis = RobotVis(display, 100, 100, 30, self.size,
                            self.max_ir_range[FRONT], self.max_ir_range[SIDE])
        self.reset_origin()
        self.front_noise_model = FrontNoiseModel();
        self.side_noise_model = SideNoiseModel();

    def reset(self):
        self.reset_motor_encoders()
        self.clear_points()
        self.clear_trail()
        self.disable_async()

    def set_controller(self, controller):
        self.controller = controller

    def set_noise_model(self, noise_model, noise_value):
        self.noise_model = noise_model
        self.noise_value = noise_value

    def set_rand_noise(self, rand_noise):
        self.rand_noise = rand_noise

    def set_asr(self, val):
        if val == 1:
            self.asr = True
        else:
            self.asr = False

    def reset_origin(self):
        self.origin_x = self.x
        self.origin_y = self.y
        self.origin_angle = self.angle

    def clear_points(self):
        self.vis.clear_points()

    def clear_trail(self):
        self.vis.clear_trail()

    def set_point(self, x, y):
        x = x * 100.0/26.0
        y = y * -100.0/26.0
        d = sqrt(x*x + y*y)
        a2 = atan2(y, x)
        nx = self.origin_x + d * cos (self.origin_angle + a2)
        ny = self.origin_y + d * sin (self.origin_angle + a2)
        self.vis.set_point(nx, ny)

    def set_trail(self):
        self.vis.set_trail(self.x, self.y)

    def set_display_text(self, line, text):
        self.vis.set_display_text(line, text)

    def redraw(self):
        self.vis.draw()
        return

    def show_front_ir(self, value):
        self.always_show_front_ir = value

    def show_side_ir(self, value):
        self.always_show_side_ir = value

    def show_us(self, value):
        self.always_show_us = value

    def set_target_ir_angle(self, side, angle):
        self.target_ir_angle[side] = angle * (pi/180)

    def set_ir_angle(self, side, angle):
        self.ir_angle[side] = angle * (pi/180)
        self.target_ir_angle[side] = self.ir_angle[side]  #manual control overrides target
        self.vis.set_ir_angle(self.ir_angle[L], self.ir_angle[R])

    def set_ir_angles(self, ir_angle_left, ir_angle_right):
        self.set_ir_angle(L, ir_angle_left)
        self.set_ir_angle(R, ir_angle_left)

    def set_ir_dist(self, sensor, side, dist, time):
        self.ir_sample_time = time
        if self.noise_model == SAMPLED_NOISE:
            if sensor == FRONT:
                #print "scale: ", self.scale, " dist: ", dist
                randdist = self.front_noise_model.add_noise(dist * self.scale) / self.scale
            if sensor == SIDE:
                randdist = self.side_noise_model.add_noise(dist * self.scale) / self.scale
        elif self.noise_model == GAUSSIAN_NOISE:
            randdist = self.rand.gauss(dist, self.noise_value*sqrt(dist)/10)
        elif self.noise_model == ANGULAR_NOISE:
            if sensor == FRONT:
                sense_value = self.dist_to_gp2d12(dist)
                sq = self.noise_value*sqrt(sense_value)/10
                randvalue = self.rand.gauss(sense_value + sq, sq)
                randdist = self.gp2d12_to_dist(randvalue)
            elif sensor == SIDE:
                sense_value = self.dist_to_gp2d120(dist)
                sq = sqrt(sense_value)/2
                randvalue = self.rand.gauss(sense_value + sq, sq)
                randdist = self.gp2d120_to_dist(randvalue)
        else:
            randdist = dist
        sensornum = self.sensor_num(sensor,side)

        if self.rand_noise:
            #randomly report an under-read every 10 seconds on average
            r = self.rand.randint(0,300)
            if r == 1:
                randdist /= 2

        self.ir_dist[sensornum] = randdist
        self.vis.set_sensor_range(sensornum, int(0.26*randdist))
        return randdist

    def get_ir_front_dist(self, side):
        return self.dist_to_gp2d12(self.ir_dist[self.sensor_num(FRONT,side)] * self.scale)

    # call read_ir_front_dist when the user actively reads, otherwise
    # call get_ir_front_dist to obtain the value
    def read_ir_front_dist(self, side):
        self.last_ir_front_reading = self.tick
        return self.dist_to_gp2d12(self.ir_dist[self.sensor_num(FRONT,side)] * self.scale)

    def get_ir_sample_time(self):
        return self.ir_sample_time

    def dist_to_gp2d12(self, dist):
        #dist is in cm
        if dist < 10:
            value = 678 * dist/10.0
        else:
            value = 6787.0/(dist+4) + 3
        return value

    def gp2d12_to_dist(self, ir):
        dist = (6787 / (ir - 3)) - 4
        return dist

    def dist_to_gp2d120(self, dist):
        #dist is in cm
        if dist < 3:
            volts = 3 * dist/3
        else:
            volts = 13 / (dist  + 0.42)
        value = 1024 * volts / 5.0
        return value

    def gp2d120_to_dist(self, ir):
        dist = (2914 / (ir + 5)) - 1;
        return dist;

    def get_ir_side_dist(self, side):
        return self.dist_to_gp2d120(self.ir_dist[self.sensor_num(SIDE,side)] * self.scale)

    # call read_ir_side_dist when the user actively reads, otherwise
    # call get_ir_side_dist to obtain the value
    def read_ir_side_dist(self, side):
        self.last_ir_side_reading = self.tick
        return self.dist_to_gp2d120(self.ir_dist[self.sensor_num(SIDE,side)] * self.scale)

    #for calculating positions of sensor readings, we need to know
    #where the sensor beam actually is.
    def get_ir_position(self, sensor, side):
        if sensor == FRONT:
            #first assume the robot and sensor are both pointing straight up
            if side == L:
                x1 = self.x + (12.5*self.size)/40
                cx = self.x + (12.5*self.size)/40
                ir_angle = (pi/4) - self.ir_angle[side]
            elif side == R:
                x1 = self.x - (12.5*self.size)/40
                cx = self.x - (12.5*self.size)/40
                ir_angle = (-pi/4) - self.ir_angle[side]
            y1 = self.y - (16*self.size)/40
            x2 = x1
            y2 = y1 - self.max_ir_range[FRONT]
            cy = self.y - (15*self.size)/40

            #now rotate the beam around the sensor to account for
            #sensor angle, then rotate the beam around the robot
            #center to account for robot angle.
            x1,y1 = self.rotate(x1,y1,cx, cy, ir_angle)
            x1,y1 = self.rotate(x1,y1,self.x, self.y, self.angle)
            x2,y2 = self.rotate(x2,y2,cx, cy, ir_angle)
            x2,y2 = self.rotate(x2,y2,self.x, self.y, self.angle)
            return x1,y1,x2,y2,self.max_ir_range[FRONT]
        elif sensor == SIDE:
            if side == L:
                x1 = self.x + (16*self.size)/40
                x2 = x1 + self.max_ir_range[SIDE]
            elif side == R:
                x1 = self.x - (16*self.size)/40
                x2 = x1 - self.max_ir_range[SIDE]
            y1 = self.y + (13*self.size)/40
            y2 = y1 
            x1,y1 = self.rotate(x1,y1,self.x, self.y, self.angle)
            x2,y2 = self.rotate(x2,y2,self.x, self.y, self.angle)

            #x1,y1 are the robot end of the sensor beam
            #x2,y2 are the far end of the sensor beam at maximum range
            return x1,y1,x2,y2,self.max_ir_range[SIDE]

    def get_us_triangle(self):
        x1 = self.x 
        y1 = self.y - (12*self.size)/40 
        x2 = self.x + self.size * 1.45
        y2 = self.y - self.size * 4
        x3 = self.x - self.size * 1.45
        y3 = self.y - self.size * 4
        x1,y1 = self.rotate(x1,y1,self.x, self.y, self.angle)
        x2,y2 = self.rotate(x2,y2,self.x, self.y, self.angle)
        x3,y3 = self.rotate(x3,y3,self.x, self.y, self.angle)

        return x1,y1,x2,y2,x3,y3,2*self.max_ir_range[FRONT]

    def get_angle_from_us(self, x, y):
        #returns the angle of a point from the position of the US
        #sensor relative to the direction of the robot

        #calculate the current location of the US sensor
        us_x = self.x 
        us_y = self.y - (12*self.size)/40 
        us_x,us_y = self.rotate(us_x,us_y,self.x, self.y, self.angle)

        #calculate location of point relative to US sensor
        his_x = x - us_x
        his_y = y - us_y

        #rotate the world so robot faces up
        his_x,his_y = self.rotate_about_origin(his_x, his_y, -self.angle)

        angle = atan2(his_x,-his_y)
            
        return angle

    def set_us_dist(self, dist):
        if dist > 1000:
            print(dist)
        self.us_dist = dist

    def get_us_dist(self):
        self.last_us_reading = self.tick
        return self.us_dist * self.scale

    # set_bump_latch enables latching bump readings.  Sometimes a
    # bounce might cause a reading of a bump switch to be missed.
    # Setting latch means a bump reading will be maintained until the
    # next reading of that sensor.
    def set_bump_latch(self, mode):
        self.latch_bump = mode

    def set_bump(self, left_bumped, right_bumped):
        if left_bumped:
            self.bump[L] = True
            self.bump_got[L] = False
        else:
            self.bump[L] = False

        if right_bumped:
            self.bump[R] = True
            self.bump_got[R] = False
        else:
            self.bump[R] = False

    def get_bump(self, side):
        self.bump_got[side] = True
        if self.bump[side]:
            return "1"
        else:
            return "0"

    def get_perimeter(self):
        #df is half the width of the front straight side                      
        df = 0.3*self.size
        #dr is half the width of the robot                                    
        dr = 0.5*self.size

        x1,y1 = self.rotate(self.x + dr, self.y - df, 
                            self.x, self.y, self.angle)
        x2,y2 = self.rotate(self.x + df, self.y - dr, 
                            self.x, self.y, self.angle)
        x3,y3 = self.rotate(self.x - df, self.y - dr, 
                            self.x, self.y, self.angle)
        x4,y4 = self.rotate(self.x - dr, self.y - df, 
                            self.x, self.y, self.angle)
        x5,y5 = self.rotate(self.x - dr, self.y + df, 
                            self.x, self.y, self.angle)
        x6,y6 = self.rotate(self.x - df, self.y + dr, 
                            self.x, self.y, self.angle)
        x7,y7 = self.rotate(self.x + df, self.y + dr, 
                            self.x, self.y, self.angle)
        x8,y8 = self.rotate(self.x + dr, self.y + df, 
                            self.x, self.y, self.angle)
        return x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7,x8,y8

    def get_linesensor_positions(self):
        xm = self.x
        ym = self.y + self.line_y
        xl = xm + self.line_spacing
        yl = ym
        xr = xm - self.line_spacing
        yr = ym
        xl,yl = self.rotate(xl, yl, self.x, self.y, self.angle)
        xr,yr = self.rotate(xr, yr, self.x, self.y, self.angle)
        xm,ym = self.rotate(xm, ym, self.x, self.y, self.angle)
        return [xl,xr,xm],[yl,yr,ym]

    def set_linesensor_readings(self, values):
        self.line_readings = values

    def get_linesensor_readings(self):
        self.last_line_reading = self.tick        
        return self.line_readings

    def init_robot_posn(self, x, y, angle):
        self.angle = angle * (pi / 180)
        self.start_angle = self.angle
        self.x = x
        self.start_x = x
        self.y = y
        self.start_y = y
        self.vis.set_posn(x, y, angle);

    def reset_posn(self):
        self.angle = self.start_angle
        self.x = self.start_x
        self.y = self.start_y
        self.target_speed = [0,0]
        self.wheel_speed = [0,0]
        self.vis.set_posn(self.x, self.y, self.angle);
        self.clear_trail()
        self.clear_points()

    def set_speed(self, side, target_speed):
        self.target_speed[side] = target_speed
        self.last_speed_change = self.real_time

    def fwd(self, target_speed):
        self.set_speed(L, target_speed)
        self.set_speed(R, target_speed)

    def stop(self):
        self.fwd(0)

    def crash(self, corner):
        if corner == -1:
            self.set_bump(False, False)
            return
        self.un_update_position()
        #should handle collision better than this!
        if corner == L:
            #front left corner
            self.set_bump(True, False)
            if self.target_speed[L] > 0 and self.wheel_speed[L] > 0:
                #bounce
                self.wheel_speed[L] = -1 *  self.wheel_accel[L]
            else:
                self.wheel_speed[L] = 0
        elif corner == R:
            #front right corner
            self.set_bump(False, True)
            if self.target_speed[R] > 0 and self.wheel_speed[R] > 0:
                self.wheel_speed[R] = -0.5 * (self.wheel_speed[R] + self.wheel_accel[R])
            else:
                self.wheel_speed[R] = 0
        else:
            #collision is at rear
            for side in L,R:
                self.wheel_speed[side] = -self.wheel_speed[side]
                self.target_speed[side] = 0

    def turn(self, target_speed):
        self.set_speed(L, target_speed)
        self.set_speed(R, 0-target_speed)
        

    def update_wheelspeed(self):
        #really simple model for now
        for side in L,R:
            effective_target_speed = self.target_speed[side]

            if self.asr == False:
                if  effective_target_speed> 0:
                    effective_target_speed -= self.wheel_drag[side]
                    if effective_target_speed < 0:
                        effective_target_speed = 0
                else:
                    effective_target_speed += self.wheel_drag[side]
                    if effective_target_speed > 0:
                        effective_target_speed = 0
            if self.wheel_speed[side] < effective_target_speed:
                self.wheel_speed[side] += self.wheel_accel[side]
                #did we overshoot?
                if self.wheel_speed[side] > effective_target_speed:
                    self.wheel_speed[side] = effective_target_speed
            elif (self.wheel_speed[side] > effective_target_speed):
                self.wheel_speed[side] -= self.wheel_accel[side]
                #did we overshoot?
                if self.wheel_speed[side] < effective_target_speed:
                    self.wheel_speed[side] = effective_target_speed

    def update_servos(self):
        #really simple model for now
        changed = False
        for side in L,R:
            if (self.target_ir_angle[side] < self.ir_angle[side]):
                changed = True
                self.ir_angle[side] -= self.ir_scan_speed
                if self.target_ir_angle[side] > self.ir_angle[side]:
                    #don't overshoot
                    self.ir_angle[side] = self.target_ir_angle[side] 
            elif self.target_ir_angle[side] > self.ir_angle[side]:
                changed = True
                self.ir_angle[side] += self.ir_scan_speed
                if self.target_ir_angle[side] < self.ir_angle[side]:
                    #don't overshoot
                    self.ir_angle[side] = self.target_ir_angle[side] 

        if changed:
            self.vis.set_ir_angle(self.ir_angle[L], self.ir_angle[R])


    def un_update_position(self):
        self.x = self.prev_x
        self.y = self.prev_y
        self.angle = self.prev_angle

    def update_position(self):
        self.update_wheelspeed()
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_angle = self.angle
        speed_gain = 55

        #dist moved by left wheel
        dl = self.wheel_speed[L] / speed_gain
        self.encoder[L] += dl

        #dist moved by right wheel
        dr = self.wheel_speed[R] / speed_gain
        self.encoder[R] += dr
        angle_change = (dl - dr) / self.wheelbase
        if (dl == dr):
            #special case to avoid divide by zero in general case
            #dx = 0
            #dy = dl
            dh = dl
        else:
            r = dl/angle_change
            dh = (2*r - self.wheelbase) * sin(angle_change/2)
            #dx = dh * sin(angle_change)
            #dy = dh * cos(angle_change)

        deltax = dh * sin(self.angle + angle_change/2)
        deltay = 0 - dh * cos(self.angle + angle_change/2)
        self.angle = self.angle + angle_change
        self.x = self.x + deltax
        self.y = self.y + deltay
        self.vis.set_posn(self.x, self.y, self.angle);

        #update wheel positions (used for drag calculations)
        radius = 47 #because robot is 100 units wide
        w_x = radius * cos(self.angle)
        w_y = radius * sin(self.angle)
        self.wheel_pos[L] = (self.x - w_x, self.y - w_y)
        self.wheel_pos[R] = (self.x + w_x, self.y + w_y)

        return self.x,self.y

    def get_wheel_pos(self):
        return self.wheel_pos[L], self.wheel_pos[R]

    def set_drag(self, left_drag, right_drag):
        self.wheel_drag[L] = left_drag
        self.wheel_drag[R] = right_drag

    def get_drag(self):
        return self.wheel_drag[L], self.wheel_drag[R]

    def get_encoder(self, side):
        v = self.encoder[side]*220.0*1.14/80.0
        self.prev_encoder[side] = v
        return self.encoder[side]*220.0*1.14/80.0

    def reset_motor_encoders(self):
        self.encoder[L] = 0.0
        self.encoder[R] = 0.0

    def sensor_num(self, sensor, side):
        return sensor*2 + side

    def set_blob(self,num,x,y):
        self.vis.set_blob(num,x,y)

    def enable_async(self, sensor):
        self.async_enabled = True
        if sensor == "US":
            self.async_us = True
        elif sensor == "IFLR":
            self.async_iflr = True
        elif sensor == "ISLR":
            self.async_islr = True
        elif sensor == "MELR":
            self.async_melr = True
        else:
            self.async_enabled = False
            return False
        return True

    def disable_async(self):
        self.async_enabled = False

    def set_async_update_interval(self, update_interval):
        self.async_update_interval = update_interval/1000.0

    def self_update(self, new_time, real_time):
        self.real_time = real_time
        self.tick += 1

        #if it's two seconds since the last motor command, shut down
        if self.real_time - self.last_speed_change > 2:
            self.target_speed = [0,0]
            self.last_speed_change = self.real_time

        self.update_servos()
        x_offset, y_offset = self.update_position()
        self.prev_time = new_time
        ir_front_visible = True
        if (self.tick - self.last_ir_front_reading) > 10 and self.always_show_front_ir == False:
            ir_front_visible = False
        ir_side_visible = True
        if (self.tick - self.last_ir_side_reading) > 10 and self.always_show_side_ir == False:
            ir_side_visible = False
        us_visible = True
        if (self.tick - self.last_us_reading) > 10 and self.always_show_us == False:
            us_visible = False
        line_visible = True
        if (self.tick - self.last_line_reading) > 10:
            line_visible = False
        self.vis.set_sensor_visibility(ir_front_visible, ir_side_visible, 
                                       us_visible, line_visible)
        #self.vis.set_ir_dists(self.ir_dist)
        #self.vis.set_us_dist(self.us_dist*self.scale)
        if self.async_enabled and \
                (real_time - self.async_last_update) \
                >= self.async_update_interval:
            self.async_last_update = real_time
            s = "S " + str(self.get_bump(L)) + " " + str(self.get_bump(R))
            if self.async_us:
                s = s + " " + str(int(self.get_us_dist()))
            if self.async_iflr:
                s = s + " " + str(int(self.read_ir_front_dist(L))) + " " + \
                    str(int(self.read_ir_front_dist(R)))
            if self.async_islr:
                s = s + " " + str(int(self.read_ir_side_dist(L))) + " " + \
                    str(int(self.read_ir_side_dist(R)))
            if self.async_melr:
                s = s + " " + str(int(self.get_encoder(L))) + " " + \
                    str(int(self.get_encoder(R)))
            s = s + "\n"
            self.controller.async_report(s)
        return x_offset, y_offset
