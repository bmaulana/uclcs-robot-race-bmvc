from __future__ import division
import sys
PYTHON_MAJOR = sys.version_info[0]
if PYTHON_MAJOR == 2:
    def bytes(a, b): return str(a)

from socket import *
import time
from src.robotmodel import L,R,M

class Controller():
    def __init__(self, port, robot, mode=0):
        self.robot = robot
        self.port = port
        self.sock = socket(AF_INET, SOCK_STREAM)
        self.udpsock = socket(AF_INET, SOCK_DGRAM)
        self.connected = False
        self.udp_connected = False
        self.tcp_connected = False
        if (mode == 0):
            self.sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
            self.sock.bind(tuple(["127.0.0.1", port]))
            self.sock.listen(1)
            self.sock.setblocking(False)
            self.test_for_incoming_tcp()

            self.udpsock.bind(tuple(["127.0.0.1", port]))
            self.udpsock.setblocking(False)
            self.robot.set_display_text(1, "localhost ready 12.5V")
            self.robot.set_display_text(2, "127.0.0.1")
        else:
            res = -1
            while (res != 0):
                time.sleep(1)
                #print("connecting...")
                self.sock = socket(AF_INET, SOCK_STREAM)
                res = self.sock.connect_ex(("127.0.0.1", port))
            #print("res = ", res)
            #print("connected to port", port)
            self.fd = self.sock
            self.fd.setblocking(False)
            self.connected = True
            self.tcp_connected = True

    def send_msg(self, msg):
        if self.udp_connected == True:
            #print(self.remoteaddr)
            self.udpsock.sendto(bytes(msg, 'ascii'), self.remoteaddr)
        else:
            self.fd.sendall(bytes(msg, 'ascii'))

    def close_conn(self):
        self.connected = False
        self.tcp_connected = False
        self.fd.close()
        self.robot.set_speed(L, 0)
        self.robot.set_speed(R, 0)
        self.robot.set_display_text(1, "localhost ready 12.5V")
        self.robot.set_display_text(2, "127.0.0.1")

    #polls the control connection.  returns true if a controller is connected
    def poll(self):
        if (self.tcp_connected == False):
            self.test_for_incoming_tcp()
        
        if self.tcp_connected == False:
            try:
                data, self.remoteaddr = self.udpsock.recvfrom(1024)
                self.udp_connected = True
            except IOError:
                if self.udp_connected == False:
                    return False
        else:
            try:
                data = self.fd.recv(80)
                if (not data):
                    self.close_conn()
                    return False
            except:
                self.data = ""
                return True
        try:
            s = str(data.decode('ascii'))
        except UnicodeDecodeError:
            self.close_conn()
            return False;
        lines = s.split('\n');
        for line in lines:
                #print line
                parts = line.split()
                if len(parts) == 0:
                    return True
                cmd = parts[0]
                if cmd == "M":
                    #motor command
                    if len(parts) < 3 or len(parts) > 4:
                        msg = 'ERR "need 2 params" ' + s
                        self.send_msg(msg)
                        return True
                    try:
                        speed = int(parts[2])
                    except ValueError:
                        msg = 'ERR "not an int" ' + s
                        self.send_msg(msg)
                        return True
                    if speed < -255 or speed > 255:
                        msg = 'ERR "invalid speed" ' + s
                        self.send_msg(msg)
                        return True
                    if parts[1] == 'L':
                        self.robot.set_speed(L, speed)
                        self.send_msg('.\n')
                        return True
                    elif parts[1] == 'R':
                        self.robot.set_speed(R, speed)
                        self.send_msg('.\n')
                        return True
                    elif parts[1] == 'LR':
                        self.robot.set_speed(L, speed)
                        try:
                            speed = int(parts[3])
                        except:
                            msg = 'ERR "not an int" ' + s
                            self.send_msg(msg)
                            return True
                        self.robot.set_speed(R, speed)
                        self.send_msg('.\n')
                        return True
                    else:
                        msg = 'ERR "invalid motor name" ' + s
                        self.send_msg(msg)
                        return True
                if cmd == "T":
                    #Display text on display
                    self.robot.set_display_text(2, line[2:])
                    return True
                if cmd == "I":
                    #Infrared servo position
                    if len(parts) < 3 or len(parts) > 4:
                        msg = 'ERR "need 2 params" ' + s
                        self.send_msg(msg)
                        return True
                    try:
                        angle = int(parts[2])
                    except ValueError:
                        msg = 'ERR "not an int" ' + s
                        self.send_msg(msg)
                        return True
                    if angle < -90 or angle > 90:
                        msg = 'ERR "invalid IR angle" ' + s
                        self.send_msg(msg)
                        return True
                    if parts[1] == 'L':
                        self.robot.set_target_ir_angle(L, angle)
                        self.send_msg('.\n')
                        return True
                    elif parts[1] == 'R':
                        self.robot.set_target_ir_angle(R, angle)
                        self.send_msg('.\n')
                        return True
                    elif parts[1] == 'LR':
                        self.robot.set_target_ir_angle(L, angle)
                        try:
                            angle = int(parts[3])
                        except ValueError:
                            msg = 'ERR "not an int" ' + s
                            self.send_msg(msg)
                            return True
                        self.robot.set_target_ir_angle(R, angle)
                        self.send_msg('.\n')
                        return True
                    else:
                        msg = 'ERR "invalid servo name" ' + s
                        self.send_msg(msg)
                        return True
                if cmd == "S":
                    #read sensor
                    if len(parts) != 2:
                        msg = 'ERR "need 2 params" ' + s
                        self.send_msg(msg)
                        return True
                    sensorname = parts[1]
                    if sensorname == "IFL":
                        #front left IR
                        reading = self.robot.read_ir_front_dist(L)
                        msg = "S IFL " + str(int(reading)) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "IFR":
                        #front right IR
                        reading = self.robot.read_ir_front_dist(R)
                        msg = "S IFR " + str(int(reading)) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "IFLR":
                        #front right IR
                        reading_left = self.robot.read_ir_front_dist(L)
                        reading_right = self.robot.read_ir_front_dist(R)
                        msg = "S IFLR " + str(int(reading_left)) + " " \
                            + str(int(reading_right)) + "\n" 

                        self.send_msg(msg)
                        return True
                    if sensorname == "ISL":
                        #front left IR
                        reading = self.robot.read_ir_side_dist(L)
                        msg = "S ISL " + str(int(reading)) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "ISR":
                        #front right IR
                        reading = self.robot.read_ir_side_dist(R)
                        msg = "S ISR " + str(int(reading)) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "ISLR":
                         #side left and right IR
                        reading_left = self.robot.read_ir_side_dist(L)
                        reading_right = self.robot.read_ir_side_dist(R)
                        msg = "S ISLR " + str(int(reading_left)) + " " \
                            + str(int(reading_right)) + "\n" 

                        self.send_msg(msg)
                        return True
                    if sensorname == "US":
                    #front ultrasound
                        reading = self.robot.get_us_dist()
                        msg = "S US " + str(int(reading)) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "BFL":
                        #front left bump switch
                        reading = self.robot.get_bump(L)
                        msg = "S BFL " + str(int(reading)) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "BFR":
                        #front right bump switch
                        reading = self.robot.get_bump(R)
                        msg = "S BFR " + str(int(reading)) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "BFLR":
                        #both front bump switch
                        reading_left = self.robot.get_bump(L)
                        reading_right = self.robot.get_bump(R)
                        msg = "S BFLR " + str(int(reading_left)) + " " \
                            + str(int(reading_right)) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "IBL":
                        #infrared bottom left (linesensors)
                        readings = self.robot.get_linesensor_readings()
                        msg = "S IBL " + str(int(readings[L])) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "IBC":
                        #infrared bottom centre (linesensors)
                        readings = self.robot.get_linesensor_readings()
                        msg = "S IBC " + str(int(readings[M])) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "IBR":
                        #infrared bottom left (linesensors)
                        readings = self.robot.get_linesensor_readings()
                        msg = "S IBR " + str(int(readings[R])) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "IBLCR":
                        #infrared bottom left (linesensors)
                        readings = self.robot.get_linesensor_readings()
                        #print(readings)
                        msg = "S IBLCR " + str(int(readings[L])) + " " \
                            + str(int(readings[M])) + " " \
                            + str(int(readings[R])) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "MEL":
                        #front left IR
                        reading = self.robot.get_encoder(L)
                        msg = "S MEL " + str(int(reading)) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "MER":
                        #front left IR
                        reading = self.robot.get_encoder(R)
                        msg = "S MER " + str(int(reading)) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "MELR":
                        #front left IR
                        reading_left = self.robot.get_encoder(L)
                        reading_right = self.robot.get_encoder(R)
                        msg = "S MELR " + str(int(reading_left)) \
                            + " " + str(int(reading_right)) + "\n" 
                        self.send_msg(msg)
                        return True
                    if sensorname == "V":
                        #voltage
                        msg = "S V 125\n" 
                        self.send_msg(msg)
                        return True
                    msg = 'ERR "bad sensor name" ' + s
                    self.send_msg(msg)
                    return True
                if cmd == "C":
                    #Configure a paramater
                    if len(parts) < 2:
                        msg = 'ERR "need at least one param" ' + s
                        self.send_msg(msg)
                        return True
                    param = parts[1]
                    if param == "RESET":
                        self.robot.reset()
                        self.robot.set_display_text(1, "ctrl connected")
                        self.robot.set_display_text(2, "127.0.0.1")
                        self.send_msg("W START\n")
                        self.send_msg("W V 125\n")
                        self.send_msg("W RUN 1.00\n")
                        return True
                    if param == "ASYNC":
                        if len(parts) < 3:
                            msg = 'ERR "need at least one interval param" ' + s
                            self.send_msg(msg)
                            return True
                        update_interval = int(parts[2])
                        #print("update interval: ", update_interval)
                        for p in parts[3:]:
                            if self.robot.enable_async(p) == False:
                                msg = 'ERR "bad sensor name" ' + p + "\n"
                                self.send_msg(msg)
                                return True
                        self.send_msg('.\n')
                        self.robot.set_async_update_interval(update_interval)
                        return True
                    if param == "LB":
                        if len(parts) != 3:
                            msg = 'ERR "need two params" ' + s
                            self.send_msg(msg)
                            return True
                        #configure bump switches to latch
                        if parts[2] == "0":
                            self.robot.set_bump_latch(False)
                            self.send_msg('.\n')
                            return True
                        if parts[2] == "1":
                            self.robot.set_bump_latch(True)
                            self.send_msg('.\n')
                            return True
                        msg = 'ERR "bad value" ' + s
                        self.send_msg(msg)
                        return True
                    if param == "RME":
                        if len(parts) != 2:
                            msg = 'ERR "need no params" ' + s
                            self.send_msg(msg)
                            return True
                        self.robot.reset_motor_encoders()
                        self.send_msg('.\n')
                        return True
                    
                    if param == "ASR":
                        if len(parts) != 3:
                            msg = 'ERR "need one param" ' + s
                            self.send_msg(msg)
                            return True
                        self.robot.set_asr(int(parts[2]))
                        self.send_msg('.\n')
                        return True
                    
                    if param == "ORIGIN":
                        if len(parts) != 2:
                            msg = 'ERR "need no params" ' + s
                            self.send_msg(msg)
                            return True
                        self.robot.reset_origin()
                        self.send_msg('.\n')
                        return True

                    if param == "POINT":
                        if len(parts) != 4:
                            msg = 'ERR "need two params" ' + s
                            self.send_msg(msg)
                            return True
                        self.robot.set_point(int(parts[2]), int(parts[3]))
                        self.send_msg('.\n')
                        return True
                    
                    if param == "TRAIL":
                        if len(parts) != 2:
                            msg = 'ERR "need no params" ' + s
                            self.send_msg(msg)
                            return True
                        self.robot.set_trail()
                        self.send_msg('.\n')
                        return True
                    
                    
                    msg = 'ERR "bad param name" ' + s
                    self.send_msg(msg)
                    return True
                    

                else:
                    msg = 'ERR "invalid command" ' + s
                    self.send_msg(msg)
                    return True
                

    def test_for_incoming_tcp(self):
        try:
            self.fd,(hostaddr,port) = self.sock.accept()
            self.fd.setblocking(False)
            #print(hostaddr,port)
            self.tcp_connected = True
            self.robot.set_display_text(1, "ctrl connected")
            self.robot.set_display_text(2, "127.0.0.1")
            self.send_msg("W START\n")
            self.send_msg("W V 125\n")
            self.send_msg("W RUN 1.00\n")
            self.robot.reset_motor_encoders()
        except:
            self.tcp_connected = False

    def async_report(self, msg):
        self.send_msg(msg)
