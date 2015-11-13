from __future__ import division 
import sys
PYTHON_MAJOR = sys.version_info[0]

# if PYTHON_MAJOR == 2:
#     import tkMessageBox
# else:
#     import tkinter.messagebox
import shlex
from src.wallmodel import WallModel
from src.linemodel import LineModel
from src.conemodel import ConeModel
from src.targetmodel import TargetModel
from src.carpetmodel import CarpetModel

class LoadError(Exception):
    """Exception raised for errors in the input.

    Attributes:
        msg  -- explanation of the error
    """

    def __init__(self, msg):
        self.msg = msg

class SceneryParser():
    def __init__(self, display, world):
        self.display = display
        self.world = world

    def openfile(self, filename):
        try:
            f = open(filename, "r")
        except:
            s = "File open failed"
            raise(LoadError(s))

        self.world.erase_scenery()
        self.world.add_default_walls()        
        linenum = 0
        for line in f:
            linenum += 1
            try:
                lexer = shlex.split(line)
            except ValueError as e:
                s = "Syntax error in line " + str(linenum) + " of scenery file: " + str(e)
                raise(LoadError(s))

            if len(lexer) == 0:
                continue
#            for token in lexer:
#                print(repr(token))
#            print("\n")
            if lexer[0] == "wall":
                if len(lexer) != 5:
                    s = "Syntax error line " + str(linenum) + ", wall command takes 4 arguments"
                    raise(LoadError(s))
                wall = WallModel(self.display, int(lexer[1]), int(lexer[2]), 
                                 int(lexer[3]), int(lexer[4]))
                self.world.add_object(wall)
            if lexer[0] == "cone":
                if len(lexer) != 4:
                    s = "Syntax error line " + str(linenum) + ", cone command takes 3 arguments"
                    raise(LoadError(s))
                cone = ConeModel(self.display, int(lexer[1]), int(lexer[2]), 
                                 int(lexer[3]))
                self.world.add_object(cone)
            if lexer[0] == "target":
                if len(lexer) != 5:
                    s = "Syntax error line " + str(linenum) + ", target command takes 4 arguments"
                    raise(LoadError(s))
                target = TargetModel(self.display, int(lexer[1]), int(lexer[2]), 
                                 int(lexer[3]), lexer[4])
                self.world.add_object(target)
            if lexer[0] == "line":
                if len(lexer) != 5:
                    s = "Syntax error line " + str(linenum) + ", line command takes 4 arguments"
                    raise(LoadError(s))
                line = LineModel(self.display, int(lexer[1]), int(lexer[2]), 
                                 int(lexer[3]), int(lexer[4]))
                self.world.add_object(line)
            if lexer[0] == "carpet":
                if len(lexer) != 11:
                    s = "Syntax error line " + str(linenum) + ", carpet command takes 10 arguments"
                    raise(LoadError(s))
                carpet = CarpetModel(self.display, int(lexer[1]), int(lexer[2]), 
                                     int(lexer[3]), int(lexer[4]), 
                                     int(lexer[5]), int(lexer[6]), 
                                     int(lexer[7]), int(lexer[8]),
                                     int(lexer[9]), int(lexer[10]))
                self.world.add_object(carpet)
            elif lexer[0] == "robot":
                if len(lexer) != 4:
                    s = "Syntax error line " + str(linenum) + ", robot command takes 3 arguments"
                    raise(LoadError(s))
                self.world.set_robot_position(int(lexer[1]), int(lexer[2]), 
                                              int(lexer[3]))
                
        
    
