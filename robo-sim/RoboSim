#! /usr/bin/env python2.7

import src.simulator
import sys

from src.simulator import Simulator
port = 0
if (len(sys.argv) == 3):
   if (sys.argv[1] == "-p"):
      port = int(sys.argv[2])
      print(port)
sim = Simulator(port)
sim.run()
