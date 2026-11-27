#!/usr/bin/env python3

import linuxcnc
import os
import sys

c = linuxcnc.command()

c.state(linuxcnc.STATE_ESTOP_RESET)
c.state(linuxcnc.STATE_ON)
c.mode(linuxcnc.MODE_AUTO)

c.program_open(os.environ.get('ORIENT_NGC', 'orient.ngc'))
c.auto(linuxcnc.AUTO_RUN, 0)
c.wait_complete()

sys.exit(0)
