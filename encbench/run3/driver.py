#!/usr/bin/env python3
# Headless "UI" for the encoding capture: machine on, home, run a program,
# then a short MDI jog sequence, then quit.
import linuxcnc, time, sys, os
HERE = os.path.dirname(os.path.abspath(__file__))

s = linuxcnc.stat(); c = linuxcnc.command(); e = linuxcnc.error_channel()

def wait(pred, t=60):
    end = time.time() + t
    while time.time() < end:
        s.poll()
        if pred(): return True
        time.sleep(0.05)
    return False

def ok(r):
    print("wait_complete", r); sys.stdout.flush()

time.sleep(1)
idle = lambda: s.interp_state == linuxcnc.INTERP_IDLE
c.state(linuxcnc.STATE_ESTOP_RESET); ok(c.wait_complete())
c.state(linuxcnc.STATE_ON); ok(c.wait_complete())
c.mode(linuxcnc.MODE_MANUAL); ok(c.wait_complete())
c.home(-1); ok(c.wait_complete())
print("homed", wait(lambda: all(s.homed[i] for i in range(s.joints)), 60)); sys.stdout.flush()
c.mode(linuxcnc.MODE_MDI); ok(c.wait_complete())
c.mdi("G10 L2 P1 Z-3"); ok(c.wait_complete()); wait(idle, 10)
time.sleep(1)
c.mode(linuxcnc.MODE_AUTO); ok(c.wait_complete())
c.program_open(os.path.join(HERE, "../../nc_files/3D_Chips.ngc")); ok(c.wait_complete())
c.auto(linuxcnc.AUTO_RUN, 0); ok(c.wait_complete())
print("started", wait(lambda: not idle(), 10))
print("finished", wait(idle, 120)); sys.stdout.flush()
c.mode(linuxcnc.MODE_MDI); ok(c.wait_complete())
for cmd in ("G0 X1 Y1", "G1 X-1 F60", "G0 Z0"):
    c.mdi(cmd); ok(c.wait_complete()); wait(idle, 30)
time.sleep(1)
c.state(linuxcnc.STATE_ESTOP); ok(c.wait_complete())
print("done"); sys.stdout.flush()
