#!/usr/bin/env python3
# The operator selects the task mode; task tracks what it runs apart from it
# (stat.task_run). What may start depends on the selected mode:
#
#   Manual  jogs, homing, and the [MDI_MACROS] macros that list MANUAL
#   MDI     MDI commands, and the macros that list MDI
#   Auto    program runs and steps, and the macros that list AUTO
#
# halui's MDI_COMMAND list runs in any mode. halui.mode.lock leaves the mode
# to the halui.mode.* pins: user interfaces cannot change it, and
# halui.program.run no longer selects Auto by itself.

import os
import sys
import time

import hal
import linuxcnc

c = linuxcnc.command()
s = linuxcnc.stat()
e = linuxcnc.error_channel()
h = hal.component("test-ui")
h.ready()

fails = 0


def check(name, ok, detail=''):
    global fails
    print('%s: %s %s' % ('ok' if ok else 'FAIL', name, detail), flush=True)
    if not ok:
        fails += 1


def wait(fn, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        s.poll()
        if fn():
            return True
        time.sleep(0.01)
    s.poll()
    return fn()


def errors():
    msgs = []
    while True:
        m = e.poll()
        if not m:
            return msgs
        msgs.append(m[1])


def refused(text):
    c.wait_complete()
    time.sleep(0.1)
    msgs = errors()
    return any(text in m for m in msgs), msgs


def idle():
    return (s.interp_state == linuxcnc.INTERP_IDLE and s.queue == 0
            and s.task_run == linuxcnc.RUN_NONE)


def x():
    return s.actual_position[0]


def mode(m):
    c.mode(m)
    c.wait_complete()
    s.poll()
    return s.task_mode == m


MODE = {linuxcnc.MODE_MANUAL: 'MANUAL', linuxcnc.MODE_MDI: 'MDI', linuxcnc.MODE_AUTO: 'AUTO'}
TRAJ = {linuxcnc.TRAJ_MODE_FREE: 'FREE', linuxcnc.TRAJ_MODE_TELEOP: 'TELEOP',
        linuxcnc.TRAJ_MODE_COORD: 'COORD'}
RUN = {linuxcnc.RUN_NONE: 'NONE', linuxcnc.RUN_PROGRAM: 'PROGRAM', linuxcnc.RUN_MDI: 'MDI'}


def modes():
    return '%s/%s/%s' % (MODE.get(s.task_mode), RUN.get(s.task_run), TRAJ.get(s.motion_mode))


wait(lambda: s.task_mode in MODE and s.motion_mode in TRAJ, 10.0)
check('starts in Manual, joint mode', s.task_mode == linuxcnc.MODE_MANUAL
      and s.task_run == linuxcnc.RUN_NONE and s.motion_mode == linuxcnc.TRAJ_MODE_FREE, modes())
check('halui sees Manual', hal.get_value('halui.mode.is-manual'))

c.state(linuxcnc.STATE_ESTOP_RESET)
c.state(linuxcnc.STATE_ON)
c.wait_complete()

# --- before homing: joint jogs in Manual only, MDI refused
c.jog(linuxcnc.JOG_INCREMENT, True, 1, 10, 0.5)
check('joint jog in Manual', wait(lambda: abs(s.joint_actual_position[1] - 0.5) < 1e-6), s.joint_actual_position[1])
check('MDI selected', mode(linuxcnc.MODE_MDI), modes())
c.jog(linuxcnc.JOG_INCREMENT, True, 1, 10, -0.5)
check('joint jog refused in MDI', *refused("Can't jog in MDI mode"))
c.home(-1)
check('homing refused in MDI', *refused("Can't home"))
c.mdi('G0 X1')
check('MDI refused before homing', *refused('not homed'))
mode(linuxcnc.MODE_MANUAL)
c.jog(linuxcnc.JOG_INCREMENT, True, 1, 10, -0.5)
wait(lambda: abs(s.joint_actual_position[1]) < 1e-6)
c.home(-1)
c.wait_complete()
check('homed in Manual', wait(lambda: all(s.homed[:3])), s.homed[:3])
c.teleop_enable(1)
c.wait_complete()
check('world mode', wait(lambda: s.motion_mode == linuxcnc.TRAJ_MODE_TELEOP), modes())

# --- MDI needs MDI mode
c.mdi('G0 X1')
check('MDI refused in Manual', *refused("Can't run an MDI command in Manual mode, select MDI"))
check('MDI', mode(linuxcnc.MODE_MDI), modes())
check('MDI idle is coordinated', wait(lambda: s.motion_mode == linuxcnc.TRAJ_MODE_COORD), modes())
c.mdi('G0 X1')
check('MDI runs', wait(lambda: s.task_run == linuxcnc.RUN_MDI, 1.0), modes())
check('MDI moved X', wait(lambda: abs(x() - 1) < 1e-6), x())
check('idle again, still MDI', wait(idle) and s.task_mode == linuxcnc.MODE_MDI
      and s.motion_mode == linuxcnc.TRAJ_MODE_COORD, modes())
check('halui sees MDI', hal.get_value('halui.mode.is-mdi'))
check('no errors from MDI', not errors())

# --- queued MDI; the mode cannot change while MDI runs
c.mdi('G4 P1')
c.mdi('G0 X1.5')
check('MDI queue runs', wait(lambda: s.task_run == linuxcnc.RUN_MDI, 1.0), modes())
c.mode(linuxcnc.MODE_MANUAL)
check('mode change refused while MDI runs', *refused("Can't select Manual while MDI runs"))
check('queued MDI finished', wait(lambda: abs(x() - 1.5) < 1e-6, 4.0), x())
check('idle after the queue, still MDI', wait(idle) and s.task_mode == linuxcnc.MODE_MDI, modes())

# --- wheel jogs only in Manual
hal.set_p('axis.x.jog-enable', '1')
hal.set_p('axis.x.jog-counts', '50')
time.sleep(0.3)
s.poll()
check('wheel does not jog in MDI', abs(x() - 1.5) < 1e-6, x())
check('Manual', mode(linuxcnc.MODE_MANUAL), modes())
check('back in world mode', wait(lambda: s.motion_mode == linuxcnc.TRAJ_MODE_TELEOP), modes())
hal.set_p('axis.x.jog-counts', '100')
check('wheel jogs in Manual', wait(lambda: abs(x() - 2) < 1e-6), x())
hal.set_p('axis.x.jog-enable', '0')
c.jog(linuxcnc.JOG_INCREMENT, False, 0, 10, 0.5)
check('world jog in Manual', wait(lambda: abs(x() - 2.5) < 1e-6), x())

# --- a jog in progress holds off a mode change
c.jog(linuxcnc.JOG_CONTINUOUS, False, 1, 1)
wait(lambda: s.current_vel > 0.5, 1.0)
c.mode(linuxcnc.MODE_MDI)
check('mode change refused while jogging', *refused('while jogging'))
c.jog(linuxcnc.JOG_STOP, False, 1)
check('jog stopped', wait(lambda: s.current_vel == 0 and not hal.get_value('motion.jog-is-active')), modes())
c.jog(linuxcnc.JOG_INCREMENT, False, 1, 10, -s.actual_position[1])
wait(lambda: abs(s.actual_position[1]) < 1e-6)

# --- programs run in Auto only
c.program_open(os.path.abspath('program.ngc'))
c.wait_complete()
c.auto(linuxcnc.AUTO_RUN, 0)
check('run refused in Manual', *refused("Can't run a program in Manual mode, select Auto"))
c.auto(linuxcnc.AUTO_STEP)
check('step refused in Manual', *refused("Can't run a program in Manual mode, select Auto"))
check('Auto', mode(linuxcnc.MODE_AUTO), modes())
c.jog(linuxcnc.JOG_INCREMENT, False, 1, 10, 0.5)
check('jog refused in Auto', *refused("Can't jog in Auto mode, select Manual"))
c.auto(linuxcnc.AUTO_RUN, 0)
check('program runs', wait(lambda: s.task_run == linuxcnc.RUN_PROGRAM, 1.0), modes())
check('halui sees Auto', hal.get_value('halui.mode.is-auto'))
check('program reached the dwell', wait(lambda: abs(x() - 2) < 1e-6), x())
c.mode(linuxcnc.MODE_MANUAL)
check('mode change refused while a program runs', *refused("Can't select Manual while a program runs"))
c.mdi('G0 X0')
check('MDI refused while the program runs', *refused('EMC_TASK_PLAN_EXECUTE'))
check('program finished', wait(lambda: abs(x() - 4) < 1e-6, 5.0), x())
check('idle after the program, still Auto', wait(idle) and s.task_mode == linuxcnc.MODE_AUTO
      and s.motion_mode == linuxcnc.TRAJ_MODE_COORD, modes())

# --- abort a paused program; the mode stays
c.auto(linuxcnc.AUTO_RUN, 0)
wait(lambda: abs(x() - 2) < 1e-6)
c.auto(linuxcnc.AUTO_PAUSE)
check('paused', wait(lambda: s.paused), modes())
c.abort()
c.wait_complete()
check('abort ends the run, still Auto', wait(idle) and s.task_mode == linuxcnc.MODE_AUTO, modes())
errors()

# --- the step starts the program paused, in Auto
c.auto(linuxcnc.AUTO_STEP)
check('step starts the program', wait(lambda: s.task_run == linuxcnc.RUN_PROGRAM and s.paused), modes())
c.abort()
c.wait_complete()
check('abort after step', wait(idle), modes())

# --- joint or world, chosen in Manual, is kept across the other modes
check('Manual', mode(linuxcnc.MODE_MANUAL), modes())
c.teleop_enable(0)
c.wait_complete()
check('joint mode', wait(lambda: s.motion_mode == linuxcnc.TRAJ_MODE_FREE), modes())
mode(linuxcnc.MODE_MDI)
check('MDI is coordinated', wait(lambda: s.motion_mode == linuxcnc.TRAJ_MODE_COORD), modes())
c.mdi('G0 X1')
check('MDI from MDI', wait(lambda: abs(x() - 1) < 1e-6), x())
wait(idle)
mode(linuxcnc.MODE_MANUAL)
check('joint mode again in Manual', wait(lambda: s.motion_mode == linuxcnc.TRAJ_MODE_FREE), modes())
mode(linuxcnc.MODE_AUTO)
c.teleop_enable(1)
c.wait_complete()
time.sleep(0.1)
s.poll()
check('teleop_enable in Auto keeps coordinated mode', s.motion_mode == linuxcnc.TRAJ_MODE_COORD, modes())
mode(linuxcnc.MODE_MANUAL)
check('... and Manual then uses world mode', wait(lambda: s.motion_mode == linuxcnc.TRAJ_MODE_TELEOP), modes())

# --- macros run by name in the modes they list
c.macro('park_y')
check('a Manual macro in Manual', wait(lambda: abs(s.actual_position[1] - 7) < 1e-6), s.actual_position[1])
check('idle after the macro, still Manual, world mode', wait(idle) and s.task_mode == linuxcnc.MODE_MANUAL
      and s.motion_mode == linuxcnc.TRAJ_MODE_TELEOP, modes())
c.jog(linuxcnc.JOG_INCREMENT, False, 1, 10, 0.5)
check('a jog right after the macro', wait(lambda: abs(s.actual_position[1] - 7.5) < 1e-6, 2) and not errors(),
      s.actual_position[1])
c.macro('x_half')
check('an MDI/Auto macro refused in Manual', *refused("Can't run macro 'x_half' in Manual mode, only in MDI, Auto"))
c.macro('nothing')
check('an unknown macro', *refused("No macro 'nothing'"))
mode(linuxcnc.MODE_AUTO)
c.macro('x_half')
check('an MDI/Auto macro in Auto', wait(lambda: abs(x() - 0.5) < 1e-6), x())
check('idle after the macro, still Auto', wait(idle) and s.task_mode == linuxcnc.MODE_AUTO, modes())
c.macro('park_y')
check('a Manual macro refused in Auto', *refused("Can't run macro 'park_y' in Auto mode, only in Manual"))
mode(linuxcnc.MODE_MANUAL)
c.jog(linuxcnc.JOG_INCREMENT, False, 1, 10, -7.5)
wait(lambda: abs(s.actual_position[1]) < 1e-6)
hal.set_p('halui.macro.park_y', '1')
check('halui.macro.park_y in Manual', wait(lambda: abs(s.actual_position[1] - 7) < 1e-6), s.actual_position[1])
hal.set_p('halui.macro.park_y', '0')
wait(idle)
errors()

# --- halui's MDI_COMMAND list runs in any mode and leaves it
for m in (linuxcnc.MODE_MANUAL, linuxcnc.MODE_AUTO, linuxcnc.MODE_MDI):
    mode(m)
    c.wait_complete()
    hal.set_p('halui.mdi-command-00', '1')
    check('halui MDI in %s' % MODE[m], wait(lambda: abs(x()) < 1e-6), x())
    check('idle after halui MDI, still %s' % MODE[m], wait(idle) and s.task_mode == m, modes())
    check('halui-mdi-is-running cleared', wait(lambda: not hal.get_value('halui.halui-mdi-is-running')))
    hal.set_p('halui.mdi-command-00', '0')
    mode(linuxcnc.MODE_MDI)
    c.mdi('G0 X1')
    wait(lambda: abs(x() - 1) < 1e-6)
    wait(idle)
check('no errors from halui MDI', not errors())
mode(linuxcnc.MODE_MANUAL)
c.macro('mdi-command-00')
check('MDI_COMMAND is the macro mdi-command-00, in Manual too', wait(lambda: abs(x()) < 1e-6), x())
wait(idle)
try:
    hal.get_value('halui.macro.mdi-command-00')
    check('MDI_COMMAND has no second pin', False)
except Exception:
    check('MDI_COMMAND has no second pin', True)
mode(linuxcnc.MODE_MDI)
c.mdi('G0 X1')
wait(lambda: idle() and abs(x() - 1) < 1e-6)
check('no errors from the MDI_COMMAND macro', not errors())

# --- halui's mode pins and halui.program.run
mode(linuxcnc.MODE_MANUAL)
hal.set_p('halui.mode.mdi', '1')
check('halui.mode.mdi', wait(lambda: s.task_mode == linuxcnc.MODE_MDI), modes())
hal.set_p('halui.mode.mdi', '0')
hal.set_p('halui.mode.manual', '1')
check('halui.mode.manual', wait(lambda: s.task_mode == linuxcnc.MODE_MANUAL), modes())
hal.set_p('halui.mode.manual', '0')
hal.set_p('halui.program.run', '1')
check('halui.program.run selects Auto and runs', wait(lambda: s.task_run == linuxcnc.RUN_PROGRAM
      and s.task_mode == linuxcnc.MODE_AUTO, 2), modes())
hal.set_p('halui.program.run', '0')
check('program finished', wait(lambda: idle() and abs(x() - 4) < 1e-6, 10), x())

# --- halui.mode.lock: the selector, held at Manual, owns the mode
hal.set_p('halui.mode.manual', '1')
hal.set_p('halui.mode.lock', '1')
check('the lock applies the held selector', wait(lambda: s.task_mode == linuxcnc.MODE_MANUAL), modes())
c.mode(linuxcnc.MODE_AUTO)
check('a user interface cannot change a locked mode', *refused('locked to the mode selector'))
hal.set_p('halui.program.run', '1')
time.sleep(0.3)
s.poll()
check('halui.program.run in a locked Manual does not run', s.task_run == linuxcnc.RUN_NONE
      and s.task_mode == linuxcnc.MODE_MANUAL, modes())
check('... and says why', *refused('select Auto'))
hal.set_p('halui.program.run', '0')
hal.set_p('halui.mode.manual', '0')
hal.set_p('halui.mode.auto', '1')
check('the selector turned to Auto', wait(lambda: s.task_mode == linuxcnc.MODE_AUTO), modes())
hal.set_p('halui.program.run', '1')
check('halui.program.run in a locked Auto runs', wait(lambda: s.task_run == linuxcnc.RUN_PROGRAM, 2), modes())
hal.set_p('halui.program.run', '0')
check('program finished', wait(lambda: idle() and abs(x() - 4) < 1e-6, 10), x())
hal.set_p('halui.mode.lock', '0')
check('unlocked: a user interface selects again', mode(linuxcnc.MODE_MANUAL), modes())
hal.set_p('halui.mode.auto', '0')

# --- motion drops to free mode while off; Auto asks for coordinated again
mode(linuxcnc.MODE_AUTO)
c.state(linuxcnc.STATE_OFF)
c.wait_complete()
wait(lambda: s.motion_mode == linuxcnc.TRAJ_MODE_FREE, 2)
c.state(linuxcnc.STATE_ON)
c.wait_complete()
check('Auto is coordinated again after machine on', wait(lambda: s.motion_mode == linuxcnc.TRAJ_MODE_COORD, 3), modes())
mode(linuxcnc.MODE_MANUAL)

msgs = errors()
check('no unexpected errors', not msgs, msgs)

c.state(linuxcnc.STATE_OFF)
c.wait_complete()
print('%d failures' % fails, flush=True)
sys.exit(1 if fails else 0)
