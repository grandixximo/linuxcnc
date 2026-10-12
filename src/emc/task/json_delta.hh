/* Encoding benchmark only: EMC_STAT -> JSON merge-patch delta against the
   previous EMC_STAT, written straight into a string with no DOM. Fields and
   names mirror build_task/build_traj/build_motion/build_io, so the delta
   applied to the reflection JSON snapshot gives the same object (arrays of
   tables are keyed by index). */
#ifndef JSON_DELTA_HH
#define JSON_DELTA_HH

#include <charconv>
#include <cstring>
#include <string>

namespace jdelta {

struct W {
    std::string s;
    bool first = true;
    void key(const char *k)
    {
        if (!first) s += ',';
        first = false;
        s += '"'; s += k; s += "\":";
    }
    void num(double v)
    {
        char b[32];
        auto r = std::to_chars(b, b + sizeof b, v);
        s.append(b, r.ptr);
    }
    void num(long long v)
    {
        char b[24];
        auto r = std::to_chars(b, b + sizeof b, v);
        s.append(b, r.ptr);
    }
    void str(const char *v)
    {
        s += '"';
        for (const char *p = v; *p; ++p) {
            if (*p == '"' || *p == '\\') { s += '\\'; s += *p; }
            else if ((unsigned char)*p < 0x20) { char b[8]; snprintf(b, sizeof b, "\\u%04x", *p); s += b; }
            else s += *p;
        }
        s += '"';
    }
    /* Open a member object; close() drops it again if nothing was written. */
    struct Mark { size_t pos; bool first; };
    Mark open(const char *k)
    {
        Mark m{s.size(), first};
        key(k);
        s += '{';
        first = true;
        return m;
    }
    void close(const Mark &m)
    {
        if (first) { s.resize(m.pos); first = m.first; }
        else { s += '}'; first = false; }
    }
};

#define JD_D(name, a, b) do { if ((a) != (b)) { w.key(name); w.num((double)(b)); } } while (0)
#define JD_I(name, a, b) do { if ((a) != (b)) { w.key(name); w.num((long long)(b)); } } while (0)
#define JD_B(name, a, b) do { if (((a) != 0) != ((b) != 0)) { w.key(name); w.s += (b) ? "true" : "false"; } } while (0)
#define JD_S(name, a, b) do { if (strcmp((a), (b))) { w.key(name); w.str(b); } } while (0)

inline void pose(W &w, const char *k, const EmcPose &a, const EmcPose &b)
{
    auto m = w.open(k);
    JD_D("x", a.tran.x, b.tran.x); JD_D("y", a.tran.y, b.tran.y); JD_D("z", a.tran.z, b.tran.z);
    JD_D("a", a.a, b.a); JD_D("b", a.b, b.b); JD_D("c", a.c, b.c);
    JD_D("u", a.u, b.u); JD_D("v", a.v, b.v); JD_D("w", a.w, b.w);
    w.close(m);
}

template <class T>
inline void arr(W &w, const char *k, const T *a, const T *b, int n)
{
    if (!memcmp(a, b, sizeof(T) * n)) return;
    w.key(k);
    w.s += '[';
    for (int i = 0; i < n; ++i) {
        if (i) w.s += ',';
        if constexpr (std::is_floating_point_v<T>) w.num((double)b[i]); else w.num((long long)b[i]);
    }
    w.s += ']';
}

inline void task(W &w, const EMC_STAT &A, const EMC_STAT &B)
{
    const auto &a = A.task, &b = B.task;
    auto m = w.open("task");
    JD_I("mode", a.mode, b.mode); JD_I("state", a.state, b.state);
    JD_I("exec_state", a.execState, b.execState); JD_I("interp_state", a.interpState, b.interpState);
    JD_I("call_level", a.callLevel, b.callLevel); JD_I("motion_line", a.motionLine, b.motionLine);
    JD_I("current_line", a.currentLine, b.currentLine); JD_I("read_line", a.readLine, b.readLine);
    JD_B("is_optional_stop_state", a.optional_stop_state, b.optional_stop_state);
    JD_B("is_block_delete_state", a.block_delete_state, b.block_delete_state);
    JD_B("is_input_timeout", a.input_timeout, b.input_timeout);
    JD_S("filename", a.file, b.file); JD_S("command", a.command, b.command);
    JD_S("ini_filename", a.ini_filename, b.ini_filename);
    pose(w, "g5x_offset", a.g5x_offset, b.g5x_offset);
    JD_I("g5x_index", a.g5x_index, b.g5x_index);
    pose(w, "g92_offset", a.g92_offset, b.g92_offset);
    JD_D("rotation_xy", a.rotation_xy, b.rotation_xy);
    pose(w, "tool_offset", a.toolOffset, b.toolOffset);
    arr(w, "active_g_codes", a.activeGCodes, b.activeGCodes, ACTIVE_G_CODES);
    arr(w, "active_m_codes", a.activeMCodes, b.activeMCodes, ACTIVE_M_CODES);
    arr(w, "active_settings", a.activeSettings, b.activeSettings, ACTIVE_SETTINGS);
    JD_I("interpreter_error_code", a.interpreter_errcode, b.interpreter_errcode);
    JD_B("is_task_paused", a.task_paused, b.task_paused);
    JD_D("delay_left", a.delayLeft, b.delayLeft);
    JD_I("queued_mdi_commands", a.queuedMDIcommands, b.queuedMDIcommands);
    JD_I("program_units", a.programUnits, b.programUnits);
    JD_I("taskbeat", a.taskbeat, b.taskbeat);
    w.close(m);
}

inline void traj(W &w, const EMC_TRAJ_STAT &a, const EMC_TRAJ_STAT &b)
{
    auto m = w.open("traj");
    JD_D("linear_units", a.linearUnits, b.linearUnits); JD_D("angular_units", a.angularUnits, b.angularUnits);
    JD_D("cycle_time", a.cycleTime, b.cycleTime); JD_I("joints", a.joints, b.joints);
    JD_I("spindles", a.spindles, b.spindles); JD_I("axis_mask", a.axis_mask, b.axis_mask);
    JD_I("mode", a.mode, b.mode); JD_B("is_enabled", a.enabled, b.enabled);
    JD_B("is_in_position", a.inpos, b.inpos); JD_I("queue", a.queue, b.queue);
    JD_I("active_queue", a.activeQueue, b.activeQueue); JD_B("is_queue_full", a.queueFull, b.queueFull);
    JD_I("id", a.id, b.id); JD_B("is_paused", a.paused, b.paused);
    JD_D("scale", a.scale, b.scale); JD_D("rapid_scale", a.rapid_scale, b.rapid_scale);
    pose(w, "position", a.position, b.position);
    pose(w, "actual_position", a.actualPosition, b.actualPosition);
    JD_D("velocity", a.velocity, b.velocity); JD_D("acceleration", a.acceleration, b.acceleration);
    JD_D("max_velocity", a.maxVelocity, b.maxVelocity); JD_D("max_acceleration", a.maxAcceleration, b.maxAcceleration);
    pose(w, "probed_position", a.probedPosition, b.probedPosition);
    JD_B("is_probe_tripped", a.probe_tripped, b.probe_tripped); JD_B("is_probing", a.probing, b.probing);
    JD_I("probeval", a.probeval, b.probeval); JD_I("kinematics_type", a.kinematics_type, b.kinematics_type);
    JD_I("motion_type", a.motion_type, b.motion_type);
    JD_D("linear_distance_to_go", a.distance_to_go, b.distance_to_go);
    pose(w, "distance_to_go", a.dtg, b.dtg);
    JD_D("current_vel", a.current_vel, b.current_vel);
    JD_B("is_feed_override_enabled", a.feed_override_enabled, b.feed_override_enabled);
    JD_B("is_adaptive_feed_enabled", a.adaptive_feed_enabled, b.adaptive_feed_enabled);
    JD_B("is_feed_hold_enabled", a.feed_hold_enabled, b.feed_hold_enabled);
    JD_B("is_single_stepping", a.single_stepping, b.single_stepping);
    w.close(m);
}

inline void motion(W &w, const EMC_MOTION_STAT &a, const EMC_MOTION_STAT &b)
{
    auto m = w.open("motion");
    traj(w, a.traj, b.traj);
    const int nj = std::min(std::max(b.traj.joints, 0), EMCMOT_MAX_JOINTS);
    auto mj = w.open("joint");
    char idx[8];
    for (int i = 0; i < nj; ++i) {
        const EMC_JOINT_STAT &x = a.joint[i], &y = b.joint[i];
        snprintf(idx, sizeof idx, "%d", i);
        auto mi = w.open(idx);
        JD_I("joint_type", x.jointType, y.jointType); JD_D("units", x.units, y.units);
        JD_D("backlash", x.backlash, y.backlash);
        JD_D("min_position_limit", x.minPositionLimit, y.minPositionLimit);
        JD_D("max_position_limit", x.maxPositionLimit, y.maxPositionLimit);
        JD_D("max_ferror", x.maxFerror, y.maxFerror); JD_D("min_ferror", x.minFerror, y.minFerror);
        JD_D("ferror_current", x.ferrorCurrent, y.ferrorCurrent);
        JD_D("ferror_high_mark", x.ferrorHighMark, y.ferrorHighMark);
        JD_D("output", x.output, y.output); JD_D("input", x.input, y.input);
        JD_D("velocity", x.velocity, y.velocity);
        JD_B("is_in_position", x.inpos, y.inpos); JD_B("is_homing", x.homing, y.homing);
        JD_B("is_homed", x.homed, y.homed); JD_B("is_fault", x.fault, y.fault);
        JD_B("is_enabled", x.enabled, y.enabled);
        JD_B("is_min_soft_limit_exceeded", x.minSoftLimit, y.minSoftLimit);
        JD_B("is_max_soft_limit_exceeded", x.maxSoftLimit, y.maxSoftLimit);
        JD_B("is_min_hard_limit_exceeded", x.minHardLimit, y.minHardLimit);
        JD_B("is_max_hard_limit_exceeded", x.maxHardLimit, y.maxHardLimit);
        JD_B("is_override_limits", x.overrideLimits, y.overrideLimits);
        w.close(mi);
    }
    w.close(mj);
    auto ma = w.open("axis");
    for (int i = 0, k = 0; i < EMCMOT_MAX_AXIS; ++i) {
        if (!(b.traj.axis_mask & (1 << i))) continue;
        const EMC_AXIS_STAT &x = a.axis[i], &y = b.axis[i];
        snprintf(idx, sizeof idx, "%d", k++);
        auto mi = w.open(idx);
        JD_D("min_position_limit", x.minPositionLimit, y.minPositionLimit);
        JD_D("max_position_limit", x.maxPositionLimit, y.maxPositionLimit);
        JD_D("velocity", x.velocity, y.velocity);
        w.close(mi);
    }
    w.close(ma);
    const int ns = std::min(std::max(b.traj.spindles, 0), EMCMOT_MAX_SPINDLES);
    auto ms = w.open("spindle");
    for (int i = 0; i < ns; ++i) {
        const EMC_SPINDLE_STAT &x = a.spindle[i], &y = b.spindle[i];
        snprintf(idx, sizeof idx, "%d", i);
        auto mi = w.open(idx);
        JD_D("speed", x.speed, y.speed); JD_D("spindle_scale", x.spindle_scale, y.spindle_scale);
        JD_D("css_maximum", x.css_maximum, y.css_maximum); JD_D("css_factor", x.css_factor, y.css_factor);
        JD_I("state", x.state, y.state); JD_I("direction", x.direction, y.direction);
        JD_B("is_brake_engaged", x.brake, y.brake); JD_I("increasing", x.increasing, y.increasing);
        JD_B("is_enabled", x.enabled, y.enabled); JD_I("orient_state", x.orient_state, y.orient_state);
        JD_I("orient_fault", x.orient_fault, y.orient_fault);
        JD_B("is_spindle_override_enabled", x.spindle_override_enabled, y.spindle_override_enabled);
        JD_B("is_homed", x.homed, y.homed);
        w.close(mi);
    }
    w.close(ms);
    arr(w, "synch_di", a.synch_di, b.synch_di, EMCMOT_MAX_DIO);
    arr(w, "synch_do", a.synch_do, b.synch_do, EMCMOT_MAX_DIO);
    arr(w, "analog_input", a.analog_input, b.analog_input, EMCMOT_MAX_AIO);
    arr(w, "analog_output", a.analog_output, b.analog_output, EMCMOT_MAX_AIO);
    arr(w, "misc_error", a.misc_error, b.misc_error, EMCMOT_MAX_MISC_ERROR);
    JD_I("debug", a.debug, b.debug);
    JD_B("is_on_soft_limit", a.on_soft_limit, b.on_soft_limit);
    JD_B("is_external_offsets_applied", a.external_offsets_applied, b.external_offsets_applied);
    pose(w, "eoffset_pose", a.eoffset_pose, b.eoffset_pose);
    JD_I("num_extra_joints", a.numExtraJoints, b.numExtraJoints);
    JD_B("is_jogging_active", a.jogging_active, b.jogging_active);
    JD_I("heartbeat", a.heartbeat, b.heartbeat);
    JD_I("echo_serial_number", a.echo_serial_number, b.echo_serial_number);
    JD_I("command_type", a.command_type, b.command_type);
    JD_I("status", a.status, b.status);
    w.close(m);
}

inline void io(W &w, const EMC_IO_STAT &a, const EMC_IO_STAT &b)
{
    auto m = w.open("io");
    JD_I("debug", a.debug, b.debug); JD_I("reason", a.reason, b.reason); JD_I("fault", a.fault, b.fault);
    auto mt = w.open("tool");
    JD_I("pocket_prepped", a.tool.pocketPrepped, b.tool.pocketPrepped);
    JD_I("tool_in_spindle", a.tool.toolInSpindle, b.tool.toolInSpindle);
    JD_I("tool_from_pocket", a.tool.toolFromPocket, b.tool.toolFromPocket);
    w.close(mt);
    auto mc = w.open("coolant");
    JD_B("is_mist", a.coolant.mist, b.coolant.mist); JD_B("is_flood", a.coolant.flood, b.coolant.flood);
    w.close(mc);
    auto mx = w.open("aux");
    JD_B("is_estopped", a.aux.estop, b.aux.estop);
    w.close(mx);
    JD_I("echo_serial_number", a.echo_serial_number, b.echo_serial_number);
    JD_I("command_type", a.command_type, b.command_type);
    JD_I("status", a.status, b.status);
    w.close(m);
}

/* Appends the delta object for b relative to a; "{}" when nothing changed. */
inline void delta(std::string &out, const EMC_STAT &a, const EMC_STAT &b)
{
    W w;
    w.s.swap(out);
    w.s.clear();
    w.s += '{';
    task(w, a, b);
    motion(w, a.motion, b.motion);
    io(w, a.io, b.io);
    JD_I("echo_serial_number", a.echo_serial_number, b.echo_serial_number);
    JD_I("command_type", a.command_type, b.command_type);
    JD_I("status", a.status, b.status);
    JD_I("debug", a.debug, b.debug);
    w.s += '}';
    out.swap(w.s);
}

#undef JD_D
#undef JD_I
#undef JD_B
#undef JD_S

}
#endif
