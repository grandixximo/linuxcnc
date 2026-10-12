/********************************************************************
* Description: ws_server.cc
*   WebSocket + FlatBuffers transport for task <-> UI communication.
*
*   The test web UI this serves is a real file, share/linuxcnc/ws_ui.html,
*   not a string literal in here: it can be edited and reloaded in the
*   browser without rebuilding task.
*
*   Design notes:
*
*   - All of the transport lives in this file. emctaskmain.cc only calls
*     wsServerStart/Stop/Publish; received commands are handed to the
*     task-internal command queue (cmd_queue.hh), which task drains.
*
*   - The task loop must never block on a socket. wsServerPublish() only
*     memcpy's EMC_STAT into a pending buffer under a short lock; the
*     FlatBuffers encoding and the fan-out to clients happen on the server
*     thread. EMC_STAT is memcpy'd rather than copy-assigned because
*     EMC_TOOL_STAT deletes its copy constructor.
*
*   - Status is a full snapshot every cycle, so a slow client can simply
*     lose intermediate frames: each session keeps only the newest status
*     frame. Nothing about a dropped status frame is unrecoverable.
*
*   - Two wire formats on the same port:
*       /ws           binary FlatBuffers  (the real protocol)
*       /ws?format=json  JSON text        (the test web UI, rendered from
*                                          an encoded EmcStat via the flatc
*                                          reflection type tables)
*     The JSON is generated from an encoded FlatBuffer, not from EMC_STAT,
*     so what the browser shows is what the binary protocol carries. It
*     carries status only: acknowledgements and operator messages are on
*     the binary channel, which the test UI does not need.
*
*   - The binary channel carries one root type, ServerMsg, with three
*     payloads: a status snapshot, an acknowledgement of one command, and
*     an operator message. Together they replace all three NML channels a
*     UI used to open. An acknowledgement goes only to the session that
*     submitted the command, which is what lets a client use a plain
*     per-connection counter where NML made it poll a shared
*     echo_serial_number.
*
*   - Commands arrive either as a binary CmdChannelMsg FlatBuffer or, from
*     the test UI, as a one-line text command. The text form is converted
*     into the same FlatBuffer and then walks the identical decode path, so
*     the debug UI exercises the real command plumbing.
*
* License: GPL Version 2
********************************************************************/

#include "ws_server.hh"
#include "cmd_queue.hh"

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/strand.hpp>

#include "flatbuffers/minireflect.h"
#include "emc/flatbuf/emc_common_generated.h"
#include "emc/flatbuf/emc_stat_generated.h"
#include "emc/flatbuf/emc_cmd_generated.h"
#include "emc/flatbuf/emc_error_generated.h"
#include "emc/flatbuf/emc_ws_generated.h"

#include <fmt/format.h>

#include "config.h"		// EMC2_HOME
#include "nml_intf/emc.hh"
#include "nml_intf/emc_nml.hh"
#include "nml_intf/emcglb.h"
#include <rtapi_string.h>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <chrono>
#include "json_delta.hh"

namespace beast     = boost::beast;
namespace http      = beast::http;
namespace websocket = beast::websocket;
namespace asio      = boost::asio;
using tcp           = asio::ip::tcp;

/* ------------------------------------------------------------------ */
/* shared state                                                       */
/* ------------------------------------------------------------------ */

/* A decoded command is built here before it is handed to the task command
   queue, so decode_command() can write into a plain buffer. */
static const size_t WS_MAX_CMD_SIZE = taskcmd::MAX_CMD_SIZE;

namespace {

struct QueuedCommand {
    alignas(16) char storage[WS_MAX_CMD_SIZE];
    size_t size = 0;
    RCS_CMD_MSG *msg() { return reinterpret_cast<RCS_CMD_MSG *>(storage); }
};

/* These NML command messages are plain data with no destructors, so storing
   them in a flat buffer and memcpy'ing is well defined and keeps the queue
   free of per-type ownership handling. */
template <class T>
void store_command(QueuedCommand &q, const T &cmd)
{
    static_assert(sizeof(T) <= WS_MAX_CMD_SIZE, "NML command too large for ws queue");
    std::memcpy(q.storage, &cmd, sizeof(T));
    q.size = sizeof(T);
}

class Session;

/* One operator message on its way out, queued by the task thread. */
struct OperatorMsg {
    int         kind;   /* WS_OPERATOR_ERROR / _TEXT / _DISPLAY */
    std::string text;
};

/* Who is waiting for a queued command to finish. */
struct CmdOwner {
    std::weak_ptr<Session> session;
    uint64_t               serial;   /* the client's own command serial */
};

struct ServerState {
    std::mutex              mu;            /* guards everything below */
    std::vector<char>       pending_stat;  /* raw EMC_STAT from task thread */
    bool                    pending_valid = false;
    std::deque<OperatorMsg> pending_errors;

    std::shared_ptr<const std::vector<uint8_t>> snapshot_bin;
    std::shared_ptr<const std::string>          snapshot_json;

    std::mutex                        sess_mu;
    std::set<std::shared_ptr<Session>> sessions;

    /* Which session to send each command acknowledgement to. Entries are
       added when a command is queued and removed when it settles; a
       command from halui has no entry and is simply not acknowledged. */
    std::mutex                          cmd_mu;
    std::map<taskcmd::Ticket, CmdOwner> cmd_owner;

    asio::io_context     ioc{1};
    std::thread          thread;
    bool                 running = false;

    /* Set while a serialize/fan-out handler is queued but has not run yet.
       Task publishes every cycle; if encoding ever falls behind the loop,
       posting unconditionally would grow the io_context queue without
       bound. Coalescing instead means a slow server thread costs dropped
       intermediate snapshots, never unbounded memory. */
    std::atomic<bool>    fanout_pending{false};
};

ServerState *g_state = nullptr;

/* ------------------------------------------------------------------ */
/* EMC_STAT  ->  FlatBuffer                                           */
/* ------------------------------------------------------------------ */

flatbuffers::Offset<EMC::TaskStat>
build_task(flatbuffers::FlatBufferBuilder &fbb, const EMC_STAT &s)
{
    auto file    = fbb.CreateString(s.task.file);
    auto command = fbb.CreateString(s.task.command);
    auto inifile = fbb.CreateString(s.task.ini_filename);

    std::vector<int32_t> gcodes(s.task.activeGCodes,
                                s.task.activeGCodes + ACTIVE_G_CODES);
    std::vector<int32_t> mcodes(s.task.activeMCodes,
                                s.task.activeMCodes + ACTIVE_M_CODES);
    std::vector<double>  settings(s.task.activeSettings,
                                  s.task.activeSettings + ACTIVE_SETTINGS);
    auto vg = fbb.CreateVector(gcodes);
    auto vm = fbb.CreateVector(mcodes);
    auto vs = fbb.CreateVector(settings);

    const EmcPose &g5x = s.task.g5x_offset;
    const EmcPose &g92 = s.task.g92_offset;
    const EmcPose &tlo = s.task.toolOffset;
    EMC::Pose p_g5x(g5x.tran.x, g5x.tran.y, g5x.tran.z, g5x.a, g5x.b, g5x.c, g5x.u, g5x.v, g5x.w);
    EMC::Pose p_g92(g92.tran.x, g92.tran.y, g92.tran.z, g92.a, g92.b, g92.c, g92.u, g92.v, g92.w);
    EMC::Pose p_tlo(tlo.tran.x, tlo.tran.y, tlo.tran.z, tlo.a, tlo.b, tlo.c, tlo.u, tlo.v, tlo.w);

    EMC::TaskStatBuilder b(fbb);
    b.add_mode(static_cast<int32_t>(s.task.mode));
    b.add_state(static_cast<int32_t>(s.task.state));
    b.add_exec_state(static_cast<int32_t>(s.task.execState));
    b.add_interp_state(static_cast<int32_t>(s.task.interpState));
    b.add_call_level(s.task.callLevel);
    b.add_motion_line(s.task.motionLine);
    b.add_current_line(s.task.currentLine);
    b.add_read_line(s.task.readLine);
    b.add_is_optional_stop_state(s.task.optional_stop_state);
    b.add_is_block_delete_state(s.task.block_delete_state);
    b.add_is_input_timeout(s.task.input_timeout);
    b.add_filename(file);
    b.add_command(command);
    b.add_ini_filename(inifile);
    b.add_g5x_offset(&p_g5x);
    b.add_g5x_index(s.task.g5x_index);
    b.add_g92_offset(&p_g92);
    b.add_rotation_xy(s.task.rotation_xy);
    b.add_tool_offset(&p_tlo);
    b.add_active_g_codes(vg);
    b.add_active_m_codes(vm);
    b.add_active_settings(vs);
    b.add_interpreter_error_code(s.task.interpreter_errcode);
    b.add_is_task_paused(s.task.task_paused != 0);
    b.add_delay_left(s.task.delayLeft);
    b.add_queued_mdi_commands(s.task.queuedMDIcommands);
    b.add_program_units(static_cast<int32_t>(s.task.programUnits));
    b.add_taskbeat(s.task.taskbeat);
    return b.Finish();
}

flatbuffers::Offset<EMC::TrajStat>
build_traj(flatbuffers::FlatBufferBuilder &fbb, const EMC_TRAJ_STAT &t)
{
    EMC::Pose pos(t.position.tran.x, t.position.tran.y, t.position.tran.z,
                  t.position.a, t.position.b, t.position.c,
                  t.position.u, t.position.v, t.position.w);
    EMC::Pose act(t.actualPosition.tran.x, t.actualPosition.tran.y, t.actualPosition.tran.z,
                  t.actualPosition.a, t.actualPosition.b, t.actualPosition.c,
                  t.actualPosition.u, t.actualPosition.v, t.actualPosition.w);
    EMC::Pose prb(t.probedPosition.tran.x, t.probedPosition.tran.y, t.probedPosition.tran.z,
                  t.probedPosition.a, t.probedPosition.b, t.probedPosition.c,
                  t.probedPosition.u, t.probedPosition.v, t.probedPosition.w);
    EMC::Pose dtg(t.dtg.tran.x, t.dtg.tran.y, t.dtg.tran.z,
                  t.dtg.a, t.dtg.b, t.dtg.c, t.dtg.u, t.dtg.v, t.dtg.w);

    EMC::TrajStatBuilder b(fbb);
    b.add_linear_units(t.linearUnits);
    b.add_angular_units(t.angularUnits);
    b.add_cycle_time(t.cycleTime);
    b.add_joints(t.joints);
    b.add_spindles(t.spindles);
    b.add_axis_mask(t.axis_mask);
    b.add_mode(static_cast<EMC::TrajMode>(t.mode));
    b.add_is_enabled(t.enabled);
    b.add_is_in_position(t.inpos);
    b.add_queue(t.queue);
    b.add_active_queue(t.activeQueue);
    b.add_is_queue_full(t.queueFull);
    b.add_id(t.id);
    b.add_is_paused(t.paused);
    b.add_scale(t.scale);
    b.add_rapid_scale(t.rapid_scale);
    b.add_position(&pos);
    b.add_actual_position(&act);
    b.add_velocity(t.velocity);
    b.add_acceleration(t.acceleration);
    b.add_max_velocity(t.maxVelocity);
    b.add_max_acceleration(t.maxAcceleration);
    b.add_probed_position(&prb);
    b.add_is_probe_tripped(t.probe_tripped);
    b.add_is_probing(t.probing);
    b.add_probeval(t.probeval);
    b.add_kinematics_type(t.kinematics_type);
    b.add_motion_type(t.motion_type);
    b.add_linear_distance_to_go(t.distance_to_go);
    b.add_distance_to_go(&dtg);
    b.add_current_vel(t.current_vel);
    b.add_is_feed_override_enabled(t.feed_override_enabled);
    b.add_is_adaptive_feed_enabled(t.adaptive_feed_enabled);
    b.add_is_feed_hold_enabled(t.feed_hold_enabled);
    b.add_is_single_stepping(t.single_stepping);
    return b.Finish();
}

flatbuffers::Offset<EMC::MotionStat>
build_motion(flatbuffers::FlatBufferBuilder &fbb, const EMC_MOTION_STAT &m)
{
    auto traj = build_traj(fbb, m.traj);

    const int njoints  = std::min(std::max(m.traj.joints, 0),   EMCMOT_MAX_JOINTS);
    const int nspindles = std::min(std::max(m.traj.spindles, 0), EMCMOT_MAX_SPINDLES);

    std::vector<flatbuffers::Offset<EMC::JointStat>> joints;
    joints.reserve(njoints);
    for (int i = 0; i < njoints; i++) {
        const EMC_JOINT_STAT &j = m.joint[i];
        EMC::JointStatBuilder b(fbb);
        b.add_joint(i);
        b.add_joint_type(j.jointType == EMC_LINEAR ? EMC::JointType::linear
                                                   : EMC::JointType::angular);
        b.add_units(j.units);
        b.add_backlash(j.backlash);
        b.add_min_position_limit(j.minPositionLimit);
        b.add_max_position_limit(j.maxPositionLimit);
        b.add_max_ferror(j.maxFerror);
        b.add_min_ferror(j.minFerror);
        b.add_ferror_current(j.ferrorCurrent);
        b.add_ferror_high_mark(j.ferrorHighMark);
        b.add_output(j.output);
        b.add_input(j.input);
        b.add_velocity(j.velocity);
        b.add_is_in_position(j.inpos != 0);
        b.add_is_homing(j.homing != 0);
        b.add_is_homed(j.homed != 0);
        b.add_is_fault(j.fault != 0);
        b.add_is_enabled(j.enabled != 0);
        b.add_is_min_soft_limit_exceeded(j.minSoftLimit != 0);
        b.add_is_max_soft_limit_exceeded(j.maxSoftLimit != 0);
        b.add_is_min_hard_limit_exceeded(j.minHardLimit != 0);
        b.add_is_max_hard_limit_exceeded(j.maxHardLimit != 0);
        b.add_is_override_limits(j.overrideLimits != 0);
        joints.push_back(b.Finish());
    }
    auto vjoints = fbb.CreateVector(joints);

    std::vector<flatbuffers::Offset<EMC::AxisStat>> axes;
    for (int i = 0; i < EMCMOT_MAX_AXIS; i++) {
        if (!(m.traj.axis_mask & (1 << i))) continue;
        const EMC_AXIS_STAT &a = m.axis[i];
        EMC::AxisStatBuilder b(fbb);
        b.add_axis(i);
        b.add_min_position_limit(a.minPositionLimit);
        b.add_max_position_limit(a.maxPositionLimit);
        b.add_velocity(a.velocity);
        axes.push_back(b.Finish());
    }
    auto vaxes = fbb.CreateVector(axes);

    std::vector<flatbuffers::Offset<EMC::SpindleStat>> spindles;
    spindles.reserve(nspindles);
    for (int i = 0; i < nspindles; i++) {
        const EMC_SPINDLE_STAT &sp = m.spindle[i];
        EMC::SpindleStatBuilder b(fbb);
        b.add_speed(sp.speed);
        b.add_spindle_scale(sp.spindle_scale);
        b.add_css_maximum(sp.css_maximum);
        b.add_css_factor(sp.css_factor);
        b.add_state(sp.state);
        b.add_direction(sp.direction);
        b.add_is_brake_engaged(sp.brake != 0);
        b.add_is_increasing(sp.increasing > 0);
        b.add_is_decreasing(sp.increasing < 0);
        b.add_is_enabled(sp.enabled != 0);
        b.add_orient_state(sp.orient_state);
        b.add_orient_fault(sp.orient_fault);
        b.add_is_spindle_override_enabled(sp.spindle_override_enabled);
        b.add_is_homed(sp.homed);
        spindles.push_back(b.Finish());
    }
    auto vspindles = fbb.CreateVector(spindles);

    std::vector<uint8_t> din(m.synch_di, m.synch_di + EMCMOT_MAX_DIO);
    std::vector<uint8_t> dout(m.synch_do, m.synch_do + EMCMOT_MAX_DIO);
    auto vdin  = fbb.CreateVector(din);
    auto vdout = fbb.CreateVector(dout);
    std::vector<double> ain(m.analog_input,  m.analog_input  + EMCMOT_MAX_AIO);
    std::vector<double> aout(m.analog_output, m.analog_output + EMCMOT_MAX_AIO);
    auto vain  = fbb.CreateVector(ain);
    auto vaout = fbb.CreateVector(aout);
    std::vector<int32_t> misc(m.misc_error, m.misc_error + EMCMOT_MAX_MISC_ERROR);
    auto vmisc = fbb.CreateVector(misc);

    const EmcPose &eo = m.eoffset_pose;
    EMC::Pose p_eo(eo.tran.x, eo.tran.y, eo.tran.z, eo.a, eo.b, eo.c, eo.u, eo.v, eo.w);

    EMC::MotionStatBuilder b(fbb);
    b.add_traj(traj);
    b.add_joint(vjoints);
    b.add_axis(vaxes);
    b.add_spindle(vspindles);
    b.add_synch_di(vdin);
    b.add_synch_do(vdout);
    b.add_analog_input(vain);
    b.add_analog_output(vaout);
    b.add_misc_error(vmisc);
    b.add_debug(m.debug);
    b.add_is_on_soft_limit(m.on_soft_limit != 0);
    b.add_is_external_offsets_applied(m.external_offsets_applied != 0);
    b.add_eoffset_pose(&p_eo);
    b.add_num_extra_joints(m.numExtraJoints);
    b.add_is_jogging_active(m.jogging_active);
    b.add_heartbeat(m.heartbeat);
    b.add_echo_serial_number(m.echo_serial_number);
    b.add_command_type(static_cast<uint32_t>(m.command_type));
    b.add_status(static_cast<int32_t>(m.status));
    return b.Finish();
}

flatbuffers::Offset<EMC::IOStat>
build_io(flatbuffers::FlatBufferBuilder &fbb, const EMC_IO_STAT &io)
{
    EMC::ToolStatBuilder tb(fbb);
    tb.add_pocket_prepped(io.tool.pocketPrepped);
    tb.add_tool_in_spindle(io.tool.toolInSpindle);
    tb.add_tool_from_pocket(io.tool.toolFromPocket);
    auto tool = tb.Finish();

    EMC::CoolantStatBuilder cb(fbb);
    cb.add_is_mist(io.coolant.mist != 0);
    cb.add_is_flood(io.coolant.flood != 0);
    auto coolant = cb.Finish();

    EMC::AuxStatBuilder ab(fbb);
    ab.add_is_estopped(io.aux.estop != 0);
    auto aux = ab.Finish();

    EMC::IOStatBuilder b(fbb);
    b.add_debug(io.debug);
    b.add_reason(io.reason);
    b.add_fault(io.fault);
    b.add_tool(tool);
    b.add_coolant(coolant);
    b.add_aux(aux);
    b.add_echo_serial_number(io.echo_serial_number);
    b.add_command_type(static_cast<uint32_t>(io.command_type));
    b.add_status(static_cast<int32_t>(io.status));
    return b.Finish();
}

flatbuffers::Offset<EMC::EmcStat>
build_stat(flatbuffers::FlatBufferBuilder &fbb, const EMC_STAT &s)
{
    auto task   = build_task(fbb, s);
    auto motion = build_motion(fbb, s.motion);
    auto io     = build_io(fbb, s.io);

    EMC::EmcStatBuilder b(fbb);
    b.add_task(task);
    b.add_motion(motion);
    b.add_io(io);
    b.add_echo_serial_number(s.echo_serial_number);
    b.add_command_type(static_cast<uint32_t>(s.command_type));
    b.add_status(static_cast<int32_t>(s.status));
    b.add_debug(s.debug);
    return b.Finish();
}

/* Bare EmcStat, for rendering the test UI's JSON. */
void encode_status(const EMC_STAT &s, std::vector<uint8_t> &out)
{
    flatbuffers::FlatBufferBuilder fbb(8192);
    fbb.Finish(build_stat(fbb, s));
    out.assign(fbb.GetBufferPointer(), fbb.GetBufferPointer() + fbb.GetSize());
}

/* ------------------------------------------------------------------ */
/* ServerMsg: what the binary channel actually carries                */
/* ------------------------------------------------------------------ */

template <class Payload>
std::shared_ptr<const std::vector<uint8_t>>
finish_server_msg(flatbuffers::FlatBufferBuilder &fbb,
                  EMC::ServerPayload type, flatbuffers::Offset<Payload> payload)
{
    EMC::ServerMsgBuilder b(fbb);
    b.add_payload_type(type);
    b.add_payload(payload.Union());
    fbb.Finish(b.Finish());
    return std::make_shared<std::vector<uint8_t>>(
        fbb.GetBufferPointer(), fbb.GetBufferPointer() + fbb.GetSize());
}

std::shared_ptr<const std::vector<uint8_t>>
encode_status_msg(const EMC_STAT &s)
{
    flatbuffers::FlatBufferBuilder fbb(8192);
    auto stat = build_stat(fbb, s);
    return finish_server_msg(fbb, EMC::ServerPayload::stat, stat);
}

std::shared_ptr<const std::vector<uint8_t>>
encode_ack(uint64_t serial, EMC::CmdState state)
{
    flatbuffers::FlatBufferBuilder fbb(128);
    auto ack = EMC::CreateCmdAck(fbb, serial, state);
    return finish_server_msg(fbb, EMC::ServerPayload::ack, ack);
}

std::shared_ptr<const std::vector<uint8_t>>
encode_operator_msg(int kind, const std::string &text)
{
    flatbuffers::FlatBufferBuilder fbb(512);
    auto str = fbb.CreateString(text);

    EMC::Message      mtype = EMC::Message::operator_error;
    flatbuffers::Offset<void> mval;
    switch (kind) {
    case WS_OPERATOR_TEXT:
        mtype = EMC::Message::operator_text;
        mval = EMC::CreateOperatorText(fbb, str).Union();
        break;
    case WS_OPERATOR_DISPLAY:
        mtype = EMC::Message::operator_display;
        mval = EMC::CreateOperatorDisplay(fbb, str).Union();
        break;
    default:
        mtype = EMC::Message::operator_error;
        mval = EMC::CreateOperatorError(fbb, str).Union();
        break;
    }

    EMC::ErrorChannelMsgBuilder eb(fbb);
    eb.add_message_type(mtype);
    eb.add_message(mval);
    auto err = eb.Finish();
    return finish_server_msg(fbb, EMC::ServerPayload::error, err);
}

/* ------------------------------------------------------------------ */
/* FlatBuffer command  ->  NML command                                */
/* ------------------------------------------------------------------ */

void to_emcpose(const EMC::Pose &p, EmcPose &out)
{
    out.tran.x = p.x();
    out.tran.y = p.y();
    out.tran.z = p.z();
    out.a = p.a();
    out.b = p.b();
    out.c = p.c();
    out.u = p.u();
    out.v = p.v();
    out.w = p.w();
}

/* Returns true if the message was understood and placed in q. */
bool decode_command(const EMC::CmdChannelMsg *msg, QueuedCommand &q)
{
    if (msg == nullptr) return false;

    switch (msg->command_type()) {

    case EMC::Command::task_set_state: {
        auto c = msg->command_as_task_set_state();
        EMC_TASK_SET_STATE m;
        m.state = static_cast<EMC_TASK_STATE>(c->state());
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_set_mode: {
        auto c = msg->command_as_task_set_mode();
        EMC_TASK_SET_MODE m;
        m.mode = static_cast<EMC_TASK_MODE>(c->mode());
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_abort: {
        EMC_TASK_ABORT m;
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_execute: {
        auto c = msg->command_as_task_plan_execute();
        EMC_TASK_PLAN_EXECUTE m;
        if (c->command() != nullptr) {
            rtapi_strxcpy(m.command, c->command()->c_str());
        }
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_open: {
        auto c = msg->command_as_task_plan_open();
        EMC_TASK_PLAN_OPEN m;
        if (c->file() != nullptr) {
            rtapi_strxcpy(m.file, c->file()->c_str());
        }
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_run: {
        auto c = msg->command_as_task_plan_run();
        EMC_TASK_PLAN_RUN m;
        m.line = c->line();
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_pause: {
        EMC_TASK_PLAN_PAUSE m;
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_resume: {
        EMC_TASK_PLAN_RESUME m;
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_step: {
        EMC_TASK_PLAN_STEP m;
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_init: {
        EMC_TASK_PLAN_INIT m;
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_synch: {
        EMC_TASK_PLAN_SYNCH m;
        store_command(q, m);
        return true;
    }
    case EMC::Command::joint_home: {
        auto c = msg->command_as_joint_home();
        EMC_JOINT_HOME m;
        m.joint = c->joint();
        store_command(q, m);
        return true;
    }
    case EMC::Command::joint_unhome: {
        auto c = msg->command_as_joint_unhome();
        EMC_JOINT_UNHOME m;
        m.joint = c->joint();
        store_command(q, m);
        return true;
    }
    case EMC::Command::joint_override_limits: {
        auto c = msg->command_as_joint_override_limits();
        EMC_JOINT_OVERRIDE_LIMITS m;
        m.joint = c->joint();
        store_command(q, m);
        return true;
    }
    case EMC::Command::jog_cont: {
        auto c = msg->command_as_jog_cont();
        EMC_JOG_CONT m;
        m.joint_or_axis = c->joint();
        m.vel = c->vel();
        m.jjogmode = c->jjogmode();
        store_command(q, m);
        return true;
    }
    case EMC::Command::jog_incr: {
        auto c = msg->command_as_jog_incr();
        EMC_JOG_INCR m;
        m.joint_or_axis = c->joint();
        m.incr = c->incr();
        m.vel = c->vel();
        m.jjogmode = c->jjogmode();
        store_command(q, m);
        return true;
    }
    case EMC::Command::jog_stop: {
        auto c = msg->command_as_jog_stop();
        EMC_JOG_STOP m;
        m.joint_or_axis = c->joint();
        m.jjogmode = c->jjogmode();
        store_command(q, m);
        return true;
    }
    case EMC::Command::traj_set_scale: {
        auto c = msg->command_as_traj_set_scale();
        EMC_TRAJ_SET_SCALE m;
        m.scale = c->scale();
        store_command(q, m);
        return true;
    }
    case EMC::Command::traj_set_rapid_scale: {
        auto c = msg->command_as_traj_set_rapid_scale();
        EMC_TRAJ_SET_RAPID_SCALE m;
        m.scale = c->scale();
        store_command(q, m);
        return true;
    }
    case EMC::Command::traj_set_max_velocity: {
        auto c = msg->command_as_traj_set_max_velocity();
        EMC_TRAJ_SET_MAX_VELOCITY m;
        m.velocity = c->velocity();
        store_command(q, m);
        return true;
    }
    case EMC::Command::traj_set_teleop_enable: {
        auto c = msg->command_as_traj_set_teleop_enable();
        EMC_TRAJ_SET_TELEOP_ENABLE m;
        m.enable = c->enable();
        store_command(q, m);
        return true;
    }
    case EMC::Command::spindle_on: {
        auto c = msg->command_as_spindle_on();
        EMC_SPINDLE_ON m;
        m.spindle = c->spindle();
        m.speed = c->speed();
        m.factor = c->factor();
        m.xoffset = c->xoffset();
        m.wait_for_spindle_at_speed = c->wait_for_spindle_at_speed();
        store_command(q, m);
        return true;
    }
    case EMC::Command::spindle_off: {
        auto c = msg->command_as_spindle_off();
        EMC_SPINDLE_OFF m;
        m.spindle = c->spindle();
        store_command(q, m);
        return true;
    }
    case EMC::Command::coolant_mist_on:  { EMC_COOLANT_MIST_ON  m; store_command(q, m); return true; }
    case EMC::Command::coolant_mist_off: { EMC_COOLANT_MIST_OFF m; store_command(q, m); return true; }
    case EMC::Command::coolant_flood_on: { EMC_COOLANT_FLOOD_ON m; store_command(q, m); return true; }
    case EMC::Command::coolant_flood_off:{ EMC_COOLANT_FLOOD_OFF m; store_command(q, m); return true; }

    case EMC::Command::set_debug: {
        auto c = msg->command_as_set_debug();
        EMC_SET_DEBUG m;
        m.debug = static_cast<unsigned>(c->debug_level());
        store_command(q, m);
        return true;
    }
    case EMC::Command::spindle_increase: {
        auto c = msg->command_as_spindle_increase();
        EMC_SPINDLE_INCREASE m;
        m.spindle = c->spindle();
        m.speed = c->speed();
        store_command(q, m);
        return true;
    }
    case EMC::Command::spindle_decrease: {
        auto c = msg->command_as_spindle_decrease();
        EMC_SPINDLE_DECREASE m;
        m.spindle = c->spindle();
        m.speed = c->speed();
        store_command(q, m);
        return true;
    }
    case EMC::Command::spindle_constant: {
        auto c = msg->command_as_spindle_constant();
        EMC_SPINDLE_CONSTANT m;
        m.spindle = c->spindle();
        m.speed = c->speed();
        store_command(q, m);
        return true;
    }
    case EMC::Command::spindle_brake_engage: {
        auto c = msg->command_as_spindle_brake_engage();
        EMC_SPINDLE_BRAKE_ENGAGE m;
        m.spindle = c->spindle();
        store_command(q, m);
        return true;
    }
    case EMC::Command::spindle_brake_release: {
        auto c = msg->command_as_spindle_brake_release();
        EMC_SPINDLE_BRAKE_RELEASE m;
        m.spindle = c->spindle();
        store_command(q, m);
        return true;
    }
    /* The schema spells this one traj_set_spindl_scale. */
    case EMC::Command::traj_set_spindl_scale: {
        auto c = msg->command_as_traj_set_spindl_scale();
        EMC_TRAJ_SET_SPINDLE_SCALE m;
        m.spindle = c->spindle();
        m.scale = c->scale();
        store_command(q, m);
        return true;
    }
    case EMC::Command::traj_clear_probe_tripped_flag: {
        EMC_TRAJ_CLEAR_PROBE_TRIPPED_FLAG m;
        store_command(q, m);
        return true;
    }
    case EMC::Command::traj_probe: {
        auto c = msg->command_as_traj_probe();
        EMC_TRAJ_PROBE m;
        to_emcpose(c->pos(), m.pos);
        m.type = c->type();
        m.vel = c->vel();
        m.ini_maxvel = c->ini_maxvel();
        m.acc = c->acc();
        m.probe_type = c->probe_type();
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_set_optional_stop: {
        auto c = msg->command_as_task_plan_set_optional_stop();
        EMC_TASK_PLAN_SET_OPTIONAL_STOP m;
        m.state = c->state();
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_set_block_delete: {
        auto c = msg->command_as_task_plan_set_block_delete();
        EMC_TASK_PLAN_SET_BLOCK_DELETE m;
        m.state = c->state();
        store_command(q, m);
        return true;
    }
    case EMC::Command::tool_load_tool_table: {
        auto c = msg->command_as_tool_load_tool_table();
        EMC_TOOL_LOAD_TOOL_TABLE m;
        if (c->file() != nullptr) {
            rtapi_strxcpy(m.file, c->file()->c_str());
        }
        store_command(q, m);
        return true;
    }
    case EMC::Command::tool_set_offset: {
        auto c = msg->command_as_tool_set_offset();
        EMC_TOOL_SET_OFFSET m;
        m.pocket = c->pocket();
        m.toolno = c->toolno();
        to_emcpose(c->offset(), m.offset);
        m.diameter = c->diameter();
        m.frontangle = c->frontangle();
        m.backangle = c->backangle();
        m.orientation = c->orientation();
        store_command(q, m);
        return true;
    }
    case EMC::Command::joint_set_backlash: {
        auto c = msg->command_as_joint_set_backlash();
        EMC_JOINT_SET_BACKLASH m;
        m.joint = c->joint();
        m.backlash = c->backlash();
        store_command(q, m);
        return true;
    }
    case EMC::Command::joint_load_comp: {
        auto c = msg->command_as_joint_load_comp();
        EMC_JOINT_LOAD_COMP m;
        if (c->file() != nullptr) {
            rtapi_strxcpy(m.file, c->file()->c_str());
        }
        m.type = c->type();
        store_command(q, m);
        return true;
    }

    case EMC::Command::joint_set_min_position_limit: {
        auto c = msg->command_as_joint_set_min_position_limit();
        EMC_JOINT_SET_MIN_POSITION_LIMIT m;
        m.joint = c->joint();
        m.limit = c->limit();
        store_command(q, m);
        return true;
    }
    case EMC::Command::joint_set_max_position_limit: {
        auto c = msg->command_as_joint_set_max_position_limit();
        EMC_JOINT_SET_MAX_POSITION_LIMIT m;
        m.joint = c->joint();
        m.limit = c->limit();
        store_command(q, m);
        return true;
    }
    case EMC::Command::traj_set_mode: {
        auto c = msg->command_as_traj_set_mode();
        EMC_TRAJ_SET_MODE m;
        m.mode = static_cast<EMC_TRAJ_MODE>(c->mode());
        store_command(q, m);
        return true;
    }
    case EMC::Command::traj_set_fo_enable: {
        auto c = msg->command_as_traj_set_fo_enable();
        EMC_TRAJ_SET_FO_ENABLE m;
        m.mode = static_cast<unsigned char>(c->mode());
        store_command(q, m);
        return true;
    }
    case EMC::Command::traj_set_so_enable: {
        auto c = msg->command_as_traj_set_so_enable();
        EMC_TRAJ_SET_SO_ENABLE m;
        m.spindle = c->spindle();
        m.mode = static_cast<unsigned char>(c->mode());
        store_command(q, m);
        return true;
    }
    case EMC::Command::traj_set_fh_enable: {
        auto c = msg->command_as_traj_set_fh_enable();
        EMC_TRAJ_SET_FH_ENABLE m;
        m.mode = static_cast<unsigned char>(c->mode());
        store_command(q, m);
        return true;
    }
    case EMC::Command::motion_adaptive: {
        auto c = msg->command_as_motion_adaptive();
        EMC_MOTION_ADAPTIVE m;
        m.status = static_cast<unsigned char>(c->status());
        store_command(q, m);
        return true;
    }
    case EMC::Command::motion_set_dout: {
        auto c = msg->command_as_motion_set_dout();
        EMC_MOTION_SET_DOUT m;
        m.index = static_cast<unsigned char>(c->index());
        m.start = static_cast<unsigned char>(c->start());
        m.end   = static_cast<unsigned char>(c->end());
        m.now   = static_cast<unsigned char>(c->now());
        store_command(q, m);
        return true;
    }
    case EMC::Command::motion_set_aout: {
        auto c = msg->command_as_motion_set_aout();
        EMC_MOTION_SET_AOUT m;
        m.index = static_cast<unsigned char>(c->index());
        m.start = c->start();
        m.end   = c->end();
        m.now   = static_cast<unsigned char>(c->now());
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_close: {
        EMC_TASK_PLAN_CLOSE m;
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_reverse: {
        EMC_TASK_PLAN_REVERSE m;
        store_command(q, m);
        return true;
    }
    case EMC::Command::task_plan_forward: {
        EMC_TASK_PLAN_FORWARD m;
        store_command(q, m);
        return true;
    }

    /* A UI asking task to put a message in front of every other UI.
       Task handles these as commands and relays them; they come back out
       through wsServerOperatorMsg() like any other operator message. */
    case EMC::Command::send_operator_error: {
        auto c = msg->command_as_send_operator_error();
        EMC_OPERATOR_ERROR m;
        if (c->error() != nullptr) rtapi_strxcpy(m.error, c->error()->c_str());
        store_command(q, m);
        return true;
    }
    case EMC::Command::send_operator_text: {
        auto c = msg->command_as_send_operator_text();
        EMC_OPERATOR_TEXT m;
        if (c->text() != nullptr) rtapi_strxcpy(m.text, c->text()->c_str());
        store_command(q, m);
        return true;
    }
    case EMC::Command::send_operator_display: {
        auto c = msg->command_as_send_operator_display();
        EMC_OPERATOR_DISPLAY m;
        if (c->display() != nullptr) rtapi_strxcpy(m.display, c->display()->c_str());
        store_command(q, m);
        return true;
    }

    default:
        return false;
    }
}

/* ------------------------------------------------------------------ */
/* text (debug) command  ->  FlatBuffer command                       */
/* ------------------------------------------------------------------ */

/* The test web UI speaks a one-line text form: "<name> [args...]".
   It is converted into the same CmdChannelMsg FlatBuffer the binary
   protocol uses, so both paths share decode_command() below it. */
bool text_to_flatbuffer(const std::string &line, flatbuffers::FlatBufferBuilder &fbb)
{
    std::string name;
    std::string rest;
    size_t sp = line.find(' ');
    if (sp == std::string::npos) {
        name = line;
    } else {
        name = line.substr(0, sp);
        rest = line.substr(sp + 1);
    }
    /* strip trailing whitespace/newlines */
    while (!rest.empty() && (rest.back() == '\n' || rest.back() == '\r' || rest.back() == ' '))
        rest.pop_back();

    auto argd = [&](int idx, double dflt) -> double {
        std::string r = rest;
        size_t pos = 0;
        for (int i = 0; i < idx; i++) {
            pos = r.find(' ', pos);
            if (pos == std::string::npos) return dflt;
            pos++;
        }
        try { return std::stod(r.substr(pos)); } catch (...) { return dflt; }
    };
    auto argi = [&](int idx, int dflt) -> int {
        return static_cast<int>(argd(idx, static_cast<double>(dflt)));
    };

    EMC::Command  utype = EMC::Command::NONE;
    flatbuffers::Offset<void> uval;

    if (name == "state") {
        EMC::TaskSetState s(argi(0, 1));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::task_set_state;
    } else if (name == "mode") {
        EMC::TaskSetMode s(argi(0, 1));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::task_set_mode;
    } else if (name == "abort") {
        EMC::TaskAbort s(0);
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::task_abort;
    } else if (name == "mdi") {
        uval = EMC::CreateTaskPlanExecuteDirect(fbb, rest.c_str()).Union();
        utype = EMC::Command::task_plan_execute;
    } else if (name == "open") {
        uval = EMC::CreateTaskPlanOpenDirect(fbb, rest.c_str()).Union();
        utype = EMC::Command::task_plan_open;
    } else if (name == "run") {
        EMC::TaskPlanRun s(argi(0, 0));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::task_plan_run;
    } else if (name == "pause") {
        EMC::TaskPlanPause s(0);
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::task_plan_pause;
    } else if (name == "resume") {
        EMC::TaskPlanResume s(0);
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::task_plan_resume;
    } else if (name == "step") {
        EMC::TaskPlanStep s(0);
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::task_plan_step;
    } else if (name == "home") {
        EMC::JointHome s(argi(0, -1));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::joint_home;
    } else if (name == "unhome") {
        EMC::JointUnhome s(argi(0, -1));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::joint_unhome;
    } else if (name == "jog_cont") {
        /* jog_cont <joint_or_axis> <vel> <jjogmode> */
        EMC::JogCont s(argi(0, 0), argd(1, 0.0), argi(2, 1));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::jog_cont;
    } else if (name == "jog_incr") {
        /* jog_incr <joint_or_axis> <incr> <vel> <jjogmode> */
        EMC::JogIncr s(argi(0, 0), argd(1, 0.0), argd(2, 0.0), argi(3, 1));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::jog_incr;
    } else if (name == "jog_stop") {
        EMC::JogStop s(argi(0, 0), argi(1, 1));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::jog_stop;
    } else if (name == "teleop") {
        EMC::TrajSetTeleopEnable s(argi(0, 1));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::traj_set_teleop_enable;
    } else if (name == "feedrate") {
        EMC::TrajSetScale s(argd(0, 1.0));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::traj_set_scale;
    } else if (name == "rapidrate") {
        EMC::TrajSetRapidScale s(argd(0, 1.0));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::traj_set_rapid_scale;
    } else if (name == "maxvel") {
        EMC::TrajSetMaxVelocity s(argd(0, 0.0));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::traj_set_max_velocity;
    } else if (name == "spindle_on") {
        /* spindle_on <spindle> <speed> */
        EMC::SpindleOn s(argi(0, 0), argd(1, 0.0), 0.0, 0.0, 0);
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::spindle_on;
    } else if (name == "spindle_off") {
        /* The schema's SpindleOff carries only the spindle number; NML's
           EMC_SPINDLE_OFF also has wait_for_spindle_at_speed (see notes.md). */
        EMC::SpindleOff s(argi(0, 0));
        uval = fbb.CreateStruct(s).Union();
        utype = EMC::Command::spindle_off;
    } else if (name == "mist_on")   { EMC::CoolantMistOn  s(0); uval = fbb.CreateStruct(s).Union(); utype = EMC::Command::coolant_mist_on;
    } else if (name == "mist_off")  { EMC::CoolantMistOff s(0); uval = fbb.CreateStruct(s).Union(); utype = EMC::Command::coolant_mist_off;
    } else if (name == "flood_on")  { EMC::CoolantFloodOn s(0); uval = fbb.CreateStruct(s).Union(); utype = EMC::Command::coolant_flood_on;
    } else if (name == "flood_off") { EMC::CoolantFloodOff s(0); uval = fbb.CreateStruct(s).Union(); utype = EMC::Command::coolant_flood_off;
    } else {
        return false;
    }

    EMC::CmdChannelMsgBuilder b(fbb);
    b.add_command_type(utype);
    b.add_command(uval);
    fbb.Finish(b.Finish());
    return true;
}

/* ------------------------------------------------------------------ */
/* the test web UI                                                    */
/* ------------------------------------------------------------------ */

/* The test web UI lives in share/linuxcnc/ws_ui.html rather than in this
   file, so it can be edited and reloaded in the browser without rebuilding
   task. It is read per request, which is fine for a page served to a handful
   of developers on loopback.

   LINUXCNC_WS_UI overrides the path, which is handy when working on the page
   from somewhere other than the installed tree. */
const char *WEB_UI_DEFAULT_PATH = EMC2_HOME "/share/linuxcnc/ws_ui.html";

std::string web_ui_path()
{
    const char *env = getenv("LINUXCNC_WS_UI");
    if (env != nullptr && *env != '\0') return std::string(env);
    return std::string(WEB_UI_DEFAULT_PATH);
}

bool read_web_ui(std::string &out, std::string &path)
{
    path = web_ui_path();
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return true;
}

/* ------------------------------------------------------------------ */
/* websocket session                                                  */
/* ------------------------------------------------------------------ */

class Session : public std::enable_shared_from_this<Session> {
public:
    explicit Session(tcp::socket &&socket)
        : ws_(std::move(socket)) {}

    void run(http::request<http::string_body> req)
    {
        json_ = req.target().find("format=json") != beast::string_view::npos;
        ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
        ws_.binary(!json_);
        auto self = shared_from_this();
        ws_.async_accept(req, [self](beast::error_code ec) {
            if (ec) return;
            {
                std::lock_guard<std::mutex> lk(g_state->sess_mu);
                g_state->sessions.insert(self);
            }
            self->send_current();
            self->do_read();
        });
    }

    bool wants_json() const { return json_; }

    /* Called on the io_context thread when a new snapshot is ready. */
    void on_snapshot()
    {
        send_current();
    }

    /* An acknowledgement or an operator message. Unlike a status snapshot
       these are events, so they are never dropped to let a later frame
       overtake them. */
    void on_message_frame(std::shared_ptr<const std::vector<uint8_t>> buf)
    {
        if (json_) return;          /* status only on the JSON channel */
        queue_frame(std::move(buf), /*is_status=*/false);
    }

    void close()
    {
        beast::error_code ec;
        ws_.next_layer().close(ec);
    }

private:
    struct Frame {
        std::shared_ptr<const std::vector<uint8_t>> bin;
        std::shared_ptr<const std::string>          txt;
        bool is_status;
    };

    void send_current()
    {
        std::shared_ptr<const std::vector<uint8_t>> bin;
        std::shared_ptr<const std::string> js;
        {
            std::lock_guard<std::mutex> lk(g_state->mu);
            bin = g_state->snapshot_bin;
            js  = g_state->snapshot_json;
        }
        if (json_) {
            if (!js) return;
            Frame f{nullptr, std::move(js), true};
            queue(std::move(f));
        } else {
            if (!bin) return;
            queue_frame(std::move(bin), /*is_status=*/true);
        }
    }

    void queue_frame(std::shared_ptr<const std::vector<uint8_t>> buf, bool is_status)
    {
        queue(Frame{std::move(buf), nullptr, is_status});
    }

    void queue(Frame f)
    {
        /* Status is a full snapshot, so a client that is behind should get
           the newest one rather than a backlog. Replacing only a status
           frame that is still last in the queue keeps acknowledgements and
           operator messages in the order task produced them, and keeps a
           status frame from overtaking the ack that refers to it. */
        if (f.is_status && !out_.empty() && out_.back().is_status) {
            out_.back() = std::move(f);
        } else {
            out_.push_back(std::move(f));
        }
        /* Acknowledgements and operator messages are never dropped in
           favour of a later frame, so a client that has stopped reading
           would grow this without bound. Well past any real backlog, take
           it to be gone and close it. */
        if (out_.size() > 1024) {
            drop("backlog", beast::error_code());
            return;
        }
        maybe_write();
    }

    void maybe_write()
    {
        if (writing_ || out_.empty()) return;
        writing_ = true;

        auto self = shared_from_this();
        Frame f = std::move(out_.front());
        out_.pop_front();

        auto done = [self](beast::error_code ec, std::size_t) {
            self->writing_ = false;
            if (ec) { self->drop("write", ec); return; }
            self->maybe_write();
        };

        if (f.txt) {
            ws_.text(true);
            auto buf = f.txt;
            ws_.async_write(asio::buffer(*buf),
                            [buf, done](beast::error_code ec, std::size_t n) { done(ec, n); });
        } else {
            ws_.binary(true);
            auto buf = f.bin;
            ws_.async_write(asio::buffer(*buf),
                            [buf, done](beast::error_code ec, std::size_t n) { done(ec, n); });
        }
    }

    void do_read()
    {
        auto self = shared_from_this();
        ws_.async_read(in_, [self](beast::error_code ec, std::size_t) {
            if (ec) { self->drop("read", ec); return; }
            self->on_message();
            self->in_.consume(self->in_.size());
            self->do_read();
        });
    }

    void on_message()
    {
        QueuedCommand q;
        bool ok = false;
        /* The client's own command counter. Zero means "not interested in
           the outcome", which is what the test UI's text commands send. */
        uint64_t serial = 0;

        if (ws_.got_binary()) {
            auto data = static_cast<const uint8_t *>(in_.data().data());
            size_t len = in_.size();
            flatbuffers::Verifier v(data, len);
            if (EMC::VerifyCmdChannelMsgBuffer(v)) {
                const EMC::CmdChannelMsg *m = EMC::GetCmdChannelMsg(data);
                serial = m->serial();
                ok = decode_command(m, q);
            }
        } else {
            std::string line = beast::buffers_to_string(in_.data());
            flatbuffers::FlatBufferBuilder fbb(512);
            if (text_to_flatbuffer(line, fbb)) {
                /* Same decode path as a binary frame. */
                ok = decode_command(EMC::GetCmdChannelMsg(fbb.GetBufferPointer()), q);
            }
        }

        if (!ok) {
            refuse(serial);
            return;
        }
        /* Into the same queue halui uses; task judges it exactly like a
           command that arrived over NML. */
        taskcmd::Ticket ticket = taskcmd::pushRaw(q.storage, q.size, 0);
        if (ticket == 0) {
            refuse(serial);
            return;
        }
        if (serial != 0) {
            std::lock_guard<std::mutex> lk(g_state->cmd_mu);
            g_state->cmd_owner[ticket] = CmdOwner{weak_from_this(), serial};
        }
    }

    /* Task never saw the command: it could not be decoded, or the queue was
       full. Say so at once rather than leave the client waiting. */
    void refuse(uint64_t serial)
    {
        if (serial == 0) return;
        on_message_frame(encode_ack(serial, EMC::CmdState::refused));
    }

    void drop(const char *what, beast::error_code ec)
    {
        if (!ec) {
            /* Our decision, not the socket's: nothing failed, so nothing
               will complete and close it for us. */
            fmt::print(stderr, "ws_server: dropping a client that stopped "
                               "reading ({})\n", what);
            close();
        } else if (ec != websocket::error::closed &&
                   ec != asio::error::eof &&
                   ec != asio::error::operation_aborted &&
                   ec != asio::error::connection_reset) {
            /* A UI going away is normal, however it goes away. Anything
               else is worth a line, because it means a client lost status
               or an acknowledgement it was waiting for. */
            fmt::print(stderr, "ws_server: session {} error: {}\n", what, ec.message());
        }

        auto self = shared_from_this();
        std::lock_guard<std::mutex> lk(g_state->sess_mu);
        g_state->sessions.erase(self);
    }

    websocket::stream<tcp::socket> ws_;
    beast::flat_buffer in_;
    std::deque<Frame> out_;
    bool writing_ = false;
    bool json_ = false;
};

/* ------------------------------------------------------------------ */
/* http: serve the test UI, or upgrade to websocket                   */
/* ------------------------------------------------------------------ */

class HttpSession : public std::enable_shared_from_this<HttpSession> {
public:
    explicit HttpSession(tcp::socket &&socket) : stream_(std::move(socket)) {}

    void run()
    {
        auto self = shared_from_this();
        http::async_read(stream_, buf_, req_,
            [self](beast::error_code ec, std::size_t) {
                if (ec) return;
                self->handle();
            });
    }

private:
    void handle()
    {
        if (websocket::is_upgrade(req_)) {
            std::make_shared<Session>(stream_.release_socket())->run(req_);
            return;
        }
        auto res = std::make_shared<http::response<http::string_body>>();
        res->version(req_.version());
        res->keep_alive(false);
        if (req_.target() == "/" || req_.target().starts_with("/index")) {
            std::string body, path;
            if (read_web_ui(body, path)) {
                res->result(http::status::ok);
                res->set(http::field::content_type, "text/html; charset=utf-8");
                res->body() = std::move(body);
            } else {
                res->result(http::status::internal_server_error);
                res->set(http::field::content_type, "text/plain");
                res->body() = "cannot read the test web UI from " + path +
                              "\nset LINUXCNC_WS_UI to override the path\n";
            }
        } else {
            res->result(http::status::not_found);
            res->set(http::field::content_type, "text/plain");
            res->body() = "not found\n";
        }
        res->prepare_payload();
        auto self = shared_from_this();
        http::async_write(stream_, *res,
            [self, res](beast::error_code, std::size_t) {
                beast::error_code ec;
                self->stream_.socket().shutdown(tcp::socket::shutdown_send, ec);
            });
    }

    beast::tcp_stream stream_;
    beast::flat_buffer buf_;
    http::request<http::string_body> req_;
};

class Listener : public std::enable_shared_from_this<Listener> {
public:
    Listener(asio::io_context &ioc, tcp::endpoint ep)
        : ioc_(ioc), acceptor_(ioc)
    {
        beast::error_code ec;
        acceptor_.open(ep.protocol(), ec);
        if (ec) return;
        acceptor_.set_option(asio::socket_base::reuse_address(true), ec);
        acceptor_.bind(ep, ec);
        if (ec) { failed_ = true; return; }
        acceptor_.listen(asio::socket_base::max_listen_connections, ec);
        if (ec) failed_ = true;
    }

    bool failed() const { return failed_; }

    void run() { accept(); }

    void stop()
    {
        beast::error_code ec;
        acceptor_.close(ec);
    }

private:
    void accept()
    {
        auto self = shared_from_this();
        acceptor_.async_accept(asio::make_strand(ioc_),
            [self](beast::error_code ec, tcp::socket socket) {
                if (ec) return;   /* acceptor closed on shutdown */
                std::make_shared<HttpSession>(std::move(socket))->run();
                self->accept();
            });
    }

    asio::io_context &ioc_;
    tcp::acceptor acceptor_;
    bool failed_ = false;
};

std::shared_ptr<Listener> g_listener;

/* Runs on the server thread: encode the pending EMC_STAT and hand it out,
   then deliver everything task produced alongside it. */
void serialize_and_fanout()
{
    /* Cleared before the snapshot is taken, so a publish arriving while this
       one is still encoding queues a fresh handler and the final frame is
       never lost. */
    g_state->fanout_pending.store(false);

    std::vector<char>        raw;
    std::deque<OperatorMsg>  errors;
    {
        std::lock_guard<std::mutex> lk(g_state->mu);
        errors.swap(g_state->pending_errors);
        if (!g_state->pending_valid) {
            raw.clear();
        } else {
            raw = g_state->pending_stat;
            g_state->pending_valid = false;
        }
    }

    std::set<std::shared_ptr<Session>> sessions;
    {
        std::lock_guard<std::mutex> lk(g_state->sess_mu);
        sessions = g_state->sessions;
    }

    bool want_bin = false, want_json = false;
    for (auto &sess : sessions) {
        if (sess->wants_json()) want_json = true;
        else                    want_bin = true;
    }

    if (!raw.empty()) {
        const EMC_STAT *st = reinterpret_cast<const EMC_STAT *>(raw.data());

        /* Encoding benchmark only: LINUXCNC_WS_CAPTURE=path appends every
           encoded EmcStat as a 4-byte length plus the buffer. */
        static FILE *cap = nullptr;
        static bool cap_init = false;
        static long cap_left = 0;
        if (!cap_init) {
            cap_init = true;
            if (const char *p = getenv("LINUXCNC_WS_CAPTURE")) {
                cap = fopen(p, "wb");
                const char *m = getenv("LINUXCNC_WS_CAPTURE_MAX");
                cap_left = m ? atol(m) : 50000;
            }
        }
        if (cap && cap_left > 0) {
            std::vector<uint8_t> capbuf;
            encode_status(*st, capbuf);
            uint32_t n = capbuf.size();
            fwrite(&n, 4, 1, cap);
            fwrite(capbuf.data(), 1, n, cap);
            if (--cap_left == 0) { fclose(cap); cap = nullptr; }
        }

        /* Encoding benchmark only: LINUXCNC_WS_TIMING=1 times the real
           FlatBuffers builder, the reflection JSON and a JSON delta from
           EMC_STAT on every frame and prints averages to stderr.
           LINUXCNC_WS_TIMING_DUMP=path keeps snapshots and deltas of the
           first frames for an offline replay check. */
        static int tim = -1;
        static FILE *tdump = nullptr;
        static std::vector<char> tprev;
        static std::string tdelta;
        static double t_fbs, t_refl, t_delta, b_fbs, b_refl, b_delta;
        static long t_n, t_dumped;
        if (tim < 0) {
            tim = getenv("LINUXCNC_WS_TIMING") != nullptr;
            if (const char *p = getenv("LINUXCNC_WS_TIMING_DUMP")) tdump = fopen(p, "w");
        }
        if (tim) {
            using tclk = std::chrono::steady_clock;
            auto us = [](tclk::duration d) { return std::chrono::duration<double, std::micro>(d).count(); };
            auto t0 = tclk::now();
            auto fb = encode_status_msg(*st);
            auto t1 = tclk::now();
            std::vector<uint8_t> rbuf;
            encode_status(*st, rbuf);
            flatbuffers::ToStringVisitor rv(" ", true, "");
            flatbuffers::IterateFlatBuffer(rbuf.data(), EMC::EmcStatTypeTable(), &rv);
            auto t2 = tclk::now();
            bool have_prev = tprev.size() == raw.size();
            if (have_prev) {
                jdelta::delta(tdelta, *reinterpret_cast<const EMC_STAT *>(tprev.data()), *st);
            }
            tprev = raw;
            auto t3 = tclk::now();
            if (have_prev) {
                t_fbs += us(t1 - t0); t_refl += us(t2 - t1); t_delta += us(t3 - t2);
                b_fbs += fb->size(); b_refl += rv.s.size(); b_delta += tdelta.size();
                if (++t_n % 10000 == 0) {
                    fprintf(stderr, "WS_TIMING n=%ld us: fbs %.2f refl_json %.2f json_delta %.2f | bytes: fbs %.0f refl_json %.0f json_delta %.0f\n",
                            t_n, t_fbs / t_n, t_refl / t_n, t_delta / t_n, b_fbs / t_n, b_refl / t_n, b_delta / t_n);
                }
            }
            if (tdump && t_dumped < 5000) {
                if (have_prev) fprintf(tdump, "D %s\n", tdelta.c_str());
                if (t_dumped % 1000 == 0) fprintf(tdump, "S %s\n", rv.s.c_str());
                if (++t_dumped == 5000) { fclose(tdump); tdump = nullptr; }
            }
        }

        /* Encode only what somebody is listening for. With no client
           attached -- the usual case -- this costs nothing beyond the
           memcpy the task loop already did. */
        std::shared_ptr<const std::vector<uint8_t>> bin;
        std::shared_ptr<const std::string>          js;

        if (want_bin) {
            bin = encode_status_msg(*st);
        }
        if (want_json) {
            /* The JSON the test UI sees is rendered from an encoded status
               buffer via the flatc reflection tables, so it can only show
               what the binary protocol actually carries.

               FlatBufferToString() would emit FlatBuffers text format, whose
               keys are unquoted -- close to JSON but not valid JSON, so
               JSON.parse() rejects it. Drive the visitor directly with
               quoting enabled instead. */
            std::vector<uint8_t> statbuf;
            encode_status(*st, statbuf);
            flatbuffers::ToStringVisitor visitor(" ", /*quotes=*/true, /*indent=*/"");
            flatbuffers::IterateFlatBuffer(statbuf.data(), EMC::EmcStatTypeTable(), &visitor);
            js = std::make_shared<std::string>(std::move(visitor.s));
        }

        {
            /* Dropping the snapshot when nothing wants it matters: a
               session that connects later is given the stored snapshot
               straight away, and handing it a stale one would be worse
               than handing it nothing. It waits one task cycle instead. */
            std::lock_guard<std::mutex> lk(g_state->mu);
            g_state->snapshot_bin  = bin;
            g_state->snapshot_json = js;
        }

        for (auto &sess : sessions) sess->on_snapshot();
    }

    /* Operator messages go to every binary client. NML's error channel is a
       read-once queue, so with two UIs attached whichever read first took
       the message and the other never saw it; here everyone gets it. */
    for (const OperatorMsg &m : errors) {
        if (!want_bin) break;
        auto frame = encode_operator_msg(m.kind, m.text);
        for (auto &sess : sessions) sess->on_message_frame(frame);
    }

    /* Command acknowledgements go only to the session that submitted the
       command. A ticket with no owner is halui's, and is simply dropped. */
    std::vector<taskcmd::Event> events;
    taskcmd::drainEvents(events);
    for (const taskcmd::Event &ev : events) {
        const bool terminal = ev.state != taskcmd::State::received;

        CmdOwner owner;
        {
            std::lock_guard<std::mutex> lk(g_state->cmd_mu);
            auto it = g_state->cmd_owner.find(ev.ticket);
            if (it == g_state->cmd_owner.end()) continue;
            owner = it->second;
            if (terminal) g_state->cmd_owner.erase(it);
        }

        EMC::CmdState state = EMC::CmdState::received;
        if (ev.state == taskcmd::State::done)  state = EMC::CmdState::done;
        if (ev.state == taskcmd::State::error) state = EMC::CmdState::error;

        if (auto sess = owner.session.lock()) {
            sess->on_message_frame(encode_ack(owner.serial, state));
        }
    }
}

} /* anonymous namespace */

/* ------------------------------------------------------------------ */
/* public interface                                                   */
/* ------------------------------------------------------------------ */

int wsServerStart(int port)
{
    if (port <= 0) return 0;          /* disabled */
    if (g_state != nullptr) return 0; /* already running */

    g_state = new ServerState();
    g_state->pending_stat.resize(sizeof(EMC_STAT));

    tcp::endpoint ep(asio::ip::make_address("127.0.0.1"),
                     static_cast<unsigned short>(port));
    g_listener = std::make_shared<Listener>(g_state->ioc, ep);
    if (g_listener->failed()) {
        fmt::print(stderr, "ws_server: cannot listen on 127.0.0.1:{}\n", port);
        g_listener.reset();
        delete g_state;
        g_state = nullptr;
        return -1;
    }
    g_listener->run();

    g_state->running = true;
    g_state->thread = std::thread([]() {
        auto guard = asio::make_work_guard(g_state->ioc);
        g_state->ioc.run();
    });

    fmt::print("ws_server: listening on http://127.0.0.1:{}/\n", port);
    return 0;
}

void wsServerStop(void)
{
    if (g_state == nullptr) return;

    if (g_listener) g_listener->stop();
    {
        std::set<std::shared_ptr<Session>> sessions;
        {
            std::lock_guard<std::mutex> lk(g_state->sess_mu);
            sessions = g_state->sessions;
            g_state->sessions.clear();
        }
        for (auto &s : sessions) s->close();
    }
    g_state->ioc.stop();
    if (g_state->thread.joinable()) g_state->thread.join();

    g_listener.reset();
    delete g_state;
    g_state = nullptr;
}

void wsServerOperatorMsg(int kind, const char *text)
{
    if (g_state == nullptr || text == nullptr || *text == '\0') return;

    /* Called from the task thread, from emcOperatorError() and friends.
       Queue only; the encoding and the fan-out happen on the server thread
       with the next status publish, which is the very next task cycle. */
    std::lock_guard<std::mutex> lk(g_state->mu);
    if (g_state->pending_errors.size() >= 256) {
        g_state->pending_errors.pop_front();
    }
    g_state->pending_errors.push_back(OperatorMsg{kind, std::string(text)});
}

void wsServerPublish(const EMC_STAT *status)
{
    if (g_state == nullptr || status == nullptr) return;

    /* Task-loop cost is one memcpy under a short lock. Encoding happens on
       the server thread. EMC_STAT is memcpy'd, not copy-assigned, because
       EMC_TOOL_STAT deletes its copy constructor. */
    {
        std::lock_guard<std::mutex> lk(g_state->mu);
        std::memcpy(g_state->pending_stat.data(), status, sizeof(EMC_STAT));
        g_state->pending_valid = true;
    }
    /* Only queue a fan-out when one is not already waiting to run. The newest
       snapshot is the only one that matters, so a server thread that falls
       behind drops intermediate frames instead of accumulating handlers. */
    if (!g_state->fanout_pending.exchange(true))
        asio::post(g_state->ioc, []() { serialize_and_fanout(); });
}
