/********************************************************************
* Description: tpn_run.c
*   Per cycle controller of the tpnext trajectory planner: picks the
*   largest jerk that keeps every speed cap and stop in the braking
*   horizon reachable, and lands stops on a cycle boundary.
*
* License: GPL Version 2
* System: Linux
********************************************************************/
#include <rtapi.h>
#include <rtapi_math.h>
#include <motion_types.h>
#include "../motion/motion.h"
#include "../tp/tp.h"
#include "tpn.h"

#define TPN_MAXCON 128
/* poles of the spindle tracking, rad/s; beyond TPN_SYNC_LAND of error
 * it also keeps the reference reachable */
#define TPN_SYNC_W 80.0
#define TPN_SYNC_LAND 0.02
/* jerks tried across the range when the lowest misses */
#define TPN_SCAN 16
/* halvings of the jerk range: a hair of the limit is not worth more */
#define TPN_BISECT 18
/* landing plans tried beyond the shortest, in steps */
#define TPN_LAND_EXTRA 4

/* deadbeat stop sequence in progress */
#define TPN_DB_MAX 40
static double db_seq[TPN_DB_MAX];
static int db_n, db_i;
static double db_S;

/* ------------------------------------------------------------ constraints */

typedef struct {
    double S;       /* where the constraint starts */
    double Vh;      /* hard: speed cap or envelope at S */
    double Vs;      /* soft: requested speed at S, < 0 for none */
    double Aentry;  /* tangential acceleration limit from S on */
    double Arun;    /* smallest acceleration limit on the way to S */
    double Jrun;    /* smallest jerk limit on the way to S */
    double Jentry;  /* jerk limit from S on */
    double ae;      /* largest acceleration either way S may be met with */
    int own;        /* carried by the envelopes of the constraints before */
    tpn_ramp r;     /* the ramp its envelope comes from, r.J = 0: none */
} tpn_con;

static tpn_con con[TPN_MAXCON];
static int ncon;
/* the lowest jerk limit the constraints gathered reach */
static double jfloor;
/* the ramps of the pieces the motion has entered that reach on past
 * it: a motion following one of them may cross the boundaries ahead
 * faster than their envelopes, still braking. Those of the move it is
 * in are read again every cycle, as the planner changes them while the
 * move is near the end of the queue; those of the moves it has left
 * stay as they were last. */
#define TPN_NACT 16
static tpn_ramp act[TPN_NACT];
static int nact, nact_old;
static int act_id = -1;

static void actAdd(tpn_ramp const *r)
{
    int k, w = nact_old;
    if (nact_old >= TPN_NACT) {
        return;
    }
    for (k = 0; k < nact; k++) {
        if (act[k].S == r->S && act[k].E == r->E && act[k].J == r->J) {
            return;
        }
        if (k >= nact_old && act[k].S < act[w].S) {
            w = k;
        }
    }
    if (nact < TPN_NACT) {
        w = nact++;
    }
    act[w] = *r;
}

/* a new cycle in the move with id: keep the ramps of the moves left,
 * drop those passed */
static void actStart(int id, double s)
{
    int k;
    if (id != act_id) {
        nact_old = nact;
        act_id = id;
    }
    nact = nact_old;
    for (k = 0; k < nact; k++) {
        if (act[k].S <= s) {
            act[k--] = act[--nact];
        }
    }
    nact_old = nact;
}
/* with those further on that only humpShort() looks at */
static int ncon_hump;
static double g_dt = 0.001;
/* the controller's position along its direction of travel: s, or -s in a
 * reverse run */
static double cx;

/* Hard feasibility of a single constraint at distance d with the brake
 * to its cap; d < 0 means the constraint starts inside this step. */
static int conOk(double v1, double a1, double d, double V, double A, double J)
{
    double bd = tpnBrakeDist(v1, a1, V, A, J);
    return bd <= tpnMax(d, 0.0) + 1e-12;
}

/* Shortest distance in which (v0, a0) gets down to vt, still braking if
 * need be: the deceleration ramps to A and holds, and is let go only
 * past vt. Zero if the speed never exceeds vt. */
static double reachDist(double v0, double a0, double vt, double A, double J)
{
    double d = 0.0;
    if (vt < 0.0) {
        vt = 0.0;
    }
    if (a0 >= 0.0) {
        double t1 = a0 / J;
        double v1 = v0 + 0.5 * a0 * t1;
        if (v1 <= vt) {
            return 0.0;
        }
        d = v0 * t1 + 0.5 * a0 * t1 * t1 - J * t1 * t1 * t1 / 6.0;
        v0 = v1;
        a0 = 0.0;
    } else if (v0 <= vt) {
        return 0.0;
    }
    /* from (vv, 0) the deceleration ramps down through a0 */
    double Aeff = tpnMax(A, -a0);
    double t0 = -a0 / J;
    double vv = v0 + 0.5 * a0 * a0 / J;
    double dpre = vv * t0 - J * t0 * t0 * t0 / 6.0;
    double dv = vv - vt, dr;
    if (dv <= 0.5 * Aeff * Aeff / J) {
        double t = sqrt(2.0 * dv / J);
        dr = vv * t - J * t * t * t / 6.0;
    } else {
        double t2 = Aeff / J;
        double v2 = vv - 0.5 * Aeff * t2;
        double t3 = (v2 - vt) / Aeff;
        dr = vv * t2 - J * t2 * t2 * t2 / 6.0 + v2 * t3 - 0.5 * Aeff * t3 * t3;
    }
    return d + tpnMax(0.0, dr - dpre);
}

/* Hard feasibility of a cap V at distance d, met with at most ae of
 * deceleration: the brake under A and J of the pieces the step is in
 * meets V with none left, or passes V with ae and goes on to
 * V - ae^2 / 2J with none left; a state
 * already below V only ramps its deceleration down to ae, and keeps above
 * the speed the piece after S, with jerk limit Ja, ramps it out from.
 * A cycle early, as the step entering that piece runs under its limits. */
static int conArriveOk(double v1, double a1, double d, double V, double A, double J,
        double ae, double Aentry, double Ja, double W, double Vn, double Ln)
{
    double tol = J * g_dt * g_dt;
    /* the step onto S, at about V */
    double dd = tpnMax(d - V * g_dt, 0.0);
    int k;
    if (a1 > 0.0) {
        /* ramp the acceleration out: if it is still speeding up at S, it
         * ramps the rest out under Ja there, and no faster, below the caps
         * W that the ramp reaches */
        double Jr = J;
        double tr = a1 / Jr;
        if (v1 * tr + 0.5 * a1 * tr * tr - Jr * tr * tr * tr / 6.0 > dd) {
            double t = v1 > 0.0 ? tpnMin(dd / v1, tr) : tr;
            for (k = 0; k < 4 && t > 0.0; k++) {
                double f = v1 * t + 0.5 * a1 * t * t - Jr * t * t * t / 6.0 - dd;
                double df = v1 + a1 * t - 0.5 * Jr * t * t;
                if (df <= 0.0) {
                    break;
                }
                t = tpnMax(0.0, tpnMin(tr, t - f / df));
            }
            double aS = a1 - Jr * t, vS = v1 + a1 * t - 0.5 * Jr * t * t;
            /* the step onto S under Ja */
            double tl = tpnMin(g_dt, aS / Ja);
            vS += aS * tl - 0.5 * Ja * tl * tl;
            aS -= Ja * tl;
            /* V holds at S with no acceleration; the ramp out ends
             * further on, where the envelope is down towards the next
             * cap Vn at Ln */
            double tz = aS / Ja, vz = vS + 0.5 * aS * aS / Ja;
            double rel = (vS * tz + 0.5 * aS * tz * tz - Ja * tz * tz * tz / 6.0);
            double Vz = Vn < V && Ln > 0.0 ? V - (V - Vn) * tpnMin(rel / Ln, 1.0) : V;
            return vz <= tpnMin(Vz, W) + tol && aS <= Aentry + 1e-9;
        }
    }
    /* the controller ramps a deceleration out at the full limit */
    double Jf = J / TPN_BRAKE_SCALE;
    if (a1 >= 0.0 ? v1 + 0.5 * a1 * a1 / J <= V : v1 <= V
            || v1 - 0.5 * a1 * a1 / Jf <= V - 0.5 * ae * ae / J) {
        if (a1 >= 0.0) {
            return 1;
        }
        /* ramp the deceleration out: where is it at S */
        double tr = -a1 / Jf, t = tr;
        if (v1 * tr + 0.5 * a1 * tr * tr + Jf * tr * tr * tr / 6.0 > dd) {
            t = v1 > 0.0 ? tpnMin(dd / v1, tr) : 0.0;
            for (k = 0; k < 4 && t > 0.0; k++) {
                double f = v1 * t + 0.5 * a1 * t * t + Jf * t * t * t / 6.0 - dd;
                double df = v1 + a1 * t + 0.5 * Jf * t * t;
                if (df <= 0.0) {
                    break;
                }
                t = tpnMax(0.0, tpnMin(tr, t - f / df));
            }
        }
        double aS = -(a1 + Jf * t), vS = v1 + a1 * t + 0.5 * Jf * t * t;
        if (aS > 0.0) {
            /* the step onto S under Ja */
            double tl = tpnMin(g_dt, aS / Ja);
            vS -= aS * tl - 0.5 * Ja * tl * tl;
            aS -= Ja * tl;
        }
        return vS <= V + tol && aS <= Aentry + 1e-9 && vS - 0.5 * aS * aS / Ja >= -tol;
    }
    /* down to V with no deceleration left, or past V with ae on the way
     * to V - ae^2 / 2J: either brake will do */
    if ((a1 >= 0.0 || v1 - 0.5 * a1 * a1 / J > V)
            && tpnBrakeDist(v1, a1, V, A, J) <= dd + 1e-12) {
        return 1;
    }
    double te = ae / J;
    double extra = V * te - 0.5 * ae * te * te + J * te * te * te / 6.0;
    return tpnBrakeDist(v1, a1, V - 0.5 * ae * ae / J, A, J) <= dd + extra + 1e-12;
}

/* The ramp an envelope comes from, from (v1, a1) at distance D from its
 * end: down to E there with no acceleration left under A and J, or below
 * E already with a deceleration J ramps out short of rest. */
static int rampOk(tpn_ramp const *r, double v1, double a1, double D, double A, double J)
{
    double E = r->E;
    double tol = J * g_dt * g_dt;
    /* done a cycle early, at the speed it ends with, the same for every
     * caller; a ramp to rest leaves the deadbeat finish room to land on
     * a cycle, as the stop check: the latest brake to rest, followed a
     * cycle at a time, may miss its peak deceleration by a cycle */
    double rest = tpnMax(D - v1 * g_dt - 4.0 * J * g_dt * g_dt * g_dt, 0.0);
#define EARLY(v) (E <= 0.0 ? rest : tpnMax(D - tpnMin(v, E) * g_dt, 0.0))
    if (-a1 > A / TPN_BRAKE_SCALE + 1e-9) {
        /* more deceleration than the pieces of the ramp take */
        return 0;
    }
    if (a1 >= 0.0) {
        /* released at once below E by the end, or braked to E there */
        double tr = a1 / J, vz = v1 + 0.5 * a1 * tr;
        if (vz <= E) {
            return v1 * tr + 0.5 * a1 * tr * tr - J * tr * tr * tr / 6.0 <= EARLY(vz) + 1e-12;
        }
        return tpnBrakeDist(v1, a1, E, A, J) <= EARLY(E) + 1e-12;
    }
    /* a deceleration ramped out at once short of rest, done by the end
     * below E, or the brake to E */
    double t = -a1 / J, vz = v1 + 0.5 * a1 * t;
    if (vz < -tol) {
        return 0;
    }
    if (vz <= E) {
        return v1 * t + 0.5 * a1 * t * t + J * t * t * t / 6.0 <= EARLY(tpnMax(vz, 0.0)) + 1e-12;
    }
    return tpnBrakeDist(v1, a1, E, A, J) <= EARLY(E) + 1e-12;
#undef EARLY
}

/* a speed that settles on a cap from below approaches it only slowly */
#define TPN_CAP_NEAR 0.98

/* requested speed of a piece, < 0 for none: a move synchronized to the
 * spindle position follows the spindle instead, unless an abort stops it.
 * Feed override and the max velocity slider apply here, every cycle. */
static double pieceSoft(TP_STRUCT const *tp, tpn_seg const *sg, int blend, double scale)
{
    double v = blend ? sg->vreq_bin : sg->vreq;
    double vls = blend ? sg->vlimit_bin : sg->vlimit_scale;
    if (sg->sync == TC_SYNC_POSITION && tpn.track) {
        return -1.0;
    }
    if (sg->sync == TC_SYNC_VELOCITY) {
        double speed = fabs(tpn.emcmotStatus->spindle_status[sg->spindle].spindleSpeedIn);
        v = speed * sg->uu_per_rev;
    }
    v *= scale;
    if (tp->vLimit > 0.0 && vls > 0.0) {
        v = tpnMin(v, tp->vLimit * vls);
    }
    return v;
}

typedef struct {
    double V, A, J;     /* caps of the pieces the step touches */
    double Jhi;         /* largest jerk that keeps the step out of pieces
                         * with a lower jerk limit, or leaves it in them
                         * at their limit */
    double Vs;          /* soft cap there, < 0 for none */
    double Sstop;       /* nearest stop ahead */
    double hump_t;      /* speed humps shorter than this are flattened */
} tpn_step;

static void addCon(double S, double Vh, double Vs, double Aentry, double Jentry,
        double Arun, double Jrun)
{
    if (ncon >= TPN_MAXCON) {
        return;
    }
    con[ncon].S = S;
    con[ncon].Vh = Vh;
    con[ncon].Vs = Vs;
    con[ncon].Aentry = Aentry;
    con[ncon].Jentry = Jentry;
    con[ncon].Arun = Arun;
    con[ncon].Jrun = Jrun;
    con[ncon].own = 1;
    con[ncon].r.J = 0.0;
    ncon++;
}

/* The acceleration each constraint may be met with, last first: no more
 * than its piece allows, nor than its piece ramps down to what the next
 * one may be met with while the motion crosses it at no more than its
 * cap. */
static void conBudget(void)
{
    int k;
    for (k = ncon - 1; k >= 0; k--) {
        tpn_con *c = &con[k];
        jfloor = tpnMin(jfloor, c->Jrun);
        double ae = c->Vh > 0.0 ? c->Aentry : 0.0;
        if (k + 1 < ncon && c->Vh > 0.0 && c->Jentry < TPN_BIG) {
            double t = (con[k + 1].S - c->S) / c->Vh;
            ae = tpnMin(ae, con[k + 1].ae + c->Jentry * TPN_BRAKE_SCALE * t);
        }
        c->ae = ae;
    }
}

static void const *rp_n, *ae_n;

static void stepInit(tpn_step *st)
{
    ncon = ncon_hump = 0;
    jfloor = TPN_BIG;
    rp_n = NULL;
    ae_n = NULL;
    st->V = TPN_BIG;
    st->A = TPN_BIG;
    st->J = TPN_BIG;
    st->Jhi = TPN_BIG;
    st->Vs = -1.0;
    st->Sstop = TPN_BIG;
    st->hump_t = 0.0;
}

/* beyond the braking distance of the fastest next state every
 * constraint can be met by stopping, with the cycle conArriveOk() brakes
 * early by to spare */
static double horizonOf(tpn_step const *st, double Arun, double Jrun, double dt)
{
    double vhi = tpn.cur_v + tpnMax(tpn.cur_a, 0.0) * dt + 0.5 * st->J * dt * dt;
    double ahi = tpnMax(tpn.cur_a, 0.0) + st->J * dt;
    return tpnBrakeDist(vhi, ahi, 0.0, TPN_BRAKE_SCALE * tpnMin(Arun, st->A),
            TPN_BRAKE_SCALE * tpnMin(Jrun, st->J))
        + 4.0 * vhi * dt + 8.0 * st->J * dt * dt * dt + 1e-6;
}

/* how much further than the braking horizon a speed hump short enough to
 * be flattened may reach */
static double humpReach(tpn_step const *st)
{
    if (st->hump_t <= 0.0) {
        return 0.0;
    }
    double vtop = st->Vs >= 0.0 ? tpnMin(st->Vs, st->V) : st->V;
    return vtop * st->hump_t + 2.0 * tpnRampDist(tpn.cur_v, vtop, st->A, st->J);
}

/* Collect the constraints ahead of cur_s up to the braking horizon of the
 * fastest state reachable this cycle, and on as far as a speed hump
 * reaches for humpShort(): beyond the horizon the controller would check
 * them with the lowest limits on the way, which the envelopes do better. */
static void gather(TP_STRUCT const *tp, double scale, int stepping, tpn_step *st)
{
    double dt = tp->cycleTime;
    double Arun = TPN_BIG, Jrun = TPN_BIG, Send = 0.0;
    int i, nrun = -1, want_one = 0;
    stepInit(st);
    if (tpn.q_len > 0) {
        actStart(seg(0)->id, tpn.cur_s);
        Send = tpnQueueEnd();
    }

    for (i = 0; i < tpn.q_len; i++) {
        tpn_seg *sg = seg(i);
        int piece;
        /* where the table fills up short of the horizon the envelopes of
         * the pieces in it carry the rest */
        if (nrun < 0 && ncon + tpnPieces(sg) + 3 > TPN_MAXCON) {
            break;
        }
        if (sg->stop_in && sg->S0 > tpn.cur_s) {
            addCon(sg->S0, 0.0, -1.0, TPN_BIG, TPN_BIG, Arun, Jrun);
            st->Sstop = tpnMin(st->Sstop, sg->S0);
        }
        /* the parts of the blend, then the interior */
        for (piece = 0; piece < tpnPieces(sg); piece++) {
            double Pa, Pb, E, soft;
            tpn_lim const *lim;
            tpn_lim const *hi;
            double E_hi;
            tpn_ramp const *rp = &sg->R[piece], *rph = &sg->R_hi[piece];
            tpn_ramp rl, rlh;
            if (!tpnPiece(sg, piece, &Pa, &Pb, &lim, &hi, &E, &E_hi)) {
                continue;
            }
            if (Pb <= tpn.cur_s && !(i == tpn.q_len - 1 && piece == tpnPieces(sg) - 1)) {
                continue;
            }
            soft = pieceSoft(tp, sg, piece < TPN_NSUB, scale);
            /* ramps to rest at the end of the queue, as it ends now, and
             * their envelopes where they are asked for below */
            int ask_hi = soft >= 0.0 && (soft > lim->V || (Pa <= tpn.cur_s && tpn.cur_v > lim->V));
            if (tpnRampToEnd(rp)) {
                tpnEndRamp(rp, 0, Send, &rl);
                if (Pa > tpn.cur_s || ask_hi) {
                    E = tpnEndAt(&rl, Pa);
                }
                rp = &rl;
            }
            if (tpnRampToEnd(rph)) {
                tpnEndRamp(rph, 1, Send, &rlh);
                if (ask_hi) {
                    E_hi = rp == &rl && rlh.A == rl.A && rlh.J == rl.J && rlh.V == rl.V
                        ? E : tpnEndAt(&rlh, Pa);
                }
                rph = &rlh;
            }
            /* the opened caps where the override asks for more than the
             * caps allow and they let the motion run faster; in the piece
             * the motion is in only once it is about to reach the caps,
             * since they leave less acceleration along the path, and for
             * as long as it runs faster than the caps allow */
            if (soft >= 0.0 && (Pa <= tpn.cur_s ? tpn.cur_v > lim->V
                        || (soft > lim->V && E_hi > E && tpn.cur_v
                            + 0.5 * tpn.cur_a * fabs(tpn.cur_a) / lim->J
                            >= TPN_CAP_NEAR * lim->V)
                    : soft > lim->V && E_hi > E)) {
                lim = hi;
                E = E_hi;
                rp = rph;
            }
            if (Pa <= tpn.cur_s && rp->J > 0.0 && rp->S > tpn.cur_s) {
                actAdd(rp);
            }
            if (Pa <= tpn.cur_s) {
                st->hump_t = tpnMax(st->hump_t, sg->hump_t);
                st->V = tpnMin(st->V, lim->V);
                st->A = tpnMin(st->A, lim->A);
                st->J = tpnMin(st->J, lim->J);
                if (soft >= 0.0) {
                    st->Vs = st->Vs < 0.0 ? soft : tpnMin(st->Vs, soft);
                }
            } else {
                /* a step that ends inside the piece runs part of the
                 * cycle there at the jerk it picks; jstar ends it on Pa */
                double jstar = (Pa - tpn.cur_s - tpn.cur_v * dt - 0.5 * tpn.cur_a * dt * dt)
                    * 6.0 / (dt * dt * dt);
                if (jstar < -lim->J) {
                    /* every jerk within the limits enters it */
                    st->J = tpnMin(st->J, lim->J);
                } else {
                    st->Jhi = tpnMin(st->Jhi, tpnMax(jstar, lim->J));
                }
                addCon(Pa, E, soft, lim->A, lim->J, Arun, Jrun);
                if (rp->J > 0.0 && rp->S > Pa) {
                    con[ncon - 1].r = *rp;
                }
            }
            Arun = tpnMin(Arun, lim->A);
            Jrun = tpnMin(Jrun, lim->J);
        }
        if (stepping && i == 0 && tpn.q_len > 1) {
            addCon(ownedEnd(sg), 0.0, -1.0, TPN_BIG, TPN_BIG, Arun, Jrun);
            con[ncon - 1].own = 0;
            st->Sstop = tpnMin(st->Sstop, ownedEnd(sg));
        }
        if (i == tpn.q_len - 1) {
            addCon(segEnd(sg), 0.0, -1.0, TPN_BIG, TPN_BIG, Arun, Jrun);
            st->Sstop = tpnMin(st->Sstop, segEnd(sg));
        }
        if (ncon >= TPN_MAXCON) {
            break;
        }
        if (want_one && ncon > 0) {
            nrun = ncon;
            break;
        }
        double past = ownedEnd(sg) - tpn.cur_s - horizonOf(st, Arun, Jrun, dt);
        if (past > 0.0) {
            if (ncon == 0) {
                /* the envelopes carry the rest only from a constraint
                 * on: the next move's */
                want_one = 1;
                continue;
            }
            if (nrun < 0) {
                nrun = ncon;
            }
            if (past > humpReach(st)) {
                break;
            }
        }
    }
    ncon_hump = ncon;
    if (nrun >= 0) {
        ncon = nrun;
    }
    conBudget();
}

/*
 * gather() for a reverse run, in x = -s: the pieces from the one the
 * motion is in back to the start of the oldest move kept that may be run
 * backwards. The planner's backward envelopes hold for the forward
 * direction only, so the envelope is built here over the constraints up
 * to the braking horizon; where the table fills up the motion stops.
 */
static void gatherReverse(TP_STRUCT const *tp, double scale, int stepping, tpn_step *st)
{
    double dt = tp->cycleTime;
    double s = tpn.cur_s;
    double Arun = TPN_BIG, Jrun = TPN_BIG;
    int i;
    stepInit(st);
    nact = nact_old = 0;
    act_id = -1;

    for (i = 0; ; i--) {
        tpn_seg *sg = seg(i);
        int piece;
        if (ncon + tpnPieces(sg) + 3 > TPN_MAXCON) {
            addCon(-ownedEnd(sg), 0.0, -1.0, TPN_BIG, TPN_BIG, Arun, Jrun);
            st->Sstop = tpnMin(st->Sstop, -ownedEnd(sg));
            break;
        }
        /* the interior, then the parts of the blend, last first */
        for (piece = tpnPieces(sg) - 1; piece >= 0; piece--) {
            double Pa, Pb, E, E_hi, soft;
            tpn_lim const *lim, *hi;
            if (!tpnPiece(sg, piece, &Pa, &Pb, &lim, &hi, &E, &E_hi) || Pa >= s) {
                continue;
            }
            soft = pieceSoft(tp, sg, piece < TPN_NSUB, scale);
            /* the opened caps as in gather() */
            if (soft >= 0.0 && hi->V > lim->V && (Pb >= s ? tpn.cur_v > lim->V
                        || (soft > lim->V && tpn.cur_v
                            + 0.5 * tpn.cur_a * fabs(tpn.cur_a) / lim->J
                            >= TPN_CAP_NEAR * lim->V)
                    : soft > lim->V)) {
                lim = hi;
            }
            if (Pb >= s) {
                st->V = tpnMin(st->V, lim->V);
                st->A = tpnMin(st->A, lim->A);
                st->J = tpnMin(st->J, lim->J);
                if (soft >= 0.0) {
                    st->Vs = st->Vs < 0.0 ? soft : tpnMin(st->Vs, soft);
                }
            } else {
                double jstar = (-Pb - cx - tpn.cur_v * dt - 0.5 * tpn.cur_a * dt * dt)
                    * 6.0 / (dt * dt * dt);
                if (jstar < -lim->J) {
                    st->J = tpnMin(st->J, lim->J);
                } else {
                    st->Jhi = tpnMin(st->Jhi, tpnMax(jstar, lim->J));
                }
                addCon(-Pb, lim->V, soft, lim->A, lim->J, Arun, Jrun);
            }
            Arun = tpnMin(Arun, lim->A);
            Jrun = tpnMin(Jrun, lim->J);
        }
        /* a corner without a blend, the end of the history, the start of
         * the move when stepping */
        int last = i == -tpn.h_len || !sg->rev_ok || !seg(i - 1)->rev_ok;
        if ((sg->stop_in && sg->S0 < s) || last || (stepping && i == 0 && ownedStart(sg) < s)) {
            addCon(-ownedStart(sg), 0.0, -1.0, TPN_BIG, TPN_BIG, Arun, Jrun);
            st->Sstop = tpnMin(st->Sstop, -ownedStart(sg));
        }
        if (last || s - ownedStart(sg) > horizonOf(st, Arun, Jrun, dt)) {
            break;
        }
    }
    /* the backward envelope of what was gathered, as the planner's
     * backwardPass() builds it forward */
    for (i = ncon - 2; i >= 0; i--) {
        if (con[i].Aentry < TPN_BIG) {
            con[i].Vh = tpnMin(con[i].Vh, con[i + 1].Vh + tpnRampReach(con[i + 1].Vh,
                        con[i].Aentry * TPN_BRAKE_SCALE, con[i].Jentry * TPN_BRAKE_SCALE,
                        con[i + 1].S - con[i].S));
        }
    }
    /* no envelope of the planner's here: every constraint is checked */
    for (i = 0; i < ncon; i++) {
        con[i].own = 0;
    }
    if (st->J >= TPN_BIG) {
        /* at the start of the move, which is the end of the reverse run:
         * the limits of the piece the move starts with */
        double Pa, Pb, E, E_hi;
        tpn_seg const *sg = seg(0);
        tpn_lim const *lim = &sg->lim_int, *hi;
        int k = sg->h_in > 0.0 ? 0 : TPN_NSUB;
        while (!tpnPiece(sg, k, &Pa, &Pb, &lim, &hi, &E, &E_hi) && k < tpnPieces(sg) - 1) {
            k++;
        }
        st->V = lim->V;
        st->A = lim->A;
        st->J = lim->J;
    }
}

typedef struct {
    double s1, v1, a1;
} tpn_next;

static void stepFrom(double s, double v, double a, double j, double dt, tpn_next *n)
{
    n->s1 = s + v * dt + 0.5 * a * dt * dt + j * dt * dt * dt / 6.0;
    n->v1 = v + a * dt + 0.5 * j * dt * dt;
    n->a1 = a + j * dt;
}

static void stepState(double j, double dt, tpn_next *n)
{
    stepFrom(cx, tpn.cur_v, tpn.cur_a, j, dt, n);
}

/* CHK_SPEED: the speed part of a hard constraint alone */
enum { CHK_HARD = 1, CHK_SOFT = 2, CHK_SPEED = 4 };

static unsigned char failmask[TPN_MAXCON];
/* the last constraint failing a check that a carrier does not spare: one
 * not own, or a soft one */
static int fail_last;

/* check constraint k (or the step caps for k < 0) against the next state */

/* An acceleration a at speed v, past the limit A of a ramp, still within
 * the limit Ain of the piece it is in, ramps down to A under J within
 * the distance L left to the next piece. */
static int accDownOk(double v, double a, double A, double Ain, double J, double L)
{
    if (a <= A + 1e-9) {
        return 1;
    }
    if (a > Ain + 1e-9) {
        return 0;
    }
    double t = (a - A) / J;
    return v * t + 0.5 * a * t * t - J * t * t * t / 6.0 <= L;
}

/* Arrival at the boundary S of own constraint c at distance d under A1
 * and J1, the limits of the way there: braked to its envelope Vh at S
 * with no acceleration left, or ramping the acceleration out across S
 * into a state from which the ramp of the envelope holds, under the
 * limits of that ramp past the piece at S, Ln long. */
static int landOk(double v1, double a1, double d, tpn_con const *c, double Ln,
        double A1, double J1)
{
    tpn_ramp const *r = &c->r;
    double tol = J1 * g_dt * g_dt;
    double dd = tpnMax(d - c->Vh * g_dt, 0.0);
    double L = r->S - c->S, rJ = tpnMin(r->J, c->Jentry * TPN_BRAKE_SCALE);
    double Jr = J1;
    double tr = fabs(a1) / Jr;
    double vz = v1 + 0.5 * a1 * tr;
    double Dr = v1 * tr + 0.5 * a1 * tr * tr - (a1 > 0.0 ? Jr : -Jr) * tr * tr * tr / 6.0;
    int k;
    if (a1 < 0.0 && vz < -tol) {
        return 0;
    }
    if (d <= 0.0) {
        /* the step crosses S: on the ramp from here; a speed cap with
         * no ramp is only landed on short of it */
        if (r->J <= 0.0) {
            return 0;
        }
        return (a1 <= 0.0 ? v1 : v1 + 0.5 * a1 * a1 / rJ) <= r->V + tol
            && -a1 <= r->A / TPN_BRAKE_SCALE + 1e-9
            && accDownOk(v1, a1, r->A / TPN_BRAKE_SCALE, c->Aentry, rJ, Ln + d)
            && rampOk(r, v1, a1, L + d, r->A, rJ);
    }
    if (vz <= c->Vh + tol || (a1 < 0.0 && v1 <= c->Vh)) {
        /* a cycle early at the speed it settles at */
        if (Dr <= tpnMax(d - tpnMin(tpnMax(vz, 0.0), c->Vh) * g_dt, 0.0)) {
            /* settled before S below the envelope */
            return vz <= c->Vh + tol || v1 <= c->Vh;
        }
        if (r->J <= 0.0) {
            return 0;
        }
        /* the state at S, still ramping out */
        double t = v1 > 0.0 ? tpnMin(d / v1, tr) : tr;
        double sj = a1 > 0.0 ? -Jr : Jr;
        for (k = 0; k < 6 && t > 0.0; k++) {
            double f = v1 * t + 0.5 * a1 * t * t + sj * t * t * t / 6.0 - tpnMax(d, 0.0);
            double df = v1 + a1 * t + 0.5 * sj * t * t;
            if (df <= 0.0) {
                break;
            }
            t = tpnMax(0.0, tpnMin(tr, t - f / df));
        }
        double vS = v1 + a1 * t + 0.5 * sj * t * t, aS = a1 + sj * t;
        if (-aS > r->A / TPN_BRAKE_SCALE + 1e-9
                || !accDownOk(vS, aS, r->A / TPN_BRAKE_SCALE, c->Aentry, rJ, Ln)) {
            return 0;
        }
        if ((aS > 0.0 ? vS + 0.5 * aS * aS / rJ : vS) > r->V + tol) {
            return 0;
        }
        return rampOk(r, vS, aS, L, r->A, rJ);
    }
    if (a1 < 0.0 && v1 - 0.5 * a1 * a1 / J1 <= c->Vh) {
        /* under J1 the brake would reach Vh still decelerating: ramp out
         * at once, then brake */
        return Dr + tpnBrakeDist(vz, 0.0, c->Vh, A1, J1) <= dd + 1e-12;
    }
    return tpnBrakeDist(v1, a1, c->Vh, A1, J1) <= dd + 1e-12;
}

/* the motion lands on the envelope of own constraint m, or on the ramp
 * it comes from */
static int landsOn(int m, tpn_next const *n, tpn_step const *st)
{
    tpn_con const *c = &con[m];
    double Ln = m + 1 < ncon ? con[m + 1].S - c->S : TPN_BIG;
    return c->Vh > 0.0 && landOk(n->v1, n->a1, c->S - n->s1, c, Ln,
            tpnMin(c->Arun, st->A) * TPN_BRAKE_SCALE, tpnMin(c->Jrun, st->J) * TPN_BRAKE_SCALE);
}

/* the motion follows the ramp of own constraint m from here */
static int followsRamp(int m, tpn_next const *n, tpn_step const *st)
{
    tpn_con const *c = &con[m];
    double d = c->S - n->s1;
    double J = tpnMin(c->Jrun, st->J) * TPN_BRAKE_SCALE, A = tpnMin(c->Arun, st->A) * TPN_BRAKE_SCALE;
    /* an acceleration the pieces of the ramp take, or released before */
    double tr = n->a1 / J;
    if (n->a1 > c->r.A / TPN_BRAKE_SCALE + 1e-9
            && n->v1 * tr + 0.5 * n->a1 * tr * tr - J * tr * tr * tr / 6.0 > d) {
        return 0;
    }
    double Jr = tpnMin(c->r.J, J), Ar = tpnMin(c->r.A, A);
    if ((n->a1 <= 0.0 ? n->v1 : n->v1 + 0.5 * n->a1 * n->a1 / Jr) > c->r.V) {
        return 0;
    }
    /* the pieces of a move often share one ramp: the last answer holds
     * for the same ramp under the same limits */
    static tpn_ramp fr_r;
    static double fr_v, fr_a, fr_s, fr_A, fr_J;
    static int fr_ok;
    if (n->v1 != fr_v || n->a1 != fr_a || n->s1 != fr_s || Ar != fr_A || Jr != fr_J
            || c->r.S != fr_r.S || c->r.E != fr_r.E || c->r.A != fr_r.A || c->r.J != fr_r.J) {
        fr_v = n->v1;
        fr_a = n->a1;
        fr_s = n->s1;
        fr_A = Ar;
        fr_J = Jr;
        fr_r = c->r;
        fr_ok = rampOk(&c->r, n->v1, n->a1, c->r.S - n->s1, Ar, Jr);
    }
    return fr_ok;
}

/* the motion follows the ramp of a piece it has entered */
static int followsAct(tpn_next const *n, tpn_step const *st)
{
    int m;
    for (m = 0; m < nact; m++) {
        tpn_ramp const *r = &act[m];
        double J = tpnMin(r->J, st->J * TPN_BRAKE_SCALE), A = tpnMin(r->A, st->A * TPN_BRAKE_SCALE);
        if (n->a1 <= r->A / TPN_BRAKE_SCALE + 1e-9
                && (n->a1 <= 0.0 ? n->v1 : n->v1 + 0.5 * n->a1 * n->a1 / J) <= r->V
                && rampOk(r, n->v1, n->a1, r->S - n->s1, A, J)) {
            return 1;
        }
    }
    return 0;
}

/* How far the next state can ramp its acceleration down by each
 * boundary: under the jerk limit of each piece on the way, for at least
 * the time it takes to get there at the fastest speed the ramp reaches.
 * Worked out once per state, and only as far as it is asked. */
static double ae_v1, ae_a1, ae_s1, ae_v;
static int ae_k;
static double ae_R[TPN_MAXCON];

/* the acceleration of the next state is down to the limit of the piece
 * of constraint k a cycle before the step that crosses into it */
static int entryOk(int k, tpn_next const *n, tpn_step const *st)
{
    tpn_con const *c = &con[k];
    double aa = n->a1;
    int i;
    if (aa <= c->Aentry + 1e-9) {
        return 1;
    }
    if (n != ae_n || n->v1 != ae_v1 || n->a1 != ae_a1 || n->s1 != ae_s1) {
        ae_n = n;
        ae_v1 = n->v1;
        ae_a1 = n->a1;
        ae_s1 = n->s1;
        ae_v = tpnMax(n->v1 + (n->a1 > 0.0 ? 0.5 * aa * aa / tpnMin(st->J, jfloor) : 0.0), 1e-9);
        ae_k = 0;
    }
    /* the piece before boundary i: the one the step runs in, then those
     * of the constraints */
    for (; ae_k <= k; ae_k++) {
        i = ae_k;
        double lo = i ? tpnMax(con[i - 1].S, n->s1) : n->s1;
        double J = i ? con[i - 1].Jentry : st->J;
        ae_R[i] = (i ? ae_R[i - 1] : 0.0) + J * tpnMax(con[i].S - lo, 0.0) / ae_v;
    }
    /* less what the last cycle of the way would add */
    double P = c->S - ae_v * g_dt, R = ae_R[k];
    for (i = k; i >= 0; i--) {
        double hi = con[i].S, lo = i ? tpnMax(con[i - 1].S, n->s1) : n->s1;
        if (hi <= P) {
            break;
        }
        if (hi > lo) {
            R -= (i ? con[i - 1].Jentry : st->J) * (hi - tpnMax(lo, P)) / ae_v;
        }
    }
    return aa - R <= c->Aentry + 1e-9;
}

/* the motion keeps under the envelope of own constraint m up to it,
 * ramping its acceleration out at once, and a deceleration it carries
 * past m is one the piece of m takes: a carrier further on may hold it */
static int passesUnder(int m, tpn_next const *n, tpn_step const *st)
{
    tpn_con const *c = &con[m];
    double J = tpnMin(c->Jrun, st->J) * TPN_BRAKE_SCALE;
    double vpk = n->a1 > 0.0 ? n->v1 + 0.5 * n->a1 * n->a1 / J : n->v1;
    return c->Vh > 0.0 && vpk <= c->Vh && -n->a1 <= c->Aentry + 1e-9
        && entryOk(m, n, st);
}

/* Whether own constraint k is carried for the next state: a ramp the
 * motion follows holds every cap it crosses and the envelope where it
 * ends everything after, and so does a landing on an envelope for the
 * constraints after it. Those the motion passes under on the way to
 * either hold as well. The nearest carrier carries the rest, so the
 * search runs once per state, and only as far as it is asked. */
static double cf_v, cf_a, cf_s;
static int cf_from, cf_next;
/* the first of the own constraints passed under since the last one that
 * is not, -1 for none */
static int cf_run;
/* the own constraint the motion lands on, -1 for none */
static int cf_land;

/* the motion ramps its acceleration out short of own constraint m, as
 * landOk() settles it */
static int settlesBefore(int m, tpn_next const *n, tpn_step const *st)
{
    tpn_con const *c = &con[m];
    double d = c->S - n->s1;
    double J = tpnMin(c->Jrun, st->J) * TPN_BRAKE_SCALE;
    double tr = fabs(n->a1) / J, vz = n->v1 + 0.5 * n->a1 * tr;
    double Dr = n->v1 * tr + 0.5 * n->a1 * tr * tr - (n->a1 > 0.0 ? J : -J) * tr * tr * tr / 6.0;
    return d > 0.0 && Dr <= tpnMax(d - tpnMin(tpnMax(vz, 0.0), c->Vh) * g_dt, 0.0);
}

/* own constraint m carries the rest: a ramp the motion follows, or an
 * envelope it lands on */
static int carrier(int m, tpn_next const *n, tpn_step const *st)
{
    if (con[m].r.J > 0.0 && followsRamp(m, n, st)) {
        cf_from = cf_run >= 0 ? cf_run : m;
        return 1;
    }
    if (landsOn(m, n, st)) {
        cf_from = cf_run >= 0 ? cf_run : m + 1;
        cf_land = m;
        return 1;
    }
    return 0;
}

/* A run passed under ends at m without a carrier there: the ones in it
 * that did not settle were not tried yet. Of those sharing one ramp, the
 * last is tried: the motion ramps its acceleration out under the jerk
 * of the pieces before it for longer, and that jerk is no lower than the
 * ramp's. */
static int runCarrier(int m, tpn_next const *n, tpn_step const *st)
{
    int i, last = -1;
    for (i = m - 1; cf_run >= 0 && i >= cf_run; i--) {
        tpn_ramp const *r = &con[i].r;
        if (!con[i].own) {
            continue;
        }
        if (last >= 0 && r->J > 0.0 && r->S == con[last].r.S && r->E == con[last].r.E
                && r->J == con[last].r.J && r->A == con[last].r.A) {
            continue;
        }
        if (carrier(i, n, st)) {
            return 1;
        }
        last = i;
    }
    return 0;
}

static int carried(int k, tpn_next const *n, tpn_step const *st)
{
    if (n != rp_n || n->v1 != cf_v || n->a1 != cf_a || n->s1 != cf_s) {
        rp_n = n;
        cf_v = n->v1;
        cf_a = n->a1;
        cf_s = n->s1;
        cf_from = followsAct(n, st) ? 0 : ncon;
        cf_next = 0;
        cf_run = -1;
        cf_land = -1;
    }
    while (cf_from > k && cf_next < ncon && (cf_next <= k || cf_run >= 0)) {
        int m = cf_next++;
        if (!con[m].own) {
            continue;
        }
        /* passed under short of where the acceleration settles: a
         * carrier there only matters if the run ends without one */
        int under = passesUnder(m, n, st);
        if (under && !settlesBefore(m, n, st)) {
            if (cf_run < 0) {
                cf_run = m;
            }
        } else if (carrier(m, n, st)) {
            break;
        } else if (under) {
            if (cf_run < 0) {
                cf_run = m;
            }
        } else if (runCarrier(m, n, st)) {
            break;
        } else {
            cf_run = -1;
        }
    }
    if (cf_from > k && cf_next >= ncon && cf_run >= 0) {
        runCarrier(ncon, n, st);
        cf_run = -1;
    }
    return k >= cf_from;
}

static int checkOne(int k, int what, tpn_next const *n, tpn_step const *st)
{
    if (k < 0) {
        double A = st->A * TPN_BRAKE_SCALE, J = st->J * TPN_BRAKE_SCALE;
        if (what == CHK_HARD || what == CHK_SPEED) {
            return conOk(n->v1, n->a1, 0.0, st->V, A, J);
        }
        return st->Vs < 0.0 || conOk(n->v1, n->a1, 0.0, st->Vs, A, J);
    }
    tpn_con const *c = &con[k];
    double d = c->S - n->s1;
    /* inside a chain of caps stepping down, a cap may be met still
     * braking: the speed only goes on down to the next one. Met at rest
     * in acceleration, each would be a small plateau of its own, and the
     * jerk would swing from one to the next. Only into a piece that can
     * hold the deceleration it is met with. */
    /* braked to under the limits of the pieces that carry the way there:
     * the envelope of the constraints of the short ones carries the
     * speed past them and their acceleration budgets what is left to
     * ramp out */
    /* the speed alone, for the fallback, under the limits of the way
     * there whether own or not: no envelope is relied on then */
    int run = !c->own || what == CHK_SPEED;
    double J = (run ? tpnMin(c->Jrun, st->J) : st->J) * TPN_BRAKE_SCALE;
    double A = (run ? tpnMin(c->Arun, st->A) : st->A) * TPN_BRAKE_SCALE;
    int deep = c->Aentry >= A;
    int chain = deep && k + 1 < ncon && con[k + 1].Vh > 0.0 && con[k + 1].Vh < c->Vh;
    int chain_s = deep && k + 1 < ncon && con[k + 1].Vs >= 0.0 && con[k + 1].Vs < c->Vs;
    if (what == CHK_HARD || what == CHK_SPEED) {
        if (c->own && what == CHK_HARD) {
            /* an acceleration the piece does not take ramps down to its
             * limit by the time the motion gets there, carried or not */
            if (c->Vh > 0.0 && !entryOk(k, n, st)) {
                return 0;
            }
            /* the envelope of a nearer one carries it, or a ramp the
             * motion follows */
            if (carried(k, n, st)) {
                return 1;
            }
        }
        if (c->Vh <= 0.0) {
            /* leave the deadbeat finish a little room to land on a cycle */
            d -= 4.0 * J * g_dt * g_dt * g_dt;
        }
        int ok;
        if (c->Vh <= 0.0 && what == CHK_HARD) {
            /* at rest with nothing left to ramp out */
            return conArriveOk(n->v1, n->a1, d, 0.0, A, J, 0.0, TPN_BIG, J, TPN_BIG, 0.0, 0.0);
        }
        if (c->Vh > 0.0 && c->S >= st->Sstop) {
            /* the motion stops before it gets there */
            return 1;
        }
        if (c->Vh > 0.0 && what == CHK_HARD) {
            /* the deceleration S may be met with */
            double Ja = tpnMin(c->Jentry * TPN_BRAKE_SCALE, J);
            double ae = tpnMin(tpnMin(c->ae, A), sqrt(2.0 * c->Vh * Ja));
            /* the caps and the jerk limits of the pieces an acceleration
             * still ramping out at S reaches, twice as the second may
             * reach further under a lower limit */
            double W = TPN_BIG;
            int m, it;
            for (it = 0; it < 2 && n->a1 != 0.0; it++) {
                double aa = fabs(n->a1);
                double R = (n->v1 + 0.5 * aa * aa / Ja) * aa / Ja;
                for (m = k + 1; m < ncon && con[m].S <= c->S + R; m++) {
                    if (con[m].Vh > 0.0) {
                        W = tpnMin(W, con[m].Vh);
                        Ja = tpnMin(Ja, con[m].Jentry * TPN_BRAKE_SCALE);
                    }
                }
            }
            ae = tpnMin(ae, sqrt(2.0 * c->Vh * Ja));
            if (n->a1 < 0.0) {
                W = TPN_BIG;
            }
            /* the next one the envelope carries */
            double Vn = c->Vh, Ln = 0.0;
            for (m = k + 1; m < ncon; m++) {
                if (con[m].own && con[m].S > c->S) {
                    Vn = con[m].Vh;
                    Ln = con[m].S - c->S;
                    break;
                }
            }
            /* down to the envelope at S, or past S on the ramp the
             * envelope comes from, the whole way under its limits, below
             * the caps the ramp crosses */
            if (c->r.J > 0.0) {
                /* carried() has looked for a landing on it */
                return cf_land == k;
            }
            /* carried() looked for a landing on an own one */
            return (c->own && cf_land == k)
                || conArriveOk(n->v1, n->a1, d, c->Vh, A, J, ae, tpnMin(c->ae, A), Ja, W, Vn, Ln);
        }
        ok = chain ? reachDist(n->v1, n->a1, c->Vh, A, J) <= tpnMax(d, 0.0) + 1e-12
            : conOk(n->v1, n->a1, d, c->Vh, A, J);
        if (what == CHK_SPEED) {
            return ok;
        }
        return ok && entryOk(k, n, st);
    }
    if (c->Vs < 0.0) {
        return 1;
    }
    return chain_s ? reachDist(n->v1, n->a1, c->Vs, A, J) <= tpnMax(d, 0.0) + 1e-12
        : conOk(n->v1, n->a1, d, c->Vs, A, J);
}

/* smallest jerk that keeps the motion from reversing */
static double jerkNoReverse(double jlo, double jhi, double J, double dt)
{
    tpn_next n;
    int k;
    stepState(jlo, dt, &n);
    if (n.v1 >= 0.0 && (n.a1 >= 0.0 || n.v1 - 0.5 * n.a1 * n.a1 / J >= -1e-12)) {
        return jlo;
    }
    for (k = 0; k < 40; k++) {
        double mid = 0.5 * (jlo + jhi);
        stepState(mid, dt, &n);
        if (n.v1 >= 0.0 && (n.a1 >= 0.0 || n.v1 - 0.5 * n.a1 * n.a1 / J >= -1e-12)) {
            jhi = mid;
        } else {
            jlo = mid;
        }
    }
    return jhi;
}

/* Jerk that brings the speed down to Vt as fast as the limits allow
 * without undershooting it. */
static double jerkTrack(double Vt, double jlo, double jhi, double J, double dt)
{
    tpn_next n;
    int k;
    stepState(jlo, dt, &n);
    if (n.v1 + 0.5 * n.a1 * fabs(n.a1) / J >= Vt) {
        return jlo;
    }
    stepState(jhi, dt, &n);
    if (n.v1 + 0.5 * n.a1 * fabs(n.a1) / J < Vt) {
        return jhi;
    }
    for (k = 0; k < 40; k++) {
        double mid = 0.5 * (jlo + jhi);
        stepState(mid, dt, &n);
        if (n.v1 + 0.5 * n.a1 * fabs(n.a1) / J >= Vt) {
            jhi = mid;
        } else {
            jlo = mid;
        }
    }
    return jhi;
}

/* The jerk limit the deceleration is ramped out with: the smallest over
 * the pieces the ramp crosses, up to the first stop, where the motion
 * rests. The hard constraints account for the limits ahead; a requested
 * stop (feed hold, pause, abort) or a soft cap out of reach (a lower
 * override) is tracked with this, or the speed runs on to rest where a
 * piece with a lower jerk limit starts inside the ramp. */
static double jerkAhead(tpn_step const *st, double dt)
{
    double J = st->J, a = tpn.cur_a, v = tpn.cur_v;
    int it, k;
    if (a >= 0.0) {
        return J;
    }
    for (it = 0; it < 3; it++) {
        double t = -a / J;
        double reach = v * t + 0.5 * a * t * t + J * t * t * t / 6.0 + 2.0 * v * dt;
        double Jn = st->J;
        for (k = 0; k < ncon && con[k].S - cx <= reach && con[k].Vh > 0.0; k++) {
            Jn = tpnMin(Jn, con[k].Jentry);
        }
        if (Jn >= J) {
            break;
        }
        J = Jn;
    }
    return J;
}

/*
 * A speed-up that has to turn into braking within [TRAJ]SPEED_HUMP_TIME
 * (over G64 R) is only a burst of jerk, as good as a vibration. While
 * speeding up the controller looks at the caps ahead, up to the first
 * one no higher than the speed it gets to, v, once its acceleration is
 * ramped out: that cap is an envelope of all those after it. Where the
 * speed hump above a cap, or above v, on the way there would be shorter,
 * it rises no further than that, for a single speed change. A stop ends
 * the look: a short move from rest is not a hump. Nonzero with the speed
 * to hold in Vhold.
 */
static int humpShort(tpn_step const *st, double *Vhold)
{
    double v = tpn.cur_v + 0.5 * tpn.cur_a * fabs(tpn.cur_a) / st->J;
    double vtop = st->Vs >= 0.0 ? tpnMin(st->Vs, st->V) : st->V;
    double hold = TPN_BIG;
    int k;
    if (v >= vtop) {
        return 0;
    }
    for (k = 0; k < ncon_hump; k++) {
        tpn_con const *c = &con[k];
        double Vk = c->Vs >= 0.0 ? tpnMin(c->Vh, c->Vs) : c->Vh;
        double d = c->S - cx;
        if (Vk <= 0.0) {
            break;
        }
        if (d <= 0.0 || Vk >= vtop) {
            continue;
        }
        /* up under the limits here, down under the lowest on the way */
        double t = tpnHumpTime(v, Vk, vtop, d, st->A, st->J,
                tpnMin(c->Arun, st->A), tpnMin(c->Jrun, st->J));
        if (t < st->hump_t) {
            hold = tpnMin(hold, tpnMax(v, Vk));
        }
        if (Vk <= v) {
            break;
        }
    }
    if (hold >= vtop) {
        return 0;
    }
    *Vhold = hold;
    return 1;
}

/* how far ahead the acceleration of the next state may still be
 * ramping out, under the lowest jerk limit on the way: constraints
 * nearer are checked for it, carried or not */
static double entryWindow(tpn_next const *n, tpn_step const *st)
{
    double J = tpnMin(st->J, jfloor), aa = fabs(n->a1);
    double v = n->v1 + (n->a1 > 0.0 ? 0.5 * aa * aa / J : 0.0);
    return tpnMax(v, 0.0) * (aa / J + 2.0 * g_dt) + 1e-9;
}

/* The next state meets the step caps in curmask and the constraints in
 * failmask. Past the first own constraint carried the own ones need no
 * hard check, beyond where the acceleration settles. */
static int stateOk(tpn_next const *n, tpn_step const *st, unsigned char curmask)
{
    int i, carried_own = 0;
    double win = n->s1 + entryWindow(n, st);
    if (((curmask & CHK_HARD) && !checkOne(-1, CHK_HARD, n, st))
            || ((curmask & CHK_SOFT) && !checkOne(-1, CHK_SOFT, n, st))) {
        return 0;
    }
    for (i = 0; i < ncon; i++) {
        unsigned char m = failmask[i];
        if (carried_own && i > fail_last) {
            break;
        }
        if (!m) {
            continue;
        }
        if ((m & CHK_HARD) && con[i].own && con[i].S > win
                && (carried_own || carried(i, n, st))) {
            carried_own = 1;
            m &= ~CHK_HARD;
        }
        if (((m & CHK_HARD) && !checkOne(i, CHK_HARD, n, st))
                || ((m & CHK_SOFT) && !checkOne(i, CHK_SOFT, n, st))) {
            return 0;
        }
    }
    return 1;
}

static double chooseJerk(TP_STRUCT const *tp, tpn_step const *st)
{
    double dt = tp->cycleTime;
    double J = st->J;
    double A = st->A;
    double jhi = tpnMin(tpnMin(J, st->Jhi), (A - tpn.cur_a) / dt);
    double jlo = tpnMax(-J, (-A - tpn.cur_a) / dt);
    tpn_next n;
    int k, i;
    int nfail = 0;

    if (jlo > jhi) {
        /* the acceleration is above the limit of the piece just entered */
        return tpn.cur_a > 0.0 ? -J : J;
    }
    jlo = jerkNoReverse(jlo, jhi, J, dt);

    /* everything that holds at jhi holds for any smaller jerk */
    stepState(jhi, dt, &n);
    /* past the first own one carried the own ones hold; past the first
     * own one that fails they are rechecked below without looking here */
    int carried_own = 0, failed_own = 0;
    double win = n.s1 + entryWindow(&n, st);
    for (i = -1; i < ncon; i++) {
        unsigned char m = 0;
        if (i >= 0 && con[i].own && failed_own) {
            m |= CHK_HARD;
        } else if (i >= 0 && con[i].own && con[i].S > win && (carried_own || carried(i, &n, st))) {
            carried_own = 1;
        } else if (!checkOne(i, CHK_HARD, &n, st)) {
            m |= CHK_HARD;
            failed_own = i >= 0 && con[i].own;
        }
        if (!checkOne(i, CHK_SOFT, &n, st)) {
            m |= CHK_SOFT;
        }
        if (i >= 0) {
            failmask[i] = m;
        }
        nfail += m != 0;
    }
    if (nfail == 0) {
        return jhi;
    }

    /* soft caps that cannot be met even braking at full rate are
     * tracked instead of searched */
    unsigned char curmask = 0;
    double Vtrack = TPN_BIG;
    stepState(jhi, dt, &n);
    if (!checkOne(-1, CHK_HARD, &n, st)) {
        curmask |= CHK_HARD;
    }
    if (!checkOne(-1, CHK_SOFT, &n, st)) {
        curmask |= CHK_SOFT;
    }
    stepState(jlo, dt, &n);
    if ((curmask & CHK_SOFT) && !checkOne(-1, CHK_SOFT, &n, st)) {
        curmask &= ~CHK_SOFT;
        Vtrack = tpnMin(Vtrack, st->Vs);
    }
    for (i = 0; i < ncon; i++) {
        if ((failmask[i] & CHK_SOFT) && !checkOne(i, CHK_SOFT, &n, st)) {
            failmask[i] &= ~CHK_SOFT;
            Vtrack = tpnMin(Vtrack, con[i].Vs);
        }
    }

    fail_last = -1;
    for (i = 0; i < ncon; i++) {
        if ((failmask[i] & CHK_SOFT) || ((failmask[i] & CHK_HARD) && !con[i].own)) {
            fail_last = i;
        }
    }
    double lo = jlo, hi = jhi;
    int ok_lo = 1;
    stepState(lo, dt, &n);
    ok_lo = stateOk(&n, st, curmask);
    double j;
    if (!ok_lo) {
        /* braking too hard may fail a ramp that ends above rest: look
         * for a jerk inside that meets them all, and search above it */
        for (k = TPN_SCAN - 1; k >= 1 && !ok_lo; k--) {
            double mid = jlo + (jhi - jlo) * k / TPN_SCAN;
            stepState(mid, dt, &n);
            ok_lo = stateOk(&n, st, curmask);
            if (ok_lo) {
                lo = mid;
            }
        }
    }
    if (!ok_lo) {
        /* No jerk meets every hard constraint: the step went a hair past
         * the point where a speed cap and the acceleration cap of the
         * piece after it turn tight together. Keep the speeds and release
         * the acceleration as fast as they allow; braking harder would
         * only push the acceleration further past the cap. */
        lo = jlo;
        hi = jhi;
        for (k = 0; k < TPN_BISECT; k++) {
            double mid = k ? 0.5 * (lo + hi) : hi;
            int ok = 1;
            stepState(mid, dt, &n);
            if ((curmask & CHK_HARD) && !checkOne(-1, CHK_SPEED, &n, st)) {
                ok = 0;
            }
            for (i = 0; i < ncon && ok; i++) {
                if ((failmask[i] & CHK_HARD) && !checkOne(i, CHK_SPEED, &n, st)) {
                    ok = 0;
                }
            }
            if (ok) {
                lo = mid;
                if (!k) {
                    break;
                }
            } else {
                hi = mid;
            }
        }
        j = lo;
    } else {
        for (k = 0; k < TPN_BISECT; k++) {
            double mid = 0.5 * (lo + hi);
            stepState(mid, dt, &n);
            if (stateOk(&n, st, curmask)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        j = lo;
    }
    if (Vtrack < TPN_BIG) {
        /* a requested stop or a lower override ramps its deceleration
         * out with the jerk limit ahead */
        j = tpnMin(j, jerkTrack(Vtrack, jlo, jhi, jerkAhead(st, dt), dt));
    }
    return j;
}

/* Jerk j_k = c0 + c1 k, k = 0 .. N-1, that takes (v, a) to (Vt, 0) in N
 * steps. */
static int landPlan(double v, double a, double Vt, int N, double dt, double *c0, double *c1)
{
    double S1 = 0.5 * N * (N - 1.0), S2 = (N - 1.0) * N * (2.0 * N - 1.0) / 6.0;
    double m10 = 0.5 * N * N, m11 = (N - 0.5) * S1 - S2;
    double r0 = -a / dt, r1 = (Vt - v - N * a * dt) / (dt * dt);
    double det = N * m11 - S1 * m10;
    if (fabs(det) < 1e-12) {
        return 0;
    }
    *c0 = (r0 * m11 - S1 * r1) / det;
    *c1 = (N * r1 - m10 * r0) / det;
    return 1;
}

/* A speed plateau. Riding the braking curve onto a speed cap ends with an
 * acceleration below one cycle of jerk, which no jerk takes to zero
 * without passing the cap: the largest jerk under it overshoots into the
 * opposite acceleration, and back, the jerk flipping sign cycle after
 * cycle. Nor can the curve be left at its end without a reversal of the
 * jerk. So leave it at its start: once N steps of jerk of one sign,
 * changing linearly, land (v, a) exactly on the cap with no
 * acceleration, follow them. A plan braking no later than the jerk
 * chosen keeps every cap; one braking later only if they all hold. */
static double landJerk(TP_STRUCT const *tp, tpn_step const *st, double j)
{
    double dt = tp->cycleTime, a = tpn.cur_a, v = tpn.cur_v;
    double Vt = st->Vs >= 0.0 ? tpnMin(st->Vs, st->V) : st->V;
    double Jf = st->J, sg = a > 0.0 ? 1.0 : -1.0;
    tpn_next n;
    int n0, N, i;
    if (Vt >= TPN_BIG || a == 0.0 || (a > 0.0) != (v < Vt)) {
        return j;
    }
    n0 = (int)ceil(fabs(a) / (Jf * dt));
    if (n0 < 2) {
        n0 = 2;
    }
    for (N = n0; N <= n0 + TPN_LAND_EXTRA; N++) {
        double c0, c1;
        if (!landPlan(v, a, Vt, N, dt, &c0, &c1)) {
            continue;
        }
        double jN = c0 + c1 * (N - 1);
        if (sg * c0 > 1e-9 * Jf || sg * jN > 1e-9 * Jf || fabs(c0) > Jf || fabs(jN) > Jf) {
            continue;
        }
        if (c0 > j) {
            int ok;
            stepState(c0, dt, &n);
            ok = checkOne(-1, CHK_HARD, &n, st) && checkOne(-1, CHK_SOFT, &n, st);
            for (i = 0; i < ncon && ok; i++) {
                ok = checkOne(i, CHK_HARD, &n, st) && checkOne(i, CHK_SOFT, &n, st);
            }
            if (!ok) {
                continue;
            }
        }
        return c0;
    }
    return j;
}

/*
 * Jerk that follows the spindle reference s_ref, v_ref, a_ref, j_ref from
 * the state (s, v, a): the feedforward jerk plus state feedback on the
 * errors it would leave at the end of the step, with all three poles at
 * -w, within the jerk limit J and the acceleration limit A.
 */
static double followJerk(double dt, double s, double v, double a, double A, double J, double w)
{
    double jf = tpn.j_ref;
    /* the errors where the step ends with the feedforward jerk */
    double es = tpn.s_ref - (s + v * dt + 0.5 * a * dt * dt + jf * dt * dt * dt / 6.0);
    double ev = tpn.v_ref - (v + a * dt + 0.5 * jf * dt * dt);
    double ea = tpn.a_ref - (a + jf * dt);
    double j = jf + w * w * w * es + 3.0 * w * w * ev + 3.0 * w * ea;
    double jlo = tpnMax(-J, (-A - a) / dt);
    double jhi = tpnMin(J, (A - a) / dt);
    j = tpnMax(jlo, tpnMin(jhi, j));
    if (fabs(es) > TPN_SYNC_LAND && jlo < jhi) {
        /* Far from the reference the linear law saturates and would
         * overshoot. In the frame moving with the reference the axis is
         * again a triple integrator: keep the jerk where it can still
         * land on the reference, the way a stop is kept reachable. */
        double side = es > 0.0 ? 1.0 : -1.0;
        double Al = tpnMax(TPN_SYNC_AMARGIN * A - fabs(tpn.a_ref), 0.1 * A);
        double Jl = TPN_SYNC_JMARGIN * J;
        double lo = jlo, hi = jhi;
        tpn_next n;
        int k;
        for (k = 0; k < 40; k++) {
            double mid = 0.5 * (lo + hi);
            stepFrom(s, v, a, mid, dt, &n);
            double gap = side * (tpn.s_ref - n.s1);
            double u = side * (n.v1 - tpn.v_ref), ar = side * (n.a1 - tpn.a_ref);
            int ok = tpnBrakeDist(u, ar, 0.0, Al, Jl) <= gap;
            /* from behind more jerk is worse, from ahead less */
            if (ok == (side > 0.0)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        j = side > 0.0 ? tpnMin(j, lo) : tpnMax(j, hi);
    }
    return j;
}

/* the spindle following jerk of the path, within the hard jerk range
 * [jerkNoReverse, jhard] of this step */
static double syncJerk(TP_STRUCT const *tp, tpn_step const *st, double jhard)
{
    double dt = tp->cycleTime;
    double j = followJerk(dt, tpn.cur_s, tpn.cur_v, tpn.cur_a, st->A, st->J, TPN_SYNC_W);
    double jlo = tpnMax(-st->J, (-st->A - tpn.cur_a) / dt);
    if (jlo > jhard) {
        return jhard;
    }
    jlo = jerkNoReverse(jlo, jhard, st->J, dt);
    return tpnMax(jlo, tpnMin(j, jhard));
}

/* where the speed ends once the acceleration is ramped out at J */
static double speedAhead(tpn_next const *n, double J)
{
    return n->v1 + 0.5 * n->a1 * fabs(n->a1) / J;
}

void tpnTapAdvance(TP_STRUCT const *tp, double *s, double *v, double *a, tpn_lim const *lim)
{
    double dt = tp->cycleTime, V = lim->V, A = lim->A, J = lim->J;
    double j = followJerk(dt, *s, *v, *a, A, J, TPN_TAP_W);
    tpn_next n;
    int k;
    stepFrom(*s, *v, *a, j, dt, &n);
    /* keep the speed where it can still be held within V either way */
    double side = speedAhead(&n, J) > V ? 1.0 : speedAhead(&n, J) < -V ? -1.0 : 0.0;
    if (side != 0.0) {
        double lo = side > 0.0 ? tpnMax(-J, (-A - *a) / dt) : j;
        double hi = side > 0.0 ? j : tpnMin(J, (A - *a) / dt);
        for (k = 0; k < 40; k++) {
            double mid = 0.5 * (lo + hi);
            stepFrom(*s, *v, *a, mid, dt, &n);
            if (side * speedAhead(&n, J) > V) {
                if (side > 0.0) {
                    hi = mid;
                } else {
                    lo = mid;
                }
            } else if (side > 0.0) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        j = side > 0.0 ? lo : hi;
        stepFrom(*s, *v, *a, j, dt, &n);
    }
    *s = n.s1;
    *v = n.v1;
    *a = n.a1;
}

/*
 * Continuous brake profile from (v, a) to rest as up to three constant
 * jerk phases; returns the number of phases.
 */
static int brakePhases(double v, double a, double A, double J, double *pj, double *pt)
{
    int n = 0;
    double t1, tA, vv;
    if (a >= 0.0) {
        vv = v + 0.5 * a * a / J;
        t1 = tpnMin(sqrt(vv / J), A / J);
        tA = vv > J * t1 * t1 ? (vv - J * t1 * t1) / A : 0.0;
        pj[n] = -J; pt[n++] = a / J + t1;
    } else {
        double Aeff = tpnMax(A, -a);
        vv = v + 0.5 * a * a / J;
        if (v - 0.5 * a * a / J <= 0.0) {
            pj[n] = J; pt[n++] = -a / J;
            return n;
        }
        t1 = tpnMin(sqrt(vv / J), Aeff / J);
        tA = vv > J * t1 * t1 ? (vv - J * t1 * t1) / Aeff : 0.0;
        pj[n] = -J; pt[n++] = tpnMax(0.0, t1 + a / J);
    }
    pj[n] = 0.0; pt[n++] = tA;
    pj[n] = J; pt[n++] = t1;
    return n;
}

/* invert the symmetric 3x3 matrix M */
static int inv3(double M[3][3], double inv[3][3])
{
    double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
               - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
               + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    if (fabs(det) < 1e-300) {
        return -1;
    }
    inv[0][0] = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) / det;
    inv[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) / det;
    inv[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / det;
    inv[1][0] = (M[1][2] * M[2][0] - M[1][0] * M[2][2]) / det;
    inv[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / det;
    inv[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) / det;
    inv[2][0] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) / det;
    inv[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) / det;
    inv[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) / det;
    return 0;
}

/*
 * Jerk sequence over N cycles that ends exactly at distance d with zero
 * speed and acceleration on a cycle boundary: the continuous brake
 * profile averaged over each cycle, plus the smallest correction that
 * removes what the averaging and the fractional last cycle leave over.
 * Returns the largest jerk of the sequence.
 */
static double deadbeat(double d, double v, double a, int N, double dt,
        int nph, double const *pj, double const *pt, double *seq)
{
    double M[3][3] = {{0}}, r[3], inv[3][3], lam[3];
    int k, p, q;
    double sN = 0.0, vN = v, aN = a, t0 = 0.0;
    /* nominal jerk: continuous profile averaged over each cycle */
    for (k = 0; k < N; k++) {
        double ta = k * dt, tb = ta + dt, acc = 0.0;
        t0 = 0.0;
        for (p = 0; p < nph; p++) {
            double lo = tpnMax(ta, t0), hi = tpnMin(tb, t0 + pt[p]);
            if (hi > lo) {
                acc += pj[p] * (hi - lo);
            }
            t0 += pt[p];
        }
        seq[k] = acc / dt;
    }
    /* response to the nominal sequence */
    for (k = 0; k < N; k++) {
        double j = seq[k];
        sN += vN * dt + 0.5 * aN * dt * dt + j * dt * dt * dt / 6.0;
        vN += aN * dt + 0.5 * j * dt * dt;
        aN += j * dt;
    }
    r[0] = d - sN;
    r[1] = -vN;
    r[2] = -aN;
    for (k = 1; k <= N; k++) {
        double m = N - k;
        double row[3] = {dt * dt * dt * (1.0 / 6.0 + 0.5 * m + 0.5 * m * m),
                         dt * dt * (0.5 + m), dt};
        for (p = 0; p < 3; p++) {
            for (q = 0; q < 3; q++) {
                M[p][q] += row[p] * row[q];
            }
        }
    }
    if (inv3(M, inv)) {
        return TPN_BIG;
    }
    for (p = 0; p < 3; p++) {
        lam[p] = inv[p][0] * r[0] + inv[p][1] * r[1] + inv[p][2] * r[2];
    }
    double jmax = 0.0;
    for (k = 1; k <= N; k++) {
        double m = N - k;
        seq[k - 1] += lam[0] * dt * dt * dt * (1.0 / 6.0 + 0.5 * m + 0.5 * m * m)
                    + lam[1] * dt * dt * (0.5 + m) + lam[2] * dt;
        jmax = tpnMax(jmax, fabs(seq[k - 1]));
    }
    return jmax;
}

/* acceleration stays within A and the motion never reverses */
static int deadbeatOk(int N, double dt, double A)
{
    double v = tpn.cur_v, a = tpn.cur_a;
    int k;
    for (k = 0; k < N; k++) {
        double j = db_seq[k];
        v += a * dt + 0.5 * j * dt * dt;
        a += j * dt;
        if (fabs(a) > A * (1.0 + 1e-6) || v < -1e-9) {
            return 0;
        }
    }
    return 1;
}

/* Land a stop on a cycle boundary: once the controller rides the brake
 * curve into a stop close enough, replace the rest of the curve by a
 * deadbeat sequence and run it to its end unless the stop moves. */
static int finishStop(TP_STRUCT const *tp, tpn_step const *st, double *j)
{
    double dt = tp->cycleTime;
    double d = st->Sstop - cx;
    double pj[3], pt[3];
    int nph, N, k;
    if (db_i < db_n && db_S == st->Sstop) {
        *j = db_seq[db_i++];
        return 1;
    }
    db_n = db_i = 0;
    if (st->Sstop >= TPN_BIG || d < 0.0) {
        return 0;
    }
    double Ab = st->A * TPN_BRAKE_SCALE, Jb = st->J * TPN_BRAKE_SCALE;
    double bd = tpnBrakeDist(tpn.cur_v, tpn.cur_a, 0.0, Ab, Jb);
    if (tpn.cur_a < 0.0 && tpn.cur_v - 0.5 * tpn.cur_a * tpn.cur_a / st->J <= 0.0) {
        /* a deceleration ramped out at the full limit: the stop check
         * brakes to rest with none left, so does the curve here */
        double t = -tpn.cur_a / st->J;
        bd = tpn.cur_v * t + 0.5 * tpn.cur_a * t * t + st->J * t * t * t / 6.0;
    }
    double slack = 8.0 * st->J * dt * dt * dt;
    int creep = tpn.cur_v <= 1e-9 && fabs(tpn.cur_a) <= 1e-9;
    if (creep ? d > tpnMin(0.01, 100.0 * slack) : d > bd + tpnMax(1e-6 * d, 0.05 * tpn.cur_v * dt) + slack) {
        return 0;
    }
    nph = brakePhases(tpn.cur_v, tpn.cur_a, Ab, Jb, pj, pt);
    double T = 0.0;
    for (k = 0; k < nph; k++) {
        T += pt[k];
    }
    N = (int)ceil(T / dt - 1e-9);
    int Nmax = creep ? TPN_DB_MAX : N + 6;
    if (N > TPN_DB_MAX - 2) {
        return 0;
    }
    for (; N <= TPN_DB_MAX && N <= Nmax; N++) {
        if (N < 3) {
            continue;
        }
        if (deadbeat(d, tpn.cur_v, tpn.cur_a, N, dt, nph, pj, pt, db_seq) <= 1.03 * st->J
                && deadbeatOk(N, dt, st->A)) {
            db_n = N;
            db_i = 1;
            db_S = st->Sstop;
            *j = db_seq[0];
            return 1;
        }
    }
    return 0;
}

void tpnRunReset(void)
{
    db_n = db_i = 0;
    nact = nact_old = 0;
    act_id = -1;
}

void tpnAdvance(TP_STRUCT const *tp, double scale, int stepping)
{
    double dt = tp->cycleTime;
    int rev = tp->reverse_run == TC_DIR_REVERSE;
    g_dt = dt;
    if (scale == 0.0 && tpn.cur_v == 0.0 && tpn.cur_a == 0.0) {
        /* paused at rest: hold still, the search would creep by the
         * smallest jerk it leaves */
        tpn.cur_j = 0.0;
        return;
    }
    cx = rev ? -tpn.cur_s : tpn.cur_s;

    tpn_step st;
    if (rev) {
        gatherReverse(tp, scale, stepping, &st);
    } else {
        gather(tp, scale, stepping, &st);
    }
    double j = chooseJerk(tp, &st);
    double jf;
    if (!rev && !tpn.track && st.hump_t > 0.0 && tpn.cur_a + j * dt > 0.0) {
        double Vh;
        if (humpShort(&st, &Vh)) {
            st.Vs = st.Vs < 0.0 ? Vh : tpnMin(st.Vs, Vh);
            j = chooseJerk(tp, &st);
        }
    }
    if (tpn.track) {
        j = syncJerk(tp, &st, j);
    }
    tpn.step_V = st.V;
    if (!tpn.track) {
        j = landJerk(tp, &st, j);
    }
    int fin = scale > 0.0 && finishStop(tp, &st, &jf);
    if (fin) {
        j = jf;
    }
    tpn_next n;
    stepState(j, dt, &n);

    /* land exactly on a stop; a deadbeat sequence with steps left ends
     * there itself, snapping early drops its last deceleration in one cycle */
    if (fin && db_i < db_n) {
        n.s1 = tpnMin(n.s1, st.Sstop);
    } else if (n.s1 >= st.Sstop - 1e-9 || (n.v1 < 1e-6 && st.Sstop - n.s1 < 1e-6 && n.a1 <= 0.0)) {
        n.s1 = tpnMin(n.s1, st.Sstop);
        if (st.Sstop - n.s1 < 1e-6) {
            n.s1 = st.Sstop;
            n.v1 = 0.0;
            n.a1 = 0.0;
        }
    }
    if (n.v1 <= 0.0) {
        n.v1 = 0.0;
        if (n.a1 < 0.0) {
            n.a1 = 0.0;
        }
    }
    /* paused or aborting and at rest */
    if (scale == 0.0 && n.v1 < 0.01 * st.J * dt * dt && fabs(n.a1) < 0.01 * st.J * dt) {
        n.v1 = 0.0;
        n.a1 = 0.0;
    }
#ifdef TPN_TRACE
    rtapi_print("T s=%.12f v=%.9f a=%.6f j=%.3f Sstop=%.12f db=%d/%d ncon=%d V=%.4f A=%.3f J=%.1f\n",
            n.s1, n.v1, n.a1, j, st.Sstop, db_i, db_n, ncon, st.V, st.A, st.J);
    if (st.Sstop < TPN_BIG) {
        rtapi_print("   d=%.9f bd=%.9f slack=%.9f\n", st.Sstop - n.s1,
                tpnBrakeDist(n.v1, n.a1, 0.0, st.A, st.J), 4.0 * st.J * dt * dt * dt);
    }
#endif
    tpn.cur_j = (n.a1 - tpn.cur_a) / dt;
    tpn.cur_s = rev ? -n.s1 : n.s1;
    tpn.cur_v = n.v1;
    tpn.cur_a = n.a1;
}
