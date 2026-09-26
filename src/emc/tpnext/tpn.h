/********************************************************************
* Description: tpn.h
*   Internal types of the tpnext trajectory planner.
*
*   tpnext plans one path parameter s along the programmed nine axis
*   path. Corners are replaced by quintic blends inside the G64 P and E
*   tolerances, every piece of the path gets tangential limits projected
*   from the per axis velocity, acceleration and jerk limits, and a jerk
*   limited controller advances s every servo cycle.
*
* License: GPL Version 2
* System: Linux
********************************************************************/
#ifndef TPN_H
#define TPN_H

#include <posemath.h>
#include "../tp/tp.h"

#define TPN_NAX 9

typedef struct {
    double v[TPN_NAX];
} tpn_vec;

/* hard limits of one piece, in path units: velocity cap, tangential
 * acceleration and tangential jerk */
typedef struct {
    double V, A, J;
} tpn_lim;

/* per axis limits */
typedef struct {
    double vel[TPN_NAX];
    double acc[TPN_NAX];
    double jerk[TPN_NAX];
} tpn_axlim;

/* per joint limits, for kinematics other than the identity */
#define TPN_NJ TP_KINS_MAX_JOINTS
typedef struct {
    int n;
    double vel[TPN_NJ];
    double acc[TPN_NJ];
    double jerk[TPN_NJ];
} tpn_jlim;

/* bounds of the joint derivatives per unit s over a piece: q' by G,
 * q'' by r G1s + G1u and q''' by r^2 G2s + r G2m + G2u, for a blend
 * part scaled by r (1 elsewhere) */
typedef struct {
    double G[TPN_NJ];
    double G1s[TPN_NJ], G1u[TPN_NJ];
    double G2s[TPN_NJ], G2m[TPN_NJ], G2u[TPN_NJ];
} tpn_jb;

/* the joints at one end of a move, for the blend there: the Jacobian,
 * its rate per unit s along the move, the tangent, and a bound of the
 * third derivative of the joints near the end */
typedef struct {
    int valid;
    double J[TPN_NJ][TPN_NAX];
    double D[TPN_NJ][TPN_NAX];
    double t[TPN_NAX];
    double G2[TPN_NJ];
} tpn_jend;

enum { TPN_LINE = 1, TPN_ARC = 2 };

typedef struct {
    int type;
    tpn_vec p0, p1;         /* start and end pose */
    double L;               /* length of the path parameter */
    tpn_vec g;              /* rate of the linearly moving axes per unit s */
    /* arc, xyz part */
    PmCartesian center, rTan, rPerp, rHelix;
    double radius, spiral, angle;
    int flat;               /* a circle, no spiral, helix or other axes */
} tpn_geom;

/* a blend is followed in TPN_NSUB parts of equal length, each with its
 * own limits, so that only the sharpest part runs at the lowest speed */
#define TPN_NSUB 6

/* at most this many parts of the interior of a move with joints */
#define TPN_NINT 16

/* quintic blend, power basis in tau = sigma / H, sigma in [0, H] */
typedef struct {
    double c[6][TPN_NAX];
    double H;
} tpn_blend;

/* one queued move */
typedef struct {
    tpn_geom geom;
    double S0;              /* path parameter at the start of the move */
    int id;
    struct state_tag_t tag;
    int canon_type;
    unsigned char enables;
    char atspeed;
    int indexer_jnum;
    int term_cond;
    double tolerance;       /* G64 P, linear axes, 0 = none */
    double ang_tolerance;   /* G64 E, angular axes, 0 = none */
    int sync;               /* TC_SYNC_* */
    int spindle;            /* the spindle it follows */
    double uu_per_rev;
    double vreq;            /* requested feed, path units */
    double vlimit_scale;    /* max velocity slider scale canon sends, 0 = none */
    syncdio_t syncdio;

    double h_in;            /* blend half length at the start, 0 = none */
    double h_out;           /* blend half length at the end */
    int stop_in;            /* motion stops at the start */
    tpn_blend bin;          /* blend with the previous move */
    tpn_lim lim_bin;        /* smallest limits over the blend */
    tpn_lim lim_sub[TPN_NSUB];  /* limits of each part of the blend */
    double vreq_bin;
    double vlimit_bin;      /* slider scale of the blend, 0 = none */
    tpn_lim lim_int;        /* limits of the unblended interior */
    /* With joints the interior is followed in nint parts, from u_int[k]
     * to u_int[k + 1] from the start of the move, each with its own
     * limits, so a joint that peaks along the move (near a singular pose)
     * slows only the part it peaks in; lim_int is then the smallest of
     * them. nint = 1: lim_int alone. */
    int nint;
    double u_int[TPN_NINT + 1];
    /* how far from its start and end a blend may reach into the move
     * for the joint model of the blend to hold, TPN_BIG without joints */
    double hj_in, hj_out;
    tpn_lim lim_ip[TPN_NINT], lim_ip_hi[TPN_NINT];
    double E_ip[TPN_NINT], E_ip_hi[TPN_NINT];
    /* the same with the curvature caps opened up to the feed at the
     * highest feed override, which the runtime takes where the override
     * asks for more than the caps above allow */
    tpn_lim lim_sub_hi[TPN_NSUB], lim_int_hi;
    int tap;                /* a rigid tap (G33.1) still following the spindle */
    double tap_scale;       /* spindle speed factor of its way out */
    double E_sub[TPN_NSUB], E_int;  /* backward envelope at the entry of each piece */
    double E_sub_hi[TPN_NSUB], E_int_hi;
    int active;
    int rev_ok;             /* may be run backwards: no tap, sync or indexer */
    double hump_t;          /* speed humps shorter than this are flattened, s */
} tpn_seg;

/* geometry */
void tpnVecFromPose(tpn_vec *v, EmcPose const *p);
void tpnPoseFromVec(EmcPose *p, tpn_vec const *v);
int tpnLineInit(tpn_geom *g, EmcPose const *start, EmcPose const *end);
int tpnArcInit(tpn_geom *g, EmcPose const *start, EmcPose const *end,
        PmCartesian const *center, PmCartesian const *normal, int turn);
void tpnGeomEval(tpn_geom const *g, double u, tpn_vec *p, tpn_vec *d1, tpn_vec *d2);
void tpnGeomBounds(tpn_geom const *g, tpn_vec *G, tpn_vec *G1, tpn_vec *G2);
/* distance from q to the move, each axis scaled by its weight in w */
double tpnGeomDist(tpn_geom const *g, tpn_vec const *q, tpn_vec const *w);

void tpnBlendInit(tpn_blend *b, double H, tpn_vec const *p0, tpn_vec const *d0,
        tpn_vec const *dd0, tpn_vec const *p1, tpn_vec const *d1, tpn_vec const *dd1);
void tpnBlendEval(tpn_blend const *b, double sigma, tpn_vec *p, tpn_vec *d1);
void tpnBlendBounds(tpn_blend const *b, tpn_vec *G, tpn_vec *G1, tpn_vec *G2);
/* the part of b between tau = t0 and t1 as a blend of its own */
void tpnBlendPart(tpn_blend const *b, double t0, double t1, tpn_blend *part);

/* limits */
/* limits of a piece whose speed is also capped at vcap, for the
 * programmed feed vwant */
void tpnLimits(tpn_axlim const *ax, tpn_vec const *G, tpn_vec const *G1,
        tpn_vec const *G2, double vcap, double vwant, tpn_lim *lim);
/* the same in two steps, for pieces whose G1 and G2 scale by r and r^2:
 * the speed caps from the unscaled bounds, which scale by 1 / sqrt(r)
 * (V2) and r^(-2/3) (V3), then the limits at a speed V. V2 and V3 leave
 * the rest of the limits for speed changes along the path; V2max and
 * V3max leave less, taken only as far as the programmed feed needs. */
typedef struct {
    double Vg, V2, V3, V2max, V3max;
    double scale;           /* share of the curvature caps taken, 1 but at corners */
    int curved;
} tpn_caps;
void tpnLimitCaps(tpn_axlim const *ax, tpn_vec const *G, tpn_vec const *G1,
        tpn_vec const *G2, tpn_caps *caps);
/* speed cap of the curvature of a piece scaled by r, for the feed vwant */
double tpnCurveCap(tpn_caps const *caps, double r, double vwant);
void tpnLimitsAt(tpn_axlim const *ax, tpn_vec const *G, tpn_vec const *G1,
        tpn_vec const *G2, double r, double V, int curved, tpn_lim *lim);

/* the caps of the axes at r merged with those of the joints at r, as
 * caps of scale 1 */
void tpnLimitCapsJ(tpn_caps const *axcaps, double r, tpn_jlim const *jl, tpn_jb const *jb,
        tpn_caps *caps);
/* lower lim->A and lim->J to what the joints leave at V */
void tpnJointLimitsAt(tpn_jlim const *jl, tpn_jb const *jb, double r, double V, int curved,
        tpn_lim *lim);
/* tpnLimits() with the joints as well */
void tpnLimitsJ(tpn_axlim const *ax, tpn_vec const *G, tpn_vec const *G1, tpn_vec const *G2,
        tpn_jlim const *jl, tpn_jb const *jb, double vcap, double vwant, tpn_lim *lim);

/* one dimensional jerk limited profile helpers */
double tpnBrakeDist(double v0, double a0, double vt, double A, double J);
/* duration of the speed change dv with zero acceleration at both ends */
double tpnRampTime(double dv, double A, double J);
/* distance of that change from v0 to v1 */
double tpnRampDist(double v0, double v1, double A, double J);
/* how long a speed hump from va to vb over D, with zero acceleration at
 * both ends, stays above both, peaking at most at vtop, speeding up under
 * Aa and Ja and slowing down under Ab and Jb; 0 if it does not get above
 * them */
double tpnHumpTime(double va, double vb, double vtop, double D, double Aa, double Ja,
        double Ab, double Jb);

/* planner state, shared by tpnext.c (module API), tpn_plan.c (queue
 * build) and tpn_run.c (per cycle controller) */

#define TPN_QSIZE DEFAULT_TC_QUEUE_SIZE
/* moves already run that the queue keeps for a reverse run */
#define TPN_HIST 200
#define TPN_BIG 1e30
#define TPN_TINY 1e-12
/* braking is planned with this fraction of the tangential limits so the
 * cycle by cycle controller has headroom to follow the curve */
#define TPN_BRAKE_SCALE 0.97

struct emcmot_status_t;
struct emcmot_config_t;

typedef struct {
    struct emcmot_status_t *emcmotStatus;
    struct emcmot_config_t *emcmotConfig;
    double (*axis_get_vel_limit)(int);
    double (*axis_get_acc_limit)(int);
    double (*axis_get_jerk_limit)(int);
    int (*axis_is_angular)(int);

    tpn_seg queue[TPN_QSIZE];
    int q_start, q_len;
    /* moves already run, kept before q_start for a reverse run */
    int h_len;
    /* G64 E for the moves queued next */
    double ang_tolerance;
    /* axes of each type, bit n = axis n, from [AXIS_n]TYPE */
    unsigned lin_mask, ang_mask;
    /* controller state along the path parameter */
    double cur_s, cur_v, cur_a, cur_j;
    /* lower bounds of the smallest acceleration and jerk limits of the
     * queued pieces, see reachable() */
    double A_lo, J_lo;
    /* the latest slow point reachable() found enough: its start, speed
     * cap and acceleration limit, and lower bounds of the limits of the
     * pieces from there to the end of the queue; valid while slow_A is
     * below TPN_BIG */
    double slow_P, slow_V, slow_Ap, slow_A, slow_J;
    /* spindle position sync: while track is set the controller follows
     * s_ref, v_ref and a_ref instead of the requested speed */
    int track;
    double s_ref, v_ref, a_ref, j_ref;
    /* velocity cap of the pieces the last step touched */
    double step_V;
    /* kinematics from motion, the joint limits at the last add, the
     * joints at the end of the queue, and the joint model at the end of
     * the last queued move (jtail) and at the start of the one being
     * added (jhead) */
    tp_kins_t kins;
    tpn_jlim jl;
    int jseed_valid;
    double jseed[TPN_NJ];
    tpn_jend jtail, jhead;
} tpn_state;

extern tpn_state tpn;

static inline tpn_seg *seg(int i)
{
    /* i < 0 for the moves already run */
    return &tpn.queue[(tpn.q_start + i + TPN_QSIZE) % TPN_QSIZE];
}

static inline double segEnd(tpn_seg const *sg)
{
    return sg->S0 + sg->geom.L;
}

/* the part of the path a move owns: from the start of its blend with the
 * previous move to the start of its blend with the next one */
static inline double ownedStart(tpn_seg const *sg)
{
    return sg->S0 - sg->h_in;
}

static inline double ownedEnd(tpn_seg const *sg)
{
    return sg->S0 + sg->geom.L - sg->h_out;
}

/* the pieces of a move: the parts of its blend with the previous move,
 * then the parts of its interior */
static inline int tpnPieces(tpn_seg const *sg)
{
    return TPN_NSUB + sg->nint;
}

/* Part k of the interior of move sg from Pa to Pb along the path, with
 * its limits, the opened ones and their envelopes. Zero if it is empty. */
static inline int tpnIntPart(tpn_seg const *sg, int k, double *Pa, double *Pb,
        tpn_lim const **lim, tpn_lim const **hi, double *E, double *E_hi)
{
    if (sg->nint == 1) {
        *Pa = sg->S0 + sg->h_in;
        *Pb = ownedEnd(sg);
        *lim = &sg->lim_int;
        *hi = &sg->lim_int_hi;
        *E = sg->E_int;
        *E_hi = sg->E_int_hi;
        return 1;
    }
    double a = sg->u_int[k], b = sg->u_int[k + 1], end = sg->geom.L - sg->h_out;
    if (b <= sg->h_in || a >= end) {
        return 0;
    }
    *Pa = a <= sg->h_in ? sg->S0 + sg->h_in : sg->S0 + a;
    *Pb = b >= end ? ownedEnd(sg) : sg->S0 + b;
    *lim = &sg->lim_ip[k];
    *hi = &sg->lim_ip_hi[k];
    *E = sg->E_ip[k];
    *E_hi = sg->E_ip_hi[k];
    return 1;
}

/* Piece k of move sg: a part of the blend (k < TPN_NSUB) or of the
 * interior, as tpnIntPart(). */
static inline int tpnPiece(tpn_seg const *sg, int k, double *Pa, double *Pb,
        tpn_lim const **lim, tpn_lim const **hi, double *E, double *E_hi)
{
    if (k >= TPN_NSUB) {
        return tpnIntPart(sg, k - TPN_NSUB, Pa, Pb, lim, hi, E, E_hi);
    }
    if (sg->h_in <= 0.0) {
        return 0;
    }
    *Pa = ownedStart(sg) + 2.0 * sg->h_in * k / TPN_NSUB;
    *Pb = k == TPN_NSUB - 1 ? sg->S0 + sg->h_in
        : ownedStart(sg) + 2.0 * sg->h_in * (k + 1) / TPN_NSUB;
    *lim = &sg->lim_sub[k];
    *hi = &sg->lim_sub_hi[k];
    *E = sg->E_sub[k];
    *E_hi = sg->E_sub_hi[k];
    return 1;
}

/* limits of the interior part a move starts with (end 0) or ends with */
static inline tpn_lim const *tpnEndLim(tpn_seg const *sg, int end, int hi)
{
    if (sg->nint == 1) {
        return hi ? &sg->lim_int_hi : &sg->lim_int;
    }
    int k = end ? sg->nint - 1 : 0;
    return hi ? &sg->lim_ip_hi[k] : &sg->lim_ip[k];
}

/* queue build, tpn_plan.c */
int tpnAddSegment(TP_STRUCT * const tp, tpn_seg *sg, int canon_type, double vel,
        double ini_maxvel, double vlimit_scale, unsigned char enables, char atspeed,
        int indexer_jnum, struct state_tag_t tag);

/* spindle synchronization, tpn_sync.c; a catch up to the spindle uses
 * this much of the acceleration and jerk limits, the rest is left for
 * following it */
#define TPN_SYNC_AMARGIN 0.9
#define TPN_SYNC_JMARGIN 0.8
/* bandwidth of the spindle estimate and of the tracking during a rigid
 * tap, rad/s: the reversals are acceleration steps it has to follow */
#define TPN_TAP_W 250.0
void tpnSyncReset(void);
int tpnSyncOn(void);
void tpnSpindleEstimate(TP_STRUCT const *tp);
/* returns nonzero while the move waits for the spindle */
int tpnSyncStart(TP_STRUCT * const tp, tpn_seg *sg);
void tpnSyncReference(TP_STRUCT const *tp);
void tpnSyncOverrun(TP_STRUCT * const tp);
/* rigid tap: one cycle of the move sg once it has started; returns one
 * of TPN_TAP_*, and the tap's position u from its start, speed and
 * acceleration */
enum { TPN_TAP_RUN, TPN_TAP_PLACE, TPN_TAP_STOPPED };
int tpnTapCycle(TP_STRUCT * const tp, tpn_seg *sg, double *u, double *v, double *a);
int tpnTapMoving(void);

/* per cycle controller, tpn_run.c */
void tpnRunReset(void);
/* advance cur_s, cur_v, cur_a and cur_j by one cycle, backwards along
 * the path while tp->reverse_run is set; cur_v and cur_a are then along
 * the direction of travel */
void tpnAdvance(TP_STRUCT const *tp, double scale, int stepping);
/* advance the signed state (s, v, a) of a rigid tap by one cycle,
 * following the spindle reference within the limits lim */
void tpnTapAdvance(TP_STRUCT const *tp, double *s, double *v, double *a, tpn_lim const *lim);

#endif
