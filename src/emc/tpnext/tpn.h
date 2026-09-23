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
#include "../tp/tp_types.h"

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

enum { TPN_LINE = 1, TPN_ARC = 2 };

typedef struct {
    int type;
    tpn_vec p0, p1;         /* start and end pose */
    double L;               /* length of the path parameter */
    tpn_vec g;              /* rate of the linearly moving axes per unit s */
    /* arc, xyz part */
    PmCartesian center, rTan, rPerp, rHelix;
    double radius, spiral, angle;
} tpn_geom;

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
    double uu_per_rev;
    double vreq;            /* requested feed, path units */
    syncdio_t syncdio;

    double h_in;            /* blend half length at the start, 0 = none */
    double h_out;           /* blend half length at the end */
    int stop_in;            /* motion stops at the start */
    tpn_blend bin;          /* blend with the previous move */
    tpn_lim lim_bin;
    double vreq_bin;
    tpn_lim lim_int;        /* limits of the unblended interior */
    double E_bin, E_int;    /* backward envelope at the entry of each piece */
    int active;
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

/* limits */
void tpnLimits(tpn_axlim const *ax, tpn_vec const *G, tpn_vec const *G1,
        tpn_vec const *G2, tpn_lim *lim);

/* one dimensional jerk limited profile helpers */
double tpnBrakeDist(double v0, double a0, double vt, double A, double J);

/* planner state, shared by tpnext.c (module API), tpn_plan.c (queue
 * build) and tpn_run.c (per cycle controller) */

#define TPN_QSIZE DEFAULT_TC_QUEUE_SIZE
#define TPN_BIG 1e30
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
    /* G64 E for the moves queued next */
    double ang_tolerance;
    /* axes of each type, bit n = axis n, from [AXIS_n]TYPE */
    unsigned lin_mask, ang_mask;
    /* controller state along the path parameter */
    double cur_s, cur_v, cur_a, cur_j;
} tpn_state;

extern tpn_state tpn;

static inline tpn_seg *seg(int i)
{
    return &tpn.queue[(tpn.q_start + i) % TPN_QSIZE];
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

/* queue build, tpn_plan.c */
int tpnAddSegment(TP_STRUCT * const tp, tpn_seg *sg, int canon_type, double vel,
        double ini_maxvel, unsigned char enables, char atspeed, int indexer_jnum,
        struct state_tag_t tag);

/* per cycle controller, tpn_run.c */
void tpnRunReset(void);
/* advance cur_s, cur_v, cur_a and cur_j by one cycle */
void tpnAdvance(TP_STRUCT const *tp, double scale, int stepping);

#endif
