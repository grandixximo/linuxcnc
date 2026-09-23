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
#include "../tp/tc_types.h"

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

#endif
