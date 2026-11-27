/********************************************************************
* Description: orient_smooth.hh
*   Smoothing of the rotary axes along runs of short lines
*   ([TRAJ]ORIENT_SMOOTH), for emccanon.
*
* License: GPL Version 2
* System: Linux
********************************************************************/
#ifndef ORIENT_SMOOTH_HH
#define ORIENT_SMOOTH_HH

#include <cmath>
#include <vector>

/* the axes a curve may bend: A, B, C, U, V and W, slot k for A + k */
#define ORIENT_NAX 6

/* One rotary axis of a run: the programmed value at each point, its jerk
 * limit (0 = none) and position limits, whether the run starts on a curve
 * sent before, with its value, rate and second derivative there, and on
 * success the value, the rate and the second derivative per unit path
 * length of the curve at each point. */
struct OrientSmoothAxis {
    std::vector<double> y;
    double jerk = 0.0;
    double min = -HUGE_VAL, max = HUGE_VAL;
    bool start_fixed = false;
    double start_val = 0.0, start_d1 = 0.0, start_d2 = 0.0;
    std::vector<double> val, d1, d2;
};

/* what the fit settled on, for tests */
struct OrientSmoothInfo {
    double spacing;             // breakpoint spacing of the curve taken
    double lambda;              // its smoothing weight
    double t_raw, t_fit;        // estimated times of the lines and of the curve
};

/* The rotary axes along a run of moves, s the path length at each point
 * (s[0] = 0, increasing). Each axis becomes one smooth curve of the path
 * length through both ends of the run, or from the curve it starts on,
 * the same smoothing for every axis, so that the distance from the
 * programmed moves over all the axes, the curve followed move by move as
 * the quintic its ends give, stays within tol, and each axis within its
 * position limits. spacing is the coarsest breakpoint spacing tried.
 * False, leaving the run as programmed, when no curve is found or none is
 * estimated to run faster than the moves under the axis jerk limits. */
bool orientSmooth(const std::vector<double> &s, std::vector<OrientSmoothAxis> &axes,
        double tol, double spacing, OrientSmoothInfo *info = nullptr);

/* A move of a run held back, a line or an arc: its path length as the
 * planner takes it and its programmed end, in; on the curve, out. */
struct OrientMove {
    double length;
    double rot[ORIENT_NAX];     // programmed A to W at its end
    unsigned mask;              // the axes on the curve, bit k for A + k
    double out[ORIENT_NAX];     // their end on the curve
    double shape[4 * ORIENT_NAX]; // d0, dd0, d1, dd1, see tpSetOrientShape()
};

/* the angular axes among A to W (bit k for A + k) and their limits */
struct OrientAxes {
    unsigned angular;
    double jerk[ORIENT_NAX];
    double min[ORIENT_NAX], max[ORIENT_NAX];
};

/* where a run sent in part leaves the curve for the rest of it */
struct OrientCarry {
    unsigned mask;              // axes on the curve there
    double val[ORIENT_NAX], d1[ORIENT_NAX], d2[ORIENT_NAX];
};

/* Fit the run of moves from start (programmed A to W), on the curve in
 * leaves off, and fill mask, out and shape of the first emit moves; out is
 * where the rest of the run then starts. False, leaving the moves as
 * programmed, when the run is not smoothed. */
bool orientSmoothRun(const double start_rot[ORIENT_NAX], const OrientAxes &axes,
        std::vector<OrientMove> &moves, size_t emit, double tol, double spacing,
        const OrientCarry &in, OrientCarry &out, OrientSmoothInfo *info = nullptr);

/* The first emit moves of the run, as orientSmoothRun() but in parts where
 * the run as a whole is not smoothed: a fit that fails is tried again on
 * the front half of the moves, down to a few moves, and where even those
 * fail the first of them stay as programmed and the fits go on after them,
 * so that what cannot be smoothed (a fast swing near a pole, a corner, a
 * limit) leaves the rest of the run smoothed. Returns the moves smoothed. */
size_t orientSmoothSend(const double start_rot[ORIENT_NAX], const OrientAxes &axes,
        std::vector<OrientMove> &moves, size_t emit, double tol, double spacing,
        const OrientCarry &in, OrientCarry &out);

#endif
