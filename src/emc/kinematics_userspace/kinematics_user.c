/********************************************************************
 * Description: kinematics_user.c
 *   Non-RT loader for kinematics modules
 *
 * Loads a kinematics .so with dlopen and calls the nonrt_attach() it
 * exports.  The module binds its own haldata to the value cells of the
 * pins the running RT instance owns and hands back its unmodified
 * forward and inverse entry points, so this process evaluates exactly
 * the kinematics the machine is running, at whatever poses it likes.
 *
 * Identity kinematics needs no module code: the module says so through
 * nonrt_ops_t and this file maps joints to axes directly.
 *
 * A module that does not export nonrt_attach() is not an error.  The
 * context comes back flagged rt_only and the caller does without
 * kinematic limits for that machine.
 *
 * Author: LinuxCNC
 * License: GPL Version 2
 * System: Linux
 *
 * Copyright (c) 2024 All rights reserved.
 ********************************************************************/

#include "kinematics_user.h"
#include "../kinematics/nonrt_kins.h"
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "config.h"  /* EMC2_HOME */

typedef int (*nonrt_attach_fn)(const char *coordinates, nonrt_ops_t *ops);

struct KinematicsUserContext {
    int initialized;
    int rt_only;               /* 1 if the module exports no nonrt_attach() */
    int is_identity;           /* 1 for identity kinematics: no module code needed */
    KINEMATICS_TYPE kins_type;
    void *rt_handle;           /* dlopen handle */
    nonrt_ops_t ops;
    int num_joints;
    int joint_to_axis[KINEMATICS_USER_MAX_JOINTS]; /* identity path only */
    char module_name[64];
};

/* ========================================================================
 * Identity joint mapping
 * ======================================================================== */

static void fill_identity_joint_map(KinematicsUserContext *ctx, const char *coords)
{
    int i, j = 0;
    for (i = 0; i < KINEMATICS_USER_MAX_JOINTS; i++) ctx->joint_to_axis[i] = -1;
    if (!coords) return;
    for (; *coords && j < ctx->num_joints; coords++) {
        int axis;
        switch (tolower((unsigned char)*coords)) {
            case 'x': axis = 0; break; case 'y': axis = 1; break;
            case 'z': axis = 2; break; case 'a': axis = 3; break;
            case 'b': axis = 4; break; case 'c': axis = 5; break;
            case 'u': axis = 6; break; case 'v': axis = 7; break;
            case 'w': axis = 8; break; default:  continue;
        }
        ctx->joint_to_axis[j++] = axis;
    }
}

/* ========================================================================
 * Module loading
 * ======================================================================== */

static int load_module(KinematicsUserContext *ctx,
                       const char *module_name,
                       const char *coordinates)
{
    char module_path[512];
    void *handle;
    nonrt_attach_fn attach;

    snprintf(module_path, sizeof(module_path),
             "%s/rtlib/%s.so", EMC2_HOME, module_name);

    handle = dlopen(module_path, RTLD_NOW | RTLD_LOCAL);
    if (!handle) {
        fprintf(stderr, "kinematicsUserInit: dlopen '%s': %s\n",
                module_path, dlerror());
        return -1;
    }
    ctx->rt_handle = handle;

    attach = (nonrt_attach_fn)dlsym(handle, "nonrt_attach");
    if (!attach) {
        fprintf(stderr, "kinematicsUserInit: '%s' exports no nonrt_attach\n",
                module_name);
        dlclose(handle);
        ctx->rt_handle = NULL;
        ctx->rt_only = 1;
        return -1;
    }

    if (attach(coordinates, &ctx->ops) != 0) {
        fprintf(stderr, "kinematicsUserInit: nonrt_attach failed for '%s'\n",
                module_name);
        dlclose(handle);
        ctx->rt_handle = NULL;
        ctx->rt_only = 1;
        return -1;
    }

    if (ctx->ops.is_identity) {
        ctx->is_identity = 1;
        ctx->kins_type = KINEMATICS_IDENTITY;
        return 0;
    }

    if (!ctx->ops.forward || !ctx->ops.inverse) {
        fprintf(stderr, "kinematicsUserInit: '%s' set no fwd/inv\n", module_name);
        dlclose(handle);
        ctx->rt_handle = NULL;
        ctx->rt_only = 1;
        return -1;
    }

    ctx->kins_type = KINEMATICS_BOTH;
    return 0;
}

/* ========================================================================
 * Public API
 * ======================================================================== */

KinematicsUserContext* kinematicsUserInit(const char* kins_type,
                                          int num_joints,
                                          const char* coordinates)
{
    KinematicsUserContext *ctx;

    if (!kins_type || num_joints < 1 || num_joints > KINEMATICS_USER_MAX_JOINTS) {
        fprintf(stderr, "kinematicsUserInit: invalid arguments\n");
        return NULL;
    }

    ctx = (KinematicsUserContext *)calloc(1, sizeof(KinematicsUserContext));
    if (!ctx) return NULL;

    ctx->num_joints = num_joints;
    strncpy(ctx->module_name, kins_type, sizeof(ctx->module_name) - 1);

    load_module(ctx, kins_type, coordinates);

    if (ctx->is_identity) {
        fill_identity_joint_map(ctx, coordinates);
    }

    ctx->initialized = 1;
    return ctx;
}

int kinematicsUserInverse(KinematicsUserContext* ctx,
                          const EmcPose* world,
                          double* joints)
{
    if (!ctx || !ctx->initialized || !world || !joints) return -1;

    if (ctx->is_identity) {
        int i;
        for (i = 0; i < ctx->num_joints; i++) {
            int ax = ctx->joint_to_axis[i];
            joints[i] = (ax >= 0) ? emcPoseGetAxis(world, ax) : 0.0;
        }
        return 0;
    }

    if (ctx->rt_only) return -1;
    return ctx->ops.inverse(world, joints, NULL, NULL);
}

int kinematicsUserForward(KinematicsUserContext* ctx,
                          const double* joints,
                          EmcPose* world)
{
    if (!ctx || !ctx->initialized || !joints || !world) return -1;

    if (ctx->is_identity) {
        int i;
        memset(world, 0, sizeof(*world));
        for (i = 0; i < ctx->num_joints; i++) {
            int ax = ctx->joint_to_axis[i];
            if (ax >= 0) emcPoseSetAxis(world, ax, joints[i]);
        }
        return 0;
    }

    if (ctx->rt_only) return -1;
    return ctx->ops.forward(joints, world, NULL, NULL);
}

int kinematicsUserIsIdentity(KinematicsUserContext* ctx)
{
    if (!ctx || !ctx->initialized) return 0;
    return ctx->is_identity;
}

int kinematicsUserGetNumJoints(KinematicsUserContext* ctx)
{
    if (!ctx || !ctx->initialized) return 0;
    return ctx->num_joints;
}

KINEMATICS_TYPE kinematicsUserGetType(KinematicsUserContext* ctx)
{
    if (!ctx || !ctx->initialized) return KINEMATICS_IDENTITY;
    return ctx->kins_type;
}

const char* kinematicsUserGetModuleName(KinematicsUserContext* ctx)
{
    if (!ctx || !ctx->initialized) return "unknown";
    return ctx->module_name;
}

int kinematicsUserRefreshParams(KinematicsUserContext* ctx)
{
    (void)ctx;
    return 0; /* nothing to refresh: the bound pins are the live values */
}

int kinematicsUserIsRtOnly(KinematicsUserContext* ctx)
{
    if (!ctx || !ctx->initialized) return 1;
    return ctx->rt_only;
}

void kinematicsUserFree(KinematicsUserContext* ctx)
{
    if (ctx) {
        if (ctx->rt_handle) dlclose(ctx->rt_handle);
        free(ctx);
    }
}
