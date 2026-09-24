/*
 * calib.c - Minimal reader for LeRobot calibration JSON + range mapping.
 *
 * The files are flat ({"joint": {"key": int, ...}, ...}) so a tiny scanner is
 * enough; no JSON library is pulled into the daemon.
 */
#include "calib.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char *const calib_joint_names[CALIB_NJ] = {
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex",   "wrist_roll",    "gripper",
};

/* Finds "key" followed by ':' inside [obj, obj+len) and parses an integer. */
static int get_int(const char *obj, const char *key, int *out)
{
    char pat[48];
    snprintf(pat, sizeof pat, "\"%s\"", key);
    const char *p = strstr(obj, pat);
    if (!p)
        return -1;
    p += strlen(pat);
    while (isspace((unsigned char)*p))
        p++;
    if (*p != ':')
        return -1;
    p++;
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p)
        return -1;
    *out = (int)v;
    return 0;
}

int calib_load(const char *path, calib_arm *arm, char *err, int errlen)
{
    char tmp[8];
    if (!err) {
        err = tmp;
        errlen = sizeof tmp;
    }
    FILE *f = fopen(path, "r");
    if (!f) {
        snprintf(err, (size_t)errlen, "cannot open %s", path);
        return -1;
    }
    char buf[16384];
    size_t n = fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[n] = '\0';

    memset(arm, 0, sizeof *arm);
    for (int k = 0; k < CALIB_NJ; k++) {
        const char *name = calib_joint_names[k];
        char pat[48];
        snprintf(pat, sizeof pat, "\"%s\"", name);
        const char *p = strstr(buf, pat);
        const char *open = p ? strchr(p, '{') : NULL;
        const char *close = open ? strchr(open, '}') : NULL;
        if (!close) {
            snprintf(err, (size_t)errlen, "%s: joint '%s' not found", path, name);
            return -2;
        }
        char obj[512];
        size_t len = (size_t)(close - open);
        if (len >= sizeof obj)
            len = sizeof obj - 1;
        memcpy(obj, open, len);
        obj[len] = '\0';

        calib_joint *j = &arm->j[k];
        int id;
        if (get_int(obj, "id", &id) || get_int(obj, "drive_mode", &j->drive_mode) ||
            get_int(obj, "homing_offset", &j->homing_offset) ||
            get_int(obj, "range_min", &j->range_min) ||
            get_int(obj, "range_max", &j->range_max)) {
            snprintf(err, (size_t)errlen, "%s: '%s' has a missing field", path, name);
            return -2;
        }
        if (id < 0 || id > 253 || j->range_max <= j->range_min) {
            snprintf(err, (size_t)errlen, "%s: '%s' has invalid id/range", path, name);
            return -2;
        }
        j->id = (uint8_t)id;
        j->is_gripper = (k == CALIB_NJ - 1);
    }
    return 0;
}

float calib_normalize(const calib_joint *j, int raw)
{
    const float lo = (float)j->range_min, hi = (float)j->range_max;
    float v = (float)raw;
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    float u = (v - lo) / (hi - lo);            /* 0..1 */
    if (j->is_gripper) {
        float n = u * 100.0f;
        return j->drive_mode ? 100.0f - n : n;
    }
    float n = u * 200.0f - 100.0f;
    return j->drive_mode ? -n : n;
}

int calib_unnormalize(const calib_joint *j, float norm)
{
    float u;
    if (j->is_gripper) {
        if (j->drive_mode) norm = 100.0f - norm;
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 100.0f) norm = 100.0f;
        u = norm / 100.0f;
    } else {
        if (j->drive_mode) norm = -norm;
        if (norm < -100.0f) norm = -100.0f;
        if (norm > 100.0f) norm = 100.0f;
        u = (norm + 100.0f) / 200.0f;
    }
    /* truncation matches LeRobot's int() */
    return (int)(u * (float)(j->range_max - j->range_min) + (float)j->range_min);
}
