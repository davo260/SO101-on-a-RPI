/* test_calib - calibration parser and LeRobot-compatible mapping, no hardware. */
#include "calib.h"

#include <math.h>
#include <stdio.h>

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); fails++; } } while (0)

int main(void)
{
    calib_arm L, F;
    char err[256];
    CHECK(calib_load("tests/fixtures/so101_leader.json", &L, err, sizeof err) == 0);
    CHECK(calib_load("tests/fixtures/so101_follower_sim.json", &F, err, sizeof err) == 0);
    CHECK(calib_load("tests/fixtures/missing.json", &L, err, sizeof err) == -1);

    CHECK(L.j[0].id == 1 && L.j[0].range_min == 729 && L.j[0].range_max == 3452);
    CHECK(L.j[0].homing_offset == -1773 && L.j[2].homing_offset == 1288);
    CHECK(L.j[5].id == 6 && L.j[5].is_gripper && !L.j[4].is_gripper);

    /* Body: range ends map to -100/+100, clamped outside */
    CHECK(fabsf(calib_normalize(&L.j[0], 729) + 100.0f) < 1e-3f);
    CHECK(fabsf(calib_normalize(&L.j[0], 3452) - 100.0f) < 1e-3f);
    CHECK(fabsf(calib_normalize(&L.j[0], 10) + 100.0f) < 1e-3f);
    CHECK(fabsf(calib_normalize(&L.j[0], (729 + 3452) / 2.0) - 0.0f) < 0.05f);
    /* Gripper: 0..100 */
    CHECK(fabsf(calib_normalize(&L.j[5], 2040)) < 1e-3f);
    CHECK(fabsf(calib_normalize(&L.j[5], 3286) - 100.0f) < 1e-3f);

    /* Leader end of range -> follower end of range */
    CHECK(calib_unnormalize(&F.j[0], calib_normalize(&L.j[0], 3452)) == 3400);
    CHECK(calib_unnormalize(&F.j[0], calib_normalize(&L.j[0], 729)) == 700);
    CHECK(calib_unnormalize(&F.j[5], 150.0f) == 3500);     /* clamped */

    /* Round trip on the same arm stays within 1 tick */
    for (int raw = 900; raw < 3000; raw += 37) {
        int back = calib_unnormalize(&L.j[1], calib_normalize(&L.j[1], raw));
        CHECK(back >= raw - 1 && back <= raw);
    }
    /* drive_mode inverts */
    calib_joint inv = L.j[0];
    inv.drive_mode = 1;
    CHECK(fabsf(calib_normalize(&inv, 729) - 100.0f) < 1e-3f);
    CHECK(calib_unnormalize(&inv, 100.0f) == 729);

    printf("%s (%d failures)\n", fails ? "FAILED" : "all calib tests passed", fails);
    return fails != 0;
}
