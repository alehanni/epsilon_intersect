#include <catch2/catch_test_macros.hpp>

#include "intersect.h"

TEST_CASE("line_intersect_gg3_basic") {

    int32_t ax, ay, bx, by, cx, cy, dx, dy;

    ax = -5;
    ay = 0;
    bx = 5;
    by = 0;

    cx = 1;
    cy = -4;
    dx = 1;
    dy = 4;

    int32_t det, sdet, tdet;
    line_intersect_gg3<int32_t>(ax, ay, bx, by, cx, cy, dx, dy, det, sdet, tdet);

    REQUIRE( det == 80 );
    REQUIRE( sdet == 40 );
    REQUIRE( tdet == 48 );
}

TEST_CASE("line_intersect_gg3_random_100") {

    

}

TEST_CASE("dist_to_line_sq_gg2_basic") {

    int32_t px, py, ax, ay, bx, by;

    px = 0;
    py = 5;

    ax = -3;
    ay = 1;
    bx = 3;
    by = 1;

    auto d = dist_to_line_sq_gg2<int32_t>(px, py, ax, ay, bx, by);

    REQUIRE( d == 16 );
}