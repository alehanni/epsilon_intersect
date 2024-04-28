#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generator_exception.hpp>
#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/generators/catch_generators_random.hpp>
#include <catch2/generators/catch_generators_range.hpp>

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

TEST_CASE("line_intersect_gg3_random_double_100") {

    using Catch::Generators::RandomFloatingGenerator;
    using Catch::Approx;

    // generate 100 seeds
    int seed = GENERATE(take(100, random(0, 10000)));

    RandomFloatingGenerator<double> rng2(-100.0, 100.0, seed);
    RandomFloatingGenerator<double> rng3(-1.0, 0.0, seed);

    // guarantee intersection by building line segments from intersection point
    double ix, iy, p1x, p1y, p2x, p2y, q1x, q1y, q2x, q2y;

    ix = rng2.get(); rng2.next();
    iy = rng2.get(); rng2.next();

    p1x = rng2.get(); rng2.next();
    p1y = rng2.get(); rng2.next();
    double s1 = rng3.get(); rng3.next();
    p2x = ix + (p1x - ix) * s1;
    p2y = iy + (p1y - iy) * s1;

    q1x = rng2.get(); rng2.next();
    q1y = rng2.get(); rng2.next();
    double s2 = rng3.get(); rng3.next();
    q2x = ix + (q1x - ix) * s2;
    q2y = iy + (q1y - iy) * s2;

    // compute intersection
    double det, sdet, tdet, t, rx, ry;
    line_intersect_gg3<double>(p1x, p1y, p2x, p2y, q1x, q1y, q2x, q2y, det, sdet, tdet);
    t = tdet / det;
    rx = p1x + (p2x - p1x) * t;
    ry = p1y + (p2y - p1y) * t;

    INFO("Intersect at x: " << rx << " y: " << ry);
    INFO("True value x: " << ix << " y: " << iy);

    REQUIRE(rx == Approx(ix));
    REQUIRE(ry == Approx(iy));
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

TEST_CASE("dist_to_line_sq_gg2_random_double_100") {
    
    using Catch::Generators::RandomFloatingGenerator;
    using Catch::Approx;

    // generate 100 seeds
    int seed = GENERATE(take(100, random(0, 10000)));

    RandomFloatingGenerator<double> rng(-100.0, 100.0, seed);
    RandomFloatingGenerator<double> rng2(0.0, 1.0, seed);
    RandomFloatingGenerator<double> rng3(-1.0, 0.0, seed);

    // build point an line so that the nearest point is known
    double nx, ny, px, py, q1x, q1y, q2x, q2y, s1, s2;

    nx = rng.get(); rng.next();
    ny = rng.get(); rng.next();

    px = rng.get(); rng.next();
    py = rng.get(); rng.next();

    // orthogonal to n -> p with positive scalar
    s1 = rng2.get(); rng2.next();
    q1x = nx + (py - ny) * s1;
    q1y = ny - (px - nx) * s1;

    // orthogonal to n -> p with negative scalar
    s2 = rng3.get(); rng3.next();
    q2x = nx + (py - ny) * s2;
    q2y = ny - (px - nx) * s2;

    // compute distance
    double d_sq_true, d_sq_comp;
    d_sq_true = (nx - px) * (nx - px) + (ny - py) * (ny - py);
    d_sq_comp = dist_to_line_sq_gg2(px, py, q1x, q1y, q2x, q2y);

    REQUIRE(d_sq_comp == Approx(d_sq_true));
}