#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generator_exception.hpp>
#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/generators/catch_generators_random.hpp>
#include <catch2/generators/catch_generators_range.hpp>

#include "intersect.h"

TEST_CASE("line_intersect_gg3_basic") {

    //int32_t ax, ay, bx, by, cx, cy, dx, dy;
    point<int32_t> a, b, c, d;

    a.x = -5;
    a.y = 0;
    b.x = 5;
    b.y = 0;

    c.x = 1;
    c.y = -4;
    d.x = 1;
    d.y = 4;

    int32_t det, sdet, tdet;
    line_intersect_gg3<int32_t>(a, b, c, d, det, sdet, tdet);

    REQUIRE( det == 80 );
    REQUIRE( sdet == 40 );
    REQUIRE( tdet == 48 );
}

TEST_CASE("line_intersect_gg3_parallel_lines") {

    //int32_t ax, ay, bx, by, cx, cy, dx, dy;
    point<int32_t> a, b, c, d;

    a.x = -5;
    a.y = 0;
    b.x = 5;
    b.y = 4;

    c.x = -5;
    c.y = 1;
    d.x = 5;
    d.y = 5;

    int32_t det, sdet, tdet;
    line_intersect_gg3<int32_t>(a, b, c, d, det, sdet, tdet);

    INFO( "det: " << det << " sdet: " << sdet << " tdet: " << tdet);
}

TEST_CASE("line_intersect_gg3_random_double_100") {

    using Catch::Generators::RandomFloatingGenerator;
    using Catch::Approx;

    // generate 100 seeds
    int seed = GENERATE(take(100, random(0, 10000)));

    RandomFloatingGenerator<double> rng2(-100.0, 100.0, seed);
    RandomFloatingGenerator<double> rng3(-1.0, 0.0, seed);

    // guarantee intersection by building line segments from intersection point
    double ix, iy;//, p1x, p1y, p2x, p2y, q1x, q1y, q2x, q2y;
    point<double> p1, p2, q1, q2;

    ix = rng2.get(); rng2.next();
    iy = rng2.get(); rng2.next();

    p1.x = rng2.get(); rng2.next();
    p1.y = rng2.get(); rng2.next();
    double s1 = rng3.get(); rng3.next();
    p2.x = ix + (p1.x - ix) * s1;
    p2.y = iy + (p1.y - iy) * s1;

    q1.x = rng2.get(); rng2.next();
    q1.y = rng2.get(); rng2.next();
    double s2 = rng3.get(); rng3.next();
    q2.x = ix + (q1.x - ix) * s2;
    q2.y = iy + (q1.y - iy) * s2;

    // compute intersection
    double det, sdet, tdet, t, rx, ry;
    line_intersect_gg3<double>(p1, p2, q1, q2, det, sdet, tdet);
    t = tdet / det;
    rx = p1.x + (p2.x - p1.x) * t;
    ry = p1.y + (p2.y - p1.y) * t;

    INFO("Intersect at x: " << rx << " y: " << ry);
    INFO("True value x: " << ix << " y: " << iy);

    REQUIRE(rx == Approx(ix));
    REQUIRE(ry == Approx(iy));
}

TEST_CASE("dist_to_seg_sq_gg2_basic") {

    //int32_t px, py, ax, ay, bx, by;
    point<int32_t> p, a, b;

    p.x = 0;
    p.y = 5;

    a.x = -3;
    a.y = 1;
    b.x = 3;
    b.y = 1;

    auto d = dist_to_seg_sq_gg2<int32_t>(p, a, b);

    REQUIRE( d == 16 );
}

TEST_CASE("dist_to_seg_sq_gg2_random_double_100") {
    
    using Catch::Generators::RandomFloatingGenerator;
    using Catch::Approx;

    // generate 100 seeds
    int seed = GENERATE(take(100, random(0, 10000)));

    RandomFloatingGenerator<double> rng(-100.0, 100.0, seed);
    RandomFloatingGenerator<double> rng2(0.0, 1.0, seed);
    RandomFloatingGenerator<double> rng3(-1.0, 0.0, seed);

    // build point an line so that the nearest point is known
    double nx, ny, s1, s2;//, px, py, q1x, q1y, q2x, q2y, s1, s2;
    point<double> p, q1, q2;

    nx = rng.get(); rng.next();
    ny = rng.get(); rng.next();

    p.x = rng.get(); rng.next();
    p.y = rng.get(); rng.next();

    // orthogonal to n -> p with positive scalar
    s1 = rng2.get(); rng2.next();
    q1.x = nx + (p.y - ny) * s1;
    q1.y = ny - (p.x - nx) * s1;

    // orthogonal to n -> p with negative scalar
    s2 = rng3.get(); rng3.next();
    q2.x = nx + (p.y - ny) * s2;
    q2.y = ny - (p.x - nx) * s2;

    // compute distance
    double d_sq_true, d_sq_comp;
    d_sq_true = (nx - p.x) * (nx - p.x) + (ny - p.y) * (ny - p.y);
    d_sq_comp = dist_to_seg_sq_gg2<double>(p, q1, q2);

    REQUIRE(d_sq_comp == Approx(d_sq_true));
}