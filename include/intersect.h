#ifndef INTERSECT_H
#define INTERSECT_H

#include <functional>

template<class T>
std::function<T(T, T)> line_eq_gg3(T x1, T y1, T x2, T y2) {
    return [x1, y1, x2, y2](T x, T y){
        return (x - x1)*(y2 - y1) - (y - y1)*(x2 - x1);
    };
}

template<class T>
void line_intersect_gg3(T p1x, T p1y, T p2x, T p2y, T q1x, T q1y, T q2x, T q2y, T &det, T &sdet, T &tdet) {

    auto line_p = line_eq_gg3<T>(p1x, p1y, p2x, p2y);
    auto line_q = line_eq_gg3<T>(q1x, q1y, q2x, q2y);

    T a = line_p(q1x, q1y);
    T b = line_p(q2x, q2y);

    T c = line_q(p1x, p1y);
    // T d = line_q(p2x, p2y);

    det = a - b;
    sdet = a; // sdet/det == s parametrizes intersection point on q
    tdet = -c; // tdet/det == t parametrizes intersection point on p
}

template<class T>
static inline T dist_sq(T ax, T ay, T bx, T by) {
    T dx = bx - ax;
    T dy = by - ay;
    return (dx * dx + dy * dy);
}

template<class T>
T dist_to_line_sq_gg2(T px, T py, T ax, T ay, T bx, T by) {
    T py_ay, bx_ax, px_ax, by_ay, bx_px, by_py, a2, t, d2_sq;

    py_ay = py - ay;
    bx_ax = bx - ax;
    px_ax = px - ax;
    by_ay = by - ay;

    // a2 = py_ay * bx_ax - px_ax * by_ay;
    // d1_sq = (a2 * a2) / (bx_ax * bx_ax + by_ay * by_ay);

    t = px_ax * bx_ax + py_ay * by_ay;
    if (t < static_cast<T>(0)) {
        d2_sq = px_ax * px_ax + py_ay * py_ay; // dist_sq(ax, ay, px, py);
    } else {
        bx_px = bx - px;
        by_py = by - py;
        t = bx_px * bx_ax + by_py * by_ay;
        if (t < static_cast<T>(0)) {
            d2_sq = bx_px * bx_px + by_py * by_py; // dist_sq(bx, by, px, py);
        } else {
            a2 = py_ay * bx_ax - px_ax * by_ay;
            d2_sq = (a2 * a2) / (bx_ax * bx_ax + by_ay * by_ay);
        }
    }

    return d2_sq;
}

template<class T>
bool aabb_intersect(T ax, T ay, T bx, T by, T cx, T cy, T dx, T dy) {
    return false;
}

#endif