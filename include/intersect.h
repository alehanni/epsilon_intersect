#ifndef INTERSECT_H
#define INTERSECT_H

#include <functional>
#include <limits>
#include <cmath>
#include <cassert>
#include <cstdio>

template<class T>
std::function<T(T, T)> line_eq_gg3(auto &p, auto &q) {
    return [p, q](T x, T y){
        return (x - p.x)*(q.y - p.y) - (y - p.y)*(q.x - p.x);
    };
}

template<class T>
void line_intersect_gg3(auto &p1, auto &p2, auto &q1, auto &q2, T &det, T &sdet, T &tdet) {

    auto line_p = line_eq_gg3<T>(p1, p2);
    auto line_q = line_eq_gg3<T>(q1, q2);

    T a = line_p(q1.x, q1.y);
    T b = line_p(q2.x, q2.y);

    T c = line_q(p1.x, p1.y);
    // T d = line_q(p2x, p2y);

    det = a - b;
    sdet = a; // s = sdet/det is the parameter for the intersection point on q
    tdet = -c; // t = tdet/det is the parameter for the intersection point on p
}

template<class T>
void line_intersect_vline(T p1x, T p2x, T qx, T &det, T &tdet) {
    tdet = qx - p1x;
    det = p2x - p1x;
}

template<class T>
void line_intersect_hline(T p1y, T p2y, T qy, T &det, T &tdet) {
    tdet = qy - p1y;
    det = p2y - p1y;
}

//template<class T>
//static inline T dist_sq(T ax, T ay, T bx, T by) {
//    T dx = bx - ax;
//    T dy = by - ay;
//    return (dx * dx + dy * dy);
//}
//
//template<class T>
//inline void normalize(T &x, T &y) {
//    constexpr T eps = static_cast<T>(1e-8);
//    T d = dist_sq(static_cast<T>(0), static_cast<T>(0), x, y);
//    assert(d > eps);
//    x = x / d;
//    y = y / d;
//}

template<class T>
T dist_to_line_sq_gg2(auto &p, auto &a, auto &b) {
    T py_ay, bx_ax, px_ax, by_ay, a2, d2_sq;

    py_ay = p.y - a.y;
    bx_ax = b.x - a.x;
    px_ax = p.x - a.x;
    by_ay = b.y - a.y;

    a2 = py_ay * bx_ax - px_ax * by_ay;
    d2_sq = (a2 * a2) / (bx_ax * bx_ax + by_ay * by_ay);

    return d2_sq;
}

template<class T>
T dist_to_seg_sq_gg2(auto &p, auto &a, auto &b) {
    T py_ay, bx_ax, px_ax, by_ay, bx_px, by_py, a2, t, d2_sq;

    py_ay = p.y - a.y;
    bx_ax = b.x - a.x;
    px_ax = p.x - a.x;
    by_ay = b.y - a.y;

    // a2 = py_ay * bx_ax - px_ax * by_ay;
    // d1_sq = (a2 * a2) / (bx_ax * bx_ax + by_ay * by_ay);

    t = px_ax * bx_ax + py_ay * by_ay;
    if (t < static_cast<T>(0)) {
        d2_sq = px_ax * px_ax + py_ay * py_ay; // dist_sq(ax, ay, px, py);
    } else {
        bx_px = b.x - p.x;
        by_py = b.y - p.y;
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
bool aabb_intersect(auto &a, auto &b) {
    return false;
}

template<class T>
struct aabb {
    T xmin, ymin, xmax, ymax;
};

template<class T>
struct point {
    T x, y;
};

template<class T>
void boxcast(auto &p1, auto &p2, auto &q1, auto &q2, aabb<T> box, T &det, T &tdet) {
    T nx, ny, wx, wy, sx, sy, ex, ey, offx, offy;

    // determine quadrant of q
    if ((q2.x - q1.x > 0) == (q2.y - q1.y > 0)) {
        // boxes located in corners nw and se
        nx = wx = std::min(q1.x, q2.x);
        ny = wy = std::min(q1.y, q2.y);
        sx = ex = std::max(q1.x, q2.x);
        sy = ey = std::max(q1.y, q2.y);
        
        if (q2.x - q1.x > 0) { // assuming ccw winding
            offx = box.xmin;
            offy = box.ymax;
        } else {
            offx = box.xmax;
            offy = box.ymin;
        }
    } else {
        // boxes located in corners sw and ne
        sx = wx = std::min(q1.x, q2.x);
        sy = wy = std::max(q1.y, q2.y);
        nx = ex = std::max(q1.x, q2.x);
        ny = ey = std::min(q1.y, q2.y);

        if (q2.x - q1.x > 0) {
            offx = box.xmax;
            offy = box.ymax;
        } else {
            offx = box.xmin;
            offy = box.ymin;
        }
    }

    // check certain pair of faces depending on quadrant of p
    T pdx, pdy;
    pdx = p2.x - p1.x;
    pdy = p2.y - p1.y;

    if (pdy > 0) {
        // check north face
        line_intersect_hline(p1.y, p2.y, ny + box.ymin, det, tdet);
        T ix = p1.x + (p2.x - p1.x) * (tdet / det);
        if (nx + box.xmin <= ix && ix <= nx + box.xmax)
            return;
    } else { // pdy < 0
        // check south face
        line_intersect_hline(p1.y, p2.y, sy + box.ymax, det, tdet);
        T ix = p1.x + (p2.x - p1.x) * (tdet / det);
        if (sx + box.xmin <= ix && ix <= sx + box.xmax)
            return;
    }

    if (pdx > 0) {
        // check west face
        line_intersect_vline(p1.x, p2.x, wx + box.xmin, det, tdet);
        T iy = p1.y + (p2.y - p1.y) * (tdet / det);
        if (wy + box.ymin <= iy && iy <= wy + box.ymax)
            return;
    } else { // pdx < 0
        // check east face
        line_intersect_vline(p1.x, p2.x, ex + box.xmax, det, tdet);
        T iy = p1.y + (p2.y - p1.y) * (tdet / det);
        if (ey + box.ymin <= iy && iy <= ey + box.ymax)
            return;
    }

    // check offset line intersect
    T sdet;
    point<T> q1_off, q2_off;
    q1_off = {q1.x + offx, q1.y + offy};
    q2_off = {q2.x + offx, q2.y + offy};
    line_intersect_gg3(p1, p2, q1_off, q2_off, det, sdet, tdet);

    if (sdet < 0.0 || sdet > det)
        tdet = -det; // invalidate tdet, TEMPORARY

    // if tdet >= 0 and tdet <= det, t = tdet/det is the parameter for the intersection point on p
}

template<class T>
void box_sweep_lines(auto &p, auto &q, aabb<T> bb, T *dest) {
    // get all the lines defining a box swept across a line segment
    T xmax, xmin, ymax, ymin;
    
    if (p.x > q.x) {
        xmax = p.x;
        xmin = q.x;
    } else {
        xmax = q.x;
        xmin = p.x;
    }

    if (p.y > q.y) {
        ymax = p.y;
        ymin = q.y;
    } else {
        ymax = q.y;
        ymin = p.y;
    }

    // write points defining the bounding box lines (ccw winding)
    // north face
    *(dest++) = xmax + bb.xmax;
    *(dest++) = ymin + bb.ymin;
    *(dest++) = xmin + bb.xmin;
    *(dest++) = ymin + bb.ymin;
    // west face
    *(dest++) = xmin + bb.xmin;
    *(dest++) = ymin + bb.ymin;
    *(dest++) = xmin + bb.xmin;
    *(dest++) = ymax + bb.ymax;
    // south face
    *(dest++) = xmin + bb.xmin;
    *(dest++) = ymax + bb.ymax;
    *(dest++) = xmax + bb.xmax;
    *(dest++) = ymax + bb.ymax;
    // east face
    *(dest++) = xmax + bb.xmax;
    *(dest++) = ymax + bb.ymax;
    *(dest++) = xmax + bb.xmax;
    *(dest++) = ymin + bb.ymin;

    // get offset diagonal lines
    if (q.x - p.x > 0 == q.y - p.y > 0) {
        // offset to top-right corner
        *(dest++) = p.x + bb.xmax;
        *(dest++) = p.y + bb.ymin;
        *(dest++) = q.x + bb.xmax;
        *(dest++) = q.y + bb.ymin;
        // offset to bottom-left corner
        *(dest++) = p.x + bb.xmin;
        *(dest++) = p.y + bb.ymax;
        *(dest++) = q.x + bb.xmin;
        *(dest++) = q.y + bb.ymax;
    } else {
        // offset to top-left corner
        *(dest++) = p.x + bb.xmin;
        *(dest++) = p.y + bb.ymin;
        *(dest++) = q.x + bb.xmin;
        *(dest++) = q.y + bb.ymin;
        // offset to bottom-right corner
        *(dest++) = p.x + bb.xmax;
        *(dest++) = p.y + bb.ymax;
        *(dest++) = q.x + bb.xmax;
        *(dest++) = q.y + bb.ymax;
    }
}

#endif