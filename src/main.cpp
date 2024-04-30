#include <cassert>

#include "raylib.h"
#include "raymath.h"
#include "intersect.h"

// small demo of robust line-line intersections

bool epsilon_intersect(Vector2 p1, Vector2 p2, Vector2 q1, Vector2 q2, float &det, float &sdet, float &tdet) {
    constexpr float eps = 0.5;
    
    // todo: pre-check using bounding boxes

    // get intersection point on line q
    float s, qix, qiy;
    line_intersect_gg3<float>(p1.x, p1.y, p2.x, p2.y, q1.x, q1.y, q2.x, q2.y, det, sdet, tdet);

    if (det < 1e-6)
        return {}; // parallel lines

    s = sdet / det;
    s = (s < 0.0) ? 0.0 : s;
    s = (s > 1.0) ? 1.0 : s;
    qix = q1.x + (q2.x - q1.x) * s;
    qiy = q1.y + (q2.y - q1.y) * s;

    // check if intersection point is within line p's tolerance
    double d_sq;
    d_sq = dist_to_line_sq_gg2(qix, qiy, p1.x, p1.y, p2.x, p2.y);

    return (d_sq <= eps * eps);
}

constexpr Vector2 segments[] = {
    {100.0, 100.0},
    {300.0, 120.0},
    {400.0, 100.0},
    {500.0, 100.0},
    {600.0, 300.0},
    {500.0, 400.0},
    {200.0, 400.0},
    {100.0, 300.0},
    {100.0, 100.0}
};

constexpr size_t n_segments = sizeof(segments) / sizeof(Vector2);

Vector2 p_prev = {320, 240};
Vector2 p_desired = {320, 240};

bool epsilon_intersect_vs_all(Vector2 p1, Vector2 p2, Vector2 &q1, Vector2 &q2, float &det, float &sdet, float &tdet) {
    // get nearest intersection when comparing against all segments
    bool intersect_atleast_once = false;

    det = sdet = tdet = 1.0; // note: intersection will be smaller than s = t = 1.0
    float _det, _sdet, _tdet;

    for (size_t i=1; i<n_segments; i++) {
        auto _q1 = segments[i - 1];
        auto _q2 = segments[i];

        if (epsilon_intersect(p1, p2, _q1, _q2, _det, _sdet, _tdet)) {
            if (_tdet * det < tdet * _det) {
                intersect_atleast_once = true;
                q1 = _q1;
                q2 = _q2;
                sdet = _sdet;
                tdet = _tdet;
                det = _det;
            }
        }
    }

    return intersect_atleast_once;
}

Vector2 get_ortho(Vector2 in) {
    Vector2 out;
    out.x = -in.y;
    out.y = in.x;
    return out;
}

void collision_step(Vector2 from, Vector2 to, Vector2 &out_pos, Vector2 &out_vel, float &dt) {
    constexpr float dt_eps = 1e-4;
    // velocity = (target_pos - in_pos) / dt => target_pos = velocity * dt + in_pos

    float det, sdet, tdet;
    Vector2 q1, q2;
    if (epsilon_intersect_vs_all(from, to, q1, q2, det, sdet, tdet)) {
        float s, t; // note: t is just the parameter name, not time
        s = sdet / det;
        t = tdet / det;
        dt -= (t > 0.0) ? t * dt : dt_eps; // t is the fraction of dt elapsed until the collision occured

        Vector2 qi1;
        qi1.x = q1.x + (q2.x - q1.x) * s;
        qi1.y = q1.y + (q2.y - q1.y) * s;

        Vector2 ortho = get_ortho(Vector2Subtract(q2, q1));
        Vector2 to2 = Vector2Add(to, ortho);
        float det2, sdet2, tdet2, s2;
        line_intersect_gg3(to.x, to.y, to2.x, to2.y, q1.x, q1.y, q2.x, q2.y, det2, sdet2, tdet2);
        assert(abs(det2) > 1e-6 && "orthogonal can't be parallel");
        s2 = sdet2 / det2;

        Vector2 qi2;
        qi2.x = q1.x + (q2.x - q1.x) * s2;
        qi2.y = q1.y + (q2.y - q1.y) * s2;

        out_pos = qi1;
        out_vel.x = (qi2.x - qi1.x) / dt; // note: dt is smaller now
        out_vel.y = (qi2.y - qi1.y) / dt;
    } else {
        out_pos = to;
        out_vel.x = (to.x - from.x) / dt;
        out_vel.y = (to.y - from.y) / dt;
        dt = 0.0;        
    }
}

void update_draw_frame() {
    
    auto mpos = GetMousePosition();
    p_desired.x = 0.9 * p_desired.x + 0.1 * mpos.x; // hit 'em with the low-pass
    p_desired.y = 0.9 * p_desired.y + 0.1 * mpos.y;
    float dt = 1.0;

    for(size_t timeout = 0; dt > 0.0 && timeout < 5; timeout++) {
        Vector2 out_pos, out_vel;
        collision_step(p_prev, p_desired, out_pos, out_vel, dt);
        p_prev = out_pos;
        p_desired = Vector2Add(out_pos, Vector2Scale(out_vel, dt));
    }
    
    // draw line between desired and corrected position
    BeginDrawing();
    ClearBackground(DARKGRAY);
    DrawCircleV(p_prev, 2.0, PINK);
    DrawLineV(p_prev, mpos, RED);
    DrawLineStrip((Vector2 *)segments, n_segments, WHITE);
    EndDrawing();
}

int main(int argc, char **argv) {
    constexpr int screen_width = 640;
    constexpr int screen_height = 480;

    InitWindow(screen_width, screen_height, "intersect");

#if defined(PLATFORM_WEB)
    emscripten_set_main_loop(update_draw_frame, 0, 1);
#else
    SetTargetFPS(60);
    while(!WindowShouldClose()) {
        update_draw_frame();
    }
#endif

    CloseWindow();

    return 0;
}