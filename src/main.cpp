
#include "raylib.h"
#include "raymath.h"
#include "intersect.h"

constexpr Vector2 segments[] = {
    {100.0, 100.0},
    {300.0, 120.0},
    {400.0, 100.0},
    {500.0, 100.0},
    {500.0, 200.0},
    {600.0, 300.0},
    {500.0, 400.0},

    // spikes
    {390.0, 400.0},
    {380.0, 380.0},
    {370.0, 400.0},
    {360.0, 380.0},
    {350.0, 400.0},
    {340.0, 380.0},
    {330.0, 400.0},
    //{320.0, 380.0},
    {310.0, 400.0},
    {300.0, 380.0},
    {290.0, 400.0},
    {280.0, 380.0},
    {270.0, 400.0},
    {260.0, 380.0},
    {250.0, 400.0},
    {240.0, 380.0},
    {230.0, 400.0},

    {200.0, 400.0},
    {100.0, 300.0},
    {100.0, 100.0}
};

constexpr size_t n_segments = sizeof(segments) / sizeof(Vector2);

void update_draw_frame() {
    
    auto mpos = GetMousePosition();

    // do a boxcast against edges
    float det, tdet, _det, _tdet;
    det = tdet = 1.0; // note: intersection will be smaller than s = t = 1.0

    Vector2 middle = {320.f, 240.f};
    aabb<float> my_box = {-6.f, -12.f, 6.f, 12.f};

    for (size_t i=1; i<n_segments; i++) {
        auto q1 = segments[i - 1];
        auto q2 = segments[i];
        boxcast<float>(middle, mpos, q1, q2, my_box, _det, _tdet);

        float t = tdet / det;
        float _t = _tdet / _det;
        if (0.0 <= _t && _t < t){
            tdet = _tdet;
            det = _det;
        }
    }

    float t, target_x, target_y;
    t = tdet / det;
    if (0.0 <= t && t <= 1.0) {
        target_x = 320.f + (mpos.x - 320.0f) * t;
        target_y = 240.f + (mpos.y - 240.0f) * t;
    } else {
        target_x = mpos.x;
        target_y = mpos.y;
    }

    // draw line between desired and corrected position
    BeginDrawing();
    ClearBackground(DARKGRAY);
    DrawLine(320, 240, target_x, target_y, RED);
    DrawRectangleLines(target_x + my_box.xmin, target_y + my_box.ymin, my_box.xmax - my_box.xmin, my_box.ymax - my_box.ymin, RED);
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