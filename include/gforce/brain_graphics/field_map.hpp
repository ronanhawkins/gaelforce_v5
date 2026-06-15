#include "pros/screen.hpp"
#include "lemlib/api.hpp"
#include <cmath>

namespace map{
// --- Map layout (right half of the 480x240 screen) ---
constexpr int    MAP_X0   = 250;          // top-left corner of map, px
constexpr int    MAP_Y0   = 20;
constexpr int    MAP_SIZE = 200;          // map is 200x200 px
constexpr double FIELD_IN = 144.0;        // VEX field is 144" square
constexpr double SCALE    = MAP_SIZE / FIELD_IN;   // px per inch

// Field coords (inches, LemLib center-origin) -> screen pixels
void fieldToScreen(double fx, double fy, int& sx, int& sy) {
    sx = MAP_X0 + (int)((fx + 72.0) * SCALE);
    sy = MAP_Y0 + (int)((72.0 - fy) * SCALE);   // flip y: screen grows downward
}

void drawField() {
    pros::screen::set_pen(pros::Color::white);
    pros::screen::draw_rect(MAP_X0, MAP_Y0, MAP_X0 + MAP_SIZE, MAP_Y0 + MAP_SIZE);

    // tile grid every 24"
    pros::screen::set_pen(0x303030);
    for (int i = 1; i < 6; i++) {
        int g = (int)(i * 24 * SCALE);
        pros::screen::draw_line(MAP_X0 + g, MAP_Y0, MAP_X0 + g, MAP_Y0 + MAP_SIZE);
        pros::screen::draw_line(MAP_X0, MAP_Y0 + g, MAP_X0 + MAP_SIZE, MAP_Y0 + g);
    }
}

void drawRobot(lemlib::Pose pose) {
    int sx, sy;
    fieldToScreen(pose.x, pose.y, sx, sy);

    pros::screen::set_pen(pros::Color::red);
    pros::screen::fill_circle(sx, sy, 5);     // robot body

    // heading indicator: LemLib 0° = up, clockwise positive
    double r = pose.theta * M_PI / 180.0;
    int hx = sx + (int)(14 * std::sin(r));
    int hy = sy - (int)(14 * std::cos(r));
    pros::screen::draw_line(sx, sy, hx, hy);
}
}