#include "gforce/brain_graphics/field_map.hpp"   // adjust to your layout
#include "pros/screen.hpp"
#include <cmath>

namespace map {
    // layout constants — internal to this file now
    constexpr int    MAP_X0   = 250;
    constexpr int    MAP_Y0   = 20;
    constexpr int    MAP_SIZE = 200;
    constexpr double FIELD_IN = 144.0;
    constexpr double SCALE    = MAP_SIZE / FIELD_IN;

    // internal helper — not in the header, file-private
    static void fieldToScreen(double fx, double fy, int& sx, int& sy) {
        sx = MAP_X0 + (int)((fx + 72.0) * SCALE);
        sy = MAP_Y0 + (int)((72.0 - fy) * SCALE);   // flip y
    }

    void drawField() {
        pros::screen::set_pen(pros::Color::white);
        pros::screen::draw_rect(MAP_X0, MAP_Y0, MAP_X0 + MAP_SIZE, MAP_Y0 + MAP_SIZE);

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
        pros::screen::fill_circle(sx, sy, 5);

        double r = pose.theta * M_PI / 180.0;
        int hx = sx + (int)(14 * std::sin(r));
        int hy = sy - (int)(14 * std::cos(r));
        pros::screen::draw_line(sx, sy, hx, hy);
    }
}