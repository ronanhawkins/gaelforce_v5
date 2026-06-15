#pragma once
#include "pros/screen.hpp"
#include "lemlib/api.hpp"
#include <cmath>

namespace mode{
    enum class Auton { MATCH_LEFT, MATCH_RIGHT, SKILLS };  //auton options options

    struct Btn { 
        int x0, y0, x1, y1; 
        const char* label; 
        Auton mode; 
    };  //button shape

    Auton selected = Auton::MATCH_LEFT;  // 3. current choice

    const Btn BTNS[] = {
        { 40,  50, 440,  95, "Match Left",  Auton::MATCH_LEFT  },
        { 40, 105, 440, 150, "Match Right", Auton::MATCH_RIGHT },
        { 40, 160, 440, 205, "Skills",      Auton::SKILLS      },
    };

    void drawSelector() {
        pros::screen::set_pen(pros::Color::black);          // clear background
        pros::screen::fill_rect(0, 0, 480, 240);

        for (const Btn & b : BTNS) {
            if (b.mode == selected) {
                pros::screen::set_pen(pros::Color::green);  // selected = filled
                pros::screen::fill_rect(b.x0, b.y0, b.x1, b.y1);
            } else {
                pros::screen::set_pen(pros::Color::white);  // others = outline
                pros::screen::draw_rect(b.x0, b.y0, b.x1, b.y1);
            }
            pros::screen::set_pen(pros::Color::white);
            pros::screen::print(pros::E_TEXT_MEDIUM, b.x0 + 15, b.y0 + 15, "%s", b.label);
        }
    }

    void runSelector() {
        drawSelector();                 // initial draw
        Auton last = selected;
        while (true) {
            pros::screen_touch_status_s_t t = pros::screen::touch_status();
            //if the screen is pressed, check if it is within the bounds of any button
            if (t.touch_status == pros::E_TOUCH_PRESSED) {
                for (const Btn& b : BTNS) {
                    if (t.x >= b.x0 && t.x <= b.x1 && t.y >= b.y0 && t.y <= b.y1) {
                        selected = b.mode;
                        return;            // selected + exit in one tap
                    }
                }
            }

            pros::delay(20);
        }
    }
}