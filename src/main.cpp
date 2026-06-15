#include "main.h"
#include "lemlib/api.hpp"
#include "gforce/gaelforce.hpp"

//initizalize function. This is the first function that runs when the program starts
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            lemlib::Pose p = chassis.getPose();

        	pros::screen::erase();          // clear whole screen
        	map::drawField();                    // static field
			map::drawRobot(p);
			// live robot
        	// numeric readout on the left side
    		pros::screen::set_pen(pros::Color::white);
    		pros::screen::print(pros::E_TEXT_MEDIUM, 1, "X: %.1f", p.x);
        	pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Y: %.1f", p.y);
        	pros::screen::print(pros::E_TEXT_MEDIUM, 3, "H: %.1f", p.theta);
            // delay to save resources
            pros::delay(50);
        }
    });
}