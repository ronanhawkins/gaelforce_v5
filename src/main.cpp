#include "main.h"
#include "lemlib/api.hpp"
#include "gforce/gaelforce.hpp"

namespace start_position {
    //left auton
    float r_X = 0;
    float r_Y = 0;
    float r_H = 0;
    //right auton
    float l_X = 0;
    float l_Y = 0;
    float l_H = 0;
    float s_X = 0;
    float s_Y = 0;
    float s_H = 0;
    // starting x position (-72 to 72) (inches), right is positive, left is negative
    // starting y position (-72 to 72) (inches), away is positive, closer is negative
    // starting heading (-180 to 180) (degrees), 0 is facing away from
    //0,0,0 is center of field, facing away from driver
}

//initizalize function. This is the first function that runs when the program starts
void initialize() {
    mode::runSelector();
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

void disabled() {} //runs when robot is disabled

//runs after initialize if connected to competition control
void competition_initialize() {
}

//auton
void autonomous() {
    switch (mode::selected) {
        case mode::Auton::MATCH_LEFT:  matchLeft();  break;
        case mode::Auton::MATCH_RIGHT: matchRight(); break;
        case mode::Auton::SKILLS:      skills();     break;
    }
}

void matchLeft() {
    chassis.setPose(start_position::l_X, start_position::l_Y, start_position::l_H);
    chassis.moveToPose(0, 0, 0,8000); // move to the center of the field
}

void matchRight() {
    chassis.setPose(start_position::r_X, start_position::r_Y, start_position::r_H);
    chassis.moveToPose(0, 0, 0,8000); // move to the center of the field
}

void skills() {
    chassis.setPose(start_position::s_X, start_position::s_Y, start_position::s_H);
    chassis.moveToPose(0, 0, 0,8000); // move to the center of the field
}

//driver control
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(10);
    }
}