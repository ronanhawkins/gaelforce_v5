#include "main.h"
#include "lemlib/api.hpp"
#include "gforce/gaelforce.hpp"

namespace start_position {
    constexpr float X = -42; // starting x position (-72 to 72) (inches), right is positive, left is negative
    constexpr float Y = -60; // starting y position (-72 to 72) (inches), away is positive, closer is negative
    constexpr float H = 0; // starting heading (-180 to 180) (degrees), 0 is facing away from driver, 90 is right
    //0,0,0 is center of field, facing away from driver
}

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

void disabled() {} //runs when robot is disabled

//runs after initialize if connected to competition control
void competition_initialize() {
    chassis.setPose(start_position::X, start_position::Y, start_position::H); // set the robot's position
}

//auton
void autonomous() {
    //move to 0,0,90,timeout in 8000 milliseconds
    chassis.moveToPose(0, 0, 90, 8000);
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