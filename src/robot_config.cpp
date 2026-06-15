#include "main.h"
#include "lemlib/api.hpp"
#include "gforce/robot_config.hpp"

namespace ports {
    constexpr int LEFT_FRONT   = 1;
	constexpr int LEFT_MIDDLE  = 2;
    constexpr int LEFT_BACK    = 3;
    constexpr int RIGHT_FRONT  = -4;
	constexpr int RIGHT_MIDDLE = -5;
    constexpr int RIGHT_BACK   = -6;
	constexpr int INERTIAL     =  7;
	constexpr int HORIZONTAL_ENCODER = 8;
	constexpr int VERTICAL_ENCODER = -9;
}

namespace encoder_config {
	// offset of horizontal tracking wheel from center, negative = back, positive = front
	constexpr double HORIZONTAL_ENCODER_OFFSET = -5.75; // inches
	// offset of vertical tracking wheel from center, negative = left, positive = right
	constexpr double VERTICAL_ENCODER_OFFSET = -2.5; // inches
}


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({ports::LEFT_FRONT, ports::LEFT_MIDDLE, ports::LEFT_BACK}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({ports::RIGHT_FRONT, ports::RIGHT_MIDDLE, ports::RIGHT_BACK}, pros::MotorGearset::blue);

// Inertial Sensor (Port 10)
pros::Imu imu(ports::INERTIAL);

// tracking wheels
// horizontal tracking wheel encoder
pros::Rotation horizontalEnc(ports::HORIZONTAL_ENCODER);
// vertical tracking wheel encoder 
pros::Rotation verticalEnc(ports::VERTICAL_ENCODER);

// horizontal tracking wheel. 2.75" diameter
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, encoder_config::HORIZONTAL_ENCODER_OFFSET);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, encoder_config::VERTICAL_ENCODER_OFFSET);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // track width in inches
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              600, // drivetrain rpm
                              2 // horizontal drift
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband (out of 127)
                                     10, // minimum output where drivetrain will move (out of 127)
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband (out of 127)
                                  10, // minimum output where drivetrain will move (out of 127)
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);