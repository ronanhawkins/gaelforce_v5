#include "main.h"
#include "gforce/gaelforce.hpp"
#include "gflib/motion.hpp"

namespace start_position {
    // left auton
    float l_X = 0;
    float l_Y = 0;
    float l_H = 0;
    // right auton
    float r_X = 0;
    float r_Y = 0;
    float r_H = 0;
    // skills
    float s_X = 0;
    float s_Y = 0;
    float s_H = 0;
    // starting x position (-72 to 72) (inches), right is positive, left is negative
    // starting y position (-72 to 72) (inches), away is positive, closer is negative
    // starting heading (-180 to 180) (degrees), 0 is facing away from driver
    // 0,0,0 is center of field, facing away from driver
}

// pause the screen task while the auton selector is active
namespace {
    volatile bool selectorActive = false;

    // Why the routine stopped, or nullptr.
    const char* volatile fault = nullptr;

    const char* reasonFor(gflib::MotionStatus st) {
        switch (st) {
            case gflib::MotionStatus::PoseUnhealthy: return "LINK LOST - pose not trusted";
            case gflib::MotionStatus::TimedOut:      return "MOTION TIMED OUT";
            case gflib::MotionStatus::Cancelled:     return "MOTION CANCELLED";
            default:                                 return "MOTION FAILED";
        }
    }

    // Runs one motion and reports whether the routine may continue. 
    // An abort stops the drivetrain and latches the reason; it never falls through
    bool step(gflib::IMotion& m) {
        const gflib::MotionStatus st = drivetrain.runMotion(m);
        if (st == gflib::MotionStatus::Settled || st == gflib::MotionStatus::EarlyExit) return true;

        drivetrain.stop();
        fault = reasonFor(st);
        return false;
    }

    // The pose belongs to the pod, so this is a request that can fail. A false
    // return means we do not know where the robot is, which is a reason not to
    // drive rather than something to log and continue past
    bool seedPose(float x, float y, float h) {
        if (drivetrain.setPose(x, y, h)) return true;
        fault = "POSE SET NOT ACKNOWLEDGED";
        return false;
    }
}

// forward declarations for the routines called from autonomous()
void matchLeft();
void matchRight();
void skills();

// initialize function. This is the first function that runs when the program starts
void initialize() {
    // Every blocking gflib call now services the link between its own ticks,
    // so the pod keeps hearing from us during a motion instead of going quiet
    // for the whole of it
    drivetrain.setServiceHook(&statusHook);

    // Wait for pose frames to start ariving
    if (!drivetrain.begin(tune::kLinkBeginTimeoutMs)) {
        fault = "NO POSE FRAMES - check the pod and the RS-485 wiring";
    }

    // thread for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            if (selectorActive) {
                pros::delay(50);
                continue;
            }
            gflib::Pose p = drivetrain.getPose();

            pros::screen::erase();          // clear whole screen
            map::drawField();               // static field
            map::drawRobot(p);              // live robot
            // numeric readout on the left side
            pros::screen::set_pen(pros::Color::white);
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "X: %.1f", p.x);
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Y: %.1f", p.y);
            pros::screen::print(pros::E_TEXT_MEDIUM, 3, "H: %.1f", p.thetaDeg);

            // Link health, because a stale pose looks exactly like a
            // stationary robot on a map
            const uint32_t now = pros::millis();
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "LINK: %s  age %lums",
                                poseSource.healthy(now) ? "ok " : "BAD",
                                static_cast<unsigned long>(poseSource.ageMs(now)));

            if (fault != nullptr) {
                pros::screen::set_pen(pros::Color::red);
                pros::screen::print(pros::E_TEXT_MEDIUM, 6, "%s", fault);
            }
            // delay to save resources
            pros::delay(50);
        }
    });
}

void disabled() {} //runs when robot is disabled

//runs after initialize if connected to competition control
void competition_initialize() {
    selectorActive = true;
    mode::runSelector();
    selectorActive = false;
}

//auton
void autonomous() {
    fault = nullptr;
    statusHook.setMotionState(gflib::MotionState::Running);

    switch (mode::selected) {
        case mode::Auton::MATCH_LEFT:  matchLeft();  break;
        case mode::Auton::MATCH_RIGHT: matchRight(); break;
        case mode::Auton::SKILLS:      skills();     break;
    }

    statusHook.setMotionState(gflib::MotionState::Idle);
}

namespace {
using gflib::operator""_r;

void driveToCentreFrom(float x, float y, float h) {
    if (!seedPose(x, y, h)) return;

    gflib::MoveToPose m(0.0_r, 0.0_r, 0.0_r,
                        drivetrain.config().lateral, drivetrain.config().angular,
                        drivetrain.config().lateralExit, drivetrain.config().move);
    step(m);   // a false return already latched the reason and stopped the drive
}
}

void matchLeft()  { driveToCentreFrom(start_position::l_X, start_position::l_Y, start_position::l_H); }
void matchRight() { driveToCentreFrom(start_position::r_X, start_position::r_Y, start_position::r_H); }
void skills()     { driveToCentreFrom(start_position::s_X, start_position::s_Y, start_position::s_H); }

//driver control
void opcontrol() {
    statusHook.setMotionState(gflib::MotionState::Idle);

    uint32_t lastDriveMs = pros::millis();

    while (true) {
        // The link is polled far faster than the drive is commanded.
        link::service();

        const uint32_t now = pros::millis();
        if (now - lastDriveMs >= tune::kLoopMs) {
            lastDriveMs = now;

            // get joystick positions
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            // move the chassis with arcade drive, shaped by gflib's curve
            drivetrain.arcadeCurved(leftY, rightX);
        }

        pros::delay(tune::kServiceMs);
    }
}
