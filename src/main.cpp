#include "main.h"
#include <cstdio>
#include "gforce/gaelforce.hpp"

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

    // Set once in initialize() and never cleared.
    const char* volatile initFault = nullptr;

    // Why the last routine stopped, or nullptr.
    const char* volatile runFault = nullptr;

    const char* reasonFor(gflib::MotionStatus st) {
        switch (st) {
            case gflib::MotionStatus::PoseUnhealthy: return "LINK LOST - pose not trusted";
            case gflib::MotionStatus::TimedOut:      return "MOTION TIMED OUT";
            case gflib::MotionStatus::Cancelled:     return "MOTION CANCELLED";
            default:                                 return "MOTION FAILED";
        }
    }

    gflib::MotionState wireState(gflib::MotionStatus st) {
        switch (st) {
            case gflib::MotionStatus::Running:       return gflib::MotionState::Running;
            case gflib::MotionStatus::Settled:       return gflib::MotionState::Settled;
            // Reaching a chain radius is a real exit, not a failure, and the
            // wire has no separate word for it.
            case gflib::MotionStatus::EarlyExit:     return gflib::MotionState::Settled;
            case gflib::MotionStatus::TimedOut:      return gflib::MotionState::TimedOut;
            case gflib::MotionStatus::Cancelled:     return gflib::MotionState::Cancelled;
            case gflib::MotionStatus::PoseUnhealthy: return gflib::MotionState::PoseUnhealthy;
        }
        return gflib::MotionState::Idle;
    }

    // Reports how the routine actually ended.
    //
    // Before this the wire only ever moved between Running and Idle, so a
    // routine that died on a stale pose was indistinguishable from one that
    // finished. Settled, TimedOut and PoseUnhealthy now reach the pod.
    struct ReportOnExit {
        ~ReportOnExit() {
            statusHook.setMotionState(drivetrain.faulted()
                                          ? wireState(drivetrain.faultStatus())
                                          : gflib::MotionState::Idle);
        }
    };
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
        initFault = "NO POSE FRAMES - check the pod and the RS-485 wiring";
    }

    // autonomous() calls initialize() again to retry the link, and each call
    // used to start another screen task that never stopped.
    static bool screenStarted = false;
    if (screenStarted) return;
    screenStarted = true;

    // thread for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            if (selectorActive) {
                pros::delay(50);
                continue;
            }
            // A copy, not the live objects
            const hal::PoseSnapshot snap = hal::readSnapshot();
            const gflib::Pose p = snap.pose;

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
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "LINK: %s  age %lums", 
                                snap.healthy ? "ok " : "BAD",
                                static_cast<unsigned long>(snap.ageMs));

            if (runFault != nullptr) {
                pros::screen::set_pen(pros::Color::red);
                pros::screen::print(pros::E_TEXT_MEDIUM, 6, "%s", runFault);
            }
            if (initFault != nullptr) {
                pros::screen::set_pen(pros::Color::red);
                pros::screen::print(pros::E_TEXT_MEDIUM, 7, "%s", initFault);
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
    // Enabling kills competition_initialize() mid selector, before it can
    // clear this, which would leave the screen task paused all match.
    selectorActive = false;

    for (int i = 0; i < 3; ++i) {
        initialize();
        pros::delay(50);
        link::service();
    }
    drivetrain.clearFault();

    // Only this run's message.
    runFault = nullptr;

    switch (mode::selected) {
        case mode::Auton::MATCH_LEFT:  matchLeft();  break;
        case mode::Auton::MATCH_RIGHT: matchRight(); break;
        case mode::Auton::SKILLS:      skills();     break;
    }

    //check for fault
    if (drivetrain.faulted()) {
        static char buf[64];
        const uint32_t at = drivetrain.faultedAtMotion();
        const char* why = reasonFor(drivetrain.faultStatus());

        // 0 means the fault came from setPose, before any motion ran
        if (at == 0) std::snprintf(buf, sizeof(buf), "%s (pose set)", why);
        else         std::snprintf(buf, sizeof(buf), "%s at motion %lu",
                                   why, static_cast<unsigned long>(at));
        runFault = buf;
    }
}

namespace {
using gflib::operator""_r;

void routine(gflib::real x, gflib::real y, gflib::real h) {
    ReportOnExit reportWhenDone;

    // Parked, waiting for the pod to echo the reset.
    statusHook.setMotionState(gflib::MotionState::Idle);
    drivetrain.setPose(x, y, h);

    // One Running for the whole driving stretch.
    statusHook.setMotionState(gflib::MotionState::Running);
    drivetrain.moveToPoint(-48.0_r, -24.0_r, 2000, 8.0_r);   // chained
    drivetrain.moveToPoint(-24.0_r, -12.0_r, 2000);          // ends the chain
    drivetrain.driveDistance(-12.0_r, 1500);
    drivetrain.turnToHeading(90.0_r, 1200);
    drivetrain.moveToPose(36.0_r, 24.0_r, 45.0_r, 3000);
}
}

void matchLeft()  { routine(start_position::l_X, start_position::l_Y, start_position::l_H); }
void matchRight() { routine(start_position::r_X, start_position::r_Y, start_position::r_H); }
void skills()     { routine(start_position::s_X, start_position::s_Y, start_position::s_H); }

//driver control
void opcontrol() {
    // Same as autonomous(): the selector may have been killed before a tap.
    selectorActive = false;

    drivetrain.clearFault();
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

            // Held
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                liftMotors.move(127);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                liftMotors.move(-127);
            } else {
                liftMotors.move(0);
            }
        }

        pros::delay(tune::kServiceMs);
    }
}
