#include "main.h"
#include "gforce/robot_config.hpp"

namespace ports {
    constexpr int LEFT_FRONT   = 1;
	constexpr int LEFT_MIDDLE  = 2;
    constexpr int LEFT_BACK    = 3;
    constexpr int RIGHT_FRONT  = -4;
	constexpr int RIGHT_MIDDLE = -5;
    constexpr int RIGHT_BACK   = -6;

	// RS-485 to the sensor pod. pros::Serial takes the port off the
	// smart-device protocol entirely, so nothing else may share it.
	constexpr int RS485 = 7;

	// Lift. The two motors face each other on the shared lift, so one is
	// reversed or they fight
	constexpr int LIFT_A = 8;
	constexpr int LIFT_B = -9;
}

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({ports::LEFT_FRONT, ports::LEFT_MIDDLE, ports::LEFT_BACK}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({ports::RIGHT_FRONT, ports::RIGHT_MIDDLE, ports::RIGHT_BACK}, pros::MotorGearset::blue);

pros::MotorGroup liftMotors({ports::LIFT_A, ports::LIFT_B}, pros::MotorGearset::green);

// LINK TO SENSOR POD

hal::V5Clock brainClock;
hal::V5Serial podLink(ports::RS485);
hal::V5Drive driveOutput(leftMotors, rightMotors);

namespace {
// gflib's scalar literal suffix, so these read 0.945_r
using gflib::operator""_r;

gflib::LinkPoseSourceConfig makeLinkConfig() {
    gflib::LinkPoseSourceConfig c;

    // Three missed reports at 100Hz, matching the pod's own
    // kBrainStatusTimeoutMs so both ends call the link dead at the same moment
    c.maxAgeMs = 30;

    // minimum confidence in pose
    c.minConfidence = 0.2_r;

    c.useDeadReckoning = true;

    return c;
}

gflib::DrivetrainConfig makeDrivetrainConfig() {
    gflib::DrivetrainConfig c;

    // [0,12] in Voltage
    c.lateral.kP = 1.0_r;
    c.lateral.kD = 0.3_r;
    c.angular.kP = 0.2_r;
    c.angular.kD = 1.0_r;

    //tunable exit conditions
    // small band (in), time inside(ms), large band(in), time inside(ms), timeout(ms)
    c.lateralExit = gflib::ExitConditions{1.0_r, 100, 3.0_r, 500, 6000};
    c.angularExit = gflib::ExitConditions{1.0_r, 100, 3.0_r, 500, 3000};

    c.loopMs = tune::kLoopMs;
    c.serviceMs = tune::kServiceMs;

    // Latency compensation stays off until the round trip has actually been
    // measured on this link.
    c.poseTransitLatencyMs = 0;
    c.poseMaxExtrapMs = 0;

    // joystick deadband, min output, expo blend (between linear and cubic)
    c.throttleCurve.deadband = 3.0_r;
    c.throttleCurve.minVolts = 1.0_r;
    c.throttleCurve.expo = 0.0_r;
    c.turnCurve = c.throttleCurve;

    return c;
}
}  // namespace

gflib::LinkPoseSource poseSource(podLink, brainClock, makeLinkConfig());
gflib::Drivetrain drivetrain(poseSource, driveOutput, brainClock, makeDrivetrainConfig());
hal::BrainStatusHook statusHook(poseSource, brainClock, driveOutput, tune::kStatusGuardMs);

namespace link {
void service() {
    poseSource.update();
    statusHook.send();

    // Same slice-time duties as the hook. Driver control never enters
    // runMotion, so nothing else would refresh what the screen task reads
    statusHook.publish();
}
}  // namespace link
