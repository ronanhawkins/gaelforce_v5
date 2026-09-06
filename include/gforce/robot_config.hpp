#pragma once
#include "main.h"
#include "gforce/link_hal.hpp"
#include "gflib/drivetrain.hpp"
#include "gflib/posesource.hpp"

extern pros:: MotorGroup leftMotors;
extern pros:: MotorGroup rightMotors;
extern pros::Controller controller;
//add other motors here, for example: extern pros::Motor armMotor;

// Rates the link and the control law run at. Separate on purpose, the pod
// leaves an 8ms listening window after each pose frame, so a reply has to be
// decided in single-digit milliseconds, while the PIDs want the 10ms tick the
// tuning was built on.
namespace tune {
    constexpr uint32_t kLoopMs = 10;

    // Divides kLoopMs exactly, so the service cadence sits at a fixed rate
    // instead of beating against the control tick. A 10ms poll would drift in
    // phase against the pod's own 10ms and the reply would eventually land on
    // top of its next frame -- flat counters for minutes, then a burst
    constexpr uint32_t kServiceMs = 2;

    // The pose frame takes 2.43ms of the pod's 8ms window to arrive and the
    // 23-byte reply needs 1.0ms, so a status is worth sending for roughly
    // 4.5ms after it lands. 3ms leaves room for one missed service slice
    constexpr uint32_t kStatusGuardMs = 3;

    // Frames must be flowing before anything drives. There is no local
    // odometry to fall back on
    constexpr uint32_t kLinkBeginTimeoutMs = 2000;
}

// The pod's side of the link. Declared in construction order: each holds
// references to the ones above it, and they all live in one translation unit
// so that order is defined rather than a static-init race
extern hal::V5Clock brainClock;
extern hal::V5Serial podLink;
extern hal::V5Drive driveOutput;
extern gflib::LinkPoseSource poseSource;
extern gflib::Drivetrain drivetrain;
extern hal::BrainStatusHook statusHook;

namespace link {
    // One poll of the link plus a guarded reply. For loops this code owns;
    // Drivetrain does the same thing itself between control ticks
    void service();
}
