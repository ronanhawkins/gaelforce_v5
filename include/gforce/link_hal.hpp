#pragma once
#include "main.h"
#include "pros/serial.hpp"   // not pulled in by api.h; generic serial lives in apix
#include "gflib/drivetrain.hpp"
#include "gflib/hal.hpp"
#include "gflib/link.hpp"
#include "gflib/posesource.hpp"

// This Brain's side of gflib's HAL. It owns motion control and NOTHING else:
// there is no IMU and no rotation sensor on this board, so the RS-485 link to
// the sensor pod is the only source of pose there is. Everything here is a
// transport.

namespace hal {

struct PoseSnapshot {
    gflib::Pose pose{};
    bool healthy = false;
    uint32_t ageMs = 0;
};

void publishSnapshot(const PoseSnapshot& s);
PoseSnapshot readSnapshot();

class V5Clock : public gflib::IClock {
    public:
        uint32_t millisNow() const override;
        void sleepMs(uint32_t ms) override;
};

// Generic serial over a smart port. The port stops speaking the smart-device
// protocol the moment pros::Serial claims it, so the port must carry nothing
// else -- see ports::RS485.
class V5Serial : public gflib::IByteStream {
    public:
        explicit V5Serial(std::uint8_t port);

        size_t read(uint8_t* dst, size_t cap) override;

        // All or nothing, never partial.
        //
        // IByteStream permits a short write, but half of a frame on a
        // half-duplex bus is worse than none: the pod has to resync past the
        // fragment, and its resyncBytes counter climbing is what Gate A
        // watches for. The pod's own Rs485Stream::write refuses the same way,
        // so both ends fail identically.
        size_t write(const uint8_t* src, size_t len) override;

    private:
        pros::Serial serial_;
};

// Volts in, millivolts out. gflib works in volts because that is what a
// motion controller can reason about; V5 motors take millivolts.
class V5Drive : public gflib::IDriveOutput {
    public:
        V5Drive(pros::MotorGroup& left, pros::MotorGroup& right);

        void setLeft(double volts) override;
        void setRight(double volts) override;

        // What was last commanded, for the BrainStatus the pod weights its
        // MCL with. A ToF return taken while parked and one taken mid-slam
        // are not the same measurement, and this is how the pod tells them
        // apart
        double lastLeftVolts() const { return lastLeft_; }
        double lastRightVolts() const { return lastRight_; }

    private:
        pros::MotorGroup& left_;
        pros::MotorGroup& right_;
        double lastLeft_ = 0.0;
        double lastRight_ = 0.0;
};

// Sends BrainStatus REACTIVELY: only ever in response to a PoseReport that has
// just been decoded, never on a timer of its own.
//
// One differential pair is one collision domain. The pod free-runs at 100Hz
// and leaves a listening window after each of its own transmissions; a second
// free-running 100Hz transmitter on the same pair drifts in phase against it
// and eventually lands on top of the pod's next frame. That failure is flat
// error counters for minutes, then a burst, then flat again -- the hardest
// possible signature to attribute after the fact.
class BrainStatusHook : public gflib::IServiceHook {
    public:
        BrainStatusHook(gflib::LinkPoseSource& source, gflib::IClock& clock,
                        const V5Drive& drive, uint32_t guardMs);

        // Runs on the motion task between control ticks. Drivetrain has
        // already called source.update() by the time this is entered, so this
        // only decides whether to answer -- it must not update, set a pose or
        // touch Drivetrain

        void onService() override { send(); publish(); }

        // Copies the current pose and link health into the shared snapshot
        void publish() const;

        // The guarded reply, also called from loops we own ourselves.
        // Returns false when it declined to send or the write was refused
        bool send();

        void setMotionState(gflib::MotionState state) { state_ = state; }

    private:
        gflib::LinkPoseSource& source_;
        gflib::IClock& clock_;
        const V5Drive& drive_;
        uint32_t guardMs_;
        gflib::MotionState state_ = gflib::MotionState::Idle;
};

}  // namespace hal
