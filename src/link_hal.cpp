#include "gforce/link_hal.hpp"
#include "gflib/util.hpp"

namespace hal {

// clock

uint32_t V5Clock::millisNow() const { return pros::millis(); }

void V5Clock::sleepMs(uint32_t ms) {
    pros::delay(ms);
}

// serial

V5Serial::V5Serial(std::uint8_t port)
    : serial_(port, static_cast<std::int32_t>(gflib::kLinkBaud)) {}

size_t V5Serial::read(uint8_t* dst, size_t cap) {
    if (cap == 0) return 0;
    const std::int32_t n = serial_.read(dst, static_cast<std::int32_t>(cap));

    // PROS_ERR on a failed read. Zero is a normal answer under IByteStream,
    // not an error, so a failure and an empty buffer report the same thing
    return n > 0 ? static_cast<size_t>(n) : 0;
}

size_t V5Serial::write(const uint8_t* src, size_t len) {
    if (len == 0) return 0;

    const std::int32_t free = serial_.get_write_free();
    if (free < 0 || static_cast<size_t>(free) < len) return 0;

    // pros::Serial::write takes a non-const buffer despite not modifying it.
    // The const_cast is the whole of the mismatch between the two interfaces
    const std::int32_t n =
        serial_.write(const_cast<std::uint8_t*>(src), static_cast<std::int32_t>(len));
    return n > 0 ? static_cast<size_t>(n) : 0;
}

// drive

V5Drive::V5Drive(pros::MotorGroup& left, pros::MotorGroup& right)
    : left_(left), right_(right) {}

namespace {
constexpr double kMaxVolts = 12.0;
constexpr double kMilliVoltsPerVolt = 1000.0;

std::int32_t toMilliVolts(double volts) {
    const double v = gflib::clamp(static_cast<gflib::real>(volts),
                                  static_cast<gflib::real>(-kMaxVolts),
                                  static_cast<gflib::real>(kMaxVolts));
    return static_cast<std::int32_t>(v * kMilliVoltsPerVolt);
}
}  // namespace

void V5Drive::setLeft(double volts) {
    lastLeft_ = volts;
    left_.move_voltage(toMilliVolts(volts));
}

void V5Drive::setRight(double volts) {
    lastRight_ = volts;
    right_.move_voltage(toMilliVolts(volts));
}

// pose snapshot

namespace {
PoseSnapshot g_snapshot;
}

void publishSnapshot(const PoseSnapshot& s) { g_snapshot = s; }
PoseSnapshot readSnapshot() { return g_snapshot; }

// brain status

BrainStatusHook::BrainStatusHook(gflib::LinkPoseSource& source, gflib::IClock& clock,
                                 const V5Drive& drive, uint32_t guardMs)
    : source_(source), clock_(clock), drive_(drive), guardMs_(guardMs) {}

void BrainStatusHook::publish() const {
    PoseSnapshot s;
    const uint32_t nowMs = clock_.millisNow();

    s.pose = source_.getPose();
    s.healthy = source_.healthy(nowMs);
    s.ageMs = source_.ageMs(nowMs);

    publishSnapshot(s);
}

bool BrainStatusHook::send() {
    if (!source_.haveReport()) return false;

    const uint32_t nowMs = clock_.millisNow();

    // Age against the LOCAL millis recorded when the report ARRIVED, which is
    // what ageMs() measures.
    //
    // Declining costs nothing. The pod tolerates three missed statuses before
    // it raises kLinkDegraded
    if (source_.ageMs(nowMs) > guardMs_) return false;

    gflib::BrainStatus s;
    s.timestampMs = nowMs;
    s.leftVolts = static_cast<float>(drive_.lastLeftVolts());
    s.rightVolts = static_cast<float>(drive_.lastRightVolts());
    s.motionState = static_cast<uint8_t>(state_);

    uint16_t flags = 0;
    const bool disabled = pros::competition::is_disabled();
    if (disabled) flags |= gflib::BrainFlags::kDisabled;
    else if (pros::competition::is_autonomous()) flags |= gflib::BrainFlags::kAutonomous;
    else flags |= gflib::BrainFlags::kDriverControl;
    s.flags = flags;

    return source_.sendStatus(s);
}

}  // namespace hal
