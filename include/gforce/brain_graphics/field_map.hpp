#pragma once
#include "gflib/pose.hpp"   // needed: drawRobot's signature uses gflib::Pose

namespace map {
    void drawField();
    void drawRobot(const gflib::Pose& pose);
}
