#pragma once
#include "lemlib/api.hpp"   // needed: drawRobot's signature uses lemlib::Pose

namespace map {
    void drawField();
    void drawRobot(lemlib::Pose pose);
}