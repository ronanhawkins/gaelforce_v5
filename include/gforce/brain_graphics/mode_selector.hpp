#pragma once

namespace mode {
    enum class Auton { MATCH_LEFT, MATCH_RIGHT, SKILLS };

    struct Btn {
        int x0, y0, x1, y1;
        const char* label;
        Auton mode;
    };

    extern Auton selected;     // declared here, defined once in the .cpp

    void drawSelector();       // declarations only — no bodies
    void runSelector();
}