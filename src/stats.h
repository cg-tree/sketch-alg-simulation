#pragma once

#include <array>
#include <utility>
#include <string>
#include "logging.h"

struct Stats {
    double startLevel    = 0.0;
    double endLevel      = 0.0;
    double gradX         = 0.0;
    double gradY         = 0.0;
    double angle         = 0.0;
    std::pair<double,double> droneA {0.0, 0.0};
    std::pair<double,double> droneB {0.0, 0.0};
    int    contourSize   = 0;
    std::pair<double,double> critPt {0.0, 0.0};
    std::array<double,2> tangentA {0.0, 0.0};
    std::array<double,2> tangentB {0.0, 0.0};
    double crossA        = 0.0;
    double crossB        = 0.0;

    // Optional free‐text debug
    std::string verboseMsg;

    void print(int epochNumber) const {
        printEpochSummary(
            epochNumber,
            startLevel,
            endLevel,
            gradX, gradY, angle,
            droneA,
            droneB,
            contourSize,
            critPt,
            tangentA,
            tangentB,
            crossA, crossB
        );
        if (!verboseMsg.empty()) {
            std::cout
                << COLOR_TEXT(YELLOW, "[DEBUG] " + verboseMsg)
                << RESET
                << std::endl;
        }
    }

    void reset() {
        startLevel = endLevel = 0.0;
        gradX = gradY = angle = 0.0;
        droneA = {0.0, 0.0};
        droneB = {0.0, 0.0};
        contourSize = 0;
        critPt = {0.0, 0.0};
        tangentA = {0.0, 0.0};
        tangentB = {0.0, 0.0};
        crossA = crossB = 0.0;
        verboseMsg.clear();
    }
};
