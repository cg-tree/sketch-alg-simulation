// logging.h

#pragma once

#include <iostream>
#include <sstream>
#include <iomanip>
#include <array>
#include <string>

// ANSI escape codes for foreground colors
// (works on most Unix/Linux/MacOS terminals and on Windows 10+ if VirtualTerminalProcessing is enabled)

#define RESET   "\033[0m" // Resets the color to normal
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"

// Wrapper macro for coloring text.
// Example: std::cout << COLOR_TEXT(RED, "[ERROR]") << std:endl;
#define COLOR_TEXT(color, text) (std::string(color) + (text) + RESET)

// Helper that formats a double with fixed precision (6 decimals)
static std::string fmtDouble(double x, int precision = 6){
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << x;
    return oss.str();
}


// ----------------------------------------------------------------
// 3. printEpochSummary: one‐line, color‐coded summary per epoch
// ----------------------------------------------------------------
//
//  - epochNumber:   integer epoch counter
//  - startLevel:    starting “level” value for this epoch
//  - endLevel:      ending “level” value for this epoch
//  - gradX, gradY:  gradient vector components
//  - angle:         gradient angle (radians or degrees, whichever you use)
//  - droneA:        (x,y) position of drone A
//  - droneB:        (x,y) position of drone B
//  - contourSize:   integer size of the local contour
//  - critPt:        (x,y) of the critical point
//  - tangentA:      array<double,2> for tangent vector at A
//  - tangentB:      array<double,2> for tangent vector at B
//  - crossA, crossB: two cross‐product values
//
// You can add or remove fields as needed. Each “section” is color‐coded.
//
static void printEpochSummary(
    int                                   epochNumber,
    double                                startLevel,
    double                                endLevel,
    double                                gradX,
    double                                gradY,
    double                                angle,
    const std::pair<double,double>&       droneA,
    const std::pair<double,double>&       droneB,
    int                                   contourSize,
    const std::pair<double,double>&       critPt,
    const std::array<double,2>&           tangentA,
    const std::array<double,2>&           tangentB,
    double                                crossA,
    double                                crossB
) {
    // 1) Header: [Epoch #] startLevel=... endLevel=...
    std::string header =
        COLOR_TEXT(YELLOW, "[Epoch " + std::to_string(epochNumber) + "]") +
        " " +
        COLOR_TEXT(WHITE, "start=" + fmtDouble(startLevel)) +
        " " +
        COLOR_TEXT(WHITE, "end="   + fmtDouble(endLevel));

    // 2) Gradient section in CYAN
    std::string gradSection =
        COLOR_TEXT(
            CYAN,
            " Grad=("   + fmtDouble(gradX) + "," + fmtDouble(gradY) + ")" +
            " θ="       + fmtDouble(angle)
        );

    // 3) Drone positions in GREEN
    std::string droneSection =
        COLOR_TEXT(
            GREEN,
            " DroneA=(" + fmtDouble(droneA.first)  + "," + fmtDouble(droneA.second) + ")" +
            " DroneB=(" + fmtDouble(droneB.first)  + "," + fmtDouble(droneB.second) + ")"
        );

    // 4) Contour info in BLUE
    std::string contourSection =
        COLOR_TEXT(
            BLUE,
            " contourSize=" + std::to_string(contourSize) +
            " critPt=("     + fmtDouble(critPt.first) + "," + fmtDouble(critPt.second) + ")"
        );

    // 5) Tangents & cross‐product in MAGENTA
    std::string tangentSection =
        COLOR_TEXT(
            MAGENTA,
            " tA=["     + fmtDouble(tangentA[0]) + "," + fmtDouble(tangentA[1]) + "]" +
            " tB=["     + fmtDouble(tangentB[0]) + "," + fmtDouble(tangentB[1]) + "]" +
            " crossA="  + fmtDouble(crossA) +
            " crossB="  + fmtDouble(crossB)
        );

    // 6) Print everything in one go, then reset color
    std::cout
        << header
        << gradSection
        << droneSection
        << contourSection
        << tangentSection
        << RESET
        << std::endl;
}

// This function performs all the printing (cout) logic for test_infrastructure() initialization
void print_test_infrastructure_info() {
    std::cout << "Initializing test... " << std::endl;
    std::cout << "Epsilon " << epsilon << std::endl;
    std::cout << "Initial direction " << alpha << std::endl;
    std::cout << "Number of Gaussians " << num << std::endl;
    std::cout << "Least difference between starting and end point " << INF << std::endl;
    std::cout << "Minimum crossings before checking termination " << CROSSBOUND << std::endl;
    std::cout << "Minimum distance factor between drones and minimum distance "
              << DIST << " " << (DIST * epsilon) << std::endl;
    std::cout << "Concentration THRESHOLD " << THRESHOLD << std::endl;
    std::cout << "Drone A starting point " 
              << drone_start_A.x << " " << drone_start_B.y << std::endl;
    std::cout << "Drone B starting point " 
              << drone_start_B.x << " " << drone_start_A.y << std::endl;
    std::cout << "Centers of gaussians " << std::endl;
    
    for (int i = 0; i < num; ++i) {
        std::cout << gaussianCenter[i].x << " " << gaussianCenter[i].y << std::endl;
    }
}
