#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <string>
// ==============================
// Structs
// ==============================
struct Vec2 {
    double x;
    double y;
};

// declaration only
Vec2 operator-(const Vec2& a, const Vec2& b);

struct Params {
    double VM;
    double VT;
    double XNP;
    double XNT;
    double HEDEG;
};

struct Trajectory {
    std::vector<Vec2> RM;
    std::vector<Vec2> RT;
    std::vector<double> T;
    std::vector<Vec2> VM;
    std::vector<Vec2> AM;
};

// ==============================
// Function declarations
// ==============================
Trajectory run_simulation(const Params& p);

void save_to_csv(const Trajectory& traj, const std::string& filename);

#endif

