#include "simulation.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

// Helper functions

double norm(const Vec2& v) {
	return std::sqrt(v.x * v.x + v.y * v.y);
}

double dot(const Vec2& a, const Vec2& b) {
	return a.x * b.x + a.y * b.y;
}

// operator overloading

Vec2 operator-(const Vec2& a, const Vec2& b) {
    return {a.x - b.x, a.y - b.y};
}

Vec2 operator+(const Vec2& a, const Vec2& b) {
    return {a.x + b.x, a.y + b.y};
}

Vec2 operator*(double s, const Vec2& v) {
    return {v.x * s, v.y * s};
}

Vec2 operator*(const Vec2& v, double s) {
    return {v.x * s, v.y * s};
}

// ==============================
// Simulation
// ==============================

Trajectory run_simulation(const Params& p){
	double spdM  = p.VM ;
	double spdT  = p.VT ;
	double XNP   = p.XNP;
	double XNT   = p.XNT;
	double HE    = p.HEDEG / 57.3;

	Vec2 RM = {0.0    , 10000.0};
	Vec2 RT = {40000.0, 10000.0};

	double BETA = M_PI;
	double T    = 0.0 ;
	
	// calculations:
    
    // target velocity vector
	Vec2 VT = {spdT * cos(BETA), spdT * sin(BETA)};
	
    // los
	Vec2 RTM    = RT - RM;
	double R    = norm(RTM);
	double XLAM = atan2(RTM.y, RTM.x);
	
    // lead angle
	double arg   = (spdT * sin(BETA + XLAM)) / spdM;
	arg	         = std::clamp(arg, -1.0, 1.0);
	double XLEAD = asin(arg);

    // missile velocity vector
	double THET  = XLAM + XLEAD;
	Vec2 VM      = {spdM * cos(THET + HE), spdM * sin(THET + HE)};

	// Relative Velocity, closing Velocity
    Vec2 VTM    =  VT - VM;
    double spdC = -(dot(RTM, VTM)) / R;
		
    Trajectory traj;
    
    int MAX_STEPS = 500000;
    int steps     = 0     ;
    
    while (spdC >= 0 && steps < MAX_STEPS){
        steps ++;
        double  H = (R < 1000.0) ? 0.0002 : 0.01;
        
        Vec2    RT_old  = RT;
        Vec2    RM_old  = RM;
        Vec2    VM_old  = VM;
        double  BETAOLD = BETA;
        
        // Target motion w XNT normal to VT so find d/dt(BETA) first, then integrate and update BETA, find VT, new. Integrate for positions.
        double  BETAD   = XNT / spdT;
        BETA    += H    * BETAD;

        VT.x     = spdT * cos(BETA);
        VT.y     = spdT * sin(BETA);
        RT.x    += H    * VT.x;
        RT.y    += H    * VT.y;
        
        // Geometry,
        // 1. calculate new los/RTM, R and set break; 2 VTM,spdC; XLAM
        RTM = RT - RM;
        R   = norm(RTM);
        if (R < 1e-3) break;
        
        // 2. Relative velocity, closing speed
        VTM  =   VT - VM;
        spdC = - dot(RTM, VTM)/R;
        
        // 3. los angle, rate, XNC, missiles acceleration
        XLAM = atan2(RTM.y, RTM.x);
        double XLAMD = (RTM.x * VTM.y - RTM.y * VTM.x) / (R * R);
        double XNC   = XNP * spdC * XLAMD;
        Vec2 AM = {
            -XNC * sin(XLAM),
             XNC * cos(XLAM)
            };
        
        // Missile update the motion
        VM.x += H * AM.x;
        VM.y += H * AM.y;
        RM.x += H * VM.x;
        RM.y += H * VM.y;
        
        // Predictor-Corrector
        VM   = 0.5 * (VM_old  + VM   + H * AM);
        RM   = 0.5 * (RM_old  + RM   + H * VM);
        RT   = 0.5 * (RT_old  + RT   + H * VT);
        BETA = 0.5 * (BETAOLD + BETA + H * BETAD);
        
        T += H;
        
        traj.RM.push_back(RM);
        traj.RT.push_back(RT);
        traj.T.push_back(T);
        traj.VM.push_back(VM);
        traj.AM.push_back(AM);

    }
    
    return traj;
    
}

// ==============================
// CSV export
// ==============================

void save_to_csv(const Trajectory& traj, const std::string& filename) {
    std::ofstream file(filename);
    
    file << "RM1,RM2,RT1,RT2,T,VM1,VM2,AM1,AM2\n";
    
    for (size_t i = 0; i < traj.RM.size(); i++) {
        file << traj.RM[i].x << ","
        << traj.RM[i].y << ","
        << traj.RT[i].x << ","
        << traj.RT[i].y << ","
        << traj.T[i]    << ","
        << traj.VM[i].x << ","
        << traj.VM[i].y << ","
        << traj.AM[i].x << ","
        << traj.AM[i].y << "\n";
    }
    
    file.close();
}
