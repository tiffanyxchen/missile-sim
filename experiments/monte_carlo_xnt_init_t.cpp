#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>
#include <iomanip>

/*
    Monte Carlo Simulation of Homing Guidance
    Converted from MATLAB to C++

    -------------------------------------------------
    Variable Definitions
    -------------------------------------------------

    VC      : Closing velocity
    XNT     : Target lateral acceleration
    VM      : Missile velocity (unused in this script)
    XNP     : Navigation constant
    TAU     : Autopilot/shaping filter time constant
    RUN     : Number of Monte Carlo runs

    TF      : Final engagement time
    TSTART  : Random target maneuver start time
    COEF    : Random maneuver direction (+1 or -1)

    Y       : Lateral displacement
    YD      : Lateral velocity
    YDD     : Lateral acceleration

    XNC     : Commanded missile acceleration
    XNL     : Actual missile acceleration after shaping filter
    XNLD    : Derivative of XNL

    RTM     : Remaining range-to-go
    TGO     : Time-to-go
    XLAMD   : LOS rate

    H       : Integration step size

    XMEAN   : Mean miss distance
    SIGMA   : Standard deviation of miss distance
*/

int main() {

    // -------------------------------------------------
    // Preallocation
    // -------------------------------------------------

    std::vector<double> Z(1000, 0.0);

    std::vector<double> ArrayTF(10, 0.0);
    std::vector<double> ArraySIGMA(10, 0.0);
    std::vector<double> ArrayXMEAN(10, 0.0);

    int count = 0;

    // -------------------------------------------------
    // Constants
    // -------------------------------------------------

    double VC = 4000.0;
    double XNT = 96.6;
    double VM = 3000.0;
    double XNP = 3.0;
    double TAU = 1.0;

    int RUN = 50;

    // -------------------------------------------------
    // Random number generator
    // -------------------------------------------------

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> dist(0.0, 1.0);

    // -------------------------------------------------
    // Loop over final engagement times
    // -------------------------------------------------

    for (int TF = 1; TF <= 10; TF++) {

        double Z1 = 0.0;
        double XMEAN = 0.0;

        // -------------------------------------------------
        // Monte Carlo runs
        // -------------------------------------------------

        for (int I = 1; I <= RUN; I++) {

            double SUM = dist(gen);

            double TSTART = TF * SUM;

            double PZ = dist(gen);

            PZ = PZ - 0.5;

            int COEF;

            if (PZ > 0.0)
                COEF = 1;
            else
                COEF = -1;

            // -------------------------------------------------
            // Initial conditions
            // -------------------------------------------------

            double Y = 0.0;
            double YD = 0.0;

            double T = 0.0;

            double H = 0.01;

            double S = 0.0;

            double XNC = 0.0;
            double XNL = 0.0;

            // State derivatives
            double YDD = 0.0;
            double XNLD = 0.0;

            // -------------------------------------------------
            // Main integration loop
            // -------------------------------------------------

            while (T <= (TF - 1e-5)) {

                // Target maneuver starts at random time

                if (T < TSTART)
                    XNT = 0.0;
                else
                    XNT = COEF * 96.6;

                // Save old states

                double YOLD = Y;
                double YDOLD = YD;
                double XNLOLD = XNL;

                int STEP = 1;
                int FLAG = 0;

                // Predictor-corrector integration

                while (STEP <= 1) {

                    if (FLAG == 1) {

                        Y = Y + H * YD;

                        YD = YD + H * YDD;

                        XNL = XNL + H * XNLD;

                        T = T + H;

                        STEP = 2;
                    }

                    // Time-to-go

                    double TGO = TF - T + 0.00001;

                    // Remaining range

                    double RTM = VC * TGO;

                    // LOS rate

                    double XLAMD =
                        (RTM * YD + Y * VC) /
                        (RTM * RTM);

                    // Proportional navigation command

                    XNC = XNP * VC * XLAMD;

                    // First-order shaping filter

                    XNLD = (XNC - XNL) / TAU;

                    // Relative lateral acceleration

                    YDD = XNT - XNL;

                    FLAG = 1;
                }

                FLAG = 0;

                // -------------------------------------------------
                // Trapezoidal correction
                // -------------------------------------------------

                Y =
                    0.5 * (YOLD + Y + H * YD);

                YD =
                    0.5 * (YDOLD + YD + H * YDD);

                XNL =
                    0.5 * (XNLOLD + XNL + H * XNLD);

                S = S + H;
            }

            // -------------------------------------------------
            // Store terminal miss distance
            // -------------------------------------------------

            Z[I] = Y;

            Z1 = Z[I] + Z1;

            XMEAN = Z1 / I;
        }

        // -------------------------------------------------
        // Compute standard deviation
        // -------------------------------------------------

        double SIGMA = 0.0;

        Z1 = 0.0;

        for (int I = 1; I <= RUN; I++) {

            Z1 =
                std::pow(Z[I] - XMEAN, 2.0) + Z1;

            if (I == 1)
                SIGMA = 0.0;
            else
                SIGMA = std::sqrt(Z1 / (I - 1));
        }

        // -------------------------------------------------
        // Store results
        // -------------------------------------------------

        count++;

        ArrayTF[count - 1] = TF;

        ArraySIGMA[count - 1] = SIGMA;

        ArrayXMEAN[count - 1] = XMEAN;
    }

    // -------------------------------------------------
    // Save results to file
    // -------------------------------------------------

    std::ofstream outfile("output/txt/datfil_mc_xnt_init_t.txt");

    outfile << std::fixed << std::setprecision(6);

    for (int i = 0; i < 10; i++) {

        outfile
            << ArrayTF[i] << " "
            << ArraySIGMA[i] << " "
            << ArrayXMEAN[i]
            << std::endl;
    }

    outfile.close();

    std::cout << "simulation finished" << std::endl;

    return 0;
}
