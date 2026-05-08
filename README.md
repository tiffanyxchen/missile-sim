# Missile Guidance Simulation 🚀

This project simulates missile-target interception using **Proportional
Navigation (PN)** guidance in C++, with Python-based trajectory
visualization.

## 🎥 Demo

![Simulation](demo1.gif)

## 📂 Project Structure

    missile-sim/
    ├── src/            # C++ simulation engine
    ├── experiments/    # parameter sweeps (e.g., XNP variation)
    ├── python/         # animation scripts
    ├── output/csv/     # simulation results

## ⚙️ Run Simulation

    cd src
    g++ main.cpp simulation.cpp -o main
    ./main

## 🎬 Run Animation

    cd python
    python animation.py

## 🧪 Parameter Sweep

    cd experiments
    g++ sweep_xnp.cpp ../src/simulation.cpp -o sweep
    ./sweep

## 📊 Output

-   CSV files → `output/csv/`
-   Optional GIFs → `output/gifs/`

## 📌 Features

-   2D missile-target dynamics
-   Proportional Navigation guidance
-   Predictor-corrector integration
-   Parameter sweep capability
-   Python animation pipeline
