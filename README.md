# pneumatic-muscle-state-feedback-control
Non-linear modeling, Jacobian linearization, and state-feedback control of a pneumatic artificial muscle actuator in MATLAB and Simulink.

This repository contains the complete project for the dynamic modeling, symbolic linearization, and advanced state-feedback control of a Pneumatic Artificial Muscle system. 

## Project Architecture
The repository is structured into two core components:
*  A MATLAB script utilizing the Symbolic Math Toolbox to define non-linear system equations, automatically derive analytical Jacobian matrices at the zero operating point and compute the feedback gain vector.
*  A Simulink simulation block diagram that evaluates control performance in closed-loop. It evaluates the idealized **State-Space model** .

## Simulation Results & Frequency Response Analysis
To validate control robustness and map the physical boundaries of the pneumatic actuator, the closed-loop tracking loop was benchmarked against sinusoidal reference signals across various operating frequencies:

### 1. Low Frequency (0.5 Hz) - Optimal Tracking
At low frequencies, the non-linear plant model (Integrator output) matches the linearized State-Space model perfectly. 

### 2. Medium Frequency (1.0 Hz) - Phase Lag Emergence
As the frequency scales up, fluidic physical constraints begin to influence the plant. Due to air compressibility and choked-flow constraints through the valve orifice, a visible phase lag develops between the reference command and the system output.

### 3. High Frequency (10.0 Hz) - High-Frequency Attenuation
At 10 Hz, the physical system hits its bandwidth threshold, acting as a natural **low-pass filter**.
