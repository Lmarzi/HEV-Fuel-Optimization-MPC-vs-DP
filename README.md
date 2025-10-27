# HEV Fuel Optimization: MPC vs. Dynamic Programming

> A project for the **Numerical Optimization for Control** course
> M.Sc. in Automation and Control Engineering
> Politecnico di Milano

---

## Authors

* Francesco Berruti
* Andrea Bonafini
* Federico Bordini
* Lorenzo Marzi

## 1. Project Goal

This project develops and compares two optimization strategies to **minimize fuel consumption** in a full hybrid electric vehicle (FHEV). The primary goal is to find the optimal torque-split strategy between the Internal Combustion Engine (ICE) and the Electric Motor (EM).

The system is modeled using a "backward-looking" approach and utilizes real-world data from a 2013 Ford Fiesta hybrid, including ICE and EM efficiency maps.

## 2. Methodologies Compared

### 1. Dynamic Programming (DP)

* **Role:** Serves as the **offline benchmark**.
* **Method:** Solves the complete optimization problem non-causally (requiring full knowledge of the entire drive cycle). It uses Bellman's optimality principle to find the global optimum for fuel consumption.

### 2. Model Predictive Control (MPC)

* **Role:** Developed as a causal, **online-capable strategy** suitable for real-time implementation.
* **Method:** Solves a Finite Horizon Optimal Control Problem (FHOCP) at each time step. The underlying Nonlinear Problem (NLP) is solved using a custom-built **Sequential Quadratic Programming (SQP)** solver.

## 3. MPC Implementation Details

* **Optimization Variable:** The core control variable is $u(t) = P_{EM} / P_{TOT}$, representing the power split ratio.
* **Cost Function:** To handle the charge-sustaining goal without full future knowledge, the MPC uses a modified cost function: $l = \dot{m}_f + s_{eq} \cdot \dot{m}_b$. Here, $s_{eq}$ is an **equivalence factor** that acts as a "virtual fuel cost" for using battery power, penalizing battery depletion at low states of charge (SoC).
* **Solver:** A custom SQP solver (`my_fmincon.m`) was implemented, using the BFGS method for the Hessian and Central Finite Differences for the gradient.

## 4. Key Results

1.  **Prediction Horizon ($N=1$):** A key finding was that increasing the MPC prediction horizon from $N=1$ to $N=5$ yielded only **marginal improvements** in performance. Therefore, a horizon of **$N=1$** was chosen, drastically reducing computational load and making the solution feasible for real-time control without needing future predictions.
2.  **Basic MPC (Unconstrained):** The MPC ($N=1$) strategy achieved fuel consumption **within ~5%** of the global optimum found by DP (average $r \approx 1.05$). This is described in the report as an "excellent result".
3.  **Modified MPC (Charge-Sustaining):** When modified to enforce the final SoC constraint, the MPC's consumption was **within ~8.7%** of the DP benchmark (average $r \approx 1.0873$), still considered "highly satisfactory".

The study concludes that a real-time MPC strategy, even with a minimal horizon, can achieve near-optimal fuel efficiency.

## 5. How to Run the Project

The project is written entirely in MATLAB.

1.  The main script to run the simulation and comparison is **`main.m`**. ( There is an error in the code with dynamic programming, therefore only MPC results are shown)
2.  Running this script will:
    * Implement the MPC algorithm.
    * Call the custom SQP solver (`my_fmincon.m`) to solve the optimization problem at each step.
    * Execute `dp_comp.m` to compute the offline Dynamic Programming benchmark solution.
    * Generate plots comparing the fuel consumption and SoC trajectories for both methods.

### Code Structure

* **`main.m`**: Main script that runs the MPC, calls the DP, and compares the results.
* **`my_hev.m`**: Contains all vehicle data, physical parameters, and the simulation model.
* **`my_fullhorizon.m`**: A handle script that defines the cost function and constraints for the optimization problem.
* **`dp_comp.m`**: Script to compute the benchmark solution using Dynamic Programming.
* **`my_fmincon.m`**: The custom implementation of the SQP solver.
* **`my_optimset.m`**: Defines all parameters and settings for the SQP solver (e.g., tolerances, BFGS method).
* **`my_gradient.m`**: Utility function for computing numerical gradients.
* **`Report.pdf`**: The full project report detailing the theory, implementation, and results.
