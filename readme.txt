Codes for the control of an autonomous car using MPC

This code has the following files (for path of 8 shape):
- dynamics_auto_car: First, build the model using the Kinematic Car Model and get the mex files in the current directory

- MPC_Run : Run the MPC Code and update state using ode45. Set the velocity and radius at lines 12 and 13 of your choice to see if the model works for the set values. Uncomment the last section to generate animation. This file also generates the plots

- MPC_Car: Function To build the matrices for MPC and uses the QPSwift solver to get the optimal decision variables values.

- gettraj: Function to generate the trajectory given radius and velocity

- Animate_Car: Function for Animation

- Robustness_final: Run To evaluate robustness. Change the value of 'Change_perc' at line 12.

For path of Sin shape:
- MPC_Run_sin: Run the MPC Code, get animations and plots

- gentraj_sim: To get the sin trajectory

- Animate_Car_sin: Function for animation
 