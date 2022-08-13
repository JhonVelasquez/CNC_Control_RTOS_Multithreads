# CNC_Control_RTOS_Multithreads

Project for implemmenting control of movement of a CNC with two axes. 

## Each section (directory) has similar codes, the difference is in the algorithm of control, there are three:
* space states with error integral control
* space states with error integral and states observer control
* space states with error integral and real states observer control

## Project contains:

### C_codes: 
codes in c for the control of the motors for X and Y axes of an CNC.

### Matlab_design: 
scripts for generating the routes of reference that is executed in the C_code

![alt text](https://github.com/JhonVelasquez/CNC_Control_RTOS_Multithreads/blob/main/Matlab_design/Simulink/Est_int_obs_m2.PNG)
![alt text](https://github.com/JhonVelasquez/CNC_Control_RTOS_Multithreads/blob/main/Matlab_design/Simulink/Est_int_obs_m1_resultado.PNG)

### Results_screenshoots: 
routes of the CNC control simulation
  
![alt text](https://github.com/JhonVelasquez/CNC_Control_RTOS_Multithreads/blob/main/Results_screenshoots/Estados_accion_integral/plot_pot_x_y.png)
