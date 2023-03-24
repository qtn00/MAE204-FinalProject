clc; clear all;
T_se =  [0 0 1 323.6; -1 0 0 -335.6; 0 -1 0 237; 0 0 0 1];
T_sc_ini = [1 0 0 450; 0 1 0 -300; 0 0 1 0; 0 0 0 1];
T_sc_fi = [0 -1 0 0; 1 0 0 100; 0 0 1 0; 0 0 0 1];
T_ce_g = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1];
T_ce_stand = [0 0 1 0; -1 0 0 0; 0 -1 0 50; 0 0 0 1];
dt = 0.5;
[traj,gripper_state] = TrajectoryGenerator(T_se,T_sc_ini,T_sc_fi,T_ce_g,T_ce_stand,dt)