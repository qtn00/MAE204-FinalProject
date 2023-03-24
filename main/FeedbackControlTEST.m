clc; clear all;
thetalistin = [-pi,-pi/2,pi/2,-pi/2,-pi/2,-pi/6];
T_sed{1} = [0 0 1 300; -1 0 0 -300; 0 -1 0 237; 0 0 0 1];
T_sedn{1} = [0 0 1 290; -1 0 0 -290; 0 -1 0 237; 0 0 0 1];
kp = 0;
ki = 0;
dt = 0.5;
[V_b,x_e,thetalist] = FeedbackControl(thetalistin,T_sed,T_sedn,kp,ki,dt)
