clc; clear all; close all;
format long g;



thetalist = [-pi/6,-pi/2,pi/2,-pi/2,-pi/2,5*pi/6];
T_sed = [0 0 1 300;-1 0 0 -300;0 -1 0 237;0 0 0 1];
T_sedn = [0 0 1 290;-1 0 0 -290;0 -1 0 237;0 0 0 1];
kp = 1; ki = 0;
dt = 0.01;
[V_b,V_error,thetalist] = FeedbackControl(thetalist,T_sed,T_sedn,kp,ki,dt);