format long;
clc
clear all

%% Initial 
thetalist = [-pi/6,-pi/2,pi/2,-pi/2,-pi/2,5*pi/2];
T_sed = [0 0 1 300;-1 0 0 -300;0 -1 0 237;0 0 0 1];
T_sedn = [0 0 1 290;-1 0 0 -290;0 -1 0 237;0 0 0 1];

Kp = zeros(4,4);
Ki = zeros(4,4);
dt = 0.01;

%% FeedbackControl
