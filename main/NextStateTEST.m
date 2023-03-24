clc; clear all;
thetalist = [-pi/6,-pi/2,pi/2,-pi/2,-pi/2,5*pi/6];
vel = [0.1,0.1,0.1,0.1,0.1,0.1];
maxvel = pi;
dt = 0.01;
Tf = 1;
N = Tf/dt;
thetalist_new = NextState(thetalist,vel,dt,maxvel)