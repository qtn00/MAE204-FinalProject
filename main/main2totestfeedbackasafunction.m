clc; clear all; close all;
format long g;



thetalist = [-pi/6,-pi/2,pi/2,-pi/2,-pi/2,5*pi/6];
T_sed = [0 0 1 300;-1 0 0 -300;0 -1 0 237;0 0 0 1];
T_sedn = [0 0 1 290;-1 0 0 -290;0 -1 0 237;0 0 0 1];
kp = 1; ki = 0;
dt = 0.01;

S1 = [0,0,1,-300,0,0]';
S2 = [0,1,0,-240,0,0]';
S3 = [0,1,0,-240,0,244]';
S4 = [0,1,0,-240,0,457]';
S5 = [0,0,-1,169,457,0]';
S6 = [0,1,0,-155,0,457]';
Slist = [S1 S2 S3 S4 S5 S6];
M = [1 0 0 457; 0 1 0 78; 0 0 1 155; 0 0 0 1];
Blist = zeros(6,6);
for i = 1:length(Slist)
    Blist(:,i) = Adjoint(TransInv(M))*Slist(:,i);
end
T_se = FKinSpace(M,Slist,thetalist.');

V_b = round(FeedbackControl(T_se,T_sed,T_sedn,kp,ki,dt),5);
Jb = round(JacobianBody(Blist,thetalist),8);
psuedoJb = round(pinv(Jb),8);
thetalist_dot = round(pinv(Jb)*V_b,4);

