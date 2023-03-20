function [V_b,X_e] = FeedbackControl(T_se,T_sed,T_sedn,kp,ki,dt)
 
format long g;

addpath('mr\')
%% Initial 

Kp = kp*eye(6,6);
Ki = ki*eye(6,6);


%% FeedbackControl
Kp = kp*eye(6,6);
Ki = ki*eye(6,6);

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

V_d = se3ToVec((1/dt)*MatrixLog6(TransInv(T_sed)*T_sedn));
% T_se = FKinSpace(M,Slist,thetalist.');
feedfor_V_d = Adjoint(TransInv(T_se)*T_sed)*V_d;

X_e = se3ToVec(MatrixLog6(TransInv(T_se)*T_sed));
V_b = feedfor_V_d + Kp*X_e + Ki*(X_e*dt);