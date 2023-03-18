format long;
clc
clear all

%% Initial 
thetalist = [-pi/6,-pi/2,pi/2,-pi/2,-pi/2,5*pi/6];
T_sed = [0 0 1 300;-1 0 0 -300;0 -1 0 237;0 0 0 1];
T_sedn = [0 0 1 290;-1 0 0 -290;0 -1 0 237;0 0 0 1];

Kp = zeros(4,4);
Ki = zeros(4,4);
dt = 0.01;

%% FeedbackControl
S1 = [0,0,1,-300,0,0]';
S2 = [0,1,0,-240,0,0]';
S3 = [0,1,0,-240,0,244]';
S4 = [0,1,0,-240,0,457]';
S5 = [0,0,-1,169,457,0]';
S6 = [0,1,0,-155,0,457]';
Slist = [S1 S2 S3 S4 S5 S6];
M = [1 0 0 457; 0 1 0 78; 0 0 1 155; 0 0 0 1];

twist = se3ToVec((1/dt)*MatrixLog6(TransInv(T_sed)*T_sedn));
T_se = FKinSpace(M,Slist,thetalist.');
CheckTwist = Adjoint(TransInv(T_se)*T_sed)*twist;
Jacobian = JacobianSpace(Slist, thetalist)
%psuedoJacobian = TransInv(M)*Jacobian'*TransInv(Jacobian*TransInv(M)*Jacobian')*twist;
