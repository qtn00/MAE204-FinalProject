function [V_b,x_e,thetalist] = FeedbackControl(thetalistin,T_sed,T_sedn,kp,ki,dt)
% FeedbackControl: calculates the task-space feedforward plus feedback
% control law
% Input:
%       thetalistin : The actual configuration of the end effector
%                     [1 x 6 vector]
%       T_sed : The desired current configuration of the end effector
%               [4 x 4 matrix]
%       T_sedn : The desired configuration of the end effector at the next
%                timestep [4 x 4 matrix]
%       kp : The P control gain [int/double]
%       ki : The I control gain [int/double]
%       dt : The timestep between reference trajectory configurations
%            [int/double]
% Output:
%       V_b : The end-effector twist expressed in the end-effector frame
%             [6 x N matrix]
%       x_e : The configuration error twist [6 x N matrix]
%       thetalist : The new list of thetas based on the control algorithm
%                   [N x 6 matrix]
% Example Input:
% clc; clear all;
% thetalistin = [-pi,-pi/2,pi/2,-pi/2,-pi/2,-pi/6];
% T_sed{1} = [0 0 1 300; -1 0 0 -300; 0 -1 0 237; 0 0 0 1];
% T_sedn{1} = [0 0 1 290; -1 0 0 -290; 0 -1 0 237; 0 0 0 1];
% kp = 0;
% ki = 0;
% dt = 0.5;
% [V_b,x_e,thetalist] = FeedbackControl(thetalistin,T_sed,T_sedn,kp,ki,dt)
% 
% Output:
% V_b =
%      0
%      0
%      0
%      0
%      0
%      0
% x_e =
%      0
%      0
%      0
%      0
%      0
%      0
% thetalist =
%    -3.1416   -1.5710    1.5710   -1.5710   -1.5710   -0.5236

format long g;

%% Initialize 
Kp = kp*eye(6,6); % P Control gain to matrix
Ki = ki*eye(6,6); % I Control gain to matrix
x_e = zeros(6,length(T_sed));
V_b = zeros(6,length(T_sed));
thetalist(1,:) = thetalistin;

%% FeedbackControl
S1 = [0,0,1,-300,0,0]';
S2 = [0,1,0,-240,0,0]';
S3 = [0,1,0,-240,0,244]';
S4 = [0,1,0,-240,0,457]';
S5 = [0,0,-1,169,457,0]';
S6 = [0,1,0,-155,0,457]';
Slist = [S1 S2 S3 S4 S5 S6];
M = [1 0 0 457; 0 1 0 78; 0 0 1 155; 0 0 0 1];

Blist = zeros(6,6);
for ii = 1:length(Slist)
    Blist(:,ii) = Adjoint(TransInv(M))*Slist(:,ii);
end

for i = 1:length(T_sed)-1
    T_se_current{i} = FKinBody(M,Blist,thetalist(i,:)');
    V_d = se3ToVec((1/dt)*MatrixLog6(TransInv(T_sed{i})*T_sedn{i}));
    feedfor_V_d = Adjoint(TransInv(T_se_current{i})*T_sed{i})*V_d;
    x_e(:,i) = se3ToVec(MatrixLog6(TransInv(T_se_current{i})*T_sed{i}));
    %%% BELOW IS FEEDFORWARD ONLY OR FEEDFORWARD PLUS P OR PI CONTROL) %%%
    V_b(:,i) = feedfor_V_d + Kp*x_e(:,i) + Ki*(sum(x_e*dt,2));
    %%% UNCOMMENT BELOW FOR P OR PI ONLY CONTROL %%%
%   V_b(:,i) = Kp*x_e(:,i) + Ki*(sum(x_e*dt,2)); 
    Jb = JacobianBody(Blist,thetalist(i,:));
    psuedoJb = pinv(Jb,1e-2);
    thetalist_dot = psuedoJb*V_b(:,i);
    thetalist(i+1,:) = NextState(thetalist(i,:),thetalist_dot',dt,10);
    for i3 = 1:6
        if thetalist(i,i3) > pi || thetalist(i,i3) < -pi
            thetalist(i,i3) = wrapToPi(thetalist(i,i3));
        end
    end

%% wrapToPi function wraps the angle to fall between -pi and pi
for i3 = 1:6
    if thetalist(end,i3) > pi || thetalist(end,i3) < -pi
        thetalist(end,i3) = wrapToPi(thetalist(end,i3));
    end
end

end