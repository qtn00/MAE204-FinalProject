function thetalist_new = NextState(thetalist,thetadot_list,timestep, max_vel)
% NextState: based on a simple first-order Euler step that output the next
% stage set of angles
% Input:
%       thetalist : The set of angles of current state of the robot
%                   [1 x 6 vector]
%       thetadot_list : Six joint angular velocities [1 x 6 vector]
%       timestep : The time step between each state [int/double]
%       max_vel : The maximum velocity magnitude [int/double]
% Output:
%       thetalist_new : The next state set of angles [1 x 6 vector]
%
% Example Input:
% clc; clear all;
% thetalist = [-pi/6,-pi/2,pi/2,-pi/2,-pi/2,5*pi/6];
% vel = [0.1,0.1,0.1,0.1,0.1,0.1];
% maxvel = pi;
% dt = 0.01;
% Tf = 1;
% N = Tf/dt;
% thetalist_new = NextState(thetalist,vel,dt,maxvel)
%
% Output:
% thetalist_new =
%    -0.5226   -1.5698    1.5718   -1.5698   -1.5698    2.6190

if thetadot_list > max_vel
    disp('Joint velocity exceeds maximum allowed')
else
    thetalist_new = thetalist + (thetadot_list*timestep);
end
end