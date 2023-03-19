function thetalist_new = NextState(thetalist,thetadot_list,timestep, max_vel)
% NextState: based on a simple first-order Euler step that output the next stage set of angles
% Input:
%       thetalist : set of angles of current state of the robot (6 jointsangles)
%       thetadot_list : six joint velocities
%       timestep : the time step between each state
%       max_vel : the maximum velocity magnitude
% Output:
%       thetalist_new : the next state set of angles
addpath('mr\')

if thetadot_list > max_vel
    disp('Joint velocity exceeds maximum allowed')
else
    thetalist_new = thetalist + (thetadot_list*timestep);
end
end