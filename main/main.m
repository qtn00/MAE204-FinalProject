clc; clear all; close all;
format long;

%% Initialization
% Initial Tse
T_se =  [0 0 1  323.6;
	    -1 0 0  -335.6;
	    0 -1 0  237;
	    0 0 0    1];
% Initial Tsc
T_sc_ini = [1 0 0 450;
    0 1 0 -300;
    0 0 1 0;
    0 0 0 1];
% Final Tsc
T_sc_fi = [0 -1 0 0;
            1 0 0 100;
            0 0 1 0;
            0 0 0 1];
% e-e relative to cube while GRASPING
T_ce_g = [0 0 1 0;
            -1 0 0 0;
            0 -1 0 0;
            0 0 0 1];
% e-e relative to cube standoff position;
T_ce_stand = [0 0 1 0;
            -1 0 0 0;
            0 -1 0 30;
            0 0 0 1];
dt = 0.01;

%% Trajectory Generator:
[traj,gripper_state] = TrajectoryGenerator(T_se,T_sc_ini,T_sc_fi,T_ce_g,T_ce_stand,dt);

output = zeros(length(traj),13);
for i = 1:length(traj)
    output(i,:) = [traj{i}(1,1:3),traj{i}(2,1:3),traj{i}(3,1:3),traj{i}(1:3,end)',gripper_state(i)];
end
csvwrite('TrajectoryOutput.csv', output);

%% Next State
% for i = 1:
    