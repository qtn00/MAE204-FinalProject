clc; clear all; close all; 
format long; 

%% Initial T matrix (input)
% Initial Tse
T_se =  [0 1 0  247;
	    1 0 0  -169;
	    0 0 -1  782;
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
T_ce_g = [1 0 0 0;
            0 0 1 0;
            0 -1 0 0;
            0 0 0 1];
% e-e relative to cube standoff position;
T_ce_stand = [1 0 0 0;
            0 0 1 0;
            0 -1 0 30;
            0 0 0 1];
dt = 0.01;

%% Start of the function
tf = 5;

% Standoff position relative to {s} above initial cube
T_se_stand_ini = T_sc_ini*T_ce_stand;

% Grasp position relative to {s} at inital cube
T_se_g_ini = T_sc_ini*T_ce_g;

% Standoff position relative to {s} above final cube 
T_se_stand_fi = T_sc_fi*T_ce_stand;

% Grasp position relative to {s} at final cube
T_se_g_fi = T_sc_fi*T_ce_g;

% Trajectory of each segment:

% % traj = [ScrewTrajectory(T_se,T_se_stand_ini,tf ,dt,3),...
% %     ScrewTrajectory(T_se_stand_ini,T_se_g_ini,tf,dt,3),...
% %     ScrewTrajectory(T_se_g_ini,T_se_stand_ini,tf,dt,3),...
% %     ScrewTrajectory(T_se_stand_ini,T_se_stand_fi,tf,dt,3),...
% %      ScrewTrajectory(T_se_stand_fi,T_se_g_fi,tf,dt,3),...
% %      ScrewTrajectory(T_se_g_fi,T_se_stand_fi,tf,dt,3)];
[traj1,gripper_state1] = ScrewTrajectory(T_se,T_se_stand_ini,tf ,dt,'open',3);
[traj2,gripper_state2] = ScrewTrajectory(T_se_stand_ini,T_se_g_ini,tf,dt,'open',3);
[traj3,gripper_state3] = ScrewTrajectory(T_se_g_ini,T_se_stand_ini,tf,dt,'close',3);
[traj4,gripper_state4] = ScrewTrajectory(T_se_stand_ini,T_se_stand_fi,tf,dt,'close',3);
[traj5,gripper_state5] = ScrewTrajectory(T_se_stand_fi,T_se_g_fi,tf,dt,'close',3);
[traj6,gripper_state6] = ScrewTrajectory(T_se_g_fi,T_se_stand_fi,tf,dt,'open',3);

% Gripper State 0 for open and 1 for close for indicing
traj = [traj1,traj2,traj3,traj4,traj5,traj6];
gripper_state = [gripper_state1,gripper_state2,gripper_state3,gripper_state4,gripper_state5,gripper_state6];

% % indexofopen = 2*length(traj)/6;
% % indexofclose = 5*length(traj)/6;
% % Gripper state value
% % gripper_state = zeros(1,numel(traj));
% % gripper_state(indexofopen:indexofclose) = 1;
% 
output = zeros(numel(traj),13);

for i = 1:length(traj)
    output(i,:) = [traj{i}(1,1:3),traj{i}(2,1:3),traj{i}(3,1:3),traj{i}(1:3,end)',gripper_state(i)];
end

csvwrite('m.csv', output);


