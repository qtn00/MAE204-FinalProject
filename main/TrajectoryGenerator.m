function [traj,gripper_state]= TrajectoryGenerator(T_se,T_sc_ini,T_sc_fi,T_ce_g,T_ce_stand,dt)
%% Start of the function
addpath('mr\')
% Tf = 7;

% Standoff position relative to {s} above initial cube
T_se_stand_ini = T_sc_ini*T_ce_stand;

% Grasp position relative to {s} at inital cube
T_se_g_ini = T_sc_ini*T_ce_g;

% Standoff position relative to {s} above final cube 
T_se_stand_fi = T_sc_fi*T_ce_stand;

% Grasp position relative to {s} at final cube
T_se_g_fi = T_sc_fi*T_ce_g;

% Trajectory of each segment:
% Moving from initial configuration to standoff
[traj1,gripper_state1] = ScrewTrajectory_modified(T_se,T_se_stand_ini,3 ,dt,'open',3);
% Moving to grasping position at inital cube position and grap the cube
[traj2,gripper_state2] = ScrewTrajectory_modified(T_se_stand_ini,T_se_g_ini,2 ,dt,'open',3);
% Moving back to standoff
[traj3,gripper_state3] = ScrewTrajectory_modified(T_se_g_ini,T_se_stand_ini,2,dt,'close',3);
% Moving to final standoff position
[traj4,gripper_state4] = ScrewTrajectory_modified(T_se_stand_ini,T_se_stand_fi,5,dt,'close',3);
% Moving to final cube position and release the cube
[traj5,gripper_state5] = ScrewTrajectory_modified(T_se_stand_fi,T_se_g_fi,2,dt,'close',3);
% Moving back to final standoff
[traj6,gripper_state6] = ScrewTrajectory_modified(T_se_g_fi,T_se_stand_fi,2,dt,'open',3);
traj = [traj1,traj2,traj3,traj4,traj5,traj6];

% Gripper State 0 for open and 1 for close for indicing
gripper_state = [gripper_state1,gripper_state2,gripper_state3,gripper_state4,gripper_state5,gripper_state6];

end

