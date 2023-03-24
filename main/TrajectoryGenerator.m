function [traj,gripper_state]= TrajectoryGenerator(T_se,T_sc_ini,T_sc_fi,T_ce_g,T_ce_stand,dt)
% TrajectoryGenerator: generates a set of tranformation matrices
% Input:
%       T_se : The initial configuration of the end-effector in reference
%              to the space frame [4 x 4 matrix]
%       T_sc_ini : The initial configuration of the cube in reference to
%                  the space frame [4 x 4 matrix]
%       T_sc_fi : The desired final configuration of the cube in reference
%                 to the space frame [4 x 4 matrix]
%       T_ce_g : The configuration of the end-effector relative to the cube
%                while grasping [4 x 4 matrix]
%       T_ce_stand : The stand-off configuration of the end-effector above
%                    the cube (before and after grasping) relative to the
%                    cube [4 x 4 matrix]
%       dt : The timestep between reference configurations [int/double]
% Output:
%       traj : The desired trajectory from the initial configuration to the
%              final configuration [1 x N cell array filled with 4x4
%              tranformation matrixes]
%       gripper_state : The state of the gripper (0 for open, 1 for closed)
%                       [1 x N matrix]
%
% Example Input:
% clc; clear all;
% T_se =  [0 0 1 323.6; -1 0 0 -335.6; 0 -1 0 237; 0 0 0 1];
% T_sc_ini = [1 0 0 450; 0 1 0 -300; 0 0 1 0; 0 0 0 1];
% T_sc_fi = [0 -1 0 0; 1 0 0 100; 0 0 1 0; 0 0 0 1];
% T_ce_g = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1];
% T_ce_stand = [0 0 1 0; -1 0 0 0; 0 -1 0 50; 0 0 0 1];
% dt = 0.5;
% [traj,gripper_state] = TrajectoryGenerator(T_se,T_sc_ini,T_sc_fi,T_ce_g,T_ce_stand,dt)
% 
% Output:
% traj = 1 x 26 cell array of 4 x 4 doubles
% gripper_state = 0     0     0     0     0     0     0     0     0     1
%                 1     1     1     1     1     1     1     1     1     1
%                 1     1     1     1     0     0

%% Initialize
% Standoff position relative to {s} above initial cube
T_se_stand_ini = T_sc_ini*T_ce_stand;

% Grasp position relative to {s} at inital cube
T_se_g_ini = T_sc_ini*T_ce_g;

% Standoff position relative to {s} above final cube 
T_se_stand_fi = T_sc_fi*T_ce_stand;

% Grasp position relative to {s} at final cube
T_se_g_fi = T_sc_fi*T_ce_g;

%% Trajectory of each segment:
% Moving from initial configuration to standoff
[traj1,gripper_state1] = ScrewTrajectory_modified(T_se,T_se_stand_ini,3 ,dt,'open',3);
% Moving to grasping position at inital cube position and grap the cube
[traj2,gripper_state2] = ScrewTrajectory_modified(T_se_stand_ini,T_se_g_ini,0.5 ,dt,'open',3);
% Moving back to standoff
[traj3,gripper_state3] = ScrewTrajectory_modified(T_se_g_ini,T_se_stand_ini,0.5,dt,'close',3);
% Moving to final standoff position
[traj4,gripper_state4] = ScrewTrajectory_modified(T_se_stand_ini,T_se_stand_fi,5,dt,'close',3);
% Moving to final cube position and release the cube
[traj5,gripper_state5] = ScrewTrajectory_modified(T_se_stand_fi,T_se_g_fi,0.5,dt,'close',3);
% Moving back to final standoff
[traj6,gripper_state6] = ScrewTrajectory_modified(T_se_g_fi,T_se_stand_fi,0.5,dt,'open',3);
traj = [traj1,traj2,traj3,traj4,traj5,traj6];

% Gripper State 0 for open and 1 for close for indicing
gripper_state = [gripper_state1,gripper_state2,gripper_state3,gripper_state4,gripper_state5,gripper_state6];

end

