clc; clear all; close all;
format long;

%% Initialization
% Initial Tse (initial state) Input of wrapper
T_reference_ini = [0 1 0 247; 1 0 0 -169; 0 0 -1 782;0  0 0 1];
% Initial Tsc Input of Wrapper
T_sc_ini = [1 0 0 450; 0 1 0 -300; 0 0 1 0; 0 0 0 1];
% Final Tsc Input of Wrapper
T_sc_fi = [0 -1 0 0; 1 0 0 100; 0 0 1 0; 0 0 0 1];
% e-e relative to cube while GRASPING
T_ce_g = [0 0 1 0; -1 0 0 0; 0 -1 0 0; 0 0 0 1];
% e-e relative to cube standoff position;
T_ce_stand = [0 0 1 0; -1 0 0 0; 0 -1 0 50; 0 0 0 1];

%%% "New Task" Case %%%
% T_reference_ini = [0 1 0 247; 1 0 0 -169; 0 0 -1 782;0  0 0 1];
% %%From 5.3%% T_se = [0 0 1 323.6; -1 0 0 -335.6; 0 -1 0 237; 0 0 0 1];
% T_sc_ini = [0 -1 0 0; 1 0 0 100; 0 0 1 0; 0 0 0 1];
% T_sc_fi = [0 -1 0 -500; 1 0 0 -200; 0 0 1 0; 0 0 0 1];

thetalistin = [0.000501442170089313+(pi/6),4.71153838730754, ...
    0.00213337924610927,-1.57207911278733,-1.57129776896494, ...
    6.77755554611493e-08]; % Actual theta values
kp = 4; % P Control gain
ki = 0; % I Control gain
dt = 0.01; % Timestep

%% Create Slist and Blist
S1 = [0,0,1,-300,0,0]';
S2 = [0,1,0,-240,0,0]';
S3 = [0,1,0,-240,0,244]';
S4 = [0,1,0,-240,0,457]';
S5 = [0,0,-1,169,457,0]';
S6 = [0,1,0,-155,0,457]';
Slist = [S1 S2 S3 S4 S5 S6];
M = [1 0 0 457; 0 1 0 78; 0 0 1 155; 0 0 0 1];
Blist = zeros(6,6);
for i3 = 1:length(Slist)
    Blist(:,i3) = Adjoint(TransInv(M))*Slist(:,i3);
end

%% Trajectory Generator:
[traj,gripper_state] = TrajectoryGenerator(T_reference_ini,T_sc_ini,T_sc_fi,T_ce_g,T_ce_stand,dt);
T_sed = traj;
T_sedn = traj(1,2:end);

%% Motion control with FeedbackControl:
[V_b,x_e,thetalist] = FeedbackControl(thetalistin,T_sed,T_sedn,kp,ki,dt);

%% Plotting Error Twist (x_e)
figure; hold on;
plot(x_e(1,:));
plot(x_e(2,:));
plot(x_e(3,:));
plot(x_e(4,:));
plot(x_e(5,:));
plot(x_e(6,:));
legend('wx (rad/cs)','wy (rad/cs)','wz (rad/cs)','vx (mm/cs)','vy (mm/cs)','vz (mm/cs)');
title('Error Twist vs Time');
xlabel('Time (cs)');
ylabel('Error Twist')

%% Create output file of thetalist and gripper state
% -The first 6 elements of output are the joint angles and the last element
% is the grippper state (0 for open and 1 for close)
% - Second out out is a 6-vector end-effector error as a function of time
% Example Output:
% output = [pi/2,pi/2,pi/6,0,-pi/4,0,1]
% V_error = [0.1,0.2,0,0.2,0.1,0; -0.2,-1,-3,0,100,-300]';

output = zeros(length(thetalist),7);
for ii = 1:length(thetalist)
    output(ii,:) = [thetalist(ii,:),gripper_state(ii)];
end
V_error = x_e;
csvwrite('12.csv',output);

