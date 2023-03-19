clc; clear all; close all;
format long;
addpath('mr\')

%% Initialization
% Initial Tse
T_se =  [0 0 1  247;
	    -1 0 0  -169;
	    0 -1 0  782;
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
kp =1; ki = 0;


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

%% Trajectory Generator:
[traj,gripper_state] = TrajectoryGenerator(T_se,T_sc_ini,T_sc_fi,T_ce_g,T_ce_stand,dt);

T_sed = traj;
T_sedn = traj(1,2:end);
thetalist(1,:) = [-pi/6,-pi/2,pi/2,-pi/2,-pi/2,5*pi/6];
V_b = zeros(6,length(traj));
v_error = zeros(6,length(traj));
thetalist_dot = zeros(6,length(traj));

for i = 1:length(traj)-1
    
    [V_b(:,i),v_error(:,i)] = FeedbackControl(T_se,T_sed{i},T_sedn{i},kp,ki,dt);
    Jb = round(JacobianBody(Blist,thetalist(i,:)),8);
    psuedoJb = round(pinv(Jb),8);
    thetalist_dot(:,i) = round(pinv(Jb)*V_b(:,i),4);
    thetalist(i+1,:) = NextState(thetalist(i,:),thetalist_dot(:,i)',dt,10);
end

for a = 1:length(v_error)
    ang(a) = norm(v_error([1:3],a));
    lin(a) = norm(v_error(4:6,a));
end
figure; hold on 
subplot(2,1,1)
plot(1:length(ang),ang);
subplot(2,1,2)
plot(1:length(lin),lin);

output = zeros(length(thetalist),7);
for ii = 1:length(thetalist)
    output(ii,:) = [thetalist(ii,:),gripper_state(ii)];
end
csvwrite('joint.csv',output);
% output = zeros(length(traj),13);
% for i = 1:length(traj)
%     output(i,:) = [traj{i}(1,1:3),traj{i}(2,1:3),traj{i}(3,1:3),traj{i}(1:3,end)',gripper_state(i)];
% end
% csvwrite('TrajectoryOutput.csv', output);

    