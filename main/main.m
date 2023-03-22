clc; clear all; close all;
format long;
addpath('mr\')
savepath
%% Initialization
% Initial Tse (initial state) Input of wrapper
T_se =  [0 0 1  323.6;
	    -1 0 0  -335.6;
	    0 -1 0  237;
	    0 0 0    1];
T_reference_ini = [0 1 0 323.6;1 0 0 -335.6; 0 0 -1 237;0 0 0 1];
% Initial Tsc Input of Wrapper
T_sc_ini = [1 0 0 450;
    0 1 0 -300;
    0 0 1 0;
    0 0 0 1];
% Final Tsc Input of Wrapper
T_sc_fi = [0 -1 0 0;
            1 0 0 100;
            0 0 1 0;
            0 0 0 1];
% e-e relative to cube while GRASPING
kp =0; ki = 0;

%%
thetalist(1,:) = [-pi/6,-pi/2,pi/2,-pi/2,-pi/2,5*pi/6]; %Initial states
T_ce_g = [0 0 1 0;
            -1 0 0 0;
            0 -1 0 0;
            0 0 0 1];
% e-e relative to cube standoff position;
T_ce_stand = [0 0 1 0;
            -1 0 0 0;
            0 -1 0 100;
            0 0 0 1];
dt = 0.01;

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
[traj,gripper_state] = TrajectoryGenerator(T_reference_ini,T_sc_ini,T_sc_fi,T_ce_g,T_ce_stand,dt);

T_sed = traj;
T_sedn = traj(1,2:end);
V_b = zeros(6,length(traj));
V_error = zeros(6,length(traj));
thetalist_dot = zeros(6,length(traj));
for i = 1:length(traj)-1
    T_se_current{i} = round(FKinSpace(M,Blist,thetalist(i,:)'),1);
    V_b(:,i) = FeedbackControl(T_se,T_sed{i},T_sedn{i},kp,ki,dt);
    Jb = round(JacobianBody(Blist,thetalist(i,:)),4);
    psuedoJb = round(pinv(Jb),4);
    thetalist_dot(:,i) = round(psuedoJb*V_b(:,i),4);
    thetalist(i+1,:) = NextState(thetalist(i,:),thetalist_dot(:,i)',dt,10);
%     for i3 = 1:6
%         if thetalist(i+1,i3) > 2*pi || thetalist(i+1,i3) < -2*pi
%             thetalist(i+1,i3) = wrapToPi(thetalist(i+1,i3));
%         end
%     end
    

end
    


%%
figure; hold on;
plot(V_error);
% figure; hold on 
% for a = 1:length(V_error)
%     ang(a) = norm(V_error([1:3],a));
%     lin(a) = norm(V_error(4:6,a));
% end
% plot(0:0.01:(length(ang)*dt)-0.01,ang);
%  
output = zeros(length(thetalist),7);
for ii = 1:length(thetalist)
    output(ii,:) = [thetalist(ii,:),gripper_state(ii)];
end
csvwrite('joint.csv',output);

% output = zeros(length(traj),13);
% for i = 1:length(traj)
%     output(i,:) = [traj{i}(1,1:3),traj{i}(2,1:3),traj{i}(3,1:3),traj{i}(1:3,end)',gripper_state(i)];
% end
% csvwrite('m.csv', output);

    
    