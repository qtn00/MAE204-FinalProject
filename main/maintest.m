clc; clear all; close all;
format short;
%% Initialization
% Initial Tse (initial state) Input of wrapper
% T_se =  [0 0 1  323.6;
% 	    -1 0 0  -335.6;
% 	    0 -1 0  237;
% 	    0 0 0    1];
T_reference_ini = [0 1 0 247; 1 0 0 -169; 0 0 -1 782; 0 0 0 1];
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
thetalistin = [-pi/6,-pi/2,pi/2,-pi/2,-pi/2,5*pi/6]; %Initial states
T_ce_g = [0 0 1 0;
            -1 0 0 0;
            0 -1 0 0;
            0 0 0 1];
% e-e relative to cube standoff position;
T_ce_stand = [0 0 1 0;
            -1 0 0 0;
            0 -1 0 50;
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
for i3 = 1:length(Slist)
    Blist(:,i3) = Adjoint(TransInv(M))*Slist(:,i3);
end
%% Trajectory Generator:
[traj,gripper_state] = TrajectoryGenerator(T_reference_ini,T_sc_ini,T_sc_fi,T_ce_g,T_ce_stand,dt);

T_sed = traj;
T_sedn = traj(1,2:end);

M = [1 0 0 457; 0 1 0 78; 0 0 1 155; 0 0 0 1];
T_reference_ini = [sind(30) cosd(30) 0 302; cosd(30) -sind(30) 0 -169; 0 0 -1 782; 0 0 0 1];
thetalist0 = [-pi/6,-pi/2, pi, pi/2, -pi*(7/6), 0];
% thetalist0 = [36.3030675466563, 487.93452536417, -56.3769730881377, -478.68144207988, 30.7175757299258, 6.28318530717987];
eomg = 0.01;
ev = 0.001;

[V_b,x_e,thetalist] = FeedbackControl(thetalistin,T_sed,T_sedn,kp,ki,dt);
[thetalistnew, success] = IKinBody(Blist, M, T_reference_ini, thetalist0.', eomg, ev);
T = FKinBody(M,Blist,thetalistnew)


% V_b = zeros(6,length(traj));
% V_error = zeros(6,length(traj));
% thetalist_dot = zeros(6,length(traj));
% for i = 1:length(traj)-1
%     T_se_current = FKinBody(M,Blist,thetalist(i,:)');
%     [V_b(:,i),V_error(:,i)] = FeedbackControl(T_se_current,T_sed{i},T_sedn{i},kp,ki,dt);
%     Jb = JacobianBody(Blist,thetalist(i,:));
%     psuedoJb = pinv(Jb,1e-2);
%     thetalist_dot = psuedoJb*V_b(:,i);
%     thetalist(i+1,:) = NextState(thetalist(i,:),thetalist_dot',dt,10);
% %     for i3 = 1:6
% %         if thetalist(i+1,i3) > 2*pi || thetalist(i+1,i3) < -2*pi
% %             thetalist(i+1,i3) = wrapToPi(thetalist(i+1,i3));
% %         end
% %     end
% %     V_error(:,i) = se3ToVec(round(MatrixLog6(TransInv(T_se_current{i})*T_sed{i}),2));
% 
% end
    


% %%
% figure; hold on;
% plot(x_e(1,:));
% plot(x_e(2,:));
% plot(x_e(3,:));
% plot(x_e(4,:));
% plot(x_e(5,:));
% plot(x_e(6,:));
% legend('wx','wy','wz','vx','vy','vz');
% % figure; hold on 
% % for a = 1:length(V_error)
% %     ang(a) = norm(V_error([1:3],a));
% %     lin(a) = norm(V_error(4:6,a));
% % end
% % plot(0:0.01:(length(ang)*dt)-0.01,ang);
% %  
% output = zeros(length(thetalist),7);
% for ii = 1:length(thetalist)
%     output(ii,:) = [thetalist(ii,:),gripper_state(ii)];
% end
% csvwrite('angle.csv',output);
% 
% % output = zeros(length(traj),13);
% % for i = 1:length(traj)
% %     output(i,:) = [traj{i}(1,1:3),traj{i}(2,1:3),traj{i}(3,1:3),traj{i}(1:3,end)',gripper_state(i)];
% % end
% % csvwrite('m.csv', output);