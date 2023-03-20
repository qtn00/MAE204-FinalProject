clc; clear all; close all;
format long g;



thetalist = [-pi/6,-pi/2,pi/2,-pi/2,-pi/2,5*pi/6];
T_sed = [0 0 1 300;-1 0 0 -300;0 -1 0 237;0 0 0 1];
T_sedn = [0 0 1 290;-1 0 0 -290;0 -1 0 237;0 0 0 1];
kp = 0; ki = 0;
dt = 0.01;
Kp = kp*eye(6,6);
Ki = ki*eye(6,6);


%% FeedbackControl
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
V_error = zeros(6,length(T_sed)-1);
V_b =zeros(6,length(T_sed)-1);
thetalist_dot = zeros(6,length(T_sed)-1);
for ii = 1:1
    T_se_current = round(FKinBody(M,Blist,thetalist(ii,:)'),1);
    V_d = se3ToVec((1/dt)*MatrixLog6(TransInv(T_sed)*T_sedn));
    V_error(:,ii) = se3ToVec(round(MatrixLog6(TransInv(T_se_current)*T_sed),2));
    V_error_sum = sum(V_error,2);
    feedfor_V_d = Adjoint(TransInv(T_se_current)*T_sed)*V_d;
    V_b(:,ii) = feedfor_V_d + Kp*V_error(:,ii) + Ki*(V_error_sum*dt);
    Jb = round(JacobianBody(Blist,thetalist(ii,:)),4);
    psuedoJb = round(pinv(Jb),4);
    thetalist_dot(:,ii) = round(psuedoJb*V_b(:,ii),4);
    thetalist(ii+1,:) = NextState(thetalist(ii,:),thetalist_dot(:,ii)',dt,10);
    for i3 = 1:6
        if thetalist(ii+1,i3) > pi || thetalist(ii+1,i3) < -pi
            thetalist(ii+1,i3) = wrapToPi(thetalist(ii+1,i3));
        end
    end
end