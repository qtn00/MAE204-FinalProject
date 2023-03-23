function [V_b,x_e,thetalist] = FeedbackControl(thetalistin,T_sed,T_sedn,kp,ki,dt)
 
format long g;

%% Initial 

Kp = kp*eye(6,6);
Ki = ki*eye(6,6);

thetalist(1,:) = thetalistin;
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
for ii = 1:length(Slist)
    Blist(:,ii) = Adjoint(TransInv(M))*Slist(:,ii);
end

x_e = zeros(6,length(T_sed));
V_b = zeros(6,length(T_sed));


for i = 1:length(T_sed)-1
    T_se_current{i} = FKinBody(M,Blist,thetalist(i,:)');
    V_d = se3ToVec((1/dt)*MatrixLog6(TransInv(T_sed{i})*T_sedn{i}));
    feedfor_V_d = Adjoint(TransInv(T_se_current{i})*T_sed{i})*V_d;
    x_e(:,i) = se3ToVec(MatrixLog6(TransInv(T_se_current{i})*T_sed{i}));
    V_b(:,i) = feedfor_V_d + Kp*x_e(:,i) + Ki*(sum(x_e*dt,2));
    Jb = JacobianBody(Blist,thetalist(i,:));
    psuedoJb = pinv(Jb,1e-2);
    thetalist_dot = psuedoJb*V_b(:,i);
    thetalist(i+1,:) = NextState(thetalist(i,:),thetalist_dot',dt,10);
    for i3 = 1:6
        if thetalist(i,i3) > pi || thetalist(i,i3) < -pi
            thetalist(i,i3) = wrapToPi(thetalist(i,i3));
        end
    end
  

end