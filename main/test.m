clc; clear all;
thetalist{1} = [-pi/6,-pi/2,pi/2,-pi/2,-pi/2,5*pi/6];
vel = [0.1,0.1,0.1,0.1,0.1,0.1];
maxvel = pi;
dt = 0.01;
Tf = 1;
N = Tf/dt;

gripper_state = ones(1,N);
gripper_state(30:50) = 0;
output = zeros(N,7);
for i = 1:N
    thetalist{i+1} = NextState(thetalist{i},vel,dt,maxvel);
    output(i,:) = [thetalist{i},gripper_state(i)];
end

csvwrite('next.csv',output)