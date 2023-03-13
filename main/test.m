clc; clear all;
thetalist{1} = [0,0,0,0,0,0];
vel = pi/4;
dt = 0.01;
Tf = 1;
N = Tf/dt;

gripper_state = ones(1,N);
gripper_state(30:50) = 0;
output = zeros(N,7);
for i = 1:N
    thetalist{i+1} = NextState(thetalist{i},vel,dt,2*pi);
    output(i,:) = [thetalist{i},gripper_state(i)];
end

csvwrite('m.csv',output)