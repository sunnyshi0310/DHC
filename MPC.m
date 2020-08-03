% Main routine for DHC simulation in linear Model Predictive Controller
% Maintainer: Lu Shi, lshi024@ucr.edu
% Last Updated: 01/29/2020

% This project is built to evaluate the performance of MPC. In this 
% simulation, nominal parameter values match those of the physical 
% Crazyflie robot.

clc
clear
M = 0.03;
Ixx = 1.43*10^(-5);
g = 9.8;
ts = 0.01;
t = 0:ts:1;
x(:,1) = [0;0;0;0;0;0];
N = length(t);
A1 = [zeros(3),eye(3);[0 0 -g;0 0 0;0 0 0],zeros(3)];
B1 = [zeros(2);zeros(2);1/M,0;0 1/Ixx];
C = eye(6);
tic
sys = ss(A1,B1,C,0,ts);
MPCobj = mpc(sys);
MPCobj.OV(3).Min = -1.57;
MPCobj.OV(3).Max = 1.57;
MPCobj.MV(1).Min = -0.3;
MPCobj.MV(1).Max = 0.3;
MPCobj.MV(2).Min = -0.0001;
MPCobj.MV(2).Max = 0.0001;
xm = mpcstate(MPCobj);
r_p=30;
uncer_m = normrnd(0,0.6*0.03);
uncer_i = normrnd(0,0.6*1.43*10^(-5));

for k = 1:N
    u(:,k) = mpcmove(MPCobj,xm,x(:,k),[3,5,atan2(5,3),0,0,0]); 
    
    % Generate uncertainty
    M = 0.0351;
    Ixx = 1.1643*10^(-5);
    
    % Propogate the system
    A1 = [zeros(3),eye(3);[0 0 -g;0 0 0;0 0 0],zeros(3)];
    B1 = [zeros(2);zeros(2);1/M,0;0 1/Ixx];
    x(:,k+1) = A1*x(:,k)+B1*u(:,k);
end
toc
umax = max(abs(u(1,:)))
usum = sum(abs(u(1,:)))
figure
subplot(2,2,1)
plot(x(1,1:end-1))
xlabel('time')
title('y')
subplot(2,2,2)
plot(x(2,1:end-1))
xlabel('time')
title('z')
subplot(2,2,3)
plot(x(3,1:end-1))
xlabel('time')
title('Pitch Angle \psi')
subplot(2,2,4)
plot(t,u(1,:),t,u(2,:))
xlabel('time')
ylabel('u')
legend('u_1','u_2')
title('input')

