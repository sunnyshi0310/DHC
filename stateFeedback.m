%% Initialization
% State [x x_dot phi phi_dot]
clear
clc
tic
% The method is tested in an inverted pendulum
% Initialize the system
M = 0.03;
Ixx = 1.43*10^(-5);
g = 9.8;
% Set initial condition
ts = 0.01;
t = 0:ts:3;
x(:,1) = [-3;-5;0;0;0;0];
x_hat(:,1) = [0;0;0;0;0;0];
N = length(t);
u_a = [0.03 0.01 -0.01 0.05 0.01 -0.05 0 0.01 -0.01;0 0.00001 -0.00001 0.000005 0.00001 -0.000005 0 0.00001 -0.00001];

% Calculate the controller
pole = [0.9;0.8;-0.9;-0.8;0.95;-0.95];
A1 = [zeros(3),eye(3);[0 0 -g;0 0 0;0 0 0],zeros(3)];
B1 = [zeros(2);zeros(2);1/M,0;0 1/Ixx];
K_p = place(A1,B1,pole);

for k = 1:N
    
    % Design corresponding controller
    u(:,k) = -K_p*x(:,k);    
    % Propagate the system

    M_quadrotor = 0.0351;
    I_xx = 1.1643*10^(-5);
    A1 = [zeros(3),eye(3);[0 0 -g;0 0 0;0 0 0],zeros(3)];
    B1 = [zeros(2);zeros(2);1/M,0;0 1/Ixx];
    x(:,k+1) = A1*x(:,k)+B1*u(:,k);
    x_hat(:,k+1) = x(:,k+1)+[3;5;0;0;0;0];
end
toc
umax = max(abs(u(1,:)))
usum = sum(abs(u(1,:)))
figure
subplot(2,2,1)
plot(t,x_hat(1,1:end-1))
xlabel('time')
ylabel('y')
title('y')
subplot(2,2,2)
plot(t,x_hat(2,1:end-1))
xlabel('time')
ylabel('z')
title('z')
subplot(2,2,3)
plot(t,x_hat(3,1:end-1))
xlabel('time')
ylabel('\psi')
title('Pitch Angle \psi')
subplot(2,2,4)
plot(t,u(1,:),t,u(2,:))
xlabel('time')
ylabel('u')
legend('u_1','u_2')
title('input')
