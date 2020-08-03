% Main routine for DHC simulation in adaptive controller
% Maintainer: Lu Shi, lshi024@ucr.edu
% Last Updated: 01/29/2020

% This project is built to evaluate the performance of adaptive
% control structure. In this simulation, nominal parameter values
% match those of the physical Crazyflie robot.

clc
clear


%% Initialization 
M_quadrotor = 0.03; % Mass(kg) of single Crazyflie
I_xx = 1.43*10^(-5); % Inertia() along x axis of single Crazyflie
g = 9.8; % Gravity ()
ts = 0.01; % Sampling time (s)
t = 0:ts:0.92; % Simulation time (s)
N = length(t); 
xDimension = 6; % Dimension of states
uDimension = 2; % Dimension of inputs
C = eye(xDimension);

x(:,1) = [0; 0; 0; 0; 0; 0]; % Initial condition

% Generate Reference
yDesireHeight = 3; %(m)
zDesireHeight = 5; %(m)
for t = 1:N+11
        ref(:,t) = [yDesireHeight; zDesireHeight; atan2(zDesireHeight,yDesireHeight); 0; 0; 0];
end


% Create the MPC controller 
A_nominal = [zeros(3),eye(3);[0 0 -g;0 0 0;0 0 0],zeros(3)];
B_nominal = [zeros(2);zeros(2);1/M_quadrotor,0;0 1/I_xx];
sys = ss(A_nominal,B_nominal,C,0,ts);            
MPCobj = mpc(sys);
MPCobj.PredictionHorizon = 10;
MPCobj.ControlHorizon = 2;
MPCobj.OV(3).Min = -pi/2;
MPCobj.OV(3).Max = pi/2;
MPCobj.MV(1).Min = -0.5;
MPCobj.MV(1).Max = 0.5;
MPCobj.MV(2).Min = -10^(-5);
MPCobj.MV(2).Max = 10^(-5);
xm = mpcstate(MPCobj);
%

massUpdate = M_quadrotor;
inertialUpdate = I_xx;


%% Simulate the system
tic
for k = 1:N
    % Design the adaptive controller
    if k>1
        massUpdate = massUpdate - 0.1*ts*(0.99*(x(1,k)-ref(1))+1*(x(2,k)-ref(2)));
        inertialUpdate = inertialUpdate - 0.00005*ts*(2*(x(1,k)-ref(1))+1.99*(x(2,k)-ref(2)));
    end
    A = [zeros(3),eye(3);[0 0 -g;0 0 0;0 0 0],zeros(3)];
    B = [zeros(2);zeros(2);1/massUpdate,0;0 1/inertialUpdate];
    MPCobj.Model.Plant = ss(A,B,C,0,ts);
    u(:,k) = mpcmove(MPCobj,xm,x(:,k),ref(:,k:k+10)');      

     
    % Propogate the system
      % 0.2
%     M_quadrotor = 0.0351;
%     I_xx = 1.1643*10^(-5);
%     % 0.6
    M_quadrotor = 0.0194;
    I_xx = 1.19436*10^(-5);
    % Nominal 
%     M_quadrotor = M_quadrotor+massDisturbance;
%     I_xx = I_xx+inertialDisturbance;
    plantA = [zeros(3),eye(3);[0 0 -g;0 0 0;0 0 0],zeros(3)];
    plantB = [zeros(2);zeros(2);1/M_quadrotor,0;0 1/I_xx];
    x(:,k+1) = plantA*x(:,k)+plantB*u(:,k);
    yref = A_nominal*x(:,k)+B_nominal*u(:,k);
end
toc


%% Results and plots
% Calculate the energy
uMax = max(abs(u(1,:)))
uSum = sum(abs(u(1,:)))

% Plot the propagation of states and inputs
figure(1)
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
plot(u(1,:))
hold on
plot(u(2,:))
xlabel('time')
ylabel('u')
legend('u_1','u_2')
title('input')


