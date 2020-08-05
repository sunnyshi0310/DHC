% Main routine for MiC simulation
% Maintainer: Lu Shi, lshi024@ucr.edu
% Last Updated: 01/28/2020

% This project is built to evaluate the performance of the hierarchical Model-identified
% Control structure. In this simulation, nominal parameter values
% match those of the physical Crazyflie robot.

% Note that the method is suitable for the system with uncertainty,
% otherwise the singularity happens. 

clc
clear


%% Initialization
M_quadrotor = 0.03; % Mass(kg) of single Crazyflie
I_xx = 1.43*10^(-5); % Inertia() along x axis of single Crazyflie
g = 9.8; % Gravity ()
ts = 0.01; % Sampling time (s)
t = 0:ts:0.4; % Simulation time (s)
N = length(t); 
x(:,1) = [-3; -5; 0; 0; 0; 0]; % Initial condition

A_nominal = [zeros(3),eye(3);[0 0 -g;0 0 0;0 0 0],zeros(3)];
B_nominal = [zeros(2);zeros(2);1/M_quadrotor,0;0 1/I_xx];
Nx = size(B_nominal,1); % Dimension of states
Nu = size(B_nominal,2); % Dimension of inputs
C = eye(Nx);
Nm = 2; % y,z
predictionHorizon = 10;

% Set the low-level linear MPC controller 
sys = ss(A_nominal,B_nominal,C,0,ts);
MPCobj = mpc(sys);
MPCobj.OV(3).Min = -1.57;
MPCobj.OV(3).Max = 1.57;
MPCobj.MV(1).Min = -1;
MPCobj.MV(1).Max = 1;
MPCobj.MV(2).Min = -0.0001;
MPCobj.MV(2).Max = 0.0001;
MPCobj.p = predictionHorizon;
xm = mpcstate(MPCobj);

for i = 1:N+Nm+2
    reference(:,i) = [0;0];
end
ref_hat(:,1) = reference(:,1);

% % Generate the uncertainty
% massDisturbance = normrnd(0,0.2*0.03);
% inertialDisturbance = normrnd(0,0.2*1.43*10^(-5));


%% Simulate the system
tic
for k = 1:N
    % Calculate control input based on refined reference
    referenceVector = [ref_hat(:,k);atan2(ref_hat(2,k),ref_hat(1,k));0;0;0];
    u(:,k) = mpcmove(MPCobj,xm,x(:,k),referenceVector); 
       
    % Generate the uncertainty
    measurementNoise = [normrnd(0,0.1,[2,1]);zeros(4,1)];

    % Propogate the system
%     M_quadrotor = M_quadrotor+massDisturbance;
%     I_xx = I_xx+inertialDisturbance;
    M_quadrotor = 0.0194;
    I_xx = 1.19436*10^(-5);
    A_true = [zeros(3),eye(3);[0 0 -g;0 0 0;0 0 0],zeros(3)];
    B_true = [zeros(2);zeros(2);1/M_quadrotor,0;0 1/I_xx];
    x(:,k+1) = A_true*x(:,k)+B_true*u(:,k);%+measurementNoise;
    
    % Update model
    if k>Nm+size(reference,1)       
        rankMeasurement = rank(ref_hat(:,k-(Nm+size(reference,1)):k))+rank(x(1:2,k-(Nm+size(reference,1)):k-1));
        if rankMeasurement == size(reference,1)+Nm
            [A,B] = mic(ref_hat(:,k-(Nm+size(reference,1)):k),x(1:2,k-(Nm+size(reference,1)):k-1),size(reference,1),Nm);
            ref_hat(:,k+1) = A*ref_hat(:,k)+B*reference(:,k+1);
        else
            ref_hat(:,k+1) = reference(:,k+1);
        end
            
    else % Initialization
        ref_hat(:,k+1) = reference(:,k+1);%+measurementNoise;
    end 
    
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

