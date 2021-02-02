close all
clear all
clc

%% Problem with Global Variables
global t1 t2 t3 zr n 
t1 = -10:0.5:10;
t2 = 0:0.5:20;
t3 = 0:0.1:4;
[t1,t2,t3] = meshgrid(t1, t2, t3);
zr = 5.9*t1.^2 + 1.3*t2.^2 - 22.00044*t1.*t2.*t3 - 81.5567*t3.^3;
n = size(zr, 1)*size(zr, 2)*size(zr,3);

%% Problem Definitoin

probl.CostFunction = @(x, G, V) CostFun(x, G, V); % Cost Function
probl.nVar = 4; % Number of unknown (decision) variables
probl.varSize = [1 probl.nVar]; % Matrix size of decision variables
probl.varMin = [-10 -20 -100 -90]; % Lower bound of decision variables
probl.varMax = [30 20 10 30]; % Upper bound of decision variables

%% Parameters of PSO

% Parameters for the pso algorithm (Clerc and Kennedy, 2002)
kappa = 1;
phi1 = 2.05;
phi2 = 2.05;
phi = phi1 + phi2;
chi = 2*kappa/abs(2 - phi - sqrt(phi*phi - 4*phi));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param.maxIt = 100; % Maximum number of iterations
param.nPop = 50; % Population size or swarm size
param.w = chi; % Intertia coefficient
param.wdamp = 1; % Damping ratio of Inertia weight
param.c1 = chi*phi1; % Personal acceleration coefficient
param.c2 = chi*phi2; % Social acceleration coefficient
param.displ = 1; % This is the flag to display the information


%% PSO main loop

out = pso_opt(probl, param);


%% Results 

plot(out.bestCosts, 'r')
title('Iterations over time')
xlabel('Iterations')
ylabel('Cost Function')

    