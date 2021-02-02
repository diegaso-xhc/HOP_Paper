close all
clear all
clc

%% Problem Definitoin

probl.CostFunction = @(x, G, V) cost(x, G, V); % Cost Function
probl.nVar = 2; % Number of unknown (decision) variables
probl.varSize = [1 probl.nVar]; % Matrix size of decision variables
probl.varMin = [0 0]; % Lower bound of decision variables
probl.varMax = [3 3]; % Upper bound of decision variables
G = [0 0;0 0;14.2160 3.0537;-0.0416 2.0404;-13.6265 4.4659;0 0];
V = [0 0 0.5637 0.4657 0.9258 0]';

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

out = pso_opt(probl, param,G,V);


%% Results 

plot(out.bestCosts, 'r')
title('Iterations over time')
xlabel('Iterations')
ylabel('Cost Function')

    