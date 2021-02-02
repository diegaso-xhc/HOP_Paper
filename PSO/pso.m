close all
clear all
clc

%% Problem Definitoin

CostFunction = @(x) CostFun(x); % Cost Function
nVar = 1; % Number of unknown (decision) variables
varSize = [1 nVar]; % Matrix size of decision variables
varMin = -20; % Lower bound of decision variables
varMax = 20; % Upper bound of decision variables

%% Parameters of PSO

maxIt = 100; % Maximum number of iterations
nPop = 50; % Population size or swarm size
w = 1; % Intertia coefficient
wdamp = 0.99; % Damping ratio of Inertia weight
c1 = 2; % Personal acceleration coefficient
c2 = 2; % Social acceleration coefficient
maxVelocity = 0.2*(varMax - varMin); % Maximum velocity for the PSO
minVelocity = -maxVelocity; % Minimum velocity for the PSO

%% Initialization

empty_particle.Position = []; % Particle template
empty_particle.Velocity = []; % Particle template
empty_particle.Cost = []; % Particle template
empty_particle.Best.Position = []; % Personal best position for this particle
empty_particle.Best.Cost = []; % Personal best velocity for this particle

particle = repmat(empty_particle, nPop, 1); % we repeat the structure of the empty particle depending on the number of members on the swarm
globalBest.Cost = inf; % The global best is not defined at first and it has the worst possible value (infinity in a minimization problem)

for i = 1: nPop
   particle(i, 1).Position = unifrnd(varMin, varMax, varSize); % Generate random solutions for our problem
   particle(i, 1).Velocity = zeros(varSize); % Initialization of the velocities with 0s   
   particle(i, 1).Cost = CostFunction(particle(i, 1).Position); % Evaluation of the particle
   particle(i, 1).Best.Position = particle(i, 1).Position;
   particle(i, 1).Best.Cost = particle(i, 1).Cost;   
   % Update the global best
   if particle(i, 1).Best.Cost < globalBest.Cost
       globalBest = particle(i, 1).Best;
   end   
end
bestCosts = zeros(maxIt, 1); % This is an a rray to contain the best values on each iteration

%% Main Loop PSO

for i = 1: maxIt
    for j = 1: nPop
       % Update velocity
       particle(j, 1).Velocity = w*particle(j, 1).Velocity +...
           c1*rand(varSize).*(particle(j, 1).Best.Position - particle(j, 1).Position)+...
           c2*rand(varSize).*(globalBest.Position - particle(j, 1).Position); 
       % Update Position
       particle(j, 1).Position = particle(j, 1).Position + particle(j, 1).Velocity;
       % Saturate lower, upper bound limit of positions and velocities
       particle(j, 1).Position = max(particle(j, 1).Position, varMin);
       particle(j, 1).Position = min(particle(j, 1).Position, varMax);
       particle(j, 1).Velocity = max(particle(j, 1).Velocity, minVelocity);
       particle(j, 1).Velocity = min(particle(j, 1).Velocity, maxVelocity);
       % Evaluate position
       particle(j, 1).Cost = CostFunction(particle(j, 1).Position);
       % Update personal best
       if particle(j, 1).Cost < particle(j, 1).Best.Cost
           particle(j, 1).Best.Position = particle(j, 1).Position;
           particle(j, 1).Best.Cost = particle(j, 1).Cost;
       end
       % Update global best
       if particle(j, 1).Best.Cost < globalBest.Cost
          globalBest = particle(j, 1).Best;          
       end      
    end
    w = w*wdamp;
    bestCosts(i) = globalBest.Cost;    
    disp(['Iter: ' num2str(i) ' Best Cost: ' num2str(globalBest.Cost)])
end

%% Results 

plot(bestCosts, 'r')
title('Iterations over time')
xlabel('Iterations')
ylabel('Cost Function')

    