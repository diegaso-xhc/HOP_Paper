function out = pso_opt(probl, param,G,V)

%% Problem Definitoin

    CostFunction = probl.CostFunction; % Cost Function
    nVar = probl.nVar; % Number of unknown (decision) variables
    varSize = [1 nVar]; % Matrix size of decision variables
    varMin = probl.varMin; % Lower bound of decision variables
    varMax = probl.varMax; % Upper bound of decision variables

    %% Parameters of PSO

    maxIt = param.maxIt; % Maximum number of iterations
    nPop = param.nPop; % Population size or swarm size
    w = param.w; % Intertia coefficient
    wdamp = param.wdamp; % Damping ratio of Inertia weight
    c1 = param.c1; % Personal acceleration coefficient
    c2 = param.c2; % Social acceleration coefficient
    displ = param.displ; % This is the flag to display the information
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
       if nVar == 1
           particle(i, 1).Position = unifrnd(varMin, varMax, varSize); % Generate random solutions for our problem
       else
           for k = 1: nVar
               particle(i, 1).Position(1, k) = unifrnd(varMin(k), varMax(k)); % Generate random solutions for our problem
           end
       end       
       particle(i, 1).Velocity = zeros(varSize); % Initialization of the velocities with 0s   
       particle(i, 1).Cost = CostFunction(particle(i, 1).Position, G, V); % Evaluation of the particle
       particle(i, 1).Best.Position = particle(i, 1).Position;
       particle(i, 1).Best.Cost = particle(i, 1).Cost;   
       % Update the global best
       if particle(i, 1).Best.Cost < globalBest.Cost
           globalBest = particle(i, 1).Best;
       end   
    end
    out.bestCosts = zeros(maxIt, 1); % This is an a rray to contain the best values on each iteration

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
           particle(j, 1).Cost = CostFunction(particle(j, 1).Position, G, V);
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
        out.bestCosts(i) = globalBest.Cost; 
        out.bestSolution = globalBest;
        out.pop = particle;
        if displ == 1
            disp(['Iter: ' num2str(i) ' Best Cost: ' num2str(globalBest.Cost)])
        end
        
    end    

end
