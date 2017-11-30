
% Problem 1 - Round shaped differential drive robot navigation
% Detailed robot model is at http://planning.cs.uiuc.edu/node659.html

% Main function

% Load model
clearvars -except modelRef environmentRef
model = modelRef;
environment = environmentRef;

%%%%%%%%%%%%%%%%%%%% Your Job 1 %%%%%%%%%%%%%%%%%%%%%%
% User initialization function
tic;
userStructure = userInit(model, environment);
toc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic;
sceneCnt = 0;
while(1)
    environment.time = environment.time + 0.01;
    
    %%%%%%%%%%%%%%%% Your Job 2 %%%%%%%%%%%%%%%%%%%%%%
    % User controller function
    [u, userStructure] = userCtrl(model, environment, userStructure);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Model update with input vector
    model = updateVehicle(model, u);
    
    % Check whether collision takes place
    % Finish loop if collision happens
    if(checkCollision(model, environment) == 1)
        'Collided'
        plotAll(model, environment);
        break;
    end
    
    % Check whether goal is achieved
    % Finish loop if goal is achieved
    if(checkGoal(model, environment) == 1)
        'Goal'
        break;
    end
    
    % Plot current situation
    % only plot 1/10 of updates, to speed up simulation
    % Change 10 to other value to speed up simulation (bigger is faster)
    if( mod(sceneCnt,10) == 0 ) 
        plotAll(model, environment, userStructure);
        sceneCnt = 0;
    end
    sceneCnt = sceneCnt + 1;
    
    
end
toc;

plotAll(model, environment);


