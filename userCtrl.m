
% Problem 1 - Round shaped differential drive robot navigation
% Your control function

function [u, userStructure] = userCtrl(model, environment, userStructure)

    % previously defined value as an example at userInit function
    % used as a global variable, so that this will change as 0, 1, 2, ....
    % at every iteration
    userStructure.exampleVariable = userStructure.exampleVariable + 1;
    
    % control input example
    u = [0.5; -0.5];
    if(environment.time < 5)
        u = [0.2; -0.5];
    elseif(environment.time < 15) 
        u = [0; 0.5];
    else
        u = [-0.3; 0];
    end
end