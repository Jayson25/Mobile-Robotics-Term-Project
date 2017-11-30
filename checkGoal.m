
% Problem 1 - Round shaped differential drive robot navigation
% Goal check function

function isAtGoal = checkGoal(model, environment)
    
    isAtGoal = 1;
    
    if(model.state(1) > environment.stateGoalToleranceHigh(1) || ...
        model.state(1) < environment.stateGoalToleranceLow(1))
        isAtGoal = 0;
    end
    
    if(model.state(2) > environment.stateGoalToleranceHigh(2) || ...
        model.state(2) < environment.stateGoalToleranceLow(2))
        isAtGoal = 0;
    end
    
    % Angle is 0 ~ 2*pi
    model.state(3) = model.state(3) - (floor(model.state(3)/(2*pi)))*2*pi;
    if(environment.stateGoalToleranceHigh(3) < environment.stateGoalToleranceLow(3))
        if((model.state(3) < environment.stateGoalToleranceHigh(3) && model.state(3) >= 0) ...
            || (model.state(3) > environment.stateGoalToleranceLow(3) && model.state(3) <= 2*pi))
        
        else
            isAtGoal = 0;
        end
    else
        if(model.state(3) > environment.stateGoalToleranceHigh(3) || ...
            model.state(3) < environment.stateGoalToleranceLow(3))
            isAtGoal = 0;
        end
    end
        
    if(model.state(4) > environment.stateGoalToleranceHigh(4) || ...
        model.state(4) < environment.stateGoalToleranceLow(4))
        isAtGoal = 0;
    end
    
    if(model.state(5) > environment.stateGoalToleranceHigh(5) || ...
        model.state(5) < environment.stateGoalToleranceLow(5))
        isAtGoal = 0;
    end
    
end


