
% Problem 1 - Round shaped differential drive robot navigation
% Vehicle model update function

% Assume round-shaped differential drive robot model
% state(1) = X position in navigation frame [m]
% state(2) = Y position in navigation frame [m]
% state(3) = Heaing represented by in navigation frame [rad]
% state(4) = Left wheel velocity [m/s]
% state(5) = Right wheel velocity [m/s]

% u(1) = Left wheel acceleration [m/s/s]
% u(2) = Right wheel acceleration [m/s/s]

function model = updateVehicle(model, u)
    
    if(u(1) > model.maxWheelAcc) u(1) = model.maxWheelAcc; end
    if(u(1) < model.minWheelAcc) u(1) = model.minWheelAcc; end
    if(u(2) > model.maxWheelAcc) u(2) = model.maxWheelAcc; end
    if(u(2) < model.minWheelAcc) u(2) = model.minWheelAcc; end
    
    % Update model, dt = 10 milliseconds with euler integration
    dt = 0.01;
    model.state(1) = model.state(1) + dt * ((model.state(4)+model.state(5))/2*cos(model.state(3)));
    model.state(2) = model.state(2) + dt * ((model.state(4)+model.state(5))/2*sin(model.state(3)));
    model.state(3) = model.state(3) + dt * ((model.state(5)-model.state(4))/model.track);
    model.state(4) = model.state(4) + dt * u(1);
    model.state(5) = model.state(5) + dt * u(2);

    if(model.state(4) > model.maxWheelVel) model.state(4) = model.maxWheelVel; end
    if(model.state(4) < model.minWheelVel) model.state(4) = model.minWheelVel; end
    if(model.state(5) > model.maxWheelVel) model.state(5) = model.maxWheelVel; end
    if(model.state(5) < model.minWheelVel) model.state(5) = model.minWheelVel; end
    
end