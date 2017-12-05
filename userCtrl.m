
% Problem 1 - Round shaped differential drive robot navigation
% Your control function

function [u, userStructure] = userCtrl(model, environment, userStructure)
    
    %checkpoint position x and y 
    objx = userStructure.checkpoint(userStructure.ci, 1);
    objy = userStructure.checkpoint(userStructure.ci, 2);
    
    %distance betweent the x and y
    dx = objx-model.state(1);
    dy = objy-model.state(2);
    
    %dist -> eucledian distance between the checkpoint and the current
    %position
    %theta -> returns the four-quadrant inverse tangent (tan-1) of Y and X
    %between the checkpoint and the current position
    dist = sqrt(dx^2 + dy^2);
    theta = atan2(dy,dx);
    
    %Dt - Time
    timeStep = 0.01;
    
    %error from the real result and expected result
    userStructure.errorAngle = [userStructure.errorAngle, theta - model.state(3)];
    userStructure.errorDist = [userStructure.errorDist, dist];
    
    %simplification of the namings from above
    eA = userStructure.errorAngle;
    eD = userStructure.errorDist;
    
    %size of the error buffer
    sErrA = size(userStructure.errorAngle);
    sErrD = size(userStructure.errorDist);
    
    %integrates
    userStructure.integDist = userStructure.integDist + eD(sErrD(2))*0.01;
    userStructure.integAngle = userStructure.integAngle + eA(sErrA(2))*0.01;
    
    %PID constant values
    Kp = 0.75;
    Ki = 0;
    Kd = 1;
    
    %size of the checkpoint list
    sci = size(userStructure.checkpoint);
    
    if userStructure.ci == sci(1)
        Kp = 0.3;
    end
    
    %PID controller for the distance 
    output = pid(eD,Kp,Ki,Kd,timeStep,userStructure.integDist);
    
    %If we didn't start the robot and the robot is not finished at rotating
    %prevents conflicts at the start of the robot
    if eA(sErrA(2)) > 0 && userStructure.started == 0
        output = 0;
    else
        userStructure.started = 1;
    end
    
    %acceleration for moving forward
    u = [output;output];
    
    Kp = 1.75;
    
    %acceleration for turning the robot
    output = pid(eA,Kp,Ki,Kd,timeStep,userStructure.integAngle);
    
    u = [u(1)-output;u(2)+output];

    %if we reach a certain point and it is not the last checkpoint
    if eD(sErrD(2)) < 0.75 && userStructure.ci < sci(1)
       
       %reset integrates and errors
       userStructure.integDist = 0; 
       userStructure.integAngle = 0;
       
       userStructure.errorDist = 0;
       userStructure.errorAngle = 0;
       
       if userStructure.ci < sci(1)
            userStructure.ci = userStructure.ci+1;
       end
    end
end

%PID controller
function output = pid (error, Kp, Ki, Kd, timeStep, integrates)
    sErr = size(error);
    output = Kp*error(sErr(2)) + Ki*integrates + Kd*(error(sErr(2))-error(sErr(2)-1))/timeStep;
end