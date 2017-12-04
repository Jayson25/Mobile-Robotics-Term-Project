
% Problem 1 - Round shaped differential drive robot navigation
% Your control function

function [u, userStructure] = userCtrl(model, environment, userStructure)
    
    %checkpoint position x and y 
    objx = userStructure.checkpoint(userStructure.ci, 1);
    objy = userStructure.checkpoint(userStructure.ci, 2);
    
    %distance betweent the x and y
    dx = objx-model.state(1);
    dy = objy-model.state(2);
    
    dist = sqrt(dx^2 + dy^2);
    theta = atan2(dy,dx);
    
    timeStep = 0.01;
    
    userStructure.errorAngle(size(userStructure.errorAngle)+1) = theta - model.state(3);
    userStructure.errorDist(size(userStructure.errorDist)+1) = dist;
    
    eA = userStructure.errorAngle;
    eD = userStructure.errorDist;
    sErrA = size(userStructure.errorAngle);
    sErrD = size(userStructure.errorDist);
    
    userStructure.integDist = userStructure.integDist + eD(sErrD(2))*0.01;
    userStructure.integAngle = userStructure.integAngle + eA(sErrA(2))*0.01;
    
    Kp = 0.45;
    Ki = 0;
    Kd = 1;
    
    output = pid(eD,Kp,Ki,Kd,timeStep,userStructure.integDist);
    
    if eA(sErrA(2)) > 0 && userStructure.started == 0
        output = 0;
    else
        userStructure.started = 1;
    end
    
    u = [output;output];
    
    Kp = 1.75;
    
    output = pid(eA,Kp,Ki,Kd,timeStep,userStructure.integAngle);
    
    u = [u(1)-output;u(2)+output];
    
    sci = size(userStructure.checkpoint);
    
    if eD(sErrD(2)) < 0.55 && userStructure.ci < sci(1)
       
       userStructure.integDist = 0; 
       userStructure.integAngle = 0;
       
       userStructure.errorDist = 0;
       userStructure.errorAngle = 0;
       
       if userStructure.ci < sci(1)
            userStructure.ci = userStructure.ci+1;
       end
    end
end

function output = pid (error, Kp, Ki, Kd, timeStep, integrates)
    sErr = size(error);
    output = Kp*error(sErr(2)) + Ki*integrates + Kd*(error(sErr(2))-error(sErr(2)-1))/timeStep;
end