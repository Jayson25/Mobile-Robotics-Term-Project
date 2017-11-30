
% Problem 1 - Round shaped differential drive robot navigation
% Plot function

function plotAll(model, env, userStructure)

    figure(1);
    plot(-9999,-9999); % dummy plot for holding on other things
    hold on;
    
    % draw current robot state
    drawRobot(model);
    
    % draw goal robot state
    modelGoal = model;
    modelGoal.state = env.stateGoal;
    drawRobotGoal(modelGoal);
    
    % draw environment
    drawEnvironment(env);
    
    hold off;
    axis equal;
    axis(env.plotArea);
    str = sprintf('Time = %f, x = %f, y = %f, theta = %f\nleft angular vel. = %f, right angular vel. = %f',env.time, model.state(1), model.state(2), model.state(3), model.state(4), model.state(5));
    title(str);
    drawnow;
    
end

function drawRobot(model)

    % rotation matrix
    rotM = [cos(model.state(3)), -sin(model.state(3)); sin(model.state(3)), cos(model.state(3))];
    
    % draw pivot
    circle(model.state(1), model.state(2), model.radius);
    hold on;
    
    % calculate navigation frame corner position
    arrow = [[-model.radius;0],[model.radius;0],[model.radius/2;model.radius/2],[model.radius;0],[model.radius/2;-model.radius/2]];
    arrowNav = rotM * arrow;
    arrowNav = arrowNav + repmat(model.state(1:2)', [1, 5]);
    for i = 1:4
        plot(arrowNav(1,i:i+1), arrowNav(2,i:i+1),'k-');
    end
    
    % draw tires
    cornerWheelNav = [   [-model.wheelRadius ; -model.wheelThickness/2-model.track/2 ; model.wheelRadius ; -model.wheelThickness/2-model.track/2] , ...
                            [model.wheelRadius ; model.wheelThickness/2-model.track/2 ; model.wheelRadius ; -model.wheelThickness/2-model.track/2] , ...
                            [-model.wheelRadius ; model.wheelThickness/2-model.track/2 ; model.wheelRadius ; model.wheelThickness/2-model.track/2] , ...
                            [-model.wheelRadius ; model.wheelThickness/2-model.track/2 ; -model.wheelRadius ; -model.wheelThickness/2-model.track/2] , ...
                            [-model.wheelRadius ; -model.wheelThickness/2+model.track/2 ; model.wheelRadius ; -model.wheelThickness/2+model.track/2] , ...
                            [model.wheelRadius ; model.wheelThickness/2+model.track/2 ; model.wheelRadius ; -model.wheelThickness/2+model.track/2] , ...
                            [-model.wheelRadius ; model.wheelThickness/2+model.track/2 ; model.wheelRadius ; model.wheelThickness/2+model.track/2] , ...
                            [-model.wheelRadius ; model.wheelThickness/2+model.track/2 ; -model.wheelRadius ; -model.wheelThickness/2+model.track/2]...
                            ];
    cornerWheelNav(1:2,:) = (rotM * cornerWheelNav(1:2,:) + repmat(model.state(1:2)', [1, 8]));
    cornerWheelNav(3:4,:) = (rotM * cornerWheelNav(3:4,:) + repmat(model.state(1:2)', [1, 8]));
    for i = 1:8
        plot([cornerWheelNav(1,i),cornerWheelNav(3,i)], [cornerWheelNav(2,i),cornerWheelNav(4,i)],'g-');
    end
end

function drawRobotGoal(model)

    % rotation matrix
    rotM = [cos(model.state(3)), -sin(model.state(3)); sin(model.state(3)), cos(model.state(3))];
    
    % draw pivot
    circleDashed(model.state(1), model.state(2), model.radius);
    hold on;
    
    % calculate navigation frame corner position
    arrow = [[-model.radius;0],[model.radius;0],[model.radius/2;model.radius/2],[model.radius;0],[model.radius/2;-model.radius/2]];
    arrowNav = rotM * arrow;
    arrowNav = arrowNav + repmat(model.state(1:2)', [1, 5]);
    for i = 1:4
        plot(arrowNav(1,i:i+1), arrowNav(2,i:i+1),'k:');
    end
    
    % draw tires
    cornerWheelNav = [   [-model.wheelRadius ; -model.wheelThickness/2-model.track/2 ; model.wheelRadius ; -model.wheelThickness/2-model.track/2] , ...
                            [model.wheelRadius ; model.wheelThickness/2-model.track/2 ; model.wheelRadius ; -model.wheelThickness/2-model.track/2] , ...
                            [-model.wheelRadius ; model.wheelThickness/2-model.track/2 ; model.wheelRadius ; model.wheelThickness/2-model.track/2] , ...
                            [-model.wheelRadius ; model.wheelThickness/2-model.track/2 ; -model.wheelRadius ; -model.wheelThickness/2-model.track/2] , ...
                            [-model.wheelRadius ; -model.wheelThickness/2+model.track/2 ; model.wheelRadius ; -model.wheelThickness/2+model.track/2] , ...
                            [model.wheelRadius ; model.wheelThickness/2+model.track/2 ; model.wheelRadius ; -model.wheelThickness/2+model.track/2] , ...
                            [-model.wheelRadius ; model.wheelThickness/2+model.track/2 ; model.wheelRadius ; model.wheelThickness/2+model.track/2] , ...
                            [-model.wheelRadius ; model.wheelThickness/2+model.track/2 ; -model.wheelRadius ; -model.wheelThickness/2+model.track/2]...
                            ];
    cornerWheelNav(1:2,:) = (rotM * cornerWheelNav(1:2,:) + repmat(model.state(1:2)', [1, 8]));
    cornerWheelNav(3:4,:) = (rotM * cornerWheelNav(3:4,:) + repmat(model.state(1:2)', [1, 8]));
    for i = 1:8
        plot([cornerWheelNav(1,i),cornerWheelNav(3,i)], [cornerWheelNav(2,i),cornerWheelNav(4,i)],'g:');
    end
end

function circle(x,y,r)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
plot(x+xp,y+yp);
end

function circleDashed(x,y,r)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
plot(x+xp,y+yp,':');
end

function drawEnvironment(env)
    envSize = size(env.corner);
    for i = 1:envSize(2)
        plot([env.corner(1,i),env.corner(3,i)], [env.corner(2,i),env.corner(4,i)],'k-');
    end
    
    return;
end
