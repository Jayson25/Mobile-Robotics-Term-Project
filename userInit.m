
% Problem 1 - Round shaped differential drive robot navigation
% User initialization function

function userStructure = userInit(model, environment)
    
    % Precision of the map (higher = better precision but slower) [note: only integers]
    precision = 5;

    userStructure.exampleVariable = 0;
    userStructure.exampleRowVector = [1,2,3];
    userStructure.exampleColumnVector = [1;2;3];
    userStructure.exampleMatrix = [1,2;3,4];
    
    % Delimitation of the map
    mapStartx = environment.plotArea(1);
    mapEndx = environment.plotArea(2);
    mapStarty = environment.plotArea(3);
    mapEndy = environment.plotArea(4);
    
    % Size of the array that will contain the map
    sizeMapx = (mapEndx-mapStartx)*precision;
    sizeMapy = (mapEndy-mapStarty)*precision;
    
    % Coordinates of the start point and goal point
    userStructure.startPoint = model.state(1, 1:2)';
    goal = environment.stateGoal(1, 1:2)';
    
    % initial angle of robot
    userStructure.startAngle = model.state(3);
    
    % Converts the above coordinates into map friendly values
    startx = (userStructure.startPoint(1) - mapStartx) * precision;
    starty = (userStructure.startPoint(2) - mapStarty)*precision;
    goalx = (goal(1) - mapStartx) * precision;
    goaly = (goal(2) - mapStarty)*precision;
    
    % Coordinates of the beginning of the map (for display purposes)
    adderx = mapStartx;
    addery = mapStarty;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                                %
    %             MAPPING            %
    %                                %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %generates a map with the obstacles and takes into account the robot's
    %size (radius)
    
    userStructure.cost = zeros(sizeMapx, sizeMapy, 3);
    userStructure.cost(:,:,:) = -1;
    userStructure.cost(startx, starty, 3) = 0;
    
    %Fill the map with 1 as empty and Inf as obstacle
    for i = 1:sizeMapx
        for j = 1:sizeMapy

            coord = [mapStartx+i/precision,mapStarty+j/precision]';
            
            if (checkObstacle(coord, model, environment) == 1)
                userStructure.map(i,j) = Inf;
            else 
                userStructure.map(i,j) = 1;
            end
       end
    end
    
    %Make sure that the goal is not an obstacle
    userStructure.map(goalx,goaly) = 1;
    
    %Parents and costs 
    parentList = pathFinding(startx, starty, goalx, goaly,sizeMapx, sizeMapy, userStructure.map);
    
    disp("Total cost to goal: " + parentList(goalx,goaly,3));
    
%     index = 1;
% 
%     for i = 1:sizeMapx
%        addery = mapStartx;
%        for j = 1:sizeMapy
% 
%            if (parentList(i,j,3) ~= Inf)
% 
%                userStructure.x(index) = adderx;
%                userStructure.y(index) = addery;
%                index = index + 1;
%            end
% 
%            addery = addery + 1/precision;
%        end
%        adderx = adderx + 1/precision;
%    end 
   
   userStructure.x(1) = parentList(goalx,goaly,1); 
   userStructure.y(1) = parentList(goalx,goaly,2);
   
   index = 1;
   
   while userStructure.x(index) ~= startx || userStructure.y(index) ~= starty
       userStructure.x(index+1) = parentList(userStructure.x(index),userStructure.y(index),1); 
       userStructure.y(index+1) = parentList(userStructure.x(index), userStructure.y(index),2);
       
       index = index + 1;
   end
   
   sizeArray = size(userStructure.y);
   userStructure.minWeight = 0;
    
   for i = 1:sizeArray(2)
    userStructure.x(i) = mapStartx + userStructure.x(i)/precision;
    userStructure.y(i) = mapStarty + userStructure.y(i)/precision;
   end
   
   userStructure.x = flip(userStructure.x);
   userStructure.y = flip(userStructure.y);
   
   userStructure.checkpoint_x = userStructure.x;
   userStructure.checkpoint_y = userStructure.y;
   
end

%Actual path finding code where a 3d matrix is generated
%dimension 1: rows
%dimension 2: columns
%dimension 3: property of a point -> [xparent, yparent, cost to get to that point]
function parentList = pathFinding(sx,sy,gx,gy,sizex,sizey,map)
    
    %initialize the parent queue line with the starting point of the robot
    parentx(1) = sx;
    parenty(1) = sy;
    
    %generate a 3d matrix described above
    parentList = zeros(sizex, sizey, 3);
    parentList(:,:,:) = Inf;
    parentList(sx, sy, 3) = 0; 

    index = 1;
    
    %actual computation
    while parentList(gx,gy,3) == Inf
        for n = -1:1
            childx = parentx(index)+n;
            for m = -1:1
                childy = parenty(index)+m;
                
                %if the child is not out of range and it is not an obstacle
                if childx > 0 && childy > 0 && childx <= sizex && childy <= sizey && map(childx,childy) ~= Inf
                    heritage = checkShortestPath(childx,childy,parentx(index),parenty(index),parentList);

                    parentList(childx,childy,1) = heritage(1);
                    parentList(childx,childy,2) = heritage(2);
                    parentList(childx,childy,3) = heritage(3);
                    
                    %check if the child is already in the parent queue line
                    isPresent = checkItem(parentx, parenty, childx, childy);
                    
                    if ~isPresent
                        sizeArray = size(parentx);
                        parentx(sizeArray(2)+1) = childx;
                        parenty(sizeArray(2)+1) = childy;
                    end
                end
            end
        end
        %increment the index of the parent queue line
        index = index + 1;
    end
end 

%check whether the child is present in the queue line and returns a boolean
function bool = checkItem(MA, MB, x, y)

    sizeArray = size(MA);
    bool = 0;
    for i = 1:sizeArray(2)
        if MA(i) == x && MB(i) == y
            bool = 1;
            return;
        end
    end
end

%compare the costs between the potential parent and the current parent (if
%one) then assign the new parent if lower cost
function heritage = checkShortestPath(cx,cy,px,py, parentList)
    
    %if diagonal
    if cx~=px && cy~=py
        cost = parentList(px,py,3) + 1.5;
    else
        cost = parentList(px,py,3) + 1;
    end
    
    if cost < parentList(cx,cy,3)
       heritage = [px,py,cost];
    else
        heritage = [parentList(cx,cy,1),parentList(cx,cy,2),parentList(cx,cy,3)];
    end
end

function isCollided = checkObstacle(coord, model, environment)
    
    isCollided = 0;

    [~, NbOfObstacleLines] = size(environment.corner);
    for j = 1:NbOfObstacleLines
        
        currObstacleLine = environment.corner(:, j);
        
       % currObstacleLine
        
        firstPointOfObstacleLine = currObstacleLine(1:2, 1);
        secondPointOfObstacleLine = currObstacleLine(3:4, 1);
        
        % 1) Find the distance between Obstacle Line segment and coord point of the graph Position
        coord_ObstacleLine_dist = point_line_segment_dist(firstPointOfObstacleLine, secondPointOfObstacleLine, coord);
        
        % 2) Determine wheter the distance is shorter than Vehicle Radious
        if(coord_ObstacleLine_dist <= model.radius)
            isCollided = 1;
            return;
        else
            isCollided = 0;
        end

    end

end

function [ dist ] = point_line_segment_dist( linePoint_1, linePoint_2, point )
  %POINT_LINE_SEGMENT_DIST
  % Return minimum distance between line segment linePoint_1 - linePoint_2 and point
  linePointsBetweenDistanceSquared = point_dist(linePoint_1, linePoint_2)*point_dist(linePoint_1, linePoint_2);  % i.e. |w-v|^2 -  avoid a sqrt
  if (linePointsBetweenDistanceSquared == 0.0)
    dist =  point_dist(point, linePoint_1);   % linePoint_1 == linePoint_2 case
    return;
  end
    
  % Line is parameterized as linePoint_1 + t (linePoint_2 - linePoint_1).
  % Find parameter t when point is projected onto the line. 
  
  t = ((point(1) - linePoint_1(1)) * (linePoint_2(1) - linePoint_1(1)) + (point(2) - linePoint_1(2)) * (linePoint_2(2) - linePoint_1(2))) / linePointsBetweenDistanceSquared;
  
  if (t < 0.0) 
    dist = point_dist(point, linePoint_1);       % Beyond the linePoint_1 end of the segment   
    return;
  elseif (t > 1.0) 
    dist = point_dist(point, linePoint_2);  % Beyond the linePoint_2 end of the segment
    return;
  end
      
  projection_point = linePoint_1 + t * (linePoint_2 - linePoint_1);  
 
  dist = point_dist(point, projection_point);

end

function [ dist ] = point_dist( point_1, point_2 )
%POINT_DIST 
    dist = sqrt((point_1(1)-point_2(1))*(point_1(1)-point_2(1)) + (point_1(2)-point_2(2))*(point_1(2)-point_2(2)) );
end
