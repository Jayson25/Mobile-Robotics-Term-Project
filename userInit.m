
% Problem 1 - Round shaped differential drive robot navigation
% User initialization function

function userStructure = userInit(model, environment)
    
    % Precision of the map (higher = better precision but slower) [note: only integers]
    precision = 5;
    
    % Delimitation of the map
    mapStartx = environment.plotArea(1);
    mapEndx = environment.plotArea(2);
    mapStarty = environment.plotArea(3);
    mapEndy = environment.plotArea(4);
    
    % Size of the array that will contain the map
    sizeMapx = (mapEndx-mapStartx)*precision;
    sizeMapy = (mapEndy-mapStarty)*precision;
    
    % error counters for the PID controllers (one for dist and one for
    % angle)
    userStructure.errorAngle = [0.0, 0.0];
    userStructure.errorDist = [0.0, 0.0];
    
    %index of the checkpoint
    userStructure.ci = 2;
    
    %boolean for started value
    userStructure.started = 0;
    
    % counter for integrates
    userStructure.integDist = 0.0;
    userStructure.integAngle = 0.0;
    
    % Coordinates of the start point and userStructure.goal point
    userStructure.startPoint = model.state(1, 1:2)';
    userStructure.goal = environment.stateGoal(1, 1:2)';
    
    % Converts the above coordinates into map friendly values
    startx = (userStructure.startPoint(1) - mapStartx) * precision;
    starty = (userStructure.startPoint(2) - mapStarty) * precision;
    goalx = (userStructure.goal(1) - mapStartx) * precision;
    goaly = (userStructure.goal(2) - mapStarty) * precision;
    
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
    
    %Make sure that the userStructure.goal is not an obstacle
    userStructure.map(goalx,goaly) = 1;
    
    %Parents and costs 
    parentList = pathFinding(startx, starty, goalx, goaly,sizeMapx, sizeMapy, userStructure.map);
    
    disp("Total cost to userStructure.goal: " + parentList(goalx,goaly,3));
   
    userStructure.x(1) = parentList(goalx,goaly,1); userStructure.y(1) = parentList(goalx,goaly,2);

    index = 1;

    while userStructure.x(index) ~= startx || userStructure.y(index) ~= starty
       userStructure.x(index+1) = parentList(userStructure.x(index),userStructure.y(index),1); 
       userStructure.y(index+1) = parentList(userStructure.x(index), userStructure.y(index),2);

       index = index + 1;
    end
    
    %flip the order of the arrays in order to have the start point as first
    %element
    userStructure.x = flip(userStructure.x);
    userStructure.y = flip(userStructure.y);

    sizeArray = size(userStructure.y);
    
    %distance between two points
    distx = abs(userStructure.x(2) - userStructure.x(1));
    disty = abs(userStructure.y(2) - userStructure.y(1));
    
    index = 1;
    add = 1;
    
    %checkpoints generation
    for i = 3:sizeArray(2)
        %distance of points that are now treaten
        currDistx = abs(userStructure.x(i)-userStructure.x(i-1));
        currDisty = abs(userStructure.y(i)-userStructure.y(i-1));
        
        %if one of the distances changes (x or y), then the previous points
        %will be a checkpoint
        if ((currDistx ~= distx) || (currDisty ~= disty) || mod(add,10) == 0 || i == sizeArray(2)-4) && i < sizeArray(2)-3
            
            index = index+1;
            userStructure.checkpoint(index,1) = userStructure.x(i-1);
            userStructure.checkpoint(index,2) = userStructure.y(i-1);
            
            add = 1;
        end
        
        %change the values of the buffer for the next points
        distx = currDistx;
        disty = currDisty;
        
        add = add + 1;
    end
    
     index = index+1;
     
     userStructure.checkpoint(index,1) = userStructure.x(sizeArray(2));
     userStructure.checkpoint(index,2) = userStructure.y(sizeArray(2));
    
    %convert the values of the matrix into map coordinates values
    for i = 1:sizeArray(2)
        userStructure.x(i) = mapStartx + userStructure.x(i)/precision;
        userStructure.y(i) = mapStarty + userStructure.y(i)/precision;
    end
   
    %same process for the checkpoint
    for i = 1:index
        userStructure.checkpoint(i,1) = mapStartx + userStructure.checkpoint(i,1)/precision;
        userStructure.checkpoint(i,2) = mapStarty + userStructure.checkpoint(i,2)/precision;
    end
    
     userStructure.checkpoint(1,1) = userStructure.x(1);
     userStructure.checkpoint(1,2) = userStructure.y(1);
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
                    child = checkShortestPath(childx,childy,parentx(index),parenty(index),parentList);

                    parentList(childx,childy,1) = child(1);
                    parentList(childx,childy,2) = child(2);
                    parentList(childx,childy,3) = child(3);
                    
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
function child = checkShortestPath(cx,cy,px,py, parentList)
    
    %if diagonal
    if cx~=px && cy~=py
        cost = parentList(px,py,3) + 1.5;
    else
        cost = parentList(px,py,3) + 1;
    end
    
    if cost < parentList(cx,cy,3)
       child = [px,py,cost];
    else
        child = [parentList(cx,cy,1),parentList(cx,cy,2),parentList(cx,cy,3)];
    end
end

function isCollided = checkObstacle(coord, model, environment)
    
    isCollided = 0;

    [~, NbOfObstacleLines] = size(environment.corner);
    for j = 1:NbOfObstacleLines
        
        currObstacleLine = environment.corner(:, j);
        
        firstPointOfObstacleLine = currObstacleLine(1:2, 1);
        secondPointOfObstacleLine = currObstacleLine(3:4, 1);
        
        % 1) Find the distance between Obstacle Line segment and coord point of the graph Position
        coord_ObstacleLine_dist = point_line_segment_dist(firstPointOfObstacleLine, secondPointOfObstacleLine, coord);
        
        % 2) Determine wheter the distance is shorter than Vehicle Radious
        if(coord_ObstacleLine_dist <= model.radius+0.15)
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