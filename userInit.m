
% Problem 1 - Round shaped differential drive robot navigation
% User initialization function

function userStructure = userInit(model, environment)

    userStructure.exampleVariable = 0;
    userStructure.exampleRowVector = [1,2,3];
    userStructure.exampleColumnVector = [1;2;3];
    userStructure.exampleMatrix = [1,2;3,4];
    
    startx = environment.plotArea(1);
    endx = environment.plotArea(2);
    
    starty = environment.plotArea(3);
    endy = environment.plotArea(4);
    
    userStructure.minWeight = 0;
    startPoint = model.state(1, 1:2)'; %first two values of model.state, same shit as model.state(1:2)' 
    goal = environment.stateGoal(1, 1:2)';
    
    adderx = startx;
    addery = starty;
    
    sizeSample = 10;
    
    %generates a map with the obstacles and takes into account the robot's
    %size (radius)
    
    for i = 1:(endx-startx)*sizeSample
        for j = 1:(endy-starty)*sizeSample
            
            coord = [startx+i/sizeSample,starty+j/sizeSample]';
            
            if (checkObstacle(coord, model, environment) == 1)
                userStructure.map(i,j) = Inf;
            
            else 
                userStructure.map(i,j) = 1;
            end
            
       end
    end
    
   % userStructure.map(goal) = 0;
   
    index = 1;
    
    indexX = startPoint(1);
    indexY = startPoint(2);
    
    while (0)
        finish_path = 1;
    end
    
    index = 1;
        
    for i = 1:(endx-startx)*sizeSample
       addery = startx;
       for j = 1:(endy-starty)*sizeSample
           
           if (userStructure.map(i,j) ~= Inf)
           
           userStructure.x(index) = adderx;
           userStructure.y(index) = addery;
           index = index + 1;
           end
           
           addery = addery + 1/sizeSample;
       end
       adderx = adderx + 1/sizeSample;
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