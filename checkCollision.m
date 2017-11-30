
% Problem 1 - Round shaped differential drive robot navigation
% Collision check function for problem 1 (round model)

function isCollided = checkCollision(model, environment)
    
    isCollided = 0;

    [~, NbOfObstacleLines] = size(environment.corner);
    for j = 1:NbOfObstacleLines
        
        currObstacleLine = environment.corner(:, j);
        firstPointOfObstacleLine = currObstacleLine(1:2, 1);
        secondPointOfObstacleLine = currObstacleLine(3:4, 1);
        currVehiclePos =  model.state(1, 1:2)';
        
        % 1) Find the distance between Obstacle Line segment and Vehicle Position
        vehicle_ObstacleLine_dist = point_line_segment_dist(firstPointOfObstacleLine, secondPointOfObstacleLine, currVehiclePos);
        
        % 2) Determine wheter the distance is shorter than Vehicle Radious
        if(vehicle_ObstacleLine_dist <= model.radius)
            isCollided = 1;
            return;
        else
            isCollided = 0;
        end

    end

end

function [ dist ] = point_line_segment_dist( linePoint_1, linePoint_2, point )
%POINT_LINE_SEGMENT_DIST 이 함수의 요약 설명 위치
%   자세한 설명 위치

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
%POINT_DIST 이 함수의 요약 설명 위치
%   자세한 설명 위치
    dist = sqrt((point_1(1)-point_2(1))*(point_1(1)-point_2(1)) + (point_1(2)-point_2(2))*(point_1(2)-point_2(2)) );

end


