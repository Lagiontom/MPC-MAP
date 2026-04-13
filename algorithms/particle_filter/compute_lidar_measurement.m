function [measurement] = compute_lidar_measurement(map, pose, lidar_config)

x = pose(1);
y = pose(2);
theta = pose(3);


num_beams = length(lidar_config);
measurement = zeros(1, num_beams);

for i = 1:num_beams
    beam_angle = theta + lidar_config(i);
    
    intersections = ray_cast([x; y], map.walls, beam_angle);
    
   
    if ~isempty(intersections)
        dx = intersections(:, 1) - x;  
        dy = intersections(:, 2) - y;  
        dist = sqrt(dx.^2 + dy.^2);   
        
        measurement(i) = min(dist);
    else
        measurement(i) = inf;
    end
end



end

