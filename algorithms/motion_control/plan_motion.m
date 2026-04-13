function [public_vars] = plan_motion(read_only_vars, public_vars)

    
    lookahead_distance = 0.5;  
    robot_speed = 0.3;        
    
   
    pose = public_vars.estimated_pose;
    path = public_vars.path;          
    
    if isempty(path)
        public_vars.motion_vector = [0, 0];
        return;
    end
    
    interwheel_dist = read_only_vars.agent_drive.interwheel_dist;
    wheel_speed_max = read_only_vars.agent_drive.max_vel;

    robot_pos = pose(1:2);
    distances = sqrt((path(:,1) - robot_pos(1)).^2 + (path(:,2) - robot_pos(2)).^2);
    

    [~, curr_idx] = min(distances);
    

    cumulative_dist = 0;
    goal_idx = curr_idx;  % Default: zůstal bychom u nejbližšího bodu
    
    for i = curr_idx:size(path, 1)-1
        segment_dist = sqrt((path(i+1,1) - path(i,1))^2 + (path(i+1,2) - path(i,2))^2);
        cumulative_dist = cumulative_dist + segment_dist;
        
      
        if cumulative_dist >= lookahead_distance
            goal_idx = i + 1;
            break;
        end
    end
    
  
    if goal_idx > size(path, 1)
        goal_idx = size(path, 1);
    end
    
    goal_point = path(goal_idx, :);
    
   
    dx = goal_point(1) - robot_pos(1);
    dy = goal_point(2) - robot_pos(2);
    alpha = atan2(dy, dx) - pose(3);
    

    alpha = atan2(sin(alpha), cos(alpha));
    

    dist_to_goal = sqrt(dx^2 + dy^2);
    

    if dist_to_goal < 1e-3
        omega = 0;
    else
       
        sin_alpha = sin(alpha);
        
        if abs(sin_alpha) > 0.02  
            R = dist_to_goal / (2 * sin_alpha);
            omega = robot_speed / abs(R);
            omega = sign(sin_alpha) * omega;
        else
            omega = 0;
        end
    end
    
  
    v_left = robot_speed - omega * interwheel_dist / 2;
    v_right = robot_speed + omega * interwheel_dist / 2;
    
  
    scale = max(1, max(abs([v_right, v_left])) / wheel_speed_max);
    
    public_vars.motion_vector = [v_right, v_left] / scale;
    
end
