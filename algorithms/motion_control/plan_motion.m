function [public_vars] = plan_motion(read_only_vars, public_vars)

    is_indoor = isempty(read_only_vars.gnss_position) || ...
                isnan(read_only_vars.gnss_position(1));

    if is_indoor
        lookahead_distance = 0.35;
        steering_gain = 2.5;
    else
        lookahead_distance = 0.8;
        steering_gain = 1.5;
    end

    robot_speed = public_vars.maxspeed;

    if read_only_vars.counter < public_vars.time_to_wait
        robot_speed = min(robot_speed, 0.2);
    end

    is_uncertain = false;

    if isfield(public_vars, 'is_ekf_init') && ...
       public_vars.is_ekf_init && ...
       isfield(public_vars, 'sigma')

        if public_vars.sigma(3,3) > 1.0
            is_uncertain = true;
        end

    elseif isfield(public_vars, 'is_pf_init') && ...
           public_vars.is_pf_init && ...
           ~isempty(public_vars.particles)

        thetas = public_vars.particles(:,3);

        circ_var = 1 - sqrt( ...
            mean(cos(thetas))^2 + ...
            mean(sin(thetas))^2);

        if circ_var > 0.5
            is_uncertain = true;
        end
    end

    if is_uncertain
        robot_speed = min(robot_speed, 0.2);
    end

    if is_indoor && ~isempty(read_only_vars.lidar_distances)

        min_dist = min(read_only_vars.lidar_distances);
        safe_dist = 0.6;

        if min_dist < safe_dist

            speed_scale = max(0, min_dist / safe_dist);

            proximity_speed = ...
                public_vars.minspeed + ...
                (robot_speed - public_vars.minspeed) * speed_scale;

            robot_speed = min(robot_speed, proximity_speed);
        end
    end

    pose = public_vars.estimated_pose;
    path = public_vars.path;

    if isempty(path)
        public_vars.motion_vector = [0, 0];
        return;
    end

    robot_pos = pose(1:2);

    distances = sqrt( ...
        (path(:,1) - robot_pos(1)).^2 + ...
        (path(:,2) - robot_pos(2)).^2);

    [~, curr_idx] = min(distances);

    cumulative_dist = 0;
    goal_idx = size(path, 1);

    for i = curr_idx:size(path,1)-1

        segment_dist = sqrt( ...
            (path(i+1,1) - path(i,1))^2 + ...
            (path(i+1,2) - path(i,2))^2);

        cumulative_dist = cumulative_dist + segment_dist;

        if cumulative_dist >= lookahead_distance
            goal_idx = i + 1;
            break;
        end
    end

    goal_point = path(goal_idx, :);

    dx = goal_point(1) - robot_pos(1);
    dy = goal_point(2) - robot_pos(2);

    alpha = atan2(dy, dx) - pose(3);
    alpha = atan2(sin(alpha), cos(alpha));

    turn_in_place_threshold = pi / 8;
    max_angular_vel = 2;

    if abs(alpha) > turn_in_place_threshold

        v = 0;
        omega = max_angular_vel * sign(alpha);

    else

        v = robot_speed * ...
            max(0.2, 1 - abs(alpha) / turn_in_place_threshold);

        dist_to_goal = sqrt(dx^2 + dy^2);

        omega = steering_gain * ...
                (2 * v * sin(alpha)) / dist_to_goal;
    end

    safety_critical_dist = 0.35;

    if ~isempty(read_only_vars.lidar_distances)

        [min_dist, min_idx] = min(read_only_vars.lidar_distances);

        if min_dist < safety_critical_dist

            obstacle_angle = ...
                read_only_vars.lidar_config(min_idx);

            if abs(obstacle_angle) < (pi / 2)

                repulsive_strength = ...
                    1 - (min_dist / safety_critical_dist);

                omega = ...
                    -sign(obstacle_angle) * ...
                    max_angular_vel * ...
                    repulsive_strength;

                v = 0;
            end
        end
    end

    interwheel_dist = ...
        read_only_vars.agent_drive.interwheel_dist;

    wheel_speed_max = ...
        read_only_vars.agent_drive.max_vel;

    v_right = v + (omega * interwheel_dist / 2);
    v_left  = v - (omega * interwheel_dist / 2);

    scale = max( ...
        1, ...
        max(abs([v_right, v_left])) / wheel_speed_max);

    public_vars.motion_vector = ...
        [v_right, v_left] / scale;

end