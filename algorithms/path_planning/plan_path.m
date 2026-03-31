function [path] = plan_path(read_only_vars, public_vars)
% week 3 - rucne definovane

path = public_vars.path;

if isempty(path)
    goal = read_only_vars.map.goal(1:2);
    
    
    %control_points = get_control_points_line(goal);
    control_points = get_control_points_circular(goal);
    %control_points = get_control_points_sine(goal);

    t = 1:size(control_points, 1);
    tt = linspace(1, size(control_points, 1), 20);
    x_spline = pchip(t, control_points(:,1), tt);
    y_spline = pchip(t, control_points(:,2), tt);

    path = [x_spline', y_spline'];
    %path = [control_points(:,1), control_points(:,2)];
end

end

function [points] = get_control_points_line(goal)

points = [ ...
        1.0, 1.0; ...
        goal(1), goal(2)];

end



function [points] = get_control_points_circular(goal)

points = [ ...
        1.0, 1.0; ...
        7.0, 3.0; ...
        goal(1), goal(2)];

end



function [points] = get_control_points_sine(goal)

points = [ ...
        1.0, 1.0; ...
        2.0, 2.2; ...
        3.0, 4.0; ...
        4.0, 5.2; ...
        5.0, 5.0; ...
        6.0, 4.8; ...
        7.0, 6.0; ...
        8.0, 7.8; ...
        goal(1), goal(2)];

end

