
function [path] = plan_path(read_only_vars, public_vars)
    %planning_required = true; 
    
    try
        if isempty(public_vars.path)
            planning_required = true;
        else
            
            planning_required = false;
        end
    catch
        planning_required = true; 
    end
    
    if planning_required
        raw_path = astar(read_only_vars, public_vars);
        %path = raw_path;
        path = smooth_path(raw_path);

    else
        path = public_vars.path;
    end
end



% function [path] = plan_path(read_only_vars, public_vars)
% % week 3 
% 
% path = public_vars.path;
% 
% % Původní plánování cesty pomocí kontrolních bodů
% %goal = read_only_vars.map.goal(1:2);
% 
%     %
%     %%control_points = get_control_points_line(goal);
%     %%control_points = get_control_points_circular(goal);
%     %%control_points = get_control_points_sine(goal);
%     %%control_points = get_control_points_indoor1(goal);
%     %control_points = get_control_points_outdoor1(goal);
% %
%     %t = 1:size(control_points, 1);
%     %tt = linspace(1, size(control_points, 1), 20);
%     %x_spline = pchip(t, control_points(:,1), tt);
%     %y_spline = pchip(t, control_points(:,2), tt);
% %
%     %path = [x_spline', y_spline'];
%     %%path = [control_points(:,1), control_points(:,2)];
% 
% 
% 
% function [points] = get_control_points_indoor1(goal)
% 
% points = [ ...
%         5, 8.5; ...
%         5, 6; ...
%         5, 4; ...
%         5, 2; ...
%         5, 1.5; ...
%         7, 1.5; ...
%         8.5, 1.5; ...
%         9, 3; ...
%         9, 5; ...
%         9, 7; ...
%         goal(1), goal(2)];
% 
% end
% 
% 
% function [points] = get_control_points_line(goal)
% 
% points = [ ...
%         1.0, 1.0; ...
%         goal(1), goal(2)];
% 
% end
% 
% 
% 
% function [points] = get_control_points_circular(goal)
% 
% points = [ ...
%         1.0, 1.0; ...
%         7.0, 3.0; ...
%         goal(1), goal(2)];
% 
% end
% 
% 
% 
% function [points] = get_control_points_sine(goal)
% 
% points = [ ...
%         1.0, 1.0; ...
%         2.0, 2.2; ...
%         3.0, 4.0; ...
%         4.0, 5.2; ...
%         5.0, 5.0; ...
%         6.0, 4.8; ...
%         7.0, 6.0; ...
%         8.0, 7.8; ...
%         goal(1), goal(2)];
% 
% end
% 
% 
% function [points] = get_control_points_outdoor1(goal)
% 
% points = [ ...
%         2, 2; ...
%         2, 3; ...
%         4, 5; ...
%         6, 8; ...
%         8, 8; ...
%         10, 8; ...
%         13, 7; ...
%         15, 5; ...
%         16, 2; ...
%         goal(1), goal(2)];
% 
% end
