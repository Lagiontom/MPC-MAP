function [path] = plan_path(read_only_vars, public_vars)
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
        path = smooth_path(raw_path);
    else
        path = public_vars.path;
    end
end