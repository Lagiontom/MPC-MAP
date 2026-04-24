function [path] = astar(read_only_vars, public_vars)
    
    start_world = public_vars.estimated_pose(1:2);
    goal_world = read_only_vars.map.goal;
    grid = read_only_vars.discrete_map.map; 
    resolution = read_only_vars.map.discretization_step;
    origin = read_only_vars.map.limits(1:2);
    
    
    clearance_cells = ceil(0.35 / resolution);  
    [rows, cols] = size(grid);
    new_grid = grid;
    [obs_r, obs_c] = find(grid == 1);
    for k = 1:length(obs_r)
        r = obs_r(k); c = obs_c(k);
        r_range = max(1,r-clearance_cells):min(rows,r+clearance_cells);
        c_range = max(1,c-clearance_cells):min(cols,c+clearance_cells);
        new_grid(r_range, c_range) = 1;
    end
    
    
    start_col = round((start_world(1) - origin(1)) / resolution) + 1;
    start_row = round((start_world(2) - origin(2)) / resolution) + 1;
    goal_col = round((goal_world(1) - origin(1)) / resolution) + 1;
    goal_row = round((goal_world(2) - origin(2)) / resolution) + 1;
    
    
    new_grid(start_row, start_col) = 0;
    new_grid(goal_row, goal_col) = 0;
    
    grid = new_grid;
    
  
    [path_rows, path_cols] = astar_core(~grid, start_row, start_col, goal_row, goal_col, rows, cols);
    
   
    path = [(path_cols-1)*resolution + origin(1), (path_rows-1)*resolution + origin(2)];
    
    
    gw = goal_world(1:2);
    path = [start_world(:)'; path; gw(:)'];
    
    
    keep = [true; sqrt(sum(diff(path).^2, 2)) > 0.01];
    path = path(keep, :);
end

function [path_rows, path_cols] = astar_core(grid, start_row, start_col, goal_row, goal_col, rows, cols)
   
    open_set = [heuristic(start_row, start_col, goal_row, goal_col), start_row, start_col];
    
    came_from = containers.Map();
    g_score = inf(rows, cols);
    g_score(start_row, start_col) = 0;
    
    while ~isempty(open_set)
        [~, idx] = min(open_set(:,1));
        current_f = open_set(idx,1);
        current_row = open_set(idx,2);
        current_col = open_set(idx,3);
        open_set(idx,:) = [];
        
        if current_row == goal_row && current_col == goal_col
            [path_rows, path_cols] = reconstruct_path(came_from, goal_row, goal_col);
            return;
        end
        
        for dr = -1:1
            for dc = -1:1
                if dr == 0 && dc == 0, continue; end
                neighbor_row = current_row + dr;
                neighbor_col = current_col + dc;
                
                if neighbor_row < 1 || neighbor_row > rows || ...
                   neighbor_col < 1 || neighbor_col > cols || ...
                   grid(neighbor_row, neighbor_col) == 0
                    continue;
                end
                
                tentative_g = g_score(current_row, current_col) + norm([dr, dc]);
                
                if tentative_g < g_score(neighbor_row, neighbor_col)
                    came_from(sprintf('%d,%d', neighbor_row, neighbor_col)) = [current_row, current_col];
                    g_score(neighbor_row, neighbor_col) = tentative_g;
                    f_score = tentative_g + heuristic(neighbor_row, neighbor_col, goal_row, goal_col);
                    open_set = [open_set; [f_score, neighbor_row, neighbor_col]];
                end
            end
        end
    end
    error('No path found!');
end

function h = heuristic(row1, col1, row2, col2)
    dx = abs(col1 - col2);
    dy = abs(row1 - row2);
    h = (dx + dy) + (sqrt(2) - 2) * min(dx, dy);
end

function [path_rows, path_cols] = reconstruct_path(came_from, goal_row, goal_col)
    path_rows = goal_row;
    path_cols = goal_col;
    current_key = sprintf('%d,%d', goal_row, goal_col);
    
    while isKey(came_from, current_key)
        current = came_from(current_key);
        path_rows = [current(1); path_rows];
        path_cols = [current(2); path_cols];
        current_key = sprintf('%d,%d', current(1), current(2));
    end
end
