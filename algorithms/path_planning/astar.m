function [path] = astar(read_only_vars, public_vars)


    path_planning_clearance = 0.7; % ZVYSENO pro vetsi vzdalenost od sten
    % 1. Načtení dat a převod na mřížku
    start_w = public_vars.estimated_pose(1:2);
    goal_w = read_only_vars.map.goal(1:2);
    grid = read_only_vars.discrete_map.map;
    res = read_only_vars.map.discretization_step;
    origin = read_only_vars.map.limits(1:2);
    [rows, cols] = size(grid);
    
    % 2. Vektorizovaná Cost Map (bez toolboxu)
    hard_c = min(0.3, path_planning_clearance) / res; 
    soft_c = (path_planning_clearance * 2.0) / res;
    
    % Předpočítání kruhové masky
    [X, Y] = meshgrid(-ceil(soft_c):ceil(soft_c));
    dist = sqrt(X.^2 + Y.^2);
    mask = dist <= soft_c;
    dX = X(mask); dY = Y(mask);
    penalties = 15 * (1 - dist(mask) / soft_c);
    penalties(dist(mask) <= hard_c) = Inf; % Zdi jsou absolutně neprůjezdné
    
    % Aplikace masky na mapu
    cost_map = zeros(rows, cols);
    [obs_r, obs_c] = find(grid == 1);
    for k = 1:length(obs_r)
        nr = obs_r(k) + dY; nc = obs_c(k) + dX;
        valid = nr >= 1 & nr <= rows & nc >= 1 & nc <= cols;
        idx = sub2ind([rows, cols], nr(valid), nc(valid));
        cost_map(idx) = max(cost_map(idx), penalties(valid));
    end
    
    % 3. Výpočet startu/cíle s ošetřením mezí
    s_c = max(1, min(cols, round((start_w(1) - origin(1))/res) + 1));
    s_r = max(1, min(rows, round((start_w(2) - origin(2))/res) + 1));
    g_c = max(1, min(cols, round((goal_w(1) - origin(1))/res) + 1));
    g_r = max(1, min(rows, round((goal_w(2) - origin(2))/res) + 1));
    
    cost_map(s_r, s_c) = 0; cost_map(g_r, g_c) = 0; % Vždy uvolníme start a cíl
    
    % 4. Spuštění A*
    [p_r, p_c] = astar_core(cost_map, s_r, s_c, g_r, g_c, rows, cols);
    
    % 5. Formátování trasy
    path = [(p_c-1)*res + origin(1), (p_r-1)*res + origin(2)];
    path = [start_w(:)'; path; goal_w(:)'];
    path = path([true; sqrt(sum(diff(path).^2, 2)) > 0.01], :);
end

function [p_r, p_c] = astar_core(cost_map, s_r, s_c, g_r, g_c, rows, cols)
    open_set = [heuristic(s_r, s_c, g_r, g_c), s_r, s_c];
    came_from = containers.Map();
    g_score = inf(rows, cols);
    g_score(s_r, s_c) = 0;
    
    while ~isempty(open_set)
        [~, idx] = min(open_set(:,1));
        c_r = open_set(idx,2); c_c = open_set(idx,3);
        open_set(idx,:) = [];
        
        if c_r == g_r && c_c == g_c
            [p_r, p_c] = reconstruct_path(came_from, g_r, g_c); return;
        end
        
        for dr = -1:1
            for dc = -1:1
                n_r = c_r + dr; n_c = c_c + dc;
                
                % Přeskočení neplatných bodů a překážek (Inf)
                if n_r < 1 || n_r > rows || n_c < 1 || n_c > cols || cost_map(n_r, n_c) == Inf
                    continue;
                end
                
                tent_g = g_score(c_r, c_c) + norm([dr, dc]) + cost_map(n_r, n_c);
                if tent_g < g_score(n_r, n_c)
                    came_from(sprintf('%d,%d', n_r, n_c)) = [c_r, c_c];
                    g_score(n_r, n_c) = tent_g;
                    f = tent_g + heuristic(n_r, n_c, g_r, g_c);
                    open_set = [open_set; [f, n_r, n_c]];
                end
            end
        end
    end
    error('Cesta nenalezena!');
end

function h = heuristic(r1, c1, r2, c2)
    dx = abs(c1 - c2); dy = abs(r1 - r2);
    h = (dx + dy) + (sqrt(2) - 2) * min(dx, dy);
end

function [p_r, p_c] = reconstruct_path(came_from, g_r, g_c)
    p_r = g_r; p_c = g_c;
    curr = sprintf('%d,%d', g_r, g_c);
    while isKey(came_from, curr)
        val = came_from(curr);
        p_r = [val(1); p_r]; p_c = [val(2); p_c];
        curr = sprintf('%d,%d', val(1), val(2));
    end
end