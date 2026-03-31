function [target] = get_target(estimated_pose, path)
% funkce najde lookahead bod na trase
lookahead_dist = 0.6; % cim bude vetsi tim buder trajektorie hladsi
x = estimated_pose(1);
y = estimated_pose(2);

% vzdalenost ke vsem bodum trasy
dist = sqrt((path(:,1) - x).^2 + (path(:,2) - y).^2);
[~, min_idx] = min(dist);

% find the first point ahead on the path at distance >= look-ahead
target_idx = size(path, 1);
for i = min_idx:size(path, 1)
    if dist(i) >= lookahead_dist
        target_idx = i;
        break;
    end
end

target = [path(target_idx, 1), path(target_idx, 2)];
end