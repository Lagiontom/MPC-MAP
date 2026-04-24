function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Summary of this function goes here

new_path = old_path;

if ~isempty(new_path) && size(new_path, 1) > 2
    weight_data = 0.1;  
    weight_smooth = 0.3; 
    tolerance = 0.00001;
    
    change = tolerance;
    max_iters = 1000;
    iter = 0;
    while change >= tolerance && iter < max_iters
        change = 0;
        for i = 2:(size(old_path, 1) - 1)
            for j = 1:2
                aux = new_path(i, j);
                new_path(i, j) = new_path(i, j) + weight_data * (old_path(i, j) - new_path(i, j)) ...
                                 + weight_smooth * (new_path(i+1, j) + new_path(i-1, j) - 2.0 * new_path(i, j));
                change = change + abs(aux - new_path(i, j));
            end
        end
        iter = iter + 1;
    end
end

end
