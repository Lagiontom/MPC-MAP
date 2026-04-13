function inside = is_inside_free_space(x, y, walls)

inside = true;

for i = 1:size(walls, 1)

    x1 = walls(i, 1);  % počátek stěny
    y1 = walls(i, 2);
    x2 = walls(i, 3);  % konec stěny
    y2 = walls(i, 4);
    

    
    numerator = abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1);
    denominator = sqrt((y2 - y1)^2 + (x2 - x1)^2);
    
    d = numerator / denominator;  
    if d < 0.02  
        inside = false;  
        return;          
    end
end



end