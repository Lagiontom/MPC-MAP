function [public_vars] = init_particle_filter(read_only_vars, public_vars)

x_min = read_only_vars.map.limits(1,1);
x_max = read_only_vars.map.limits(1,3);
y_min = read_only_vars.map.limits(1,2);
y_max = read_only_vars.map.limits(1,4);


walls = read_only_vars.map.walls;


num_particles = read_only_vars.max_particles;


particles = zeros(num_particles, 3);


is_valid_position = @(x, y) is_inside_free_space(x, y, walls);


count = 0;

while count < num_particles
   
    x_rand = x_min + (x_max - x_min) * rand();  
    y_rand = y_min + (y_max - y_min) * rand(); 
    
   
    theta_rand = -pi + (2 * pi) * rand();
    

    if is_valid_position(x_rand, y_rand)
     
        count = count + 1;
        particles(count, :) = [x_rand, y_rand, theta_rand];
    end
  
end


public_vars.particles = particles; 
public_vars.weights = ones(num_particles, 1) / num_particles;


end


