function [public_vars] = student_workspace(read_only_vars,public_vars)

if isempty(read_only_vars.gnss_position) || isnan(read_only_vars.gnss_position(1))
    current_mode = 'indoor';
else
    current_mode = 'outdoor';
end

if ~isfield(public_vars, 'iteration_counter')
    public_vars.iteration_counter = 0;
end
public_vars.iteration_counter = public_vars.iteration_counter + 1;

if ~isfield(public_vars, 'pf_stabilization_counter')
    public_vars.pf_stabilization_counter = 0;
end


if ~isfield(public_vars, 'ekf_stabilization_counter')
    public_vars.ekf_stabilization_counter = 0;
end

if ~isfield(public_vars, 'is_pf_init')
    public_vars.is_pf_init = false;
end
if ~isfield(public_vars, 'is_ekf_init')
    public_vars.is_ekf_init = false;
    public_vars.ekf_init_counter = 0;
end
if ~isfield(public_vars, 'last_mode')
    public_vars.last_mode = 'unknown'; 
end

if strcmp(current_mode, 'indoor')
    
    if ~public_vars.is_pf_init
        public_vars = init_particle_filter(read_only_vars, public_vars);
        public_vars.is_pf_init = true;
    elseif strcmp(public_vars.last_mode, 'outdoor')
        public_vars.pf_stabilization_counter = 10; 
        
        last_known_pose = [];
        if isfield(read_only_vars, 'gnss_history') && ~isempty(read_only_vars.gnss_history)
            valid_history = read_only_vars.gnss_history(~isnan(read_only_vars.gnss_history(:,1)), :);
            if ~isempty(valid_history)
                last_gnss_pos = valid_history(end, :);
                last_theta = public_vars.mu(3); 
                last_known_pose = [last_gnss_pos, last_theta];
            end
        end
        
        if isempty(last_known_pose) && public_vars.is_ekf_init
            last_known_pose = public_vars.mu';
        end
        if ~isempty(last_known_pose)
            num_particles = read_only_vars.max_particles;
            particles = zeros(num_particles, 3);
            walls = read_only_vars.map.walls;
            count = 0;
            
            init_pos_std = 0.15; 
            init_theta_std = 0.1; 
            
            while count < num_particles
                x_rand = last_known_pose(1) + randn() * init_pos_std;
                y_rand = last_known_pose(2) + randn() * init_pos_std;
                theta_rand = last_known_pose(3) + randn() * init_theta_std;
                if is_inside_free_space(x_rand, y_rand, walls)
                    count = count + 1;
                    particles(count, :) = [x_rand, y_rand, mod(theta_rand + pi, 2 * pi) - pi];
                end
            end
            public_vars.particles = particles;
        else
            public_vars = init_particle_filter(read_only_vars, public_vars);
        end
        public_vars.is_pf_init = true;
    end
    if public_vars.is_pf_init
        public_vars.particles = update_particle_filter(read_only_vars, public_vars);
        mean_x = mean(public_vars.particles(:,1));
        mean_y = mean(public_vars.particles(:,2));
        mean_theta = atan2(mean(sin(public_vars.particles(:,3))), mean(cos(public_vars.particles(:,3))));
        public_vars.estimated_pose = [mean_x, mean_y, mean_theta];
    end
    
else
    public_vars.particles = [];
    
    if ~public_vars.is_ekf_init 
        if strcmp(public_vars.last_mode, 'indoor')
            public_vars = init_kalman_filter(read_only_vars, public_vars); 
            public_vars.mu = public_vars.estimated_pose'; 
            public_vars.sigma = diag([0.2^2, 0.2^2, 0.1^2]); 
            public_vars.is_ekf_init = true;
            public_vars.ekf_stabilization_counter = 10; 
            public_vars.ekf_init_counter = public_vars.ekf_init_counter + 1;
            if public_vars.ekf_init_counter < public_vars.gnss_init_samples
                public_vars.motion_vector = [0, 0];
                public_vars.estimated_pose = [2, 2, pi/2]; 
                return; 
            else
                rov_mod = read_only_vars;
                valid_idx = ~isnan(rov_mod.gnss_history(:,1));
                rov_mod.gnss_history = rov_mod.gnss_history(valid_idx, :);
                public_vars = init_kalman_filter(rov_mod, public_vars);
                public_vars.is_ekf_init = true;
            end
        end
    else
        if strcmp(public_vars.last_mode, 'indoor')
            public_vars.mu(1) = public_vars.estimated_pose(1);
            public_vars.mu(2) = public_vars.estimated_pose(2);
            public_vars.mu(3) = public_vars.estimated_pose(3);
            public_vars.sigma = diag([0.5^2, 0.5^2, 1.0^2]); 
            public_vars.ekf_stabilization_counter = 10; 
        end
    end
    
    if public_vars.is_ekf_init
        if strcmp(public_vars.last_mode, 'indoor')
            temp_rov = read_only_vars;
            temp_rov.gnss_position = [NaN, NaN]; 
            [public_vars.mu, public_vars.sigma] = update_kalman_filter(temp_rov, public_vars);
        else
            [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
        end
        public_vars.estimated_pose = public_vars.mu';
    end
end
public_vars.last_mode = current_mode;
public_vars.mocap_pose = read_only_vars.mocap_pose;


if strcmp(current_mode, 'indoor') && public_vars.pf_stabilization_counter > 0
    public_vars.pf_stabilization_counter = public_vars.pf_stabilization_counter - 1;
    public_vars.motion_vector = [0, 0]; 
elseif strcmp(current_mode, 'outdoor') && public_vars.ekf_stabilization_counter > 0
  
    public_vars.ekf_stabilization_counter = public_vars.ekf_stabilization_counter - 1;
    public_vars.motion_vector = [0, 0]; 
else
    if(public_vars.iteration_counter > 10)
       public_vars.path = plan_path(read_only_vars, public_vars);
       public_vars = plan_motion(read_only_vars, public_vars);
    end
end
end