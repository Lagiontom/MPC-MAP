function [public_vars] = student_workspace(read_only_vars, public_vars)
    public_vars.maxspeed = 1; 
    public_vars.minspeed = 0.5;
    
    if any(isnan(read_only_vars.gnss_position))
            public_vars.time_to_wait = 1;  
        else
            public_vars.time_to_wait = 100;  
    end

    
    
    

    wait_cycles_gnss = 10;

    if read_only_vars.counter == 1
        public_vars.pf_enabled = false;
        public_vars.kf_enabled = false;
        
        if any(isnan(read_only_vars.gnss_position))
            public_vars.target_cycle = 1; 
            public_vars.initial_pose = [2, 7, pi/2]; 
        else
            public_vars.target_cycle = read_only_vars.counter + wait_cycles_gnss; 
            public_vars.initial_pose = [read_only_vars.gnss_position(1), read_only_vars.gnss_position(2), pi/4];
        end
    end

    if read_only_vars.counter < public_vars.target_cycle
        public_vars.motion_vector = [0, 0]; 
        public_vars.estimated_pose = public_vars.initial_pose; 
        return; 
    elseif read_only_vars.counter == public_vars.target_cycle
        if any(isnan(read_only_vars.gnss_position))
            public_vars = init_particle_filter(read_only_vars, public_vars);
            public_vars.estimated_pose = public_vars.initial_pose; 
            public_vars.pf_enabled = true;
        else
            public_vars = init_kalman_filter(read_only_vars, public_vars);
            

            public_vars.mu = [public_vars.initial_pose(1); 
                              public_vars.initial_pose(2); 
                              public_vars.initial_pose(3)];
            public_vars.sigma = diag([0.5, 0.5, 0.1]);
            
            public_vars.kf_enabled = true;
        end
    end

    is_indoor = any(isnan(read_only_vars.gnss_position));


    if is_indoor && ~public_vars.pf_enabled
    public_vars = init_particle_filter(read_only_vars, public_vars);
    

    if isfield(public_vars, 'kf_enabled') && public_vars.kf_enabled
        last_pose = public_vars.estimated_pose;
        N = size(public_vars.particles, 1);
        public_vars.particles(:,1) = last_pose(1) + 0.3 * randn(N, 1);
        public_vars.particles(:,2) = last_pose(2) + 0.3 * randn(N, 1);
        public_vars.particles(:,3) = last_pose(3) + 0.5 * randn(N, 1);
    end
    
    public_vars.pf_enabled = true;
    public_vars.kf_enabled = false;
    end


    if ~is_indoor && ~public_vars.kf_enabled
    last_pose = public_vars.estimated_pose;
    
    public_vars = init_kalman_filter(read_only_vars, public_vars);
    
    public_vars.mu = [last_pose(1); 
                      last_pose(2); 
                      last_pose(3)];
    

    public_vars.sigma = diag([0.5, 0.5, 0.1]); 
    
    public_vars.kf_enabled = true;
    public_vars.pf_enabled = false;
    end

    is_ready_to_plan = true;

    if public_vars.pf_enabled
        [public_vars.particles, public_vars.mu_pf] = update_particle_filter(read_only_vars, public_vars);
        public_vars.estimated_pose = public_vars.mu_pf;
        

        thetas = public_vars.particles(:,3);
        circ_var = 1 - sqrt(mean(cos(thetas))^2 + mean(sin(thetas))^2);
        

        if std(public_vars.particles(:,1)) > 0.5 || std(public_vars.particles(:,2)) > 0.5 || circ_var > 0.2
            is_ready_to_plan = false;
        end
    elseif public_vars.kf_enabled
        [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
        public_vars.estimated_pose = public_vars.mu';
    end

    if is_ready_to_plan
        public_vars.path = plan_path(read_only_vars, public_vars);
        public_vars = plan_motion(read_only_vars, public_vars);
    else

        public_vars.motion_vector = [-0.5, 0.5];
    end
    disp(read_only_vars.counter);
end