function [mu, Sigma] = update_kalman_filter(read_only_vars, public_vars)
   
    dt = read_only_vars.sampling_period;
    L = read_only_vars.agent_drive.interwheel_dist;
    u = public_vars.motion_vector; 
    
    z = read_only_vars.gnss_position'; 
    
    mu_old = public_vars.mu;
    Sigma_old = public_vars.sigma;
      
    R = public_vars.kf.R; 
        
    [mu_bar, Sigma_bar] = ekf_predict(mu_old, Sigma_old, u, R, dt, L);
    [mu, Sigma] = kf_measure(mu_bar, Sigma_bar, z, public_vars.kf);
end
