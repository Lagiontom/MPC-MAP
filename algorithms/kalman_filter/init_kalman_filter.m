function [public_vars] = init_kalman_filter(read_only_vars, public_vars)

    public_vars.kf.C = [1 0 0; 0 1 0];
    
  
    public_vars.kf.Q = diag([0.5, 0.5]); 
    
    public_vars.kf.R = diag([0.00005, 0.00005, 0.0005]);


    public_vars.mu = [0; 0; 0];
    public_vars.sigma = diag([0.5, 0.5, 10]); 

end