function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Initializes the Kalman filter parameters.
 

gnss_data = read_only_vars.gnss_history;
 

initial_mean = mean(gnss_data);
gnss_cov = cov(gnss_data);
 

disp('Inicializace Kalmanova filtru...');
disp('Změřená počáteční pozice (mean):');
disp(initial_mean);
disp('Kovarianční matice GNSS senzoru - Q (šum měření):');
disp(gnss_cov);
 

public_vars.kf.C = [1 0 0; 0 1 0];

public_vars.kf.Q = gnss_cov;

public_vars.kf.R = diag([0.0001, 0.0001, 0.0001]);
 

public_vars.mu = [initial_mean(1); initial_mean(2); 0];

public_vars.sigma = diag([gnss_cov(1,1), gnss_cov(2,2), 10]); 
 
end