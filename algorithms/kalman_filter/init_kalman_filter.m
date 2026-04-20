function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here


gnss_data = read_only_vars.gnss_history;
initial_mean = mean(gnss_data);


gnss_cov = cov(gnss_data);

disp('Inicializace GNSS');
disp('Změřená počáteční pozice - mean:');
disp(initial_mean);
disp('Kovarianční matice GNSS senzoru - R:');
disp(gnss_cov);


public_vars.kf.C = [1 0 0; 0 1 0];


% Q = Matice nejistoty měření 
public_vars.kf.Q = gnss_cov;

% R = Matice nejistoty procesu 
public_vars.kf.R = diag([0.01, 0.01, 0.01]);

% for task 3
%public_vars.mu = [2; 2; pi/2];
%public_vars.sigma = zeros(3, 3);


 public_vars.mu = [initial_mean(1); initial_mean(2); 0];
 public_vars.sigma = diag([gnss_cov(1,1), gnss_cov(2,2), 10]); % Velká nejistota pro úhel (10)

end
