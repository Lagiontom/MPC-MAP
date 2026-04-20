function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
INIT_STEPS = 100;
if (read_only_vars.counter < INIT_STEPS)
    % Během prvních 100 iterací stojíme na místě a sbíráme GNSS data pro určení šumu
    public_vars.motion_vector = [0, 0];
    public_vars.estimated_pose = [2, 2, pi/2]; % provizorní odhad pro vizualizaci
    return; % Přeskočíme zbytek kódu, dokud nemáme dost dat
elseif (read_only_vars.counter == INIT_STEPS)
    % Ve 100. iteraci inicializujeme Kalmanův filtr na základě nasbíraných dat
    public_vars = init_kalman_filter(read_only_vars, public_vars);
end

% 9. Update particle filter (Vypnuto pro Kalmanův filtr)
% public_vars.particles = update_particle_filter(read_only_vars, public_vars);
% 
% % 10. Update Kalman filter
 [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
% 
% % 11. Estimate current robot position
 public_vars.mocap_pose = read_only_vars.mocap_pose;
 % public_vars.estimated_pose = estimate_pose(public_vars); % Staré z PF
 public_vars.estimated_pose = public_vars.mu'; % Nové předání výstupu z EKF
% 
% % 12. Path planning
 public_vars.path = plan_path(read_only_vars, public_vars);
% 
% % 13. Plan next motion command
 public_vars = plan_motion(read_only_vars, public_vars);



end
