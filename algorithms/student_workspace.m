function [public_vars] = student_workspace(read_only_vars, public_vars)
% algorithms/student_workspace.m

% persistent data_collected lidar_data gnss_data sample_idx
% 
% % inicializace pro sber 100 vzorku
% if isempty(data_collected) || read_only_vars.counter == 1
%     data_collected = false;
%     lidar_data = zeros(100, 8);
%     gnss_data = zeros(100, 2);
%     sample_idx = 1;
% end
% 
% % sber dat dokud nemame 100 iteraci
% if ~data_collected
% 
%     % oprava: nacteni vsech 8 kanalu lidaru misto jen jednoho
%     lidar_data(sample_idx, :) = read_only_vars.lidar_distances(1:8); 
% 
%     % nacteni obou os gnss
%     gnss_data(sample_idx, :) = [read_only_vars.gnss_position(1,1), read_only_vars.gnss_position(1,2)];
% 
%     sample_idx = sample_idx + 1;
% 
%     % vyhodnoceni po nasbirani 100 vzorku
%     % vyhodnoceni po nasbirani 100 vzorku
%     % vyhodnoceni po nasbirani 100 vzorku
%     if sample_idx > 100
%         data_collected = true;
% 
%         % --- Task 2: Sensor uncertainty ---
%         sigma_lidar = StandardDev(lidar_data);
%         sigma_gnss = StandardDev(gnss_data);
% 
%         figure('Name', 'Task 2: Histograms');
% 
%         % rozlozeni 8 kanalu lidaru do samostatnych grafu (mrizka 3x3)
%         for i = 1:8
%             subplot(3, 3, i);
%             histogram(lidar_data(:, i), 'FaceColor', '#0072BD', 'EdgeColor', 'w');
%             title(['LiDAR ch ', num2str(i)]);
%             grid on;
%         end
% 
%         % gnss data (osa x i y) v jednom spolecnem grafu na 9. pozici
%         subplot(3, 3, 9);
%         hold on;
%         histogram(gnss_data(:, 1), 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
%         histogram(gnss_data(:, 2), 'FaceColor', 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
%         title('GNSS X a Y');
%         legend('X', 'Y', 'Location', 'best');
%         grid on;
%         hold off;
% 
%         % --- Task 3: Covariance matrix ---
%         cov_lidar = cov(lidar_data)
%         cov_gnss = cov(gnss_data)
% 
% 
%         % --- Task 4: Normal distribution ---
%         x_vals = linspace(-0.2, 0.2, 1000); 
%         barvy = lines(8);
% 
%         figure('Name', 'Task 4: Sensor Noise PDF');
% 
%         % pdf pro lidar
%         subplot(2, 1, 1);
%         hold on;
%         for i = 1:8
%             pdf_lidar = norm_pdf(x_vals, 0, sigma_lidar(i));
%             plot(x_vals, pdf_lidar, 'Color', barvy(i,:), 'LineWidth', 1.5);
%         end
%         grid on;
%         legend('Ch 1', 'Ch 2', 'Ch 3', 'Ch 4', 'Ch 5', 'Ch 6', 'Ch 7', 'Ch 8', 'Location', 'eastoutside');
%         title('Hustota pravdepodobnosti (PDF) - LiDAR');
%         hold off;
% 
%         % pdf pro gnss
%         subplot(2, 1, 2);
%         hold on;
%         pdf_gnss_x = norm_pdf(x_vals, 0, sigma_gnss(1));
%         pdf_gnss_y = norm_pdf(x_vals, 0, sigma_gnss(2));
%         plot(x_vals, pdf_gnss_x, 'b', 'LineWidth', 1.5);
%         plot(x_vals, pdf_gnss_y, 'g', 'LineWidth', 1.5);
%         grid on;
%         legend('GNSS X', 'GNSS Y', 'Location', 'eastoutside');
%         title('Hustota pravdepodobnosti (PDF) - GNSS');
%         hold off;
%     end
% end

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);
end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

end