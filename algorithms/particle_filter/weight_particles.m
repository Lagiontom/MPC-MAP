function [weights] = weight_particles(particle_measurements, lidar_distances)
    N = size(particle_measurements, 1);
    weights = zeros(N, 1);
    sigma = 1.0;  
    max_range = 30; % Vzdálenost větší než mapa
    

    real_meas = lidar_distances;
    real_meas(~isfinite(real_meas)) = max_range;
    
    for i = 1:N
        pred_meas = particle_measurements(i, :);
        

        pred_meas(~isfinite(pred_meas)) = max_range;
        

        diff = pred_meas - real_meas;
        weights(i) = exp(-mean(diff.^2) / (2 * sigma^2));
    end
    
    if sum(weights) == 0 || isnan(sum(weights))
        weights = ones(N, 1) / N;
    else
        weights = weights / sum(weights);
    end
end