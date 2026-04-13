function [weights] = weight_particles(particle_measurements, lidar_distances)

N = size(particle_measurements, 1);
weights = ones(N, 1) / N;
sigma = 1.0;  


for i = 1:N
    predicted_measurements = particle_measurements(i, :);
    
    diff = predicted_measurements - lidar_distances;

    weights(i) = exp(-sum(diff.^2) / (2 * sigma^2));
end

weights = weights / sum(weights);

end



