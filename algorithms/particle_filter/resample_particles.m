function [new_particles] = resample_particles(particles, weights)
    N = size(particles, 1);
    new_particles = zeros(N, 3);
    weights = weights / sum(weights);
    cumulative_weights = cumsum(weights);
    r = rand / N; 
    j = 1;

    for i = 1:N
        U = r + (i - 1) / N;
        while U > cumulative_weights(j)
            j = j + 1;
        end
        new_particles(i, :) = particles(j, :);
    end

  
    jitter_sigma = [0.02, 0.02, 0.01]; 
    noise = randn(N, 3) .* jitter_sigma; 
    new_particles = new_particles + noise;


    new_particles(:, 3) = mod(new_particles(:, 3) + pi, 2 * pi) - pi;
end