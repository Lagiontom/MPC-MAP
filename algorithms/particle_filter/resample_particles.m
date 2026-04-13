function [new_particles] = resample_particles(particles, weights)

N = size(particles, 1);


sigma = [0.1, 0.1, 0.05];  
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


noise = randn(N, 3) .* sigma; 
new_particles = new_particles + noise;

new_particles(:, 3) = wrapToPi(new_particles(:, 3));

end

