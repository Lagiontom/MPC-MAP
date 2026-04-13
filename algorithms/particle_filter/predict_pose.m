function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)

x = old_pose(1);
y = old_pose(2);
theta = old_pose(3);


dt = read_only_vars.sampling_period;
v_l = motion_vector(2);  
v_r = motion_vector(1);  
L = read_only_vars.agent_drive.interwheel_dist;  

% uhlova rychlost
v = (v_r + v_l) / 2;      
w = (v_r - v_l) / L;   


% pridani sumu
sigma_v = 0.04 * abs(v);      
sigma_w = 0.04 * abs(w);      
sigma_theta = 0.04;           

v_noisy = v + sigma_v * randn;     
w_noisy = w + sigma_w * randn;      

% pridani kinematiky
if abs(w_noisy) > 1e-3      
        delta_theta = w_noisy * dt;     
    x_new = x + (v_noisy / w_noisy) * (sin(theta + delta_theta) - sin(theta));
    y_new = y + (v_noisy / w_noisy) * (cos(theta) - cos(theta + delta_theta));
else
    x_new = x + v_noisy * dt * cos(theta);
    y_new = y + v_noisy * dt * sin(theta);
end


theta_new = theta + w_noisy * dt + sigma_theta * randn;  

theta_new = mod(theta_new + pi, 2 * pi) - pi;

new_pose = [x_new, y_new, theta_new];

end
