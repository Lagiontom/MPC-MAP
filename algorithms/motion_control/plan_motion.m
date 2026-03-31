function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Pure Pursuit according to the lecture kinematic model.

% nastaveni
v_max = 0.35;              % Rychlost
stop_tolerance = 0.20;     % Tolerance pro zastaveni u ciloveho bodu

% diferencialni podvozek parametry
d = read_only_vars.agent_drive.interwheel_dist; % Vzdalenost mezi koly
wheel_speed_max = read_only_vars.agent_drive.max_vel; % Maximalni rychlost kol
omega_max = 2 * wheel_speed_max / d; % Maximalni uhelova rychlost

% Current pose estimate and planned path
pose = public_vars.estimated_pose; 
path = public_vars.path; 



%  look-ahead bod na trase
target = get_target(pose, path);

% transformace do lokalniho souradnicoveho systemu robota
dx = target(1) - pose(1);
dy = target(2) - pose(2);
x_G =  cos(pose(3)) * dx + sin(pose(3)) * dy;
y_G = -sin(pose(3)) * dx + cos(pose(3)) * dy;
l = max(hypot(x_G, y_G), 1e-3);

%   R = l^2 / (2*y_G), omega = v / R
if abs(y_G) < 1e-6
    v = v_max;
    omega = 0;
else
    R = l^2 / (2 * y_G);
    v = v_max;
    omega = v / R;

    if abs(omega) > omega_max
        omega = omega_max * sign(omega);
        v = min(v_max, abs(R) * omega_max);
    end
end

%  inverzní kinematika [v, omega] -> [v_R, v_L]
v_right = v + omega * d / 2;
v_left = v - omega * d / 2;

scale = max(1, max(abs([v_right, v_left])) / wheel_speed_max);
public_vars.motion_vector = [v_right, v_left] / scale;%Zapsani do vektoru
end