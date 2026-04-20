function [mu_bar, Sigma_bar] = ekf_predict(mu, Sigma, u, R, dt, L)

    vR = u(1);
    vL = u(2);
    v = (vR + vL) / 2;
    omega = (vR - vL) / L;
    
    theta = mu(3);
    

    mu_bar = zeros(3,1);
    mu_bar(1) = mu(1) + cos(theta) * v * dt;
    mu_bar(2) = mu(2) + sin(theta) * v * dt;
    mu_bar(3) = mu(3) + omega * dt;
    

    G = eye(3);
    G(1,3) = -sin(theta) * v * dt;
    G(2,3) = cos(theta) * v * dt;
    

    Sigma_bar = G * Sigma * G' + R;
end