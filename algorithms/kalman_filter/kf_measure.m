function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)
%KF_MEASURE Summary of this function goes here


    C = kf.C;
    Q = kf.Q;


    K = sigma * C' / (C * sigma * C' + Q);
    

    new_mu = mu + K * (z - C * mu);
    

    I = eye(size(sigma, 1));
    new_sigma = (I - K * C) * sigma;
end
