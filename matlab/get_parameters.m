function [lambda2, omega2, eta1, eta2, R, D] = get_parameters()
    % Measure lambda2 from the mean (theta_dot/theta) for test_transient
    lambda2 = 0.0389^2;
    % Measure eta1 and eta2 as the mean (phi/PWM) with test_speed
    eta1 = 15;
    eta2 = 15;
    % Measure R and D manually
    R = 50e-3 / 2;
    D = 116e-3 / 2;
    % Measure from test_oscillatory, by making k large and measuring the
    % frequency of oscillation, then omega2 ~= (omega2_measured/K)
    omega2 = 0.1;
end