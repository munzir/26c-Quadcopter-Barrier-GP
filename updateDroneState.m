function [state, obs] = updateDroneState(state, R, I, tau, g, m, F, kd, wind_comp, dt, unmodel_dynamics, model_wind)
    p = getP(state);        q = getQ(state);        r = getR(state);
    
    % Compute orientation angle derivaties
    [phi_dot, theta_dot, psi_dot] = getEulerDeriv(state);
    
    % Compute body rate velocity derivatives
    [p_dot, q_dot, r_dot] = getBodyVelocityDot(state, R, tau, I, p, q, r, kd, unmodel_dynamics);
        
    % Compute positional double derivatives (accelerations)
    [ddx, ddy, ddz] = linearAccel(state, R, g, m, F, kd, wind_comp, model_wind);
    
    % Store observations
    obs = [ddx, ddy, ddz, p_dot, q_dot, r_dot];
    
    % Compute position derivatives (velocities)
    x_dot = getXdot(state);
    y_dot = getYdot(state);
    z_dot = getZdot(state);

    % Compute states' derivatives
    state_dot = [ x_dot      y_dot     z_dot ...
                phi_dot  theta_dot   psi_dot  ...
                    ddx        ddy       ddz  ...
                  p_dot      q_dot    r_dot];
    %
    state = state + state_dot*dt;
end