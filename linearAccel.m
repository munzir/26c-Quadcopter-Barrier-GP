function [xdd, ydd, zdd] = linearAccel(state, R, g, m, Ftotal, kd, wind_comp, model_wind)
    % Compute accelerations
    gz = [0 0 g];
    Fz = [0 0 -Ftotal];

    %         % Unmodeled Dynamics
    dx = getXdot(state);    dy = getYdot(state);    dz = getZdot(state);
    P = eye(3);     % Projection matrix
    P(3,3) = 0;
    D = -0.25*(kd/m)*R*P*R'*[dx dy dz]';
    D = 0;

    if model_wind == 1
        lin_accel = gz' + (1/m)*R*Fz' + D + (1/m)*wind_comp';
    else
        lin_accel = gz' + (1/m)*R*Fz' + D;
    end

    xdd = lin_accel(1) ;
    ydd = lin_accel(2) ;
    zdd = lin_accel(3) ;
end