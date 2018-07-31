function [p_dot, q_dot, r_dot] = getBodyVelocityDot(state, R, tau, I, p, q, r, kd, unmodel_dynamics)
    tx = tau(1);    ty = tau(2);    tz = tau(3);
    Ix = I(1);      Iy = I(2);      Iz = I(3);

    % Compute body velocity derivatives w/o denominator
    p_ = tx - (Iz-Iy) * q * r ;
    q_ = ty - (Ix-Iz) * p * r ;
    r_ = tz - (Iy-Ix) * q * p ;
    
    % Unmodeled dynamics
    %unmodel_dynamics = 0;
    if unmodel_dynamics == 1
        dx = getXdot(state);    dy = getYdot(state);    dz = getZdot(state);
        P = eye(3); P(3,3) = 0;     % Projection matrix
        B = 2*kd*R*P*R'*[dx dy dz]';
        
        p_dot = p_/Ix + B(1);
        q_dot = q_/Iy + B(2);
        r_dot = r_/Iz + B(3);
    else
        p_dot = p_/Ix;
        q_dot = q_/Iy;
        r_dot = r_/Iz;
    end

end