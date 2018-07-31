function [w1, w2, w3, w4] = setAngularVelocities(...
                                kAlpha, u1, pd_cmd, qd_cmd, rd_cmd, ...
                                state, I ,     kf,     km, l)
    Ix = I(1);  Iy = I(2);  Iz = I(3);
    
    p = getP(state);    q = getQ(state);    r = getR(state);
    
    F_  = u1/ kf;
    
    if isempty(kAlpha)
        pd_pred = pd_cmd;
        qd_pred = qd_cmd;
        rd_pred = rd_cmd;
    else
        pd_pred = pd_cmd - kAlpha(4);
        qd_pred = qd_cmd - kAlpha(5);
        rd_pred = rd_cmd - kAlpha(6);
    end
    
    u2  = pd_pred*Ix + (Iz-Iy)*q*r;
    Mx_ = u2/(kf*l);
    
    u3  = qd_pred*Iy + (Ix-Iz)*p*r;
    My_ = u3/(kf*l);
    

    u4  = rd_pred*Iz + (Iy-Ix)*p*q;
    Mz_ = u4/km;
    
    w1 = (1/4)*(F_ + Mx_ + My_ + Mz_);
    w2 = (1/4)*(F_ - Mx_ + My_ - Mz_);
    w3 = (1/4)*(F_ - Mx_ - My_ + Mz_);
    w4 = (1/4)*(F_ + Mx_ - My_ - Mz_);
    
    w1 = -sqrt(w1); % since Tau1 (ccw) +ve
    w2 =  sqrt(w2);
    w3 = -sqrt(w3);
    w4 =  sqrt(w4);
end

