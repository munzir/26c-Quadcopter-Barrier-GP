clc; 
clear;
close all;
% clf;

%% Initial Setup
mode = 4;
while (mode ~= 0) && (mode ~= 1)
    mode = input('Training: 0   |   Testing: 1   |   Exit: 2    =   ' );
    if mode == 0
        type = 'train';
        add  = 'training/';
    elseif mode == 1
        type = 'test';
        add  = 'testing/';
    elseif mode == 2
        return  % to terminate the script
    end
    dataPath = '/home/mouhyemen/desktop/research/safeLearning/data/';
end
fileNumber = input('File Number #:  ', 's');
plot_flag   = 1;
use_QP_u1   = 0;
use_QP_txy  = 0;
unmodel_dynamics = 1;
model_wind  = 1;

save( strcat(dataPath,'modes'), 'use_QP_u1', 'use_QP_txy', 'unmodel_dynamics');

%% Drone characteristics
tic
fprintf('Setting up drone, PID, & barrier parameters ... ');

kf = 1.0;           % constant relating force and rotor speed
km = 1.0;           % constant relating torque and rotor speed
kd = 5.0;          % drag constant

m  = 1.40;          % mass of quad in kg
g  = 9.81;          % gravity

L  = 0.25;          % rotor-to-rotor distance
l  = L/(2*sqrt(2));

Ix = 0.1;           % inertia along x-axis
Iy = 0.1;           % inertia along y-axis
Iz = 0.2;           % inertia along z-axis

%% Wind disturbance
scale = 3;

% Arbitrary wind position and direction
% wind_pos    = rand(1, 3);
% wind_comp   = scale*rand(1, 3);
% wind_comp(3) = wind_comp(3) + 1;
% flag = randi(19);
% if flag >= 15
%     wu = wind_comp(1); wv = wind_comp(2); ww = wind_comp(3);
%     wind_comp = [wu wv ww];
% elseif flag >= 10
%     wu = -wind_comp(1); wv = wind_comp(2); ww = wind_comp(3);
%     wind_comp = [wu wv ww];
% elseif flag >= 5
%     wu = wind_comp(1); wv = -wind_comp(2); ww = wind_comp(3);
%     wind_comp = [wu wv ww];
% elseif flag >= 0
%     wu = wind_comp(1); wv = wind_comp(2); ww = -wind_comp(3);
%     wind_comp = [wu wv ww];
% end

% Fixed wind position & direction
wind_pos    = [0.11091      0.99649    0.0053003];
wind_comp   = scale*[2.2576      3.3518       3.1059];

wu = wind_comp(1);
wv = wind_comp(2);
ww = wind_comp(3);

wind_x = wind_pos(1);
wind_y = wind_pos(2);
wind_z = wind_pos(3);

% quiver3(wind_x, wind_y, wind_z, ...
%             wu,     wv,     ww, ...
%        'Color',  [0.25 0.75 0], ...
%    'LineWidth',  3.0, 'MaxHeadSize', 1.5, 'MarkerSize', 30) ; %, 'MaxHeadSize', 10.5);
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% view(70,15);
% hold on;
save( strcat(dataPath, 'droneParams'), 'm', 'g', 'Ix', 'Iy', 'Iz', 'wind_pos', 'wind_comp');

%% PID parameters for each controller
% x, y, z kp and kd values
kp_xyz = [30.0, 30.0, 30.0];   % x, y, z kp values
kd_xyz = [5.0, 5.0, 5.0];   % x, y, z, kd values (2nd order)

kp_roll_pitch = [20.0, 20.0]; % roll, pitch kp values (1st order)
kp_yaw = 5.0;

kp_pqr = [20.0 20.0 20.0];  % p, q, r kp values (1st order)

%% Barrier Parameters
px = 2;     % ellipse pos axis along x-axis
py = 2;     % ellipse pos axis along y-axis
pz = 2;   % ellipse pos axis along z-axis

vx = 4;   % ellipse vel axis along x
vy = 0.7;   % ellipse vel axis along y-axis
vz = 0.5;   % ellipse vel axis along z-axis

cz  = 2;     % elliposidal center for z-axis
cx  = 0;     % elliposidal center for x-axis
alpha2 = 100;
alpha1 = 10;
alpha0 = 10000;
alphaz = 10000;

%% Barrier Conditions
if use_QP_u1 == 0 && use_QP_txy == 0
    fprintf('\n ------------ NOT USING QP SOLVER AT ALL -------------------- \n\n');
elseif use_QP_u1 == 1 && use_QP_txy == 0
    fprintf('\n ------------ USING QP SOLVER FOR U1 ------------------------ \n\n');
elseif use_QP_txy == 1 && use_QP_u1 == 0
    fprintf('\n ------------ USING QP SOLVER FOR Tx & Ty ------------------- \n\n');
elseif use_QP_txy == 1 && use_QP_u1 == 1
    fprintf('\n ------------ USING QP SOLVER FOR BOTH ---------------------- \n\n');
end

%% Reference Trajectory (Flight plan)
trajectory = load(strcat(dataPath, type, 'Trajectory.mat'));

numTrajectories = numel(fieldnames(trajectory));
trajectory = struct2cell(trajectory);

total_time = 20; % 20 seconds
dtScale = 1;

%% DRONE FLIGHT
%% Drone Flight for each trajectory
totalDataPoints = length(trajectory{1})*numTrajectories;
fprintf('Total Trajectory Points: %d\n', totalDataPoints);
for k = 1:numTrajectories
    tic
    fprintf('Setting up reference for trajectory:   %2d     |   ', k);
    
    x_ref = trajectory{k}(1,:);
    y_ref = trajectory{k}(2,:);
    z_ref = trajectory{k}(3,:);
    
    xd_ref = trajectory{k}(4,:);
    yd_ref = trajectory{k}(5,:);
    zd_ref = trajectory{k}(6,:);
    
    xdd_ref = trajectory{k}(7,:);
    ydd_ref = trajectory{k}(8,:);
    zdd_ref = trajectory{k}(9,:);
    
    dt      = dtScale*total_time/length(x_ref);
    time    = linspace(0, total_time, floor(total_time/dt) );
    
    psi_ref = atan2(yd_ref, xd_ref);
    
    %% Reference and Drone States description
    refTrajectory = [x_ref; y_ref; z_ref; xd_ref; yd_ref; zd_ref; ...
        xdd_ref; ydd_ref; zdd_ref; psi_ref]';
    save( strcat( dataPath, 'refTrajectory'), 'refTrajectory');
    droneState = [ x_ref(1)   y_ref(1)       z_ref(1)  ...
        0.0        0.0         psi_ref(1)  ...
        xd_ref(1)  yd_ref(1)      zd_ref(1)  ...
        0.0        0.0            0.0       ] ;
    toc
    
    %% Drone Flight
    
    fprintf('Begin drone flight for trajectory:     %2d     |   ', k);
    inner_loop = 10;
    totTime = 0;
    idx = 1;
    for i=1:length(x_ref)
        %     i
        tStart = tic;
        %% Hover (altitude) controller
        [u1, zdd_cmd] = zController(z_ref(i), zd_ref(i), zdd_ref(i), droneState, kp_xyz, kd_xyz, m);
        
        %% Position (XY) controller
        x_info = [x_ref(i) xd_ref(i) xdd_ref(i)];
        y_info = [y_ref(i) yd_ref(i) ydd_ref(i)];
        [xdd_cmd, ydd_cmd] = xyController(x_info, y_info, droneState, kp_xyz, kd_xyz);
        
        %% Do not use this control scheme if QP applied only for tx and ty
        if use_QP_u1 == 1 || use_QP_txy == 0
            u1 = m*sqrt(xdd_cmd^2 + ydd_cmd^2 + (zdd_cmd-g)^2);
        end
        
        %% Barrier certificate on z-component
        if use_QP_u1 == 1
            H = 1;
            f = -u1;
            
            x  = getX(droneState);      y   = getY(droneState);     z   = getZ(droneState);
            dx = getXdot(droneState);   dy  = getYdot(droneState);  dz  = getZdot(droneState);
            dx_hist(i) = dx;
            dy_hist(i) = dy;
            [R13, R23, R33, ~] = getRotCol3(droneState);
            
            %%         % Single barrier on Z-VELOCITY
            h = 1 - (dz/vz)^2 ;
            A = -2*dz*R33/(vz^2*m);
            b = alphaz*h - 2*dz*g/vz^2;
            % ______________________________________________________________
            
            %%         % Single barrier on Z-POSITION
            %         h = 1 - ((z-cz)/pz)^2;
            %         dh = -2*(z-cz)*dz/pz^2;
            %
            %         A = -2*(z-cz)*R33/(m*pz^2) ;
            %         b = alpha0*h + alpha1*dh - 2*(z-cz)*g/pz^2 - 2*dz^2/pz^2;
            % ______________________________________________________________
            
            %%         % Single barrier on Z-POSITION & Z-VELOCITY
            %         h = 1 - ((z-cz)/pz)^2 - (dz/vz)^2;
            %         A = -2*dz*R33/(m*vz^2);
            %         b = alpha*h - 2*dz*g/vz^2 - 2*dz*(z-cz)/pz^2;
            % ______________________________________________________________
            
            %% QP-solver on u1
            uBound = 2*g;
            lBound = 0;
            
            options = optimset('display', 'off');
            u1_QP = quadprog(H, f, A, b, [], [], lBound, uBound, [], options);
            
            if ~isempty(u1_QP)
                u1_QP_hist(i) = u1_QP;
                u1 = u1_QP;
            end
        end
        
        %% Attitude controller
        for j=1:inner_loop
            %% Get updated orientation angles and body velocity rates
            x  = getX(droneState);      y   = getY(droneState);     z   = getZ(droneState);
            dx = getXdot(droneState);   dy  = getYdot(droneState);  dz  = getZdot(droneState);
            [R13, R23, R33, R] = getRotCol3(droneState);
            R11 = R(1,1);   R12 = R(1,2);
            R21 = R(2,1);   R22 = R(2,2);
            phi = getPhi(droneState);   theta = getTheta(droneState);   psi = getPsi(droneState);
            p = getP(droneState);       q = getQ(droneState);           r = getR(droneState);
            
            %% Controllers
            % implementing roll-pitch and yaw controllers
            [p_cmd, q_cmd] = rollPitchController(...
                u1, xdd_cmd, ydd_cmd, phi, theta, psi, kp_roll_pitch, m);
            r_cmd = yawController(psi_ref(i), psi, kp_yaw);
            
            % implementing pqr (body-rate) controller
            [pd_cmd, qd_cmd, rd_cmd] = pqrController(p_cmd, q_cmd, r_cmd, p, q, r, kp_pqr);
            
            %% Set propeller speeds
            % Set angular velocities on each propeller
            I = [Ix Iy Iz];
            [w1, w2, w3, w4] = setAngularVelocities([], u1, pd_cmd, qd_cmd, rd_cmd, droneState, I, kf, km, l);
            w_total = w1 + w2 + w3 + w4;
            
            %% Compute forces and torque moments
            % Forces
            f1 = kf*w1^2;   f2 = kf*w2^2;   f3 = kf*w3^2;   f4 = kf*w4^2;
            Ftotal = f1 + f2 + f3 + f4;
            
            % Torques/Moments
            t1 =  km*w1^2;  t2 = -km*w2^2;  t3 =  km*w3^2;  t4 = -km*w4^2;
            
            tx = (f1 + f4 - f2 - f3)*l;
            ty = (f1 + f2 - f3 - f4)*l;
            tz = t1 + t2 + t3 + t4;
            tau = [tx ty tz];
            
            %% Implement barriers on Tau-x and Tau-y
            if use_QP_txy == 1
                H = eye(2);
                f = -[tx ty]';
                
                [R13dot, R23dot] = getRotCol3Deriv(R, p, q);
                dddx = -u1*R13dot/m;     % third derivative of x
                dddy = -u1*R23dot/m;     % third derivative of y
                
                ddx  = -u1*R13/m;       % second derivative of x
                ddy  = -u1*R23/m;       % second derivative of y
                
                % Derivative of rot matrix R_ZYX elements
                [R11dot, R12dot, R21dot, R22dot, R33dot] = getRotDerivative(droneState);
                W       = [ R21     -R11 ;  R22     -R12 ];
                Wdot    = [R21dot -R11dot; R22dot -R12dot];
                V       = pinv(W);
                Vdot    = get2x2MatrixDeriv(W, Wdot);
                % inertial terms for pdot and qdot
                dp_term1 = -q*r*(Iz-Iy) ;
                dq_term1 = -p*r*(Ix-Iz) ;
                
                % For simplified expressions
                Jx = -6*ddx*dddx/vx^2 ;
                Jy = -6*ddy*dddy/vy^2 ;
                u_by_m = u1/m ;
                
                L  = u_by_m*R33*V;
                K  = -u_by_m*R33dot*V*[p q]' - u_by_m*R33*Vdot*[p q]' - L*[dp_term1/Ix dq_term1/Iy]';
                Kx = K(1);
                Ky = K(2);
                
                Ixy = [1/Ix 0 ; 0 1/Iy] ;
                gamma = [2*dx/vx^2       2*dy/vy^2];
                
                %% Single barrier on X-VELOCITY & Y-VELOCITY
                h    = 1 - dx^2/vx^2 - dy^2/vy^2;
                dh   = -(2*ddx*dx)/vx^2 - (2*ddy*dy)/vy^2;
                ddh  = -(2*ddx^2)/vx^2 - (2*ddy^2)/vy^2 - (2*dddx*dx)/vx^2 - (2*dddy*dy)/vy^2;
                
                % Simplifying expressions
                beta = alpha2*ddh + alpha1*dh + alpha0*h ;
                
                A = -gamma* L * Ixy ;
                b = beta - gamma*K - Jx - Jy;
                
                % QP solver
                lBound = [-5.0 -5.0];
                uBound = [ 5.0  5.0];
                
                options = optimset('display', 'off');
                tau_QP = quadprog(H, f, A, b, [], [], lBound, uBound, [], options);
                tx_QP = tau_QP(1);
                ty_QP = tau_QP(2);
                
                
            end
            
            %% Update drone states
            [droneState, obs] = updateDroneState(droneState+eps,         R,                 I, ...
                                                            tau,         g,                 m, ... 
                                                         Ftotal,        kd,         wind_comp, ...
                                                  dt/inner_loop, unmodel_dynamics, model_wind   );
            
        end
        
        %% Save history information
        if mod(i,2) == 0
            xref_hist(idx)        = x_ref(i);
            yref_hist(idx)        = y_ref(i);
            zref_hist(idx)        = z_ref(i);
            time_hist(idx)        = time(i);
            u1_hist(idx)          = u1;
            F_hist(idx,1)         = Ftotal;
            if use_QP_txy == 1
                tau                 = [tx_QP ty_QP tz];
                tau_hist_QP(idx,:)  = tau;
            elseif use_QP_txy == 0
                tau                 = [tx ty tz];
                tau_hist(idx,:)     = tau;
            end
            R13_hist(idx,1)       = R13;
            R23_hist(idx,1)       = R23;
            R33_hist(idx,1)       = R33;
            stateHistory(idx,:)   = droneState;
            observations(idx,:)   = obs;
            idx = idx + 1;
        end
        
        %% Time information
        timeElapsed         = toc(tStart);
        totTime             = totTime + timeElapsed;
    end
    avgTime = totTime/length(time);
    fprintf('Time/iteration = %4.3f milliseconds.\n', avgTime*1000);
    
    %% Save barrier/no-barrier states based on conditions
    % EXAMPLE USAGE: save filename variable_name
    if use_QP_u1 == 0 && use_QP_txy == 0
        x_noBarrier     = stateHistory(:,1);
        y_noBarrier     = stateHistory(:,2);
        z_noBarrier     = stateHistory(:,3);
        xdot_noBarrier  = stateHistory(:,7);
        ydot_noBarrier  = stateHistory(:,8);
        zdot_noBarrier  = stateHistory(:,9);
        state_noBarrier = [  x_noBarrier    y_noBarrier     z_noBarrier ...
            xdot_noBarrier ydot_noBarrier zdot_noBarrier ];
        
        save(strcat(dataPath, 'state_noBarrier'), 'state_noBarrier');
    else
        x_wBarrier      = stateHistory(:,1);
        y_wBarrier      = stateHistory(:,2);
        z_wBarrier      = stateHistory(:,3);
        xdot_wBarrier   = stateHistory(:,7);
        ydot_wBarrier   = stateHistory(:,8);
        zdot_wBarrier   = stateHistory(:,9);
        state_wBarrier  = [x_wBarrier y_wBarrier z_wBarrier xdot_wBarrier ydot_wBarrier zdot_wBarrier];
        save( strcat(dataPath, 'state_wBarrier'), 'state_wBarrier');
    end
    save( strcat( dataPath, 'stateHistory'), 'stateHistory' );
    
    %% Save training/testing data
    fieldname = strcat('inputs_', num2str(k));
    if use_QP_txy == 0
        S.(fieldname) = [stateHistory F_hist R13_hist R23_hist R33_hist tau_hist];
    else
        S.(fieldname) = [stateHistory F_hist R13_hist R23_hist R33_hist tau_hist_QP];
    end
    
    fieldname = strcat('observations_', num2str(k));
    S.(fieldname) = observations;
    
    if unmodel_dynamics == 1
        tail1 = 'EPS';
        if use_QP_u1 == 1 && use_QP_txy == 0
            tail2 = 'u1';
            name = strcat(dataPath, add, type, '_', fileNumber, '_', tail1, '_', tail2);
        elseif use_QP_txy == 1 && use_QP_u1 == 0
            tail = 'txy';
            name = strcat(dataPath, add, type, '_', fileNumber, '_', tail1, '_', tail2);
        elseif use_QP_txy == 1 && use_QP_u1 == 1
            tail = 'u1_txy';
            name = strcat(dataPath, add, type, '_', fileNumber, '_', tail1, '_', tail2);
        else
            name = strcat(dataPath, add, type, '_', fileNumber, '_', tail1);
        end
    else
        if use_QP_u1 == 1 && use_QP_txy == 0
            tail = 'u1';
            name = strcat(dataPath, add, type, '_', fileNumber, '_', tail);
        elseif use_QP_txy == 1 && use_QP_u1 == 0
            tail = 'txy';
            name = strcat(dataPath, add, type, '_', fileNumber, '_', tail);
        elseif use_QP_txy == 1 && use_QP_u1 == 1
            tail = 'u1_txy';
            name = strcat(dataPath, add, type, '_', fileNumber, '_', tail);
        else
            name = strcat(dataPath, add, type, '_', fileNumber);
        end
    end
    
    save(name, '-struct', 'S');    
    
    %% PLOTTING - Comparing Drone [x,y,z,yaw] against reference
    tic
    fprintf('Plot drone flight for trajectory:      %2d     |   ', k);
    
    %% Determine number of rows/columns based on conditions
    if use_QP_u1 == 1 || use_QP_txy == 1
        row = 3;    col = 3;
    elseif (use_QP_u1 == 0 || use_QP_txy == 0) && plot_flag == 1
        row = 2; col = 2;
    else
        row = 1; col = 1;
    end
    
    %% Plotting based on conditions
    if (plot_flag == 1)
        figure;
        %% Comparing drone's trajectory with reference trajectory
        subplot(row, col,1);
        
        x_flight = stateHistory(:,1)';
        y_flight = stateHistory(:,2)';
        z_flight = stateHistory(:,3)';
        
        plot3(x_ref, y_ref, z_ref, 'r'); grid on; hold on;
        plot3(x_flight, y_flight, z_flight, 'b');
        
        title('Flight Path','Interpreter','Latex');
        xlabel('X [meters]','Interpreter','Latex');
        ylabel('Y [meters]','Interpreter','Latex');
        zlabel('Z [meters]','Interpreter','Latex');
        limit = 4;
        axis([  min(x_ref)-limit/4    max(x_ref)+limit/4 ...
            min(y_ref)-limit/4    max(y_ref)+limit/4 ...
            min(z_ref)-limit    max(z_ref)+limit   ]);
        
        view(30,30);
        
        %% Comparing behavior with and without barriers
        
        if use_QP_u1 || use_QP_txy
            load state_noBarrier.mat
            load state_wBarrier.mat
            x_noBarrier     = state_noBarrier(:,1);
            y_noBarrier     = state_noBarrier(:,2);
            z_noBarrier     = state_noBarrier(:,3);
            xdot_noBarrier  = state_noBarrier(:,4);
            ydot_noBarrier  = state_noBarrier(:,5);
            zdot_noBarrier  = state_noBarrier(:,6);
            
            color = 'black';
            
            subplot(row, col, 4);
            plot(time, x_noBarrier, 'r', time, x_wBarrier, 'b'); hold on;
            line([min(time),max(time)],[max(x_noBarrier), max(x_noBarrier)], 'Color',color,'LineStyle','--')
            line([min(time),max(time)],[min(x_noBarrier), min(x_noBarrier)], 'Color',color,'LineStyle','--')
            xlabel('Time','Interpreter','Latex');
            ylabel('x(t)','Interpreter','Latex');
            title('x(t)');
            legend('Without Barrier' , ['With Barrier = ' num2str(px) ], ['Ref limit = ' num2str(max(x_noBarrier)) ]);
            grid on;
            
            
            subplot(row, col, 5);
            plot(time, y_noBarrier, 'r', time, y_wBarrier, 'b'); hold on;
            line([min(time),max(time)],[max(y_noBarrier), max(y_noBarrier)], 'Color',color,'LineStyle','--')
            line([min(time),max(time)],[min(y_noBarrier), min(y_noBarrier)], 'Color',color,'LineStyle','--')
            xlabel('Time','Interpreter','Latex');
            ylabel('y(t)','Interpreter','Latex');
            title('y(t)');
            legend('Without Barrier' , ['With Barrier = ' num2str(py) ], ['Ref limit = ' num2str(max(y_noBarrier)) ]);
            grid on;
            
            
            subplot(row, col, 6);
            plot(time, z_noBarrier, 'r', time, z_wBarrier, 'b'); hold on;
            line([min(time),max(time)],[max(z_noBarrier), max(z_noBarrier)], 'Color',color,'LineStyle','--')
            line([min(time),max(time)],[min(z_noBarrier), min(z_noBarrier)], 'Color',color,'LineStyle','--')
            xlabel('Time','Interpreter','Latex');
            ylabel('z(t)','Interpreter','Latex');
            title('z(t)');
            legend('Without Barrier' , ['With Barrier = ' num2str(pz) ], ['Ref limit = ' num2str(max(z_noBarrier)) ]);
            grid on;
            
            
            
            subplot(row, col, 7);
            plot(time, xdot_noBarrier, 'r', time, xdot_wBarrier, 'b'); hold on;
            line([min(time),max(time)],[max(xdot_noBarrier), max(xdot_noBarrier)], 'Color',color,'LineStyle','--')
            line([min(time),max(time)],[min(xdot_noBarrier), min(xdot_noBarrier)], 'Color',color,'LineStyle','--')
            xlabel('Time','Interpreter','Latex');
            ylabel('$\dot{x}(t)$','Interpreter','Latex');
            title('xdot(t)');
            legend('Without Barrier' , ['With Barrier = ' num2str(vx) ], ['Ref limit = ' num2str(max(xdot_noBarrier)) ]);
            grid on;
            
            
            subplot(row, col, 8);
            plot(time, ydot_noBarrier, 'r', time, ydot_wBarrier, 'b'); hold on;
            line([min(time),max(time)],[max(ydot_noBarrier), max(ydot_noBarrier)], 'Color',color,'LineStyle','--')
            line([min(time),max(time)],[min(ydot_noBarrier), min(ydot_noBarrier)], 'Color',color,'LineStyle','--')
            xlabel('Time','Interpreter','Latex');
            ylabel('$\dot{y}(t)$','Interpreter','Latex');
            title('ydot(t)');
            legend('Without Barrier' , ['With Barrier = ' num2str(vy) ], ['Ref limit = ' num2str(max(ydot_noBarrier)) ]);
            grid on;
            
            
            subplot(row, col, 9);
            plot(time, zdot_noBarrier, 'r', time, zdot_wBarrier, 'b'); hold on;
            line([min(time),max(time)],[max(zdot_noBarrier), max(zdot_noBarrier)], 'Color',color,'LineStyle','--')
            line([min(time),max(time)],[min(zdot_noBarrier), min(zdot_noBarrier)], 'Color',color,'LineStyle','--')
            xlabel('Time','Interpreter','Latex');
            ylabel('$\dot{z}(t)$','Interpreter','Latex');
            title('zdot(t)');
            legend('Without Barrier' , ['With Barrier = ' num2str(vz) ], ['Ref limit = ' num2str(max(zdot_noBarrier)) ]);
            grid on;
        end
        
        if use_QP_u1 == 0 && use_QP_txy == 0
            %% Comparing drone's heading with reference heading
            subplot(row, col,2);
            
            u = cos(psi_ref);
            v = sin(psi_ref);
            w = zeros(1,length(psi_ref));
            
            drone_u = cos(stateHistory(:,6))';
            drone_v = sin(stateHistory(:,6))';
            drone_w = zeros(1, length(stateHistory(:,6)));
            
            step = 2;
            quiver3(x_ref(1:step:end), y_ref(1:step:end), z_ref(1:step:end), ...
                u(1:step:end), v(1:step:end), w(1:step:end), 0.5, 'r');
            grid on; hold on;
            step = 1;
            quiver3(x_flight(1:step:end), y_flight(1:step:end), z_flight(1:step:end), ...
                drone_u(1:step:end), drone_v(1:step:end), drone_w(1:step:end), 0.5, 'Color', [0.25 0.75 0]);
            
            title('Drone heading','Interpreter','Latex');
            xlabel('X [meters]','Interpreter','Latex');
            ylabel('Y [meters]','Interpreter','Latex');
            zlabel('Z [meters]','Interpreter','Latex');
            %         legend('Reference', 'Actual');
            limit = 0.5;
            axis([  min(x_ref)-limit    max(x_ref)+limit ...
                min(y_ref)-limit    max(y_ref)+limit ...
                min(z_ref)-limit    max(z_ref)+limit   ]);
            
            view(30,30)
            
            %% Computing error in drone and reference trajectory
            subplot(row, col,3);
            
            err_x = (xref_hist - x_flight).^2;
            err_y = (yref_hist - y_flight).^2;
            err_z = (zref_hist - z_flight).^2;
            
            err_pos = sqrt(err_x + err_y + err_z);
            %         plot(time, err_pos, 'b'); grid on;
            plot(time_hist, err_x   , 'r', time_hist, err_z, 'b'); grid on; hold on;
            plot(time_hist, err_y   , 'Color', [0.25 0.75 0]);
            plot(time_hist, err_pos , 'Color', [1 0.75 0] );
            title('Error in drone flight','Interpreter','Latex');
            xlabel('Time (seconds)','Interpreter','Latex');
            ylabel('Error (meters) for X, Y, Z','Interpreter','Latex');
            limit = 0.1;
            
            %% Comparing drone yaw and reference yaw against time
            subplot(row, col,4)
            
            drone_psi = stateHistory(:,6)';
            plot(time, psi_ref, 'r');
            grid on; hold on;
            plot(time_hist, drone_psi, 'b');
            title('Yaw behavior','Interpreter','Latex');
            xlabel('Time (seconds)','Interpreter','Latex');
            ylabel('$\psi$ (radians)','Interpreter','Latex');
            legend('Reference', 'Actual');
        end
    end
    toc
    fprintf('-----------------------------------------------------------------------------------------------\n');
end

