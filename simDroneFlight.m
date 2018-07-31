clear; clc; 
close all;

pause on;
% close all;

%% Setup
L  = 0.2;   % rotor-to-rotor distance
l  = L/(2*sqrt(2));
dataPath = '/home/mouhyemen/desktop/research/safeLearning/data/';
load( strcat(dataPath, 'stateHistory.mat' ) );
load( strcat(dataPath, 'refTrajectory.mat' ) );

%%
x_ref = refTrajectory(:,1);
y_ref = refTrajectory(:,2);
z_ref = refTrajectory(:,3);

% legend('Reference');
%%
% subplot(2,2,1);
figure;
% pause(5);
x_hist = stateHistory(:,1) ;
y_hist = stateHistory(:,2) ;
z_hist = stateHistory(:,3) ;

step = 1;
for i=1:step:length(stateHistory)
    x = stateHistory(i,1);
    y = stateHistory(i,2);
    z = stateHistory(i,3);
    
    phi   = stateHistory(i,4);
    theta = stateHistory(i,5);
    psi   = stateHistory(i,6);
    
    x1 = [x-l  x+l];
    y1 = [y-l  y+l];
    
    x2 = [x-l  x+l];
    y2 = [y+l  y-l];
    
    z1 = [z z];
    
    % Vertices matrix
    V1 = [x1(:) y1(:) z1(:)];
    V2 = [x2(:) y2(:) z1(:)];
    [Vr1, Vr2] = getRotVertex(V1, V2, phi, theta, psi);
    
    plot3(x_ref, y_ref, z_ref, 'r-');
    grid on;
    hold on;
    plot3(x_hist(1:i), y_hist(1:i), z_hist(1:i), 'b--');
    
    plot3(Vr1(:,1), Vr1(:,2), Vr1(:,3), 'k.-', 'MarkerSize', 10);   %Rotated around centre of line
    plot3(Vr2(:,1), Vr2(:,2), Vr2(:,3), 'k.-', 'MarkerSize', 10);   %Rotated around centre of line
    
    title('Flight Path','Interpreter','Latex');
    xlabel('X [meters]','Interpreter','Latex');
    ylabel('Y [meters]','Interpreter','Latex');
    zlabel('Z [meters]','Interpreter','Latex');
    limit = 0.25;
    axis([  min( min(x_ref), min(x_hist) )-limit    max( max(x_ref), max(x_hist) )+limit ...
            min( min(y_ref), min(y_hist) )-limit    max( max(y_ref), max(y_hist) )+limit ...
            min( min(z_ref), min(z_hist) )-limit    max( max(z_ref), max(z_hist) )+limit ...   
            ]);
    
    view(30,30)
    
    pause(0.09);
    hold off;
end
disp('Drone flight completed');