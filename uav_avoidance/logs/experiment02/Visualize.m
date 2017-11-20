function Visualize()
%VISUALIZE Visualizes the experiment 02

% Loading the log files
data=load('uav2.txt');

% Extracting the number of measurements
mess = size(data,1);

measurement = data(:,4:6)';
x = measurement(1,:);
y = measurement(2,:);
z = measurement(3,:);

dt_ = 0.1;
window = 10;
step = window/dt_;

reference_z = [3.5 6.0];
reference_x = [0.0 5.0];

close all
for t = (step+1 + 3000):200:length(x)    
    % Extracting the diferentials
    dx = x(t-step:t);
    dy = y(t-step:t);
    dz = z(t-step:t);
    dt = dt_*[t-step:t];

    % Representation of z with respect to the x coordinate
    subplot(2,2,1)
    plot(dx,dz)
    axis square
    xlim([-7 7])
    ylim([0 7])
    xlabel('position x[m]')
    ylabel('altitude z[m]')
    
    % Representation of z with respect of the time
    subplot(2,2,3)
    plot(dt, dz)
    axis square
    ylim([0 7])
    xlabel('time t[s]')
    ylabel('altitude z[m]')
    
    % Representation of x with respect to the y coordinate
    subplot(2,2,2)
    plot(dx,dy)
    axis square
    xlim(reference_x + 2*[-1 +1])
    ylim(max((reference_x) + 1)*[-1 1])
    xlabel('position x[m]')
    ylabel('position y[m]')
    
    % Representation of x with respect of the time
    subplot(2,2,4)
    plot(dt, dx)
    axis square
    ylim(reference_x + 2*[-1 +1])
    xlabel('time t[s]')
    ylabel('position x[m]')
    
    pause(dt_)
end

figure 
plot(dt_*[1:length(x)],x ...
    ,dt_*[1:length(x)],reference_x(1)*[ones(size(x))]...
    ,dt_*[1:length(x)],reference_x(2)*[ones(size(x))])
title('X breacking')

figure 
plot(dt_*[1:length(z)],z ...
    ,dt_*[1:length(z)],reference_z(1)*[ones(size(z))]...
    ,dt_*[1:length(z)],reference_z(2)*[ones(size(z))])
title('Z breacking')

% 1.25m de frenada
 

end

