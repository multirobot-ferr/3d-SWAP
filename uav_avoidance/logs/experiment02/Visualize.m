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

dt = 0.1;
window = 10;
step = window/dt;

close all
for t = (step+1):length(x)
    subplot(2,2,1)
    plot3(x(t-step:t),y(t-step:t),z(t-step:t))
    axis square
    xlim([-7 7])
    ylim([-7 7])
    zlim([0 7])
    
    subplot(2,2,2)
    plot(x(t-step:t),z(t-step:t))
    axis square
    xlim([-7 7])
    ylim([0 7])
    xlabel('Altitude')
    
    subplot(2,2,4)
    plot(dt*[t-step:t],z(t-step:t))
    axis square
    ylim([0 7])
    xlabel('Altitude')
    
    pause(dt)
end

 
% % Ploting everything
% close all
% axis square
% plot(measurements(1,:), measurements(2,:), '.r', ...
%      measurements_no_out(1,:), measurements_no_out(2,:), '.b', ...
%      p_mean(1), p_mean(2), 'or', ...
%      e1x, e1y, '-r', ...
%      p_mean_no_out(1), p_mean_no_out(2), 'ob', ...
%      e2x, e2y, '-b' ...
%      )
% legend('Measurements Discarted', 'Measurements', 'Mean', 'Max error mean', 'Mean no outlayers','Max error mean no out')
%  
% xlabel(['Mean error ' num2str(max(errors)) ', Mean no outlayers error ' num2str(max(errors2))])
% ylabel(['Measurements ' num2str(mess)])
% disp(['Mean at (' num2str(p_mean) '), error in mean ' num2str(max(errors))])
% disp(['Mean at (' num2str(p_mean) '), error in mean no outlayers ' num2str(max(errors2))])
 
end

