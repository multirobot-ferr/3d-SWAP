function ComputePositionError()
%COMPUTEPOSITIONERROR This script computes the error in position of an UAV
%that remains static in a position

% Loading the log files
data=load('uav2.txt');

% Extracting the number of measurements
mess = size(data,1);

measurements = data(:,4:5)';

 
% Computing the simple mean
p_mean = [mean(measurements(1,:)) mean(measurements(2,:))];
 
% Deleating outlayers from the measurement set
errors = sqrt((p_mean(1) - measurements(1,:)).^2 + (p_mean(2) - measurements(2,:)).^2);
threshold = mean(errors);    % Errors larger than this measurement are discarded
 
measurements_no_out = measurements(:,find(errors < threshold));
p_mean_no_out = [mean(measurements_no_out(1,:)) mean(measurements_no_out(2,:))];
 
% Ploting everything
close all
plot(measurements(1,:), measurements(2,:), '.r', ...
     measurements_no_out(1,:), measurements_no_out(2,:), '.b', ...
     p_mean(1), p_mean(2), 'or', ...
     p_mean_no_out(1), p_mean_no_out(2), 'ob')
legend('Measurements Discarted', 'Measurements', 'Mean', 'Mean no outlayers')


errors_2 = sqrt((p_mean_no_out(1) - measurements(1,:)).^2 + (p_mean_no_out(2) - measurements(2,:)).^2);
disp(['Mean at (' num2str(p_mean) '), error in mean ' num2str(max(errors))])
disp(['Mean at (' num2str(p_mean) '), error in mean no outlayers ' num2str(max(errors_2))])

err_x = (max(errors))*cos(-pi:0.2:pi) + p_mean(1);
err_y = (max(errors))*sin(-pi:0.2:pi) + p_mean(2);

hold on
plot(err_x, err_y, '-r')

err_x = (max(errors_2))*cos(-pi:0.1:pi) + p_mean_no_out(1);
err_y = (max(errors_2))*sin(-pi:0.1:pi) + p_mean_no_out(2);
plot(err_x, err_y, '-b')
 
end