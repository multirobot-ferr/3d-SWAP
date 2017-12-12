function [ output_args ] = ComputePositionError( )
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

% Computing the error without outliyers
errors2 = sqrt((p_mean_no_out(1) - measurements(1,:)).^2 + (p_mean_no_out(2) - measurements(2,:)).^2);

e1x = max(errors)*cos([-pi:0.1:pi -pi]) + p_mean(1);
e1y = max(errors)*sin([-pi:0.1:pi -pi]) + p_mean(2);

e2x = max(errors2)*cos([-pi:0.1:pi -pi]) + p_mean_no_out(1);
e2y = max(errors2)*sin([-pi:0.1:pi -pi]) + p_mean_no_out(2);
 
% Ploting everything
close all
axis square
plot(measurements(1,:), measurements(2,:), '.r', ...
     measurements_no_out(1,:), measurements_no_out(2,:), '.b', ...
     p_mean(1), p_mean(2), 'or', ...
     e1x, e1y, '-r', ...
     p_mean_no_out(1), p_mean_no_out(2), 'ob', ...
     e2x, e2y, '-b' ...
     )
legend('Measurements Discarted', 'Measurements', 'Mean', 'Max error mean', 'Mean no outlayers','Max error mean no out')
 
xlabel(['Mean error ' num2str(max(errors)) ', Mean no outlayers error ' num2str(max(errors2))])
ylabel(['Measurements ' num2str(mess)])
disp(['Mean at (' num2str(p_mean) '), error in mean ' num2str(max(errors))])
disp(['Mean at (' num2str(p_mean) '), error in mean no outlayers ' num2str(max(errors2))])
 



end