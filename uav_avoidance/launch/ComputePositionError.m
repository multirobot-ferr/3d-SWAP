function [ output_args ] = ComputePositionError( input_args )
%COMPUTEPOSITIONERROR This script computes the error in position of an UAV
%that remains static in a position

% Loading the log files
data=load(input_args);

% Real position
p = [-10.0, -10.0];

% Number of measurements
mess = 150;

% Variance error (assuming a gaussian error)
var = [1.0, 1.0]; % sigma^2

disp(['Real position: (' num2str(p) ')'])
disp(['95.4% of the measurements in square (' num2str(-2*var + p) ') and (' num2str(2*var + p) ')'])

measurements = zeros(2,mess);
for m = 1:mess
measurements(:,m)=data(m,1:2);
end

% % Generating the measurements
% measurements = zeros(2,mess);
% for m = 1:mess
%     measurements(:,m) = normrnd(p,sqrt(var));
% end
% 
% warning('Uncomment')
% This represents a really extrange outlayer
%measurements(:,1) = p + [10 10];

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
     p(1), p(2),'og', ...
     p_mean(1), p_mean(2), 'or', ...
     p_mean_no_out(1), p_mean_no_out(2), 'ob')
legend('Measurements Discarted', 'Measurements','Ground Truth', 'Mean', 'Mean no outlayers')

disp(['Mean at (' num2str(p_mean) '), error in mean ' num2str(sqrt(sum((p_mean - p).^2)))])
disp(['Mean at (' num2str(p_mean) '), error in mean no outlayers ' num2str(sqrt(sum((p_mean_no_out - p).^2)))])




end

