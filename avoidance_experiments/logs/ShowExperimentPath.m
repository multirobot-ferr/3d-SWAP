function ShowExperimentPath(exp_num)
%SHOWEXPERIMENTPATH Shows the path of the specified experiment

% Loading the log files
param = load(['experiment' sprintf('%02d',exp_num) '/uav2vars.txt']);
data  = load(['experiment' sprintf('%02d',exp_num) '/uav2.txt']);

size(data)

% Extracting the data
for r=1:4
    idx = (r-1)*3 + 1;
    for coord = 1:3
        robot(r,coord,:) = [data(:,idx + coord - 1)];
    end
end

figure
hold on
t = 1000;
for r=1:4
    x(:) = [robot(r,1,1:t)]
    y(:) = [robot(r,2,1:t)]
    z(:) = [robot(r,3,1:t)];
    plot(x,y)
end

end

