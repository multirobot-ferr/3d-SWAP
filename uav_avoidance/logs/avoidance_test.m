%
% Copyright (c) 2017, University of Duisburg-Essen, swap-ferr
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice, this
%    list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
%    this list of conditions and the following disclaimer in the documentation
%    and/or other materials provided with the distribution.
%
% 3. Neither the name of swap-ferr nor the names of its
%    contributors may be used to endorse or promote products derived from
%    this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
% DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
% @file avoidance_test.m
% @author Eduardo Ferrera
% @version 1.2
% @date    12/3/17
%
% @short: Reads a log file created with swap.
%
% In order to use this funtion, type: 
%   avoidance_test(n)
% Where n is the number of uav that generated the log file.
% 

function avoidance_test( uav_test )
% Shows what happened with the specific robot
% Possible inputs: 1, 2 or 3

zoom = 3;

param= [];
data = [];

% Testing for errors
switch uav_test 
    case 1
    case 2
    case 3 
    otherwise
        error('Number not valid');
end

% Loading the log files
param = load(['uav' num2str(uav_test) 'vars.txt']);
data  = load(['uav' num2str(uav_test) '.txt']);

% Parameters
safety_r = param(1);
bracking_distance = param(2);
pos_err = param(3);

safety_x = safety_r*cos([-pi:pi/16:pi -pi]);
safety_y = safety_r*sin([-pi:pi/16:pi -pi]);
reserv_x = (2*(bracking_distance + pos_err) + safety_r)*cos([-pi:pi/16:pi -pi]);
reserv_y = (2*(bracking_distance + pos_err) + safety_r)*sin([-pi:pi/16:pi -pi]);

close all
takeOffTime = 50;
for t = takeOffTime:size(data,1)
    % Extracting the data of this time
    data_now = data(t, :);
    
    % Extracting the position of all uav's
    for uav_id = 1:3
        for coord = 1:3
            pos(uav_id, coord) = data_now((uav_id - 1)*3 + coord);
        end
    end
    
    % Centering all in the current robot's perspective
    offset_uav = pos(uav_test,:);
    for uav_id = 1:3
        pos(uav_id, :) = pos(uav_id, :) - offset_uav;
    end
    offset = (3*3)+1;  % Deleating 3 uavs, 3 coordinates
    
    % Extracting references
    % offset [ref_goal_x1 ref_goal_y1 ref_goal_x2 ref_goal_y2]
    ref_desired_x = data_now(offset+0:2:offset+2+0);
    ref_desired_y = data_now(offset+1:2:offset+2+1);
    offset = offset + 4;
    % offset [ref2take_x1 ref2take_y1 ref2take_x2 ref2take_y2]
    ref2take_x    = data_now(offset+0:2:offset+2+0);
    ref2take_y    = data_now(offset+1:2:offset+2+1);
    offset = offset + 4;
    
    % Extracting other measurements
    x_mes = [];
    y_mes = [];
    for col=( offset:2:size(data_now,2) )
        x_mes = [x_mes data_now(:,col)];
        y_mes = [y_mes data_now(:,col+1)];
    end
    
    limits = [-1 1 -1 1]*zoom*(safety_r+bracking_distance+pos_err);
    
    % Ploting all
    hold off
    plot(pos(:,1),pos(:,2), 'xr',          ... % Positions of the robots
        ref_desired_x, ref_desired_y, 'g', ... % Desired orientation
        ref2take_x, ref2take_y, 'r'        ... % Reference to take
        ); 
    hold on
    for uav_id = 1:3
        plot( safety_x + pos(uav_id,1), safety_y + pos(uav_id,2), 'g-.'); % Safety radius
        
        if (uav_id ~= uav_test)        
            plot( reserv_x + pos(uav_id,1), reserv_y + pos(uav_id,2), 'g-'); % Reserved space
        end
    end
    plot(x_mes, y_mes)
   
    axis(limits);
    title(['Simulation step ' num2str(t) '/' num2str(size(data,1))])
    drawnow;
    pause(0.1)
    
end