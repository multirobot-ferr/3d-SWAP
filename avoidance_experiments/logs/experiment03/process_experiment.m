function process_experiment()
% PROCESS_EXPERIMENT calculates the minimial distance between uavs

close all
%sshpass -p "password" scp -r user@example.com:/some/remote/path /some/local/path
data = load('uav2.txt');

plot( data(:,4), data(:,5),'.b', data(:,7), data(:,8),'.g')
axis square
pause(1)


%inicializar minima distancia
mindist=1000;
% Number of measurements
mess = length(data);

% measurements of uav2 and uav3
measurements_uav2 = data(1:mess,4:6);
measurements_uav3 = data(1:mess,7:9);

%last position (true position)
last_position_uav3 = [measurements_uav3(mess, 1) measurements_uav3(mess, 2) measurements_uav3(mess, 3)];
errors=zeros(mess,1);
% Medidas de uav3 a descartar
for m = 1:mess
    errors(m,1) = sqrt((last_position_uav3(1) - measurements_uav3(m,1))^2 + (last_position_uav3(2) - measurements_uav3(m,2))^2);

end

% Solo se calcula minima distancia cuando uav3 esté en posición de avoidance  
for m=1:mess
    if(errors(m,1)<0.5)
        dist=sqrt((measurements_uav3(m,1)-measurements_uav2(m,1))^2+(measurements_uav3(m,2)-measurements_uav2(m,2))^2);
        if (dist<mindist)
            mindist=dist;
        end
    end
end
        

disp('la minima distancia entre UAV2 y UAV3 es ')
disp(mindist)

hold on

df_x = mindist*cos(-pi:0.1:pi) + last_position_uav3(1);
df_y = mindist*sin(-pi:0.1:pi) + last_position_uav3(2);
plot(last_position_uav3(1), last_position_uav3(2), '* ', df_x, df_y, 'r')
axis square
        

end