function get_information_from_uav()
% PROCESS_EXPERIMENT recovers the measurement taken from the uavs 

%sshpass -p "password" scp -r user@example.com:/some/remote/path /some/local/path
password = input('Introduce mabzircs machines password:','s');

for experiment = 1:7
    for robot = 2:4
        str = ['sshpass -p ' password ' scp grvc@mbzirc_' num2str(robot) ':/home/grvc/catkin_ws/src/mbzirc/avoidance_experiments/logs/experiment' sprintf('%02d',experiment) '/uav* ' cd '/experiment' sprintf('%02d',experiment)];
        disp(['Copying experiment ' sprintf('%02d',experiment) ', robot ' num2str(robot)]);
        disp(str)
        system(str);
    end
end

end