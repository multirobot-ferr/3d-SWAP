function get_information()
% PROCESS_EXPERIMENT recovers the measurement taken from the uavs 

%sshpass -p "password" scp -r user@example.com:/some/remote/path /some/local/path
password = input('Introduce mabzircs machines password:','s');

for robot = 2
    system(['sshpass -p ' password ' scp grvc@mbzirc_' num2str(robot) ':/home/grvc/catkin_ws/src/mbzirc/uav_avoidance/logs/experiment02/uav* ' cd])
end

end
