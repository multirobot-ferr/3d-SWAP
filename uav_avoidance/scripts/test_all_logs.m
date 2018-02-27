function test_all_logs()
% TEST_ALL_LOGS tests if all logs are saved in the computer, and if not,
% tryes to recover them from the UAVs comuters

password = input('Introduce mabzircs machines password:','s');

for exp = 1:5
    disp(['Testing the bags of the experiment ' num2str(exp)])
    
    archives = dir(['../bags/experiment0' num2str(exp)]);
    
    bag_found = false;
    log_found = false;
    for idx = 1:length(archives)
        if ( strcmp(archives(idx).name, ['experiment0' num2str(exp) '_01.bag']))
            disp(['  ' archives(idx).name ' found in local'])
            bag_found = true;
        end
    end
    
    archives = dir(['../logs/experiment0' num2str(exp)]);
    for idx = 1:length(archives)
        for uav = 2:3
            log_found = false;
            if ( strcmp(archives(idx).name, ['uav' num2str(uav) '.txt']))
                disp(['  ' archives(idx).name ' found in local'])
                log_found = true;
            end
            if (~log_found)
                system(['sshpass -p ' password ' scp grvc@mbzirc_' num2str(uav) ...
                    ':/home/grvc/catkin_ws/src/mbzirc/uav_avoidance/logs/experiment0' num2str(exp) '/uav* ' ...
                    [cd '/../logs/experiment0' num2str(exp) '/']]);
            end
        end
    end
    
    

end

% 
% %sshpass -p "password" scp -r user@example.com:/some/remote/path /some/local/path
% 
% 
% for robot = 2:3
%     system(['sshpass -p ' password ' scp grvc@mbzirc_' num2str(robot) ':/home/grvc/catkin_ws/src/mbzirc/uav_avoidance/logs/experiment03/uav* ' cd])
% end

end
