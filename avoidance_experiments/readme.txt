Para realizar experimentos con uav_avoidance en el simulador:

1. roslaunch avoidance_experiments simulator_carting.launch

Con esa orden lanzamos gazebo con 3 UAVs preparados para simular mediante el SITL de px4 y visualizar la simulación en gazebo y en RVIZ.

Para configurar las posiciones iniciales de los UAVs configurar el archivo de parámetros:

avoidance_experiments/config/simulator/simulator_params.yaml

2. roslaunch avoidance_experiments exp(numero exp)_r(id robot)_prepare.launch

Lanzar un roslaunch por cada UAV que esté incluido en el experimento. Los UAV despegarán y se situarán en sus posiciones iniciales.

3. roslaunch avoidance_experiments exp(numero exp)_start.launch

Los UAVs comienzan el experimento

4. roslaunch avoidance_experiments exp(numero exp)_replay.launch

Para reproducir en rviz la simulación del último experimento


Para realizar experimentos reales con uav_avoidance:

1. roslaunch avoidance_experiments core_r(robot_id).launch

2. roslaunch avoidance_experiments swap_r(robot_id).launch


