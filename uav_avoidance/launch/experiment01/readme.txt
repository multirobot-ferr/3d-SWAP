En la prueba 1 - calcular el error de posicionamiento

Lanzar roscore en la estación base
Lanzar visualización RVIZ en la estación base
Posicionar el UAV
Test de UAV
Activar el UAL server
    
    roslaunch uav_avoidance core_real_1.launch
    roslaunch uav_avoidance ual_real_1.launch

Ejecutar el nodo swap. Este no comanda nada directamente al UAV. El nodo swap se subscribirá a la posición del UAV y creará un archivo txt para procesarlo con un script de matlab


Calcular el error de posición mediante script de matlab
    
    /uav_avoidance/launch/ComputePositionError.m



Prueba 1:
    Escenario: Un único UAV.
    Objetivo: Determinar el error en position del UAV (Parameter Tunning)
    Prueba: 
    Mantener el UAV estático en el suelo durante un periodo de tiempo relativamente alto (varios minutos). Realizar la media de todas las medidas para determinar la posición exacta del UAV. Colocarlo en distintas posiciones medidas con cinta métrica respecto a la original para estimar una cota del error en posición. 


To execute the first experiment:
- Start the simulator
roslaunch uav_avoidance simulator.launch

- Start the simple system to save everything
roslaunch uav_avoidance exp01_start.launch

This creates a bag in bags/experiment01 and a log in logs/experiment01
