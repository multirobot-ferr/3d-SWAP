Experimento 1

El objetivo de este experimento es medir el error en posición. 
El UAV_2 se coloca en el suelo para medir el error de la pos durante un tiempo.

Experimento 2

El objetivo de este experimento es sintonizar los PID que controlarán la posición del UAV durante el experimento.
Existirá una prueba por cada eje (x,y,z). En cada prueba se comandarán distintos waypoints para comprobar la respuesta del PID en ese eje.
En cada prueba el UAV_2 volará a una posición inicial para posteriormente alternará una serie de waypoints en el eje que corresponda a la prueba.

Experimento 3

El objetivo de este experimento es probar SWAP en un solo UAV.
El UAV_2 volará hasta una posición inicial, a una altura de 5m, y esperará. 
El UAV_3 se desplazará a la posición del experimento volando a una altura de 3m y aterrizará.
Cuando se le ordene, el UAV_2 comenzará la misión que consistirá en:
Dirigirse a un waypoint que haga que se produzca una posible colisión.
El UAV_2 realizará SWAP para evitar al UAV_3 que se encuentra en el suelo.
Cuando llegue a dicho waypoint aterrizará en ese punto.

Experimento 4

El objetivo de este experimento es probar SWAP con dos UAV.

El UAV_2 volará hasta una posición inicial y se mantendrá esperando el comienzo del experimento a una altura de 3m.
El UAV_3 hará lo mismo pero a una altura de 6m.
Cuando se ordene comenzará el experimento que consistirá en:
- Primer cruce misma trayectoria dirección contraria
- Se desplazan cada uno a un waypoint diferente que no hará que se crucen
- Segundo cruce UAVs se dirigen a la posción donde estaba el otro
- Vuelven a cambiar de waypoint
- Tercer cruce: intercambian waypoints
- Cambio de waypoint
- cuarto cruce
- UAVs se dirigen al mismo waypoint donde se mantendrán bailando ya que buscan el mismo punto.
- UAV_3 se dirige a otro waypoint en el que aterrizará.
- UAV_2 se dirige a otro waypoint en el que aterrizará.


Experimento 5

El objetivo de este experimento es comprobar el funcionamiento de Swap con tres UAVs realizando el movimiento de evitación a la vez. La altura de los UAVs no cambiará en todo el experimento, siendo de 3, 6 y 9m respectivamente.

1. Los UAVs se situarán en una posición inicial que serán los vértices de un triángulo.

2. Los UAVs intercambiarán los vértices de ese triangulo de manera que coincidan en la parte central del triángulo.

3. Vuelven a intercambiar los vértices para simular el movimiento de evitación en diferentes posiciones

3. Los UAVs 1 y 2 se mantendrán buscando una posición cercana que haga que se produzca un baile similar al del experimento 4. Entonces, el UAV 3 irá a un waypoint en el que sea necesario pasar por esa zona de baile, evitando a los dos.

4. Misma trayectoria del punto 3 pero con dirección contraria.

5. UAVs se desplazan a posición inicial para aterrizar.

Experimento 6


Experimento similar al experimento cuatro, en el que dos UAVs intercambian las posiciones haciendo Swap, pero en este caso con cambios en la altura. El objetivo de este experimento es comprobar los diferentes estados en altura priorizando que los UAVs nunca crucen sus planos x-y para asegurar la máxima seguridad.

1. Primer cruce, dos UAVs en dirección contraria a una diferencia de altura de 2 m (uav2->3m y uav3->5m) en el que los UAVs se evitarán haciendo Swap.

2. Segundo cruce, uno contra otro a una diferencia de altura de 7m (uav2->3m y uav3->10) en el que los uavs se ignorarán ya que están a una altura demasiado grande para que los cilindros se toquen. (Le he preguntado a victor y no hay problema por volar tan alto en el karting)

3. En el tercer cruce los uavs irán navegando hacia su waypoint cambiando su altura. El uav2 volando a 3m de altura y el uav3 de 10m a un mínimo de 7m. De esta forma, cuando se crucen en el plano x-y, entrarán en el estado Z-BLOCKED y seguirán hacia su waypoint de destino sin variar la altura hasta que dejen de ser conflictivos, previniendo el movimiento Swap.

Para este experimento el cilindro swap tiene una altura de 4m y el cilindro Z-blocked de 10m. De esta forma, en el pimer cruce estarán en estado swap, en el segundo en estado free y el tercero en estado Z-Blocked.



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


