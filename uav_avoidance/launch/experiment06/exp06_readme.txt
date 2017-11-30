Experimento similar al experimento cuatro, en el que dos UAVs intercambian las posiciones haciendo swap, pero en este caso con cambios en la altura. El objetivo de este experimento es comprobar los diferentes estados en altura priorizando que los uavs nunca crucen sus planos xy para asegurar la máxima seguridad.

1. Primer cruce, dos uavs en dirección contraria a una altura de 3 m (uav2->3m y uav3->6m) en el que los uavs se evitarán haciendo swap.

2. Segundo cruce uno contra otro a una altura de 17m (uav2->3m y uav3->20) en el que los uavs se ignorarán ya que están a una altura demasiado grande para que los cilindros intersecten.

3. En el tercer cruce los uavs irán navegando hacia su waypoint cambiando su altura. El uav2 irá de 3m a un máximo de 6m y el uav3 de 20m a un mínimo de 10m. De esta forma, cuando se crucen en el plano x-y, entrarán en el estado Z-BLOCKED y seguirán hacia su waypoint de destino sin variar la altura.
