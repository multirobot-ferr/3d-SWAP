#include <fstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char** argv)
{
    //UAVs
    system("rostopic pub /quad1/hal/go_to_wp std_msgs/String \"data: '{-50,-20,10},0'\" & ");
    system("rostopic pub /quad2/hal/go_to_wp std_msgs/String \"data: '{-50,0,10},0'\" & ");
    system("rostopic pub /quad3/hal/go_to_wp std_msgs/String \"data: '{-50,20,10},0'\" & ");

    //Black Cylinder
    system("rostopic pub /object_black_cylinder_mov_1/hal/go_to_wp std_msgs/String \"data: '{0,0,0.07},0'\" & ");
    system("rostopic pub /object_black_cylinder_mov_2/hal/go_to_wp std_msgs/String \"data: '{-50,-15,0.07},0'\" & ");
    //Black Box
    system("rostopic pub /object_black_box_mov_1/hal/go_to_wp std_msgs/String \"data: '{45,13,0.07},0'\" & ");
    system("rostopic pub /object_black_box_mov_2/hal/go_to_wp std_msgs/String \"data: '{-32,-22,0.07},0'\" & ");
    //Blue Cylinder
    system("rostopic pub /object_blue_cylinder_mov_1/hal/go_to_wp std_msgs/String \"data: '{10,-19,0.07},0'\" & ");
    //Blue Box
    system("rostopic pub /object_blue_box_mov_1/hal/go_to_wp std_msgs/String \"data: '{36,25,0.07},0'\" & ");
    //Red Cylinder
    system("rostopic pub /object_red_cylinder_mov_1/hal/go_to_wp std_msgs/String \"data: '{-38,15,0.07},0'\" & ");
    system("rostopic pub /object_red_cylinder_mov_2/hal/go_to_wp std_msgs/String \"data: '{-1,-3,0.07},0'\" & ");
    //Red Box
    system("rostopic pub /object_red_box_mov_1/hal/go_to_wp std_msgs/String \"data: '{-3,-28,0.07},0'\" & ");
    system("rostopic pub /object_red_box_mov_2/hal/go_to_wp std_msgs/String \"data: '{-17,13,0.07},0'\" & ");

    return 0;
}





