#include <stdlib.h>
#include <stdio.h>

int main(int argc, char** argv)
{
    system("rostopic pub /object_black_cylinder_mov_1/hal/path std_msgs/String \"data: '[{0,0,0.07},0]'\" & ");

    return 0;
}





