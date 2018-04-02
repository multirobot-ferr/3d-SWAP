# 3d-SWAP

This repository contains all the code for running 3D-SWAP and the experiments used to test the algorithm

## Dependencies

 * [grvc-ual v2.0](https://github.com/grvcTeam/grvc-ual)
 * [Armadillo](http://arma.sourceforge.net/download.html)


## How to install

Before you can actually set up the 3D-SWAP, you need to install the dependencies.

You can find the instructions for the UAL installation [here](https://github.com/grvcTeam/grvc-ual/wiki/How-to-build-and-install-grvc-ual)

Once dependencies have been installed, 3D-SWAP can be set up. The 3D-SWAP repository contains a folder with a set of packages. These packages need to reside in the src folder of your catkin workspace. To clone the repositorie into the right place:
    ```   
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/multirobot-ferr/3d-SWAP.git
    $ cd ~/catkin_ws/
    $ catkin_make
    $ source ~/.bashrc
    ```

## How to use 3D-SWAP ##

There are some examples that you can run for testing the 3D-SWAP. How to run the cube simulation is explained here.

 * simulator_cube.launch: launches up to 4 UAVs in SITL simulation. UAVs are controlled through the ROS services provided by the UAV abstraction Layer.

    `$ roslaunch avoidance_experiments simulator_cube.launch`   

 * cube_prepare.launch: UAVs are placed in the four squares of a cube waiting for the starting launch to exchange their positions. 

    `$ roslaunch avoidance_experiments cube_prepare.launch`

 * expcube_start.launch: UAVs start the experiment.

     `$ roslaunch avoidance_experiments expcube_start.launch`