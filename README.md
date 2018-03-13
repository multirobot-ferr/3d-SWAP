# 3d-SWAP

This repository contains all the code for running 3D-SWAP and the experiments used to test the algorithm

## Dependencies

 * [grvc-ual](https://github.com/grvcTeam/grvc-ual)
 * [grvc-utils](https://github.com/grvcTeam/grvc-utils)
 * [PX4 Firmware](https://github.com/PX4/Firmware) at tag [v1.6.3](https://github.com/PX4/Firmware/tree/v1.6.3)

## How do I get set up?

``` 
git clone https://github.com/multirobot-ferr/3d-SWAP.git
```

## How to use 3D-SWAP ##

There are some examples that you can run for testing the 3D-SWAP. How to run the cube simulation is explained here.

 * simulator_cube.launch: launches up to 4 UAVs in SITL simulation. UAVs are controlled through the ROS services provided by the UAV abstraction Layer.

    `$ roslaunch avoidance_experiments simulator_cube.launch`

 * cube_prepare.launch: UAVs are placed in the four squares of a cube waiting for the starting launch to exchange their positions. 

    `$ roslaunch avoidance_experiments cube_prepare.launch`

 * expcube_start.launch: UAVs start the experiment.