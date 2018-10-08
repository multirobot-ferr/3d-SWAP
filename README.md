# 3D-SWAP

This repository contains all the code for running 3D-SWAP and the experiments used to test the algorithm

# Installing 3D-SWAP

## Installing dependencies

Before you can actually set up the 3D-SWAP, you need to install the following dependencies:

 * The algorithm works over [ros Kinetic Kame](http://wiki.ros.org/kinetic), you can finde the instructions to install it [here](http://wiki.ros.org/kinetic/Installation)

 * In order to intereact with UAVs, the system also uses the [grvc-ual v2.0](https://github.com/grvcTeam/grvc-ual) interface, you can also find installation instructions [here](https://github.com/grvcTeam/grvc-ual/wiki/How-to-build-and-install-grvc-ual)

 * Finally 3D-SWAP uses the [Armadillo C++ library](http://arma.sourceforge.net/download.html), to install it type:
```
    $ sudo apt-get install libarmadillo-dev
```

There are some ROS related packages that our system uses to show prettier representations. They are optional and probably you have them already installed, but if not, you can install them with the following command:
```
    $ sudo apt-get install ros-kinetic-map-server 
```

## Creating your workspace

In order to use 3D-SWAP you need a catkin workspace. You should have one alredy, but if not, follow the instructions in [create a ROS Workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

We will assume you created your workspace in the **~/catkin_ws** folder.

## Donwloading the source

Once dependencies have been installed, 3D-SWAP can be set up. The 3D-SWAP repository contains a folder with a set of packages. These packages need to reside in the src folder of your catkin workspace. To clone the repositorie into the right place:
       
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/multirobot-ferr/3d-SWAP.git
    $ cd ~/catkin_ws/
    $ catkin_make
    $ source ~/.bashrc


## How to use 3D-SWAP ##

### Simple performance demo

If you just want to try 3D-SWAP, you can directly run our cube simulation demo. In it, basically you have 4 UAVs that places themselves in a cube of 20x20x20m centered at (0,0,13) (UAVs are always at least 3m away from the floor). Those UAVs exchange their positions creating a pretty conflictive situation in the center of the cube.

To test it, you have to first launch the 4 UAVs in SITL simulation. The UAVs are controlled through the ROS services provided by the UAV abstraction Layer.

    $ roslaunch avoidance_experiments simulator_cube.launch 
    
You should have rightnow a gazebo simulation with 4 UAVs, and an RVIZ representation of the system.

To make the UAVs take-off and place themselves in their respective starting positions, execute the folling command:

    $ roslaunch avoidance_experiments cube_prepare.launch

Finally, once they are in their stating positions, just give them the green light to move:

    $ roslaunch avoidance_experiments expcube_start.launch
