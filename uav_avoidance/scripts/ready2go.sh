#!/bin/bash

sleep 2;
rosservice call /uav_1/Start "data: true"
rosservice call /uav_2/Start "data: true"
rosservice call /uav_3/Start "data: true"
