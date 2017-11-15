#!/bin/bash

sleep 2;
echo "Argumento:"
echo $1
echo 'Executing rosservice call /$1/Start "data: true"'
rosservice call /$1/Start "data: true"
