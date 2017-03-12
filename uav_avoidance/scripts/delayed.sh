#!/bin/bash
#
#
# Copyright (c) 2017, University of Duisburg-Essen, swap-ferr
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of swap-ferr nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# 
# @author Eduardo Ferrera
# @version 2.0
# @date    7/3/17 
# 
# @short: This script is ment to run a .sh command in a delayed form.
# 
# Example to wake up a ros node, inside a launch file, with a delay of 3 seconds:
#
# 	<node name="auv_avoidance" pkg="uav_avoidance" type="delayed.sh"
#		args="3 roslaunch uav_avoidance avoidance.launch robot_number:=1" output="screen">
#	</node>


echo " "
echo " "

# Check if the first argument is a number or not
if ! [[ ${*: 1 : 1} == ?(-)+([0-9]) ]]; then
	echo ${*: 1 : 1}

	DELAY_IN_SECS=5;
	echo The system assumes a delay of $DELAY_IN_SECS seconds;

	# Executing all the incomming parameters
	INSTRUCTIONS=${*: 1: $#}
else
	DELAY_IN_SECS=${*: 1: 1}

	# Executing all the incomming parameters but not the fist one
	INSTRUCTIONS=${*: 2: $#}
fi

echo Executing the command:
echo \"$INSTRUCTIONS\";
echo Executing in $DELAY_IN_SECS seconds...

sleep $DELAY_IN_SECS;

$INSTRUCTIONS
