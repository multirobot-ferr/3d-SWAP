#################################################################################
#										#
#				VISION NODE RUN FILE				#
#										#
#################################################################################

#######################
## Camera Parameters ##
#######################
# Focal length in meters
CAMERA_FOCALLENGTH="1078.905191"
# Sensor width in meters
CAMERA_SENSOR_WIDTH="4.8e-3"
# Sensor height in meters
CAMERA_SENSOR_HEIGHT="3.6e-3"
# Image width in pixels
CAMERA_IMAGE_WIDTH="1280"
# Image height in pixels
CAMERA_IMAGE_HEIGHT="960"
# Pixel Size
CAMERA_PIXEL_SIZE="1"

##############################################
## Choose between different camera sources. ##
##############################################
# Possibilities are: video|ros|real
CAMERA_TYPE="ros"

# In case of using video
VIDEO_PATH="b2.mp4"

# In case of ros topic
CAMERA_TOPIC="/camera_1/image_raw_1"

# In case of usb device
CAMERA_INDEX="0"

##########################
##	Publish data	##
##########################
# Name for a topic to publish the list of candidates
PUBLISH_DATA=true
CANDIDATES_TOPIC="/candidateList"

##################################
##	Drone data		##
##################################
# Topic where to read the data from the drone
DRONE_POSE_TOPIC="/quad1/hal/position"

##################################
##	Debugging Flags		##
##################################
# true if want to visualize the results from the process into 
# a separate window, false to disable de visualization
VISUALIZE_RESULTS=true

##################################
##	Object clues		##
##################################
MIN_OBJ_SIZE=0.15
MAX_OBJ_SIZE=2.0

#------------------------------------------DONT TOUCH BELOW THIS LINE!!---------------------------------------------
##################   
##	RUN	##
##################

# Config camera
RUN_ARGUMENTS=" -c $CAMERA_TYPE"
if [ "$CAMERA_TYPE" == "video" ]; then
	RUN_ARGUMENTS="$RUN_ARGUMENTS -p $VIDEO_PATH"
elif [ "$CAMERA_TYPE" == "ros" ];then
	RUN_ARGUMENTS="$RUN_ARGUMENTS -t $CAMERA_TOPIC"
elif [ "$CAMERA_TYPE" == "real" ];then
	RUN_ARGUMENTS="$RUN_ARGUMENTS -i $CAMERA_INDEX"
else
	echo "Unknown source for the images, please check the CAMERA_TYPE variable"
	exit
fi

# Config Publish
if [ "$PUBLISH_DATA" ]; then
	RUN_ARGUMENTS="$RUN_ARGUMENTS --publishTopic $CANDIDATES_TOPIC"
fi

# Visualization
if [ $VISUALIZE_RESULTS == true ]; then
	RUN_ARGUMENTS="$RUN_ARGUMENTS -v"
fi

# Drone
RUN_ARGUMENTS="$RUN_ARGUMENTS --dronePoseTopic $DRONE_POSE_TOPIC"

# Camera parameters
RUN_ARGUMENTS="$RUN_ARGUMENTS --cf $CAMERA_FOCALLENGTH --csw $CAMERA_SENSOR_WIDTH --csh $CAMERA_SENSOR_HEIGHT --ciw $CAMERA_IMAGE_WIDTH --cih $CAMERA_IMAGE_HEIGHT --cps $CAMERA_PIXEL_SIZE"

# Object data
RUN_ARGUMENTS="$RUN_ARGUMENTS --minos $MIN_OBJ_SIZE --maxos $MAX_OBJ_SIZE"

# Run vision Node
./vision_node $RUN_ARGUMENTS

