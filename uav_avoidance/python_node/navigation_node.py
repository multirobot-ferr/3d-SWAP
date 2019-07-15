#!/usr/bin/env python
import rospy
from uav_abstraction_layer.srv import TakeOff, Land
from uav_abstraction_layer.msg import State
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped 
from nav_msgs.msg import Path
import time
from numpy import linalg as LA
import rospkg
rospack = rospkg.RosPack()
import math
import json
import threading


drones = [] 

def pose_callback(data,drone_id_and_n_drones):
    for i in range(drone_id_and_n_drones[1]):
        drones[i].pose_list[drone_id_and_n_drones[0]] = data

class DroneNavigation:
    vel = TwistStamped()
    pose_list = {}
    state = State()
    waypoints = []
    take_off_service =""

    #callback ual state
    def uav_state_callback(self,data):
        self.state = data

    #class initialization
    def __init__(self, drone, namespace, n_drones):
        
        for i in range(n_drones):
            self.pose_list[i+1] = PoseStamped()
        self.drone = drone
        self.namespace = namespace
        self.pose_pub = rospy.Publisher('/'+namespace+'_'+str(drone)+'/ual/set_pose',PoseStamped,queue_size=1)
        self.vel_pub = rospy.Publisher('/'+namespace+'_'+str(drone)+'/ual/set_velocity',TwistStamped,queue_size=1)
        rospy.Subscriber('/'+namespace+'_'+str(drone)+'/ual/state',State, self.uav_state_callback,queue_size=1)
        self.take_off_service = '/'+namespace+'_'+str(drone)+'/ual/take_off'
        self.land_service = '/'+namespace+'_'+str(drone)+'/ual/land'
        folder_pack = rospack.get_path('uav_avoidance')
        with open(folder_pack+'/python_node/'+str(drone)+'.json') as json_file:  
            self.waypoints = json.load(json_file)

    def collision_avoidance_function(self):
        ####################################################
        ### INCLUDE COLLISION AVOIDANCE FUNCTION HERE#############
        ####################################################
        #Drone 1 pose -> self.pose_list[1].pose.position.x
                   # self.pose_list[1].pose.position.y
                   # self.pose_list[1].pose.position.z
        #Drone 2 pose -> self.pose_list[2].pose.position.x
                    #self.pose_list[2].pose.position.y
                    #self.pose_list[2].pose.position.z
                # ........
        collision = False
        if(collision):
            self.vel.twist.linear.x = 0.0
            self.vel.twist.linear.y = 0.0
            self.vel.twist.linear.z = 0.0
            return True
        else:
            return False
   
    
    def cal_distance_to_waypoint(self,x,y,z):
        distance_to_waypoint = 1
        aux = LA.norm([self.pose_list[self.drone].pose.position.x-x, self.pose_list[self.drone].pose.position.y-y,self.pose_list[self.drone].pose.position.z-z])        
        if aux>distance_to_waypoint:
            return False
        else:
            return True

    def takeOff(self):
        try:
            take_off_client = rospy.ServiceProxy(self.take_off_service, TakeOff)
            take_off_client.call(3.0,True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def mission_thread(self):
        threading.Thread(target=self.mission).start()
    
    def mission(self):
        for i in self.waypoints:
            while(self.cal_distance_to_waypoint(i[0],i[1],i[2])==False):
                if(self.collision_avoidance_function()):
                    print("Collision detected")
                    self.vel_pub(self.vel)
                else:
                    pos = PoseStamped()
                    pos.header.frame_id = 'map'
                    pos.pose.position.x = i[0]
                    pos.pose.position.y = i[1]
                    pos.pose.position.z = i[2]
                    pos.pose.orientation.w = 1
                    self.pose_pub.publish(pos)
                time.sleep(0.1)
        try:
            land_client = rospy.ServiceProxy(self.land_service, Land)
            land_client.call(True)
            print("UAV ",self.drone,": landing")
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
                    

if __name__== "__main__":
    rospy.init_node('mission_node')
    print "\nSelect the number of drones:\n"
    n_drones = int(raw_input(" >> "))
    
    for i in range(n_drones):
        drones.append(DroneNavigation(i+1,'uav', n_drones))
    for i in range(n_drones):
        rospy.Subscriber("/uav_"+str(i+1)+"/ual/pose", PoseStamped, pose_callback, (i+1, n_drones), queue_size=1)
    for i in range(n_drones):
        drones[i].takeOff()
    print "\nPress a key to start the mission:\n"
    key = raw_input(" >> ")
    for i in range(n_drones):
        drones[i].mission_thread()
    


