//------------------------------------------------------------------------------
// GRVC MBZIRC Vision
//------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//----------

#include <uav_state_machine/state_machine.h>

//-------------------------------------------------------------------------------------------------------------------------------
Uav_State_Machine::Uav_State_Machine(grvc::utils::ArgumentParser _args){

     // Init services.
    pos_error_srv   = new hal::Server::PositionErrorService::Client ("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/hal/pos_error",   _args);
    takeOff_srv     = new  hal::Server::TakeOffService::Client      ("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/hal/take_off",    _args);
    land_srv        = new  hal::Server::LandService::Client         ("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/hal/land",        _args);
   
    while(!takeOff_srv->isConnected()) {
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    ros::NodeHandle nh;

    target_service   = nh.advertiseService("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/visual_servoing/enabled", targetCallback);
    takeoff_service  = nh.advertiseService("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/visual_servoing/takeoff", takeoffCallback);
    land_service     = nh.advertiseService("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/visual_servoing/land", landCallback);

    ros::Subscriber altitudeSubs;
    if(ros::isInitialized()){
        altitudeSubs = nh.subscribe<std_msgs::Float64>("/mavros_"+_args.getArgument<std::string>("uavId","1")+"/global_position/rel_alt", 10, altitudeCallback);
    }
    
    candidateSubscriber = nh.subscribe<std_msgs::String>("/candidateList", 1, candidateCallback);
    joystickSubscriber = nh.subscribe<sensor_msgs::Joy>("/joy", 100, joystickCb);

    if(!candidateSubscriber){
        std::cout << ("Can't start candidate subscriber.") << std::endl;
        return -1;
    }else{
        std::cout << "Subscribed to candidate topic" << std::endl;
    }

    std::cout << "Initialized" << std::endl;
    std::cout << "Connected to hal" << std::endl;
}
//--------------------------------------------------------------------------------------------------------------------------------
Uav_State_Machine::step(){
    
    while(ros::ok()){

        switch(mState){
            case REPOSE:
                reposeCallback();
                break;
            
            case TAKINGOFF:
                takeoffCallback();
                break;

            case HOVER:
                hoverCallback();
                break;

            case CATCHING:
                catchingCallback();
                break;

            case LAND:
                landCallback();
                break;
        }

    }

}
//---------------------------------------------------------------------------------------------------------------------------------
void candidateCallback(const std_msgs::String::ConstPtr _msg){
    //mbzirc::Candidate specs;
    //specs.color = 1;
    hal::TaskState ts;

    std::stringstream msg;
    msg << _msg->data;
    mbzirc::CandidateList candidateList;
    msg >> candidateList;

    if(target_state){
        if(candidateList.candidates.size() > 0){
        
        mbzirc::Candidate target = bestCandidateMatch(candidateList, specs);
        std::cout << "tracking candidate with error " << target.location.transpose() << std::endl;
        target.location[2] = gFlyTargetAltitude-gAltitude;
        pos_error_srv->send(target.location, ts);

        }
    }
   /* if(candidateList.candidates.size() > 0){
        mbzirc::Candidate target = bestCandidateMatch(candidateList, specs);
        std::cout << "tracking candidate with error " << target.location.transpose() << std::endl;
        target.location[2] = 15-gAltitude;
        pos_error_srv->send(target.location, ts);

    }else{
        hal::Vec3 zeroError = {0,0,0};
        pos_error_srv->send(zeroError, ts);
    }*/
    this_thread::sleep_for(chrono::milliseconds(100));
}
//---------------------------------------------------------------------------------------------------------------------------------
void joystickCb(const sensor_msgs::Joy::ConstPtr& _joy) {
        //Callback
        gFlyTargetAltitude += _joy->axes[1];
        std::cout << gFlyTargetAltitude << std::endl;
}
//---------------------------------------------------------------------------------------------------------------------------------
void altitudeCallback(const std_msgs::Float64::ConstPtr _msg){
    gAltitude = _msg->data;
}
//---------------------------------------------------------------------------------------------------------------------------------
bool targetServiceCallback(uav_visual_servoing::target_service::Request  &req,
         uav_visual_servoing::target_service::Response &res)
{
    if(mState == eState::HOVER){
        mTarget.color = req.color;
        mTarget.shape = req.shape;
        mTarget.location[0] = req.position[0];
        mTarget.location[1] = req.position[1];
        res.success = true;
        mState = eState::CATCHING;

        return true;
    }else{
        res.success = false;

        return false;
    }
    
}

//---------------------------------------------------------------------------------------------------------------------------------
void reposeCallback(){
    //GotoTargetPosition
    //TargetTracking
    //Pickup
    //GotoDeploy
}

//---------------------------------------------------------------------------------------------------------------------------------
void hoverCallback(){
    //GotoTargetPosition
    //TargetTracking
    //Pickup
    //GotoDeploy
}

//---------------------------------------------------------------------------------------------------------------------------------
void catchingCallback(){
    //GotoTargetPosition
    //TargetTracking
    //Pickup
    //GotoDeploy
}
//---------------------------------------------------------------------------------------------------------------------------------
bool takeoffCallback(uav_visual_servoing::takeoff_service::Request  &req,
         uav_visual_servoing::takeoff_service::Response &res)
{
    gFlyTargetAltitude = req.altitude;
    hal::TaskState ts;
    takeOff_srv ->send(gFlyTargetAltitude, ts);

    res.success = true;

    return true;
}
//---------------------------------------------------------------------------------------------------------------------------------
bool landCallback(uav_visual_servoing::land_service::Request  &req,
         uav_visual_servoing::land_service::Response &res)
{
    hal::TaskState ts;
    land_srv ->send(ts);

    res.success = true;

    return true;
}