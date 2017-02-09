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
bool UavStateMachine::Init(grvc::utils::ArgumentParser _args){

     // Init services.
    pos_error_srv   = new  grvc::hal::Server::PositionErrorService::Client ("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/hal/pos_error",   _args);
    takeOff_srv     = new  grvc::hal::Server::TakeOffService::Client      ("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/hal/take_off",    _args);
    land_srv        = new  grvc::hal::Server::LandService::Client         ("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/hal/land",        _args);
    waypoint_srv    = new  grvc::hal::Server::WaypointService::Client     ("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/hal/go_to_wp",    _args);
    abort_srv       = new  grvc::hal::Server::AbortService::Client     ("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/hal/abort",    _args);

    while(!takeOff_srv->isConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ros::NodeHandle nh;

    target_service   = nh.advertiseService("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/uav_state_machine/enabled", &UavStateMachine::targetServiceCallback, this);
    takeoff_service  = nh.advertiseService("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/uav_state_machine/takeoff", &UavStateMachine::takeoffCallback, this);
    land_service     = nh.advertiseService("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/uav_state_machine/land",    &UavStateMachine::landCallback, this);
    waypoint_service = nh.advertiseService("/mbzirc_"+_args.getArgument<std::string>("uavId","1")+"/uav_state_machine/waypoint",&UavStateMachine::searchingCallback, this);


    /**if(ros::isInitialized()){
        positionSubs = nh.subscribe<geometry_msgs::PoseStamped>("/mavros_"+_args.getArgument<std::string>("uavId","1")+"/local_position/pose", 10, &UavStateMachine::positionCallback, this);
        altitudeSubs = nh.subscribe<std_msgs::Float64>("/mavros_"+_args.getArgument<std::string>("uavId","1")+"/global_position/rel_alt", 10, &UavStateMachine::altitudeCallback, this);
    }**/

    positionSubs = nh.subscribe<geometry_msgs::PoseStamped>("/mavros_"+_args.getArgument<std::string>("uavId","1")+"/local_position/pose", 10, &UavStateMachine::positionCallback, this);
    altitudeSubs = nh.subscribe<std_msgs::Float64>("/mavros_"+_args.getArgument<std::string>("uavId","1")+"/global_position/rel_alt", 10, &UavStateMachine::altitudeCallback, this);   
    joystickSubscriber = nh.subscribe<sensor_msgs::Joy>("/joy", 100, &UavStateMachine::joystickCb, this);
    std::cout << "Initialized" << std::endl;
    std::cout << "Connected to hal" << std::endl;

    return true;
}
//--------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::step(){
    
    while(ros::ok()){

        switch(mState){
            case eState::REPOSE:
            {
                break;
            }
            case eState::TAKINGOFF:
            {
                grvc::hal::TaskState ts;
                takeOff_srv ->send(mFlyTargetAltitude, ts);
                mState = eState::HOVER;
                break;
            }
            case eState::HOVER:
            {   

                break;
            }
            case eState::SEARCHING:
            {   
                //When the UAV finish the track, it starts again the same track
                grvc::hal::TaskState ts;
                waypoint_srv->send(mWaypointList[mWaypointItem], ts);
                if(ts==TaskState::finished)
                {
                    mWaypointItem++;
                    if(mWaypointItem > (mWaypointList.size()-1)){
                        mWaypointItem = 0;
                        mState = eState::HOVER;         
                    }
                }
                break;
            }
            case eState::CATCHING:
            {
                catchingCallback();
                break;
            }
            case eState::LAND:
            {
                grvc::hal::TaskState ts;
                land_srv ->send(ts);
                mState = eState::REPOSE;
                break;
            }
        }

    }

}
//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::candidateCallback(const uav_state_machine::candidate_list::ConstPtr& _msg){
    //mbzirc::Candidate specs;
    //specs.color = 1;
    if(mState == eState::CATCHING){
        grvc::hal::TaskState ts;
        uav_state_machine::candidate_list candidateList = *_msg;

        if(candidateList.candidates.size() > 0){
	    uav_state_machine::candidate matchedCandidate;
            if(bestCandidateMatch(candidateList, mTarget, matchedCandidate)){
                std::cout << "tracking candidate with error " << matchedCandidate.position << std::endl;
                matchedCandidate.position.z = mFlyTargetAltitude-mCurrentAltitude;
		Eigen::Matrix<double, 3, 1> targetPosition;
		targetPosition << matchedCandidate.position.x, matchedCandidate.position.y, matchedCandidate.position.z;
                pos_error_srv->send(targetPosition, ts);
            }else{
                std::cout << "Cant find a valid candidate" << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::joystickCb(const sensor_msgs::Joy::ConstPtr& _joy) {
        //Callback
        mFlyTargetAltitude = mCurrentAltitude + _joy->axes[1];
        std::cout << mFlyTargetAltitude << std::endl;
}
//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::altitudeCallback(const std_msgs::Float64::ConstPtr& _msg){
    mCurrentAltitude = _msg->data;
}
//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg){
    mCurrentWaypoint = {{_msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z}, 0.0};

}
//---------------------------------------------------------------------------------------------------------------------------------
bool UavStateMachine::targetServiceCallback(uav_state_machine::target_service::Request  &req,
         uav_state_machine::target_service::Response &res)
{
    std::cout << "Received target of color: " << req.color << ", and position :["<< req.position[0] << ", " << req.position[1] << "];" << std::endl;
    if(mState == eState::HOVER){
        mTarget.color = req.color;
        mTarget.shape = req.shape;
        mTarget.position.x = req.position[0];
        mTarget.position.y = req.position[1];
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
    //TargetTrackingcandidateCallback
    //Pickup
    //GotoDeploy
}

//---------------------------------------------------------------------------------------------------------------------------------
bool UavStateMachine::searchingCallback(uav_state_machine::waypoint_service::Request &req, uav_state_machine::waypoint_service::Response &res)
{
    //GotoPosition
    //1- start searching 0- stop searching 2- restart searching 
    //The difference between start and restart is that restart continues by the last wp  
    if(req.action == 0)
    {
        if(mState == eState::SEARCHING)
        {
            mState = eState::HOVER;
            grvc::hal::TaskState ts;
            abort_srv->send(ts);
            waypoint_srv->send(mCurrentWaypoint, ts);
            res.success = true;
        }else{
            res.success = false;
        }
    }
    else if(req.action == 1)
    {
        if(mState == eState::HOVER){
            mWaypointItem = 0;
            mWaypointList.clear();
            for(int i=0; i< req.waypoint_track.size(); i++)  
               mWaypointList.push_back({{req.waypoint_track[i].x, req.waypoint_track[i].y, req.waypoint_track[i].z}, 0.0});
            mState = eState::SEARCHING;
            res.success = true;
        }else{
            res.success = false;
        }
    }
    else if(req.action == 2)
    {
        if(mState == eState::HOVER){
            mState = eState::SEARCHING;
            res.success = true;
        }else{
            res.success = false;
        }
    }
    
    return true;
    //TargetTracking
    //Pickup
    //GotoDeploy
}

//---------------------------------------------------------------------------------------------------------------------------------
void UavStateMachine::catchingCallback(){
    //GotoTargetPosition

    //TargetTracking (aka visual servoing)
    /// Init subscriber to candidates
    ros::NodeHandle nh;
    ros::Subscriber candidateSubscriber = nh.subscribe<uav_state_machine::candidate_list>("/candidateList", 1, &UavStateMachine::candidateCallback, this);

    if(!candidateSubscriber){
        std::cout << ("Can't start candidate subscriber.") << std::endl;
	mState = eState::HOVER;
	return;
    }else{
        std::cout << "Subscribed to candidate topic" << std::endl;
    }

    while(mState == eState::CATCHING){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    //Pickup
    //GotoDeploy
}
//---------------------------------------------------------------------------------------------------------------------------------
bool UavStateMachine::takeoffCallback(uav_state_machine::takeoff_service::Request  &req, uav_state_machine::takeoff_service::Response &res)
{
    
    if(mState == eState::REPOSE){
        mFlyTargetAltitude = req.altitude;
        mState = eState::TAKINGOFF;
        res.success = true;

    }else{
        res.success = false;
    }
    return true;
}
//---------------------------------------------------------------------------------------------------------------------------------
bool UavStateMachine::landCallback(uav_state_machine::land_service::Request  &req, uav_state_machine::land_service::Response &res)
{
    mFlyTargetAltitude = 0;
    mState = eState::LAND;
    res.success = true;

    return true;
}
//----------------------------------------------------------------------------------------------------------------------------------
bool UavStateMachine::bestCandidateMatch(const uav_state_machine::candidate_list _list, const uav_state_machine::candidate &_specs, uav_state_machine::candidate &_result){
    double bestScore= 0;
    bool foundMatch = false;
    for(auto&candidate:_list.candidates){
        double score = 0;
        if(candidate.color == _specs.color){
            score +=1;
        }

        //if(candidate.shape == _specs.shape){
        //    score +=1;
        //}

        //if((candidate.location - _specs.location).norm() < (_result.location - _specs.location).norm()){
        //    score +=1;
        //}

        if(score > bestScore){
            _result = candidate;
            foundMatch = true;
        }
    }
    return foundMatch;
}
