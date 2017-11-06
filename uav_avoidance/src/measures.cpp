

/**
 * @file measures.cpp
 * @author Alfonso Alcántara
 * @date   2/11/17
 *
 * @short: the node to check measures in real experiments
 *
 *
 */

#include <uav_avoidance/measures.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <functional>


int main(int argc, char **argv) {
    // name remapping
    ros::init(argc, argv, "measures");

    Measures measure;


    measure.Spin();


    return 0;
}

/**
 * Default constructor of the class
 */
Measures::Measures()
{
    // Preparing private acquisition of parameters
    pnh_ = new ros::NodeHandle("~");

    // Sleeping some random time to not collapse the system due to a big wake up
    // Waiting a time between 1 and 2 seconds
    // TODO Seems to have allways the same number -> 1.4
    double sleep_time = 1.0 + (rand() % 10 + 1)/10.0;
    ROS_INFO("SWAP: Waiting %f seconds before start", sleep_time);
    ros::Duration( sleep_time).sleep();

    // Getting the uav_id
    if (!pnh_->getParam("uav_id", uav_id_)) {
        ROS_FATAL("Measures: uav_id is not set. Closing the measures system");
    }



    // Specific namespace for UAL
    std::string ual_ns;
    if (!pnh_->getParam("ual_namespace", ual_ns))
        ual_ns = "";

    // Subscribing to the position of UAV
        std::string uav_topic_name = "/" + ual_ns + "uav_" + std::to_string(uav_id_) + "/ual" + pose_uav_topic.c_str();
        pos_uav_sub_ = nh_.subscribe(  uav_topic_name.c_str() , 1, &Measures::PoseReceived, this);


}

/**
 * Destructor to release the memory
 */
Measures::~Measures()
{

    // Releasing the memory
    delete pnh_;
}



/**
 * Executes the main loop of swap
 */
void Measures::Spin()
{
    while (ros::ok())
    {
        SpinOnce();

        // Sleeping to save time
       ros::Duration(0.05).sleep();
    }
}

/**
 * Executes the main loop of swap once
 */
void Measures::SpinOnce()
{

}


/**
 * Callback for own pose estimatimation
 */
void Measures::PoseReceived(const geometry_msgs::PoseStamped::ConstPtr& uav_pose)
{

    tf2::Quaternion q3(uav_pose->pose.orientation.x,
                       uav_pose->pose.orientation.y,
                       uav_pose->pose.orientation.z,
                       uav_pose->pose.orientation.w);
    q3.normalize();     //Avoids a warning


    double x   = uav_pose->pose.position.x;
    double y   = uav_pose->pose.position.y;
    double z   = uav_pose->pose.position.z;
    double yaw = tf2::getYaw(q3);



}

