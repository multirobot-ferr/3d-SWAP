#include <list>
#include <iterator>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/LinkState.h>

struct Frame {
    Frame(double _x, double _y, double _z) {
        position.x = _x;
        position.y = _y;
        position.z = _z;
     }
    geometry_msgs::Point position;
};

struct KeyFrame {
    KeyFrame(Frame _frame, ros::Time _time): frame(_frame) {
        time = _time;
    }
    Frame frame;
    ros::Time time;
 
    bool operator<(const KeyFrame& _key) { return time < _key.time; }
 
    static Frame interpolate(const KeyFrame& _prev, const KeyFrame& _next, const ros::Duration& _elapsed) {
        double t_prev = _prev.time.toSec();
        double t_next = _next.time.toSec();
        double t_curr = _elapsed.toSec();
        double t = (t_curr - t_prev) / (t_next - t_prev);
        double x = (1.0 - t) * _prev.frame.position.x + t * _next.frame.position.x;
        double y = (1.0 - t) * _prev.frame.position.y + t * _next.frame.position.y;
        double z = (1.0 - t) * _prev.frame.position.z + t * _next.frame.position.z;
        return Frame(x, y, z);
    }
};

class GazeboAnimatedLink {
public:
    GazeboAnimatedLink(const std::string& _link_name, const std::string& _reference_frame = "map") {
        link_name_ = _link_name;
        reference_frame_ = _reference_frame;
        std::string link_state_pub_topic = "/gazebo/set_link_state";
        link_state_publisher_ = nh_.advertise<gazebo_msgs::LinkState>(link_state_pub_topic, 1);
    }

    void addKeyFrame(const KeyFrame& _key_frame) {
        key_frames_.push_back(_key_frame);
    }

    void play(double _fps) {
        if (key_frames_.size() < 2) {
            std::cerr << "At least two key frames needed!" << std::endl;  // TODO: Throw?
            return;
        }
        playing_ = true;
        key_frames_.sort();
        std::list<KeyFrame>::iterator next_frame = key_frames_.begin();
        // Wait for simulation time initialization
        while (ros::Time::now() == ros::Time(0)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        ros::Time play_started = ros::Time::now();
        ros::Rate rate(_fps);
        while (playing_ && ros::ok()) {
            ros::Duration elapsed = ros::Time::now() - play_started;
            //ROS_INFO("elapsed = %f", elapsed.toSec());
            while (next_frame->time.toSec() < elapsed.toSec()) {
                next_frame++;
                if (next_frame == key_frames_.end()) {
                    playing_ = false;
                    return;
                }
            }
            Frame curr_frame = next_frame->frame;
            if (next_frame != key_frames_.begin()) {
                // Interpolate!
                auto prev_frame = std::prev(next_frame);
                curr_frame = KeyFrame::interpolate(*prev_frame, *next_frame, elapsed);
            }
            gazebo_msgs::LinkState current;
            current.link_name = link_name_;
            current.pose.position.x = curr_frame.position.x;
            current.pose.position.y = curr_frame.position.y;
            current.pose.position.z = curr_frame.position.z;
            current.reference_frame = reference_frame_;
            link_state_publisher_.publish(current);
            ros::spinOnce();
            rate.sleep();
        }
    }

    void stop() { playing_ = false; }

protected:
    std::string link_name_;
    std::string reference_frame_;
    std::list<KeyFrame> key_frames_;
    bool playing_ = false;
    ros::NodeHandle nh_;
    ros::Publisher link_state_publisher_;
};

int main(int _argc, char** _argv) {
    ros::init(_argc, _argv, "gazebo_move");
    ROS_INFO("Starting gazebo_move");
    GazeboAnimatedLink yellow("Yelow cylinder object::grab_here");  // WATCHOUT: misspelled!
    yellow.addKeyFrame(KeyFrame(Frame(0.0, 0.0, 0.0), ros::Time(0.1)));
    yellow.addKeyFrame(KeyFrame(Frame(1.0, 1.0, 1.0), ros::Time(5.0)));
    yellow.addKeyFrame(KeyFrame(Frame(0.0, 0.0, 0.0), ros::Time(10.0)));
    yellow.play(25);
    return 0;
}
