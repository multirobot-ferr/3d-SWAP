#include <list>
#include <iterator>
#include <ros/ros.h>
//#include <gazebo_msgs/LinkState.h>

struct Frame {
    Frame(double _test) { test = _test; }
    double test;
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
        return Frame((1.0 - t) * _prev.frame.test + t * _next.frame.test);
    }
};

class GazeboAnimatedLink {
public:
    GazeboAnimatedLink(const std::string& _link_name, const std::string& _reference_frame = "map") {
        link_name_ = _link_name;
        reference_frame_ = _reference_frame;
        //std::string link_state_pub_topic = "/gazebo/set_link_state";
        //link_state_publisher_ = nh_.advertise<gazebo_msgs::LinkState>(link_state_pub_topic, 1);
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
        ros::Time play_started = ros::Time::now();
        ros::Rate rate(_fps);
        while (playing_ && ros::ok()) {
            ros::Duration elapsed = ros::Time::now() - play_started;
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
            ROS_INFO("test = %f", curr_frame.test);
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
    GazeboAnimatedLink yellow("Yellow cylinder object::grab_here");
    yellow.addKeyFrame(KeyFrame(10.0, ros::Time(10.0)));
    yellow.addKeyFrame(KeyFrame(0.0, ros::Time(5.0)));
    yellow.addKeyFrame(KeyFrame(15.0, ros::Time(15.0)));
    yellow.play(25);
    return 0;
}
