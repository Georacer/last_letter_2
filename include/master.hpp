#include <last_letter_libs.hpp>

class Master
{
  public:
    // Model model;
    ros::NodeHandle nh;
    ros::Subscriber gazebo_sub;
    Model model;
    Master();
    // ~Master();
    void gazebo_clk_clb(const rosgraph_msgs::Clock::ConstPtr&);
};
