#include <last_letter_2_headers.hpp>
#include "dynamics.hpp"
#include "aerodynamics.hpp"
#include "propulsion/propulsion.hpp"
#include "factory.hpp"
#include "model.hpp"




class Master
{
  public:
    // Model model;
    ros::NodeHandle nh;
    ros::Subscriber gazebo_sub;
    Model model;
    Master();
    // ~Master();
    void gazeboClockClb(const rosgraph_msgs::Clock::ConstPtr&);
};


