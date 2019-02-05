#include <ros/ros.h>
#include <last_letter_2/get_control_signals_srv.h>
#include <last_letter_2/control_signals.h>


float delta_a=0, delta_e=0, delta_r=0, delta_t=0;

void signal_callback(const last_letter_2::control_signals::ConstPtr &msg)
{
    delta_a=msg->delta_a;
    delta_e=msg->delta_e;
    delta_r=msg->delta_r;
    delta_t=msg->delta_t;
}

bool send_control_signals(last_letter_2::get_control_signals_srv::Request &req,
        last_letter_2::get_control_signals_srv::Response &res)
{
    res.signals.delta_a=delta_a;
    res.signals.delta_e=delta_e;
    res.signals.delta_r=delta_r;
    res.signals.delta_t=delta_t;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    printf("controller_node: just start\n");

    ros::NodeHandle nh;	
    ros::Subscriber joystick_input = nh.subscribe("last_letter_2/Joy_signals", 1000, signal_callback);
    ros::ServiceServer control_signals_server = nh.advertiseService("last_letter_2/control_signals", send_control_signals);
    
    while(ros::ok())
    {
        printf("controller_node: start spin\n");
        ros::spin();
    }

    ros::shutdown();

    return 0;
}
