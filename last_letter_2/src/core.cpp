#include <master.hpp>
#include "model.cpp"
#include "master.cpp"
#include "dynamics.cpp"
#include "aerodynamics.cpp"
#include "propulsion/propulsion.cpp"
#include "factory.cpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "core");
    printf("master: just start\n");
    Master master;
    printf("master: just create master class object\n");
    while(ros::ok())
    {
        printf("master: start spinning\n");
        ros::spin();
    }


    ros::shutdown();

    return 0;
}
