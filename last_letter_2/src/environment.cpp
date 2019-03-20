#include <ros/ros.h>
#include <last_letter_2_msgs/airdata_srv.h>
#include <last_letter_2_msgs/air_data.h>
#include <last_letter_2_msgs/model_states.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <iostream>

#define Rd 287.05307  //Gas constant for dry air, J/kg K
#define L0 -6.5  //Temperature lapse rate, at sea level deg K/km

ros::Publisher pub;

class Environment
{
    public:
        last_letter_2_msgs::air_data airdata;
        last_letter_2_msgs::model_states states;
        double windDir, windRef, windRefAlt,surfSmooth, kwind;
        double T0; //Temperature at sea level, degrees K
        double P0; //Pressure at sea level, in HG
        double rho; //Density at sea level, kg/m**3
        double windDistU;
        double windDistV[2], windDistW[2];
        double Lu,Lw,sigmau,sigmaw;
        double allowTurbulence, dt, grav0;
        Environment();
        bool calcAirdata(last_letter_2_msgs::airdata_srv::Request& req, last_letter_2_msgs::airdata_srv::Response& res );
        void calcWind();
        void calcDens();
        void calcPres();
        void calcTemp();

};

Environment::Environment()
{
    grav0 = 9.81; // need fix:  last_letter_msgs::Geoid::EARTH_grav;
    if(!ros::param::getCached("/world/deltaT", dt)) {ROS_FATAL("Invalid parameters for -deltaT- in param server!"); ros::shutdown();}
    if(!ros::param::getCached("/environment/Dryden/use", allowTurbulence)) {ROS_FATAL("Invalid parameters for -/environment/Dryden/use- in param server!"); ros::shutdown();}

    //initialize atmosphere stuff
    if(!ros::param::getCached("/environment/groundTemp", T0)) {ROS_FATAL("Invalid parameters for -/environment/groundTemp- in param server!"); ros::shutdown();}
    T0 += 274.15;
    if(!ros::param::getCached("/environment/groundPres", P0)) {ROS_FATAL("Invalid parameters for -/environment/groundPres- in param server!"); ros::shutdown();}
    if(!ros::param::getCached("/environment/rho", rho)) {ROS_FATAL("Invalid parameters for -/environment/rho- in param server!"); ros::shutdown();}

   //Initialize bias wind engine
    if(!ros::param::getCached("/environment/windRef", windRef)) {ROS_FATAL("Invalid parameters for -/environment/windRef- in param server!"); ros::shutdown();}
    if(!ros::param::getCached("/environment/windRefAlt", windRefAlt)) {ROS_FATAL("Invalid parameters for -/environment/windRefAlt- in param server!"); ros::shutdown();}
    if(!ros::param::getCached("/environment/windDir", windDir)) {ROS_FATAL("Invalid parameters for -/environment/windDir- in param server!"); ros::shutdown();}
    if(!ros::param::getCached("/environment/surfSmooth", surfSmooth)) {ROS_FATAL("Invalid parameters for -/environment/surfSmooth- in param server!"); ros::shutdown();}
    windDir = windDir*M_PI/180; //convert to rad
    kwind = windRef/pow(windRefAlt,surfSmooth);

    //Initialize turbulence engine
    if(!ros::param::getCached("/environment/Dryden/Lu", Lu)) {ROS_FATAL("Invalid parameters for -/environment/Dryden/Lu- in param server!"); ros::shutdown();}
    if(!ros::param::getCached("/environment/Dryden/Lw", Lw)) {ROS_FATAL("Invalid parameters for -/environment/Dryden/Lw- in param server!"); ros::shutdown();}
    if(!ros::param::getCached("/environment/Dryden/sigmau", sigmau)) {ROS_FATAL("Invalid parameters for -/environment/Dryden/sigmau- in param server!"); ros::shutdown();}
    if(!ros::param::getCached("/environment/Dryden/sigmaw", sigmaw)) {ROS_FATAL("Invalid parameters for -/environment/Dryden/sigmaw- in param server!"); ros::shutdown();}
    windDistU = 0;
    for (int i=0;i<2;i++)
    {
        windDistV[i] = 0;
        windDistW[i] = 0;
    }

}

bool Environment::calcAirdata(last_letter_2_msgs::airdata_srv::Request &req, last_letter_2_msgs::airdata_srv::Response &res)
{
    states = req.states;

    calcTemp();
    calcWind();
    calcDens();
    calcPres();
    pub.publish(airdata);

    // Send back airdata
    res.airdata = airdata;
    return true;
}

///////////////////////
//Calculate temperature
void Environment::calcTemp()
{
    double altitude = -states.z;
    airdata.temperature = T0 + altitude / 1000.0 * L0;
}

void Environment::calcWind()
{
    //Call functions
    double Va, input, temp[2];
    airdata.wind_x = -cos(windDir) * kwind * pow(abs(states.z) + 0.001, surfSmooth);
    airdata.wind_y = -sin(windDir) * kwind * pow(abs(states.z) + 0.001, surfSmooth); //abs is used to avoid exp(x,0) which may return nan
    airdata.wind_z = 0;
    geometry_msgs::Vector3 wind;
    wind.x = airdata.wind_x;
    wind.y = airdata.wind_y;
    wind.z = airdata.wind_z;

    if (isnan(wind.x) || isnan(wind.y) || isnan(wind.z))
    {
        ROS_FATAL("earth wind NAN in environmentNode!");
        std::cout << wind.x << " " << wind.y << " " << wind.z << std::endl;
        ros::shutdown();
    }

    //claculate turbalent wind
    geometry_msgs::Vector3 airspeed;
    airspeed.x = states.u - wind.x;
    airspeed.y = states.v - wind.y;
    airspeed.z = states.w - wind.z;

    Va = sqrt(pow(airspeed.x, 2) + pow(airspeed.y, 2) + pow(airspeed.z, 2));

    if (allowTurbulence)
    {
        input = (((double)rand()) / (RAND_MAX)-0.5); //turbulence u-component

        windDistU = windDistU * (1 - Va / Lu * dt) + sigmau * sqrt(2 * Va / (M_PI * Lu)) * dt * input;

        input = (((double)rand()) / (RAND_MAX)-0.5); //turbulence v-component
        temp[0] = windDistV[0];
        temp[1] = windDistV[1];
        windDistV[1] = -pow(Va / Lu, 2) * dt * temp[0] + temp[1] + sigmau * sqrt(3 * Va / (M_PI * Lu)) * Va / (sqrt(3) * Lu) * dt * input;
        windDistV[0] = (1.0 - 2.0 * Va / Lu * dt) * temp[0] + dt * temp[1] + sigmau * sqrt(3 * Va / (M_PI * Lu)) * dt * input;

        input = ((double)rand() / (RAND_MAX)-0.5); //turbulence w-component
        temp[0] = windDistW[0];
        temp[1] = windDistW[1];
        windDistW[1] = -pow(Va / Lw, 2) * dt * temp[0] + temp[1] + sigmaw * sqrt(3 * Va / (M_PI * Lw)) * Va / (sqrt(3) * Lw) * dt * input;
        windDistW[0] = (1.0 - 2.0 * Va / Lw * dt) * temp[0] + dt * temp[1] + sigmaw * sqrt(3 * Va / (M_PI * Lw)) * dt * input;
    }

    if (isnan(windDistU) || isnan(windDistV[0]) || isnan(windDistW[0]))
    {
        ROS_FATAL("turbulence NAN in environmentNode!");
        std::cout << windDistU << " " << windDistV[0] << " " << windDistW[0] << std::endl;
        ros::shutdown();
    }

    // environment.wind = Reb/wind; //Rotate bias wind in body axes

    airdata.wind_x += windDistU; // add turbulence
    airdata.wind_y += windDistV[0];
    airdata.wind_z += windDistW[0];

    if (isnan(airdata.wind_x) || isnan(airdata.wind_y) || isnan(airdata.wind_z))
    {
        ROS_FATAL("body wind NAN in environmentNode!");
        std::cout << airdata.wind_x << " " << airdata.wind_y << " " << airdata.wind_z << std::endl;
        ros::shutdown();
    }
}

void Environment::calcDens()
{
    double altitude = -states.z;
    double Hb = 0, Tb = T0, Pb = P0, L = L0;
    double alt2pressRatio = (Pb / P0) * pow(1 - (L / Tb) * (altitude / 1000.0 - Hb), ((1000.0 * grav0) / (Rd * L))); //Corrected to 1 - (L/...)
    double alt2tempRatio = airdata.temperature / T0;
    double density = rho * alt2pressRatio / alt2tempRatio;
    airdata.density = density;
}

///////////////////////////////
//Calculate barometric pressure
void Environment::calcPres()
{
    double altitude = -states.z;
    double pressure;
    double Hb = 0, Tb = T0, Pb = P0, L = L0;
    pressure = Pb * pow(1 - (L / Tb) * (altitude / 1000.0 - Hb), ((1000.0 * grav0) / (Rd * L))); //Corrected to 1 - (L/...)
    airdata.pressure = pressure;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "environment_node");
    Environment env_object;
    ros::NodeHandle nh;
    ros::ServiceServer envir_server = nh.advertiseService("last_letter_2/airdata", &Environment::calcAirdata, &env_object);
    pub = nh.advertise<last_letter_2_msgs::air_data>("last_letter_2/Environment", 1);

    while (ros::ok())
    {
        ros::spin();
    }

    ros::shutdown();

    return 0;
}
