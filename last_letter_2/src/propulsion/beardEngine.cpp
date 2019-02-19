BeardEngine::BeardEngine(Model *parent) : Propulsion(parent)
{
    initParam();
    printf("beard engine constructor\n");
}

void BeardEngine::calcThrust()
{
    double delta_t = model->control_signals.delta_t;	
    prop_wrenches.thrust = 1.0 / 2.0 * rho * s_prop * c_prop * (pow(delta_t * k_motor, 2) - pow(model->airspeed, 2));
}

void BeardEngine::calcTorque()
{
    double delta_t = model->control_signals.delta_t;
    prop_wrenches.torque = -k_t_p * pow((k_omega * delta_t), 2);
}

void BeardEngine::initParam()
{
    int id = 1;   // need fix
    char paramMsg[50];
    sprintf(paramMsg, "motor%i/s_prop", id);
    if (!ros::param::getCached(paramMsg, s_prop))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "motor%i/c_prop", id);
    if (!ros::param::getCached(paramMsg, c_prop))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "motor%i/k_motor", id);
    if (!ros::param::getCached(paramMsg, k_motor))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "motor%i/k_omega", id);
    if (!ros::param::getCached(paramMsg, k_omega))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "motor%i/k_t_p", id);
    if (!ros::param::getCached(paramMsg, k_t_p))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
}
