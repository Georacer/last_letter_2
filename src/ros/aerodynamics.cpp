
Aerodynamics::Aerodynamics(Model *parent)
{
    model = parent;
    initParam();
}

void Aerodynamics::calcWrench()
{
    calcAdditionalData();
    calcForces();
    calcTorques();
}

void Aerodynamics::calcAdditionalData()
{

    // airspeed, alpha, beta
    float u_r = model->model_states.u - model->airdata.wind_x;
    float v_r = model->model_states.v - model->airdata.wind_y;
    float w_r = model->model_states.w - model->airdata.wind_z;
    airspeed = sqrt(pow(u_r, 2) + pow(v_r, 2) + pow(w_r, 2));
    alpha = atan2(w_r, u_r);
    beta;
    if (u_r == 0)
    {
        if (v_r == 0)
        {
            beta = 0;
        }
        else
        {
            beta = asin(v_r / abs(v_r));
        }
    }
    else
    {
        beta = atan2(v_r, u_r);
    }
}

void Aerodynamics::calcForces()
{
    float qbar = 0.5 * rho * pow(airspeed, 2) * s;
    float sigmoid = (1 + exp(-M * (alpha - a0)) + exp(M * (alpha + a0))) / (1 + exp(-M * (alpha - a0))) / (1 + exp(M * (alpha + a0)));
    if (isnan(sigmoid))
        sigmoid = 0;
    float linear = (1.0 - sigmoid) * (c_lift_0 + c_lift_a * alpha);                         //Lift at small AoA
    float flatPlate = sigmoid * (2 * copysign(1, alpha) * pow(sin(alpha), 2) * cos(alpha)); //Lift beyond stall
    float c_lift_alpha = linear + flatPlate;

    float AR = pow(b, 2) / s;
    float c_drag_alpha = c_drag_p + pow(c_lift_0 + c_lift_a * alpha, 2) / (M_PI * oswald * AR);

    float ca = cos(alpha);
    float sa = sin(alpha);

    float p = model->model_states.p;
    float q = model->model_states.q;
    float r = model->model_states.r;
    float delta_a = model->control_signals.delta_a;
    float delta_e = model->control_signals.delta_e;
    float delta_r = model->control_signals.delta_r;
    float delta_t = model->control_signals.delta_t;

    // force calculation - - - - - - - - - - -expressed to body frame
    if (airspeed == 0)
    {
        aero_wrenches.drag = 0;
        aero_wrenches.lift = 0;
        aero_wrenches.fy = 0;
    }
    else
    {
        aero_wrenches.drag = qbar * ((-c_drag_alpha * ca + c_lift_alpha * sa) + (-c_drag_q * ca + c_lift_q * sa) * 0.5 / airspeed * c * q + (-c_drag_deltae * ca + c_lift_deltae * sa) * delta_e);
        aero_wrenches.lift = qbar * ((-c_drag_alpha * sa - c_lift_alpha * ca) + (-c_drag_q * sa - c_lift_q * ca) * 0.5 / airspeed * c * q + (-c_drag_deltae * sa - c_lift_deltae * ca) * delta_e);
        aero_wrenches.fy = qbar * (c_y_0 + c_y_b * beta + c_y_p * b / 2 / airspeed * p + c_y_r * b / 2 / airspeed * r + c_y_deltaa * delta_a + c_y_deltar * delta_r);
    }
}

void Aerodynamics::calcTorques()
{
    float qbar = 0.5 * rho * pow(airspeed, 2) * s;

    float ca = cos(alpha);
    float sa = sin(alpha);

    float p = model->model_states.p;
    float q = model->model_states.q;
    float r = model->model_states.r;
    float delta_a = model->control_signals.delta_a;
    float delta_e = model->control_signals.delta_e;
    float delta_r = model->control_signals.delta_r;
    float delta_t = model->control_signals.delta_t;

    if (airspeed == 0)
    {
        aero_wrenches.l = 0;
        aero_wrenches.m = 0;
        aero_wrenches.n = 0;
    }
    else
    {
        aero_wrenches.l = qbar * (b * (c_l_0 + c_l_b * beta + c_l_p * b / 2 / airspeed * p + c_l_r * b / 2 / airspeed * r + c_l_deltaa * delta_a + c_l_deltar * delta_r));
        aero_wrenches.m = qbar * (c * (c_m_0 + c_m_a * alpha + c_m_q * c / 2 / airspeed * q + c_m_deltae * delta_e));
        aero_wrenches.n = qbar * (b * (c_n_0 + c_n_b * beta + c_n_p * b / 2 / airspeed * p + c_n_r * b / 2 / airspeed * r + c_n_deltaa * delta_a + c_n_deltar * delta_r));
    }
}

void Aerodynamics::initParam()
{
    int id = 1;
    char paramMsg[50];
    sprintf(paramMsg, "airfoil%i/c_drag_p", id);
    if (!ros::param::getCached(paramMsg, c_drag_p))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_drag_deltae", id);
    if (!ros::param::getCached(paramMsg, c_drag_deltae))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_drag_q", id);
    if (!ros::param::getCached(paramMsg, c_drag_q))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_drag_0", id);
    if (!ros::param::getCached(paramMsg, c_drag_0))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_lift_0", id);
    if (!ros::param::getCached(paramMsg, c_lift_0))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_lift_deltae", id);
    if (!ros::param::getCached(paramMsg, c_lift_deltae))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_lift_q", id);
    if (!ros::param::getCached(paramMsg, c_lift_q))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_lift_a", id);
    if (!ros::param::getCached(paramMsg, c_lift_a))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/alpha_stall", id);
    if (!ros::param::getCached(paramMsg, a0))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/s", id);
    if (!ros::param::getCached(paramMsg, s))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/b", id);
    if (!ros::param::getCached(paramMsg, b))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c", id);
    if (!ros::param::getCached(paramMsg, c))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_y_0", id);
    if (!ros::param::getCached(paramMsg, c_y_0))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_y_b", id);
    if (!ros::param::getCached(paramMsg, c_y_b))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_y_p", id);
    if (!ros::param::getCached(paramMsg, c_y_p))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_y_r", id);
    if (!ros::param::getCached(paramMsg, c_y_r))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_y_deltaa", id);
    if (!ros::param::getCached(paramMsg, c_y_deltaa))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_y_deltar", id);
    if (!ros::param::getCached(paramMsg, c_y_deltar))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_l_0", id);
    if (!ros::param::getCached(paramMsg, c_l_0))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_l_p", id);
    if (!ros::param::getCached(paramMsg, c_l_p))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_l_b", id);
    if (!ros::param::getCached(paramMsg, c_l_b))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_l_r", id);
    if (!ros::param::getCached(paramMsg, c_l_r))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_l_deltaa", id);
    if (!ros::param::getCached(paramMsg, c_l_deltaa))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_l_deltar", id);
    if (!ros::param::getCached(paramMsg, c_l_deltar))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_m_0", id);
    if (!ros::param::getCached(paramMsg, c_m_0))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_m_a", id);
    if (!ros::param::getCached(paramMsg, c_m_a))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_m_q", id);
    if (!ros::param::getCached(paramMsg, c_m_q))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_m_deltae", id);
    if (!ros::param::getCached(paramMsg, c_m_deltae))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_n_0", id);
    if (!ros::param::getCached(paramMsg, c_n_0))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_n_b", id);
    if (!ros::param::getCached(paramMsg, c_n_b))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_n_p", id);
    if (!ros::param::getCached(paramMsg, c_n_p))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_n_r", id);
    if (!ros::param::getCached(paramMsg, c_n_r))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_n_deltaa", id);
    if (!ros::param::getCached(paramMsg, c_n_deltaa))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/c_n_deltar", id);
    if (!ros::param::getCached(paramMsg, c_n_deltar))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    sprintf(paramMsg, "airfoil%i/oswald", id);
    if (!ros::param::getCached(paramMsg, oswald))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
}