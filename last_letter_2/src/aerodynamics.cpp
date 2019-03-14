



Aerodynamics::Aerodynamics(Model *parent)
{
    model = parent;
}

void Aerodynamics::calcWrench()
{
    loadAirdataTriplet();
    calcForces();
    calcTorques();
}

void Aerodynamics::loadAirdataTriplet()
{
    // airspeed, alpha, beta
    airspeed= model->airspeed;
    alpha=model->alpha;
    beta=model->beta;
}

// Class NoAero contructor
NoAerodynamics::NoAerodynamics(Model *parent) : Aerodynamics(parent)
{   
    printf("no aerodynamics built\n");
}

void NoAerodynamics::calcForces()
{

}

void NoAerodynamics::calcTorques()
{

}

StdLinearAero::StdLinearAero(Model *parent) : Aerodynamics(parent)
{
    initParam();
    printf("stdLinearAero built\n");
}

void StdLinearAero::calcForces()
{
    double qbar = 0.5 * rho * pow(airspeed, 2) * s;

    double c_lift_alpha = liftCoeff(alpha);
    double c_drag_alpha = dragCoeff(alpha);

    double ca = cos(alpha);
    double sa = sin(alpha);

    double p = model->model_states.p;
    double q = model->model_states.q;
    double r = model->model_states.r;
    double delta_a = model->control_signals.delta_a;
    double delta_e = model->control_signals.delta_e;
    double delta_r = model->control_signals.delta_r;
    double delta_t = model->control_signals.delta_t;

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

void StdLinearAero::calcTorques()
{
    double qbar = 0.5 * rho * pow(airspeed, 2) * s;

    double ca = cos(alpha);
    double sa = sin(alpha);

    double p = model->model_states.p;
    double q = model->model_states.q;
    double r = model->model_states.r;
    double delta_a = model->control_signals.delta_a;
    double delta_e = model->control_signals.delta_e;
    double delta_r = model->control_signals.delta_r;
    double delta_t = model->control_signals.delta_t;

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

double StdLinearAero::liftCoeff(double alpha)
{
    double sigmoid = (1 + exp(-M * (alpha - a0)) + exp(M * (alpha + a0))) / (1 + exp(-M * (alpha - a0))) / (1 + exp(M * (alpha + a0)));
    if (isnan(sigmoid))
        sigmoid = 0;
    double linear = (1.0 - sigmoid) * (c_lift_0 + c_lift_a * alpha);                         //Lift at small AoA
    double flatPlate = sigmoid * (2 * copysign(1, alpha) * pow(sin(alpha), 2) * cos(alpha)); //Lift beyond stall
    return linear + flatPlate;
}

double StdLinearAero::dragCoeff(double alpha)
{
    double AR = pow(b, 2) / s;
    double c_drag_alpha = c_drag_p + pow(c_lift_0 + c_lift_a * alpha, 2) / (M_PI * oswald * AR);
    return c_drag_alpha;
}

// Class constructor
HCUAVAero::HCUAVAero (Model * parent) : StdLinearAero(parent)
{
	int id = 1;
	char s[100];
	Factory factory;
	// Create CLift polynomial
	sprintf(s,"airfoil%i/cLiftPoly",id);
	liftCoeffPoly =  factory.buildPolynomial(s);
	// Create CDrag polynomial
	sprintf(s,"airfoil%i/cDragPoly",id);
	dragCoeffPoly =  factory.buildPolynomial(s);
}

//////////////////////////
//C_lift_alpha calculation
double HCUAVAero::liftCoeff (double alpha)
{
	return liftCoeffPoly->evaluate(alpha);
}

//////////////////////////
//C_drag_alpha calculation
double HCUAVAero::dragCoeff (double alpha)
{
	return dragCoeffPoly->evaluate(alpha);
}

void StdLinearAero::initParam()
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