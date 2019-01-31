#include <last_letter_libs.hpp>


// class Model;

class Aerodynamics 
{
  public:
	float rho = 1.2250;  // need fix. rho=model->airdata.rho
	float M = 50;
	float oswald;
	float a0; 
	float c_drag_q, c_drag_deltae, c_drag_p, c_drag_0;
	float c_lift_0, c_lift_deltae, c_lift_q, c_lift_a;
	float b, c, s;
	float c_y_0, c_y_b, c_y_p, c_y_r, c_y_deltaa, c_y_deltar;
	float c_l_0, c_l_p, c_l_b, c_l_r, c_l_deltaa, c_l_deltar;
	float c_n_0, c_n_p, c_n_b, c_n_r, c_n_deltaa, c_n_deltar;
	float c_m_0, c_m_a, c_m_q, c_m_deltae;
	float airspeed, alpha, beta;

    last_letter_2::Aero_wrenches aero_wrenches;
    Model *model;
    Aerodynamics(Model *);
    // ~Aerodynamics();
	void init_param();
	void calc_wrench();
	void calc_additional_data();
    void calc_forces();
    void calc_torques();
};

 class Propulsion
 {
   public:
   float rho = 1.2250;  // need fix. rho=model->airdata.rho
   float s_prop, c_prop, k_motor, k_omega, k_t_p;
   float airspeed;
    last_letter_2::Prop_wrenches prop_wrenches;
    Model *model;
    Propulsion(Model *);
    // ~Propulsion();
	void init_param();
	void calc_additional_data();
	void calc_wrench();
    void calc_thrust();
    void calc_torque();
 };

class Model
{
  public:
	last_letter_2::Model_states model_states;
	last_letter_2::Airdata airdata;
	last_letter_2::Control_signals control_signals;
	last_letter_2::planeForces planeForces;
	
	ros::NodeHandle nh;
    ros::ServiceClient states_client;
	ros::ServiceClient control_signals_client;
	ros::ServiceClient airdata_client;
	ros::ServiceClient apply_wrench_client;
	ros::ServiceClient sim_step_client;
	ros::ServiceClient pauseGazebo;

	ros::Publisher signals_publisher;
	last_letter_2::model_states_srv states_srv;
	last_letter_2::control_signals_srv signals_srv;
	last_letter_2::airdata_srv air_data;
	last_letter_2::apply_wrench apply_wrench_srv;

	Aerodynamics aerodynamics;
	Propulsion propulsion;
	// Airdata airdata;
	Model();
    void model_step();
	void get_states();
	void get_control_signals();
	void get_airdata();
	void calc_wrenches();
	void apply_wrenches();
	void simulation_step();

};

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
