
 class Model
{
  public:
    last_letter_2_msgs::model_states model_states;
    last_letter_2_msgs::air_data airdata;
    last_letter_2_msgs::control_signals control_signals;
    last_letter_2_msgs::model_wrenches model_wrenches;
    double airspeed,alpha,beta;
    double u_r, v_r, w_r;   //relative body wind
    
    ros::NodeHandle nh;
    ros::ServiceClient states_client;
    ros::ServiceClient control_signals_client;
    ros::ServiceClient airdata_client;
    ros::ServiceClient apply_wrench_client;
    ros::ServiceClient sim_step_client;
    ros::ServiceClient pauseGazebo;

    ros::Publisher signals_publisher;
    last_letter_2_msgs::get_model_states_srv states_srv;
    last_letter_2_msgs::get_control_signals_srv signals_srv;
    last_letter_2_msgs::airdata_srv air_data;
    last_letter_2_msgs::apply_wrench_srv apply_wrench_srv;

    Dynamics dynamics;
    // Airdata airdata;
    Model();
    void modelStep();
    void getStates();
    void getControlSignals();
    void getAirdata();
    void calcAirdataTriplet();
    void calcWrenches();
    void applyWrenches();
    void simulationStep();
};