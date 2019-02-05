#include <last_letter_libs.hpp>

class Model
{
    public:
        last_letter_2::model_states model_states;
        last_letter_2::air_data airdata;
        last_letter_2::control_signals control_signals;
        last_letter_2::model_wrenches model_wrenches;

        ros::NodeHandle nh;
        ros::ServiceClient states_client;
        ros::ServiceClient control_signals_client;
        ros::ServiceClient airdata_client;
        ros::ServiceClient apply_wrench_client;
        ros::ServiceClient sim_step_client;
        ros::ServiceClient pauseGazebo;
        
        ros::Publisher signals_publisher;
        last_letter_2::get_model_states_srv states_srv;
        last_letter_2::get_control_signals_srv signals_srv;
        last_letter_2::airdata_srv air_data;
        last_letter_2::apply_wrench_srv apply_wrench_srv;

        Aerodynamics aerodynamics;
        Propulsion propulsion;
        // Airdata airdata;
        Model();
        void modelStep();
        void getStates();
        void getControlSignals();
        void getAirdata();
        void calcWrenches();
        void applyWrenches();
        void simulationStep();
};