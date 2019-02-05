#include <ros/ros.h>
#include <string>
// #include <class_config.hpp>
#include <rosgraph_msgs/Clock.h>
#include <last_letter_2/get_model_states_srv.h>
#include <last_letter_2/get_control_signals_srv.h>
#include <last_letter_2/airdata_srv.h>
#include <last_letter_2/air_data.h>
#include <last_letter_2/control_signals.h>
#include <last_letter_2/model_states.h>
#include <last_letter_2/aero_wrenches.h>
#include <last_letter_2/prop_wrenches.h>
#include <geometry_msgs/Point.h>
#include <last_letter_2/model_wrenches.h>
#include <last_letter_2/apply_wrench_srv.h>
#include <std_srvs/Empty.h>
#include <math.h>
#include <ctime>

class Aerodynamics;
class Propulsion;
class Model;
class Master;