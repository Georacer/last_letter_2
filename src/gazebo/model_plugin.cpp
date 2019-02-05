#include <ros/ros.h>
#include "ros/callback_queue.h"
#include <ros/service.h>
#include <rosgraph_msgs/Clock.h>
#include <gazebo/gazebo_client.hh> //gazebo version >6
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <last_letter_2/Model_states.h>
#include <last_letter_2/apply_wrench_srv.h>
#include <last_letter_2/get_model_states_srv.h>
#include <last_letter_2/Model_wrenches.h>
#include <boost/bind.hpp>
#include <ctime> // for timer
#include <iostream>

namespace gazebo
{
class model_plugin : public ModelPlugin
{
    // Pointer to the model
  private:
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection; //A class that encapsulates a connection

    ///  A node use for ROS transport
    ros::NodeHandle *rosNode;

    ///  A ROS subscriber
    //ros::Subscriber rosSub;

    // ROS publisher
    ros::Publisher state_pub;
    ros::ServiceServer apply_wrenches_server;
    ros::ServiceServer return_states_server;

    ///  A ROS callbackqueue that helps process messages
    ros::CallbackQueue wrenches_rosQueue;
    ros::CallbackQueue states_rosQueue;

    ///  A thread the keeps running the rosQueue
    std::thread rosQueueThread;

  public:
    model_plugin() : ModelPlugin() //constructor
    {
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) //Called when a Plugin is first created,
    {                                                         //and after the World has been loaded.Νot be blocking.
        this->model = _model;
        ROS_INFO("model_plugin just started");

        this->rosNode = new ros::NodeHandle; //Create a ros node for transport
        while (!this->rosNode->ok())
        {
            ROS_INFO("Waiting for node to rise");
        }

        // Spin up the queue helper thread.
        this->rosQueueThread =
            std::thread(std::bind(&model_plugin::QueueThread, this));
        //Connect a callback to the world update start signal.
        this->updateConnection = event::Events::ConnectWorldUpdateEnd(std::bind(&model_plugin::OnUpdate, this));
        ros::AdvertiseServiceOptions so = (ros::AdvertiseServiceOptions::create<last_letter_2::apply_wrench_srv>("last_letter_2/apply_wrench_srv",
                                                                                                                 boost::bind(&model_plugin::apply_wrench_on_model, this, _1, _2), ros::VoidPtr(), &this->wrenches_rosQueue));
        this->apply_wrenches_server = this->rosNode->advertiseService(so);
        so = (ros::AdvertiseServiceOptions::create<last_letter_2::get_model_states_srv>("last_letter_2/model_states",
                                                                                        boost::bind(&model_plugin::return_states, this, _1, _2), ros::VoidPtr(), &this->states_rosQueue));
        this->return_states_server = this->rosNode->advertiseService(so);
        // Publish code
        this->state_pub = this->rosNode->advertise<last_letter_2::Model_states>("last_letter_2/model_states", 1000);
    }

    //  ROS helper function that processes messages
    void QueueThread()
    {
        // static const double timeout = 0.01;
        ROS_INFO(" i am in QueueThread now\n");
        while (this->rosNode->ok())
        {
            // this->rosQueue.callAvailable(ros::WallDuration(timeout));
            this->wrenches_rosQueue.callAvailable();
            this->states_rosQueue.callAvailable();
        }
    }

    bool apply_wrench_on_model(last_letter_2::apply_wrench_srv::Request &req,
                               last_letter_2::apply_wrench_srv::Response &res)
    {
        ignition::math::Vector3d force, torque;
        float thrust;

        thrust = req.model_wrenches.thrust;
        force[0] = thrust;
        force[1] = 0;
        force[2] = 0;
        model->GetLink("airfoil")->AddLinkForce(force);
        model->GetJoint("body_to_arm")->SetVelocity(0, thrust);

        force[0] = req.model_wrenches.forces[0];
        force[1] = req.model_wrenches.forces[1];
        force[2] = req.model_wrenches.forces[2];
        model->GetLink("airfoil")->AddLinkForce(force);

        torque[0] = req.model_wrenches.torques[0];
        torque[1] = req.model_wrenches.torques[1];
        torque[2] = req.model_wrenches.torques[2];
        model->GetLink("airfoil")->AddRelativeTorque(torque);

        return true;
    }

    bool return_states(last_letter_2::get_model_states_srv::Request &req,
                       last_letter_2::get_model_states_srv::Response &res)
    {
        ignition::math::Vector3d relLinVel;
        relLinVel = model->GetLink("airfoil")->RelativeLinearVel();
        ignition::math::Vector3d rotation;
        rotation = model->GetLink("airfoil")->WorldPose().Rot().Euler();
        ignition::math::Vector3d relAngVel;
        relAngVel = model->GetLink("airfoil")->RelativeAngularVel();
        ignition::math::Vector3d position;
        position = model->GetLink("airfoil")->WorldPose().Pos();

        last_letter_2::Model_states model_states;
        model_states.header.stamp = ros::Time::now();
        model_states.x = position[0];
        model_states.y = position[1];
        model_states.z = position[2];
        model_states.roll = rotation[0];
        model_states.pitch = -rotation[1];
        model_states.yaw = -rotation[2];
        model_states.u = relLinVel[0];
        model_states.v = -relLinVel[1];
        model_states.w = -relLinVel[2];
        model_states.p = relAngVel[0];
        model_states.q = -relAngVel[1];
        model_states.r = -relAngVel[2];
        res.model_states = model_states;

        return true;
    }

    void OnUpdate()
    {
    }
};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(model_plugin)
} // namespace gazebo
