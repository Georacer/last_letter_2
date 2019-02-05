#include <gazebo/gazebo_client.hh> //gazebo version >6
#include <ros/ros.h>
#include <gazebo/physics/World.hh>
#include <ignition/math/Vector3.hh>
#include "ros/callback_queue.h"
#include <boost/bind.hpp>
#include <last_letter_2/apply_wrench_srv.h>
#include <last_letter_2/model_wrenches.h>
#include <rosgraph_msgs/Clock.h>
#include <ctime> // for timer
#include <ros/service.h>
#include <thread>
#include <std_srvs/Empty.h>

namespace gazebo
{
class stepper : public WorldPlugin
{
  // Pointer to the World
private:
  physics::WorldPtr World;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection; //A class that encapsulates a connection

  ///  A node use for ROS transport
  ros::NodeHandle *rosNode;

  ///  A ROS subscriber
  //ros::Subscriber rosSub;

  // ROS publisher
  // ros::Publisher pub_;
  ros::ServiceServer srv_;

  ///  A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;

  ///  A thread the keeps running the rosQueue
  std::thread rosQueueThread;

public:
  stepper() : WorldPlugin() //constructor
  {
  }

  void Load(physics::WorldPtr _World, sdf::ElementPtr _sdf) //Called when a Plugin is first created,
  {                                                         //and after the World has been loaded.Νot be blocking.
    this->World = _World;
    ROS_INFO("stepper just started");

    this->rosNode = new ros::NodeHandle; //Create a ros node for transport
    while (!this->rosNode->ok())
    {
      ROS_INFO("Waiting for node to rise");
    }
    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&stepper::QueueThread, this));
    // Connect a callback to the world update start signal.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&stepper::OnUpdate, this));
    ros::AdvertiseServiceOptions so = (ros::AdvertiseServiceOptions::create<last_letter_2::apply_wrench_srv>("last_letter_2/step",
                                                                                                             boost::bind(&stepper::give_step, this, _1, _2), ros::VoidPtr(), &this->rosQueue));
    this->srv_ = this->rosNode->advertiseService(so);
  }

  //  ROS helper function that processes messages
  void QueueThread()
  {
    static const double timeout = 0.01;
    ROS_INFO(" i am in QueueThread now\n");
    while (this->rosNode->ok())
    {
      // this->rosQueue.callAvailable(ros::WallDuration(timeout));
      this->rosQueue.callAvailable();
    }
  }

  bool give_step(last_letter_2::apply_wrench_srv::Request &req,
                 last_letter_2::apply_wrench_srv::Response &res)
  {
    this->World->Step(1);

    return true;
  }

  void OnUpdate()
  {
  }
};
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(stepper)
} // namespace gazebo
