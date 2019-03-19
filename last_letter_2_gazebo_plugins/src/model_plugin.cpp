#include <ros/ros.h>
#include "ros/callback_queue.h"
#include <ros/service.h>
#include <rosgraph_msgs/Clock.h>
#include <gazebo/gazebo_client.hh> //gazebo version >6
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <last_letter_2_msgs/model_states.h>
#include <last_letter_2_msgs/apply_wrench_srv.h>
#include <last_letter_2_msgs/get_model_states_srv.h>
#include <last_letter_2_msgs/model_wrenches.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <kdl_parser/kdl_parser.hpp>
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
    ros::ServiceServer returnStates_server;

    ///  A ROS callbackqueue that helps process messages
    ros::CallbackQueue wrenches_rosQueue;
    ros::CallbackQueue states_rosQueue;

    ///  A thread the keeps running the rosQueue
    std::thread rosQueueThread;

    last_letter_2_msgs::model_states model_states;

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
        ros::AdvertiseServiceOptions so = (ros::AdvertiseServiceOptions::create<last_letter_2_msgs::apply_wrench_srv>("last_letter_2/apply_wrench_srv",
                                                                                                                      boost::bind(&model_plugin::applyWrenchOnModel, this, _1, _2), ros::VoidPtr(), &this->wrenches_rosQueue));
        this->apply_wrenches_server = this->rosNode->advertiseService(so);
        so = (ros::AdvertiseServiceOptions::create<last_letter_2_msgs::get_model_states_srv>("last_letter_2/model_states",
                                                                                             boost::bind(&model_plugin::returnStates, this, _1, _2), ros::VoidPtr(), &this->states_rosQueue));
        this->returnStates_server = this->rosNode->advertiseService(so);
        // Publish code
        this->state_pub = this->rosNode->advertise<last_letter_2_msgs::model_states>("last_letter_2/model_states", 1000);

        modelStateInit();
    }

    void modelStateInit()
    {
        //Get initial states from parameter server
        XmlRpc::XmlRpcValue list;

        if (!ros::param::getCached("init/position", list)) { ROS_FATAL("Invalid parameters for init/position in param server!"); ros::shutdown();}
        ignition::math::Vector3d xyz_pose(list[0], list[1], list[2]);
        ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        if (!ros::param::getCached("init/orientation", list)) { ROS_FATAL("Invalid parameters for init/orientation in param server!"); ros::shutdown();}
        ignition::math::Vector3d rpy_pose(list[0], list[1], list[2]);
        if (!ros::param::getCached("init/velLin", list)) { ROS_FATAL("Invalid parameters for init/velLin in param server!"); ros::shutdown();}
        ignition::math::Vector3d velLin(list[0], list[1], list[2]);
        if (!ros::param::getCached("init/velAng", list)) { ROS_FATAL("Invalid parameters for init/velAng in param server!"); ros::shutdown();}
        ignition::math::Vector3d velAng(list[0], list[1], list[2]);

        //Set the initial position and rotation
        ignition::math::Pose3d init_pose;
        init_pose.Set(xyz_pose, rpy_pose);
        this->model->SetWorldPose(init_pose);

        // Tranform linear and angular velocity from body frame to world frame for initialized launching
        KDL::Frame tranformation_matrix;
        tf2::Stamped<KDL::Vector> v_out;

        tranformation_matrix = KDL::Frame(KDL::Rotation::EulerZYX(-rpy_pose[2], -rpy_pose[1], rpy_pose[0]), KDL::Vector(0, 0, 0));
        v_out = tf2::Stamped<KDL::Vector>(tranformation_matrix.Inverse() * KDL::Vector(velLin[0], velLin[1], velLin[2]), ros::Time::now(), "airfoil");

        velLin[0] = v_out[0];
        velLin[1] = v_out[1];
        velLin[2] = v_out[2];

        v_out = tf2::Stamped<KDL::Vector>(tranformation_matrix.Inverse() * KDL::Vector(velAng[0], velAng[1], velAng[2]), ros::Time::now(), "airfoil");

        velAng[0] = v_out[0];
        velAng[1] = v_out[1];
        velAng[2] = v_out[2];

        // Set velocities on model
        this->model->SetLinearVel(velLin);  //NWU frame, keep yaw and pitch at zero
        this->model->SetAngularVel(velAng); //NWU frame
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

    //service that apply the calculated aerodynamic and propulsion wrenches on relative links of model
    bool applyWrenchOnModel(last_letter_2_msgs::apply_wrench_srv::Request &req,
                            last_letter_2_msgs::apply_wrench_srv::Response &res)
    {
        ignition::math::Vector3d force, torque;
        double thrust;

        thrust = req.model_wrenches.thrust;
        force[0] = thrust;
        force[1] = 0;
        force[2] = 0;
        model->GetLink("airfoil")->AddLinkForce(force);
        model->GetJoint("body_FLU_to_arm")->SetVelocity(0, thrust);

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

    bool returnStates(last_letter_2_msgs::get_model_states_srv::Request &req,
                      last_letter_2_msgs::get_model_states_srv::Response &res)
    {
        res.model_states = model_states;
        return true;
    }

    void OnUpdate()
    {
        ignition::math::Vector3d relLinVel;
        relLinVel = model->GetLink("airfoil")->RelativeLinearVel();
        ignition::math::Vector3d rotation;
        rotation = model->GetLink("airfoil")->WorldPose().Rot().Euler();
        ignition::math::Vector3d relAngVel;
        relAngVel = model->GetLink("airfoil")->RelativeAngularVel();
        ignition::math::Vector3d position;
        position = model->GetLink("airfoil")->WorldPose().Pos();

        model_states.header.stamp = ros::Time::now();
        model_states.header.frame_id = "body_FLU";
        model_states.x = position[0];
        model_states.y = position[1];
        model_states.z = position[2];
        model_states.roll = rotation[0];
        model_states.pitch = rotation[1];
        model_states.yaw = rotation[2];
        model_states.u = relLinVel[0];
        model_states.v = relLinVel[1];
        model_states.w = relLinVel[2];
        model_states.p = relAngVel[0];
        model_states.q = relAngVel[1];
        model_states.r = relAngVel[2];
        this->state_pub.publish(model_states);

        //publish tranform between gazebo inertia NWU and body frame FLU
        //if declaration is placed in data area of class, causes problems. So it is placed here.
        static tf2_ros::TransformBroadcaster broadcaster_;
        geometry_msgs::TransformStamped transformStamped_;
        tf2::Quaternion quat_;

        transformStamped_.header.stamp = ros::Time::now();
        transformStamped_.header.frame_id = "inertial_NWU";
        transformStamped_.child_frame_id = "body_FLU";
        transformStamped_.transform.translation.x = model_states.x;
        transformStamped_.transform.translation.y = model_states.y;
        transformStamped_.transform.translation.z = model_states.z;
        quat_.setRPY(model_states.roll, model_states.pitch, model_states.yaw);
        transformStamped_.transform.rotation.x = quat_.x();
        transformStamped_.transform.rotation.y = quat_.y();
        transformStamped_.transform.rotation.z = quat_.z();
        transformStamped_.transform.rotation.w = quat_.w();

        broadcaster_.sendTransform(transformStamped_);
        
        //publish body static tranformations between body_FLU and body_FRD
        transformStamped_.header.stamp = ros::Time::now();
        transformStamped_.header.frame_id = "body_FLU";
        transformStamped_.child_frame_id = "body_FRD";
        transformStamped_.transform.translation.x = 0;
        transformStamped_.transform.translation.y = 0;
        transformStamped_.transform.translation.z = 0;
        quat_.setRPY(M_PI, 0, 0);
        transformStamped_.transform.rotation.x = quat_.x();
        transformStamped_.transform.rotation.y = quat_.y();
        transformStamped_.transform.rotation.z = quat_.z();
        transformStamped_.transform.rotation.w = quat_.w();

        broadcaster_.sendTransform(transformStamped_);
    }
};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(model_plugin)
} // namespace gazebo
