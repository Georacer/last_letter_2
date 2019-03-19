#include "noEngine.cpp"
#include "beardEngine.cpp"
#include "electricEngine.cpp"

Propulsion::Propulsion(Model *parent) : tfListener(tfBuffer)
{
    model = parent;
}

void Propulsion::calcWrench()
{
    rotateWind();
    calcThrust();
    calcTorque();
}

void Propulsion::rotateWind()
{
    //load the wind vector which will be tranformed
    std::string motor_name_link="arm";
    v_in.header.frame_id = model->airdata.header.frame_id;
    v_in.vector.x = model->airdata.wind_x;
    v_in.vector.y = model->airdata.wind_y;
    v_in.vector.z = model->airdata.wind_z;
    // std::cout<<"frame="<<v_in.header.frame_id<<"    x="<<v_in.vector.x<<" y="<<v_in.vector.y<<" z="<<v_in.vector.z<<std::endl;
    v_out = tfBuffer.transform(v_in, motor_name_link);
    // std::cout<<"frame="<<v_out.header.frame_id<<"         x="<<v_out.vector.x<<" y="<<v_out.vector.y<<" z="<<v_out.vector.z<<std::endl<<std::endl;

    normalWind = v_out.vector.x;
    if (!std::isfinite(normalWind)) { ROS_FATAL("propulsion.cpp: NaN value in normalWind"); ros::shutdown(); }
    if (std::fabs(normalWind) > 1e+160) { ROS_FATAL("propulsion.cpp/rotateWind: normalWind over 1e+160"); ros::shutdown(); }
}