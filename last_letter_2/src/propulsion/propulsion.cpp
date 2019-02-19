
#include "noEngine.cpp"
#include "beardEngine.cpp"

Propulsion::Propulsion(Model *parent)
{
    model=parent;
}

void Propulsion::calcWrench()
{
    calcThrust();
    calcTorque();
}
