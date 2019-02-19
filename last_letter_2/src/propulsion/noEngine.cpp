NoEngine::NoEngine(Model *parent) : Propulsion(parent)
{
    printf("No engine constructor\n");
}

void NoEngine::calcThrust()
{
    prop_wrenches.thrust = 0;
}

void NoEngine::calcTorque()
{
    prop_wrenches.torque = 0;
}